# Solar MPPT tracker

Maximum-power-point tracker for direct-drive solar aircraft: PV array → ESC →
BLDC, **no battery anywhere**. The MCU logic supply sits on the same bus as the
panel, so a bus collapse is a loss of the aircraft, and most of the design is
about that rather than about squeezing out the last percent of power.

Files: `Src/mppt.c`, `Inc/mppt.h`, regression harness in `test/mppt_sim/`.

---

## 1. Enabling it

The module is **off** unless the board's block in `Inc/targets.h` defines
`USE_MPPT`. Today only `EGAN_MPPT_L431` does.

When it is off, `Src/mppt.c` compiles to a single typedef and the three call
sites in `Src/main.c` preprocess to `((void)0)` via macros at the bottom of
`mppt.h`. Every other target in the tree is unaffected, byte for byte.

```c
#ifdef EGAN_MPPT_L431
...
#define USE_MPPT
#endif
```

Any constant in `mppt.h` can be overridden from the board block — they are all
`#ifndef`-guarded.

---

## 2. Where it hooks into AM32

Three call sites, no deletions.

| Location | Call | Why there |
|---|---|---|
| `Src/main.c` includes | `#include "mppt.h"` | Must follow `targets.h`, which is where `USE_MPPT` comes from |
| End of `main()`, just before `while (1)` | `mppt_init()` | ADC DMA has been running through the whole startup tune, so `ADC_raw_volts` is live; duty is still 0 and the panel is open-circuit, which hands init a real Voc measurement for free |
| End of the 1 kHz block in `tenKhzRoutine()` | `mppt_1khz_update()` | Sees the settled results of AM32's current-limit / stall / speed loops |
| In `tenKhzRoutine()`, after `last_duty_cycle = duty_cycle` | `mppt_apply_duty(&duty_cycle)` | **After** the ramp/slew block and **after** the ramp state is captured |

That last one is the only structural change to AM32, and it is worth
understanding before touching it.

Duty *reductions* have to bypass AM32's slew limiter — a bus collapse must be
caught in one tick, and a ramp rate of a few duty units per step would take
~100 ms to shed load.

`last_duty_cycle = duty_cycle` was also moved to *before* the ceiling is
applied, so AM32's ramp tracks its own intent rather than the clamped value.
Otherwise every MPPT duty reduction resets the ramp, and with `RAMP_SPEED_*`
of 1 duty unit per 20 kHz tick AM32 then has to crawl back: a 26 ms Voc sweep
cost **78 ms** of recovery instead of 28 ms.

The consequence is that AM32's ramp no longer paces the MPPT, so the MPPT has
to pace itself — that is what `MPPT_DUTY_RISE_MAX` is for. It is not optional;
see §6.6.

---

## 3. Runtime states

| State | Meaning |
|---|---|
| `OFF` | Disarmed / not running. A conservative duty ceiling still applies |
| `SEED` | Jumped to 0.78 × Voc, letting the inner PI settle |
| `TRACK` | Normal tracking — see the two modes below |
| `RECOVER` | Bus collapse — shedding load. Safety path |
| `VOC` | Duty 0, measuring true open-circuit voltage |

`mppt.state`, `mppt.vref`, `mppt.voc_est`, `mppt.duty_applied`,
`mppt.hill_climb`, `mppt.collapse_events`, `mppt.reseed_events` and
`mppt.voc_sweeps` are all safe to stream over AM32's serial telemetry while
commissioning.

### 3.1 Two tracking modes, chosen automatically

`TRACK` runs one of two things depending on whether the current channel
carries usable information:

- **Fractional-Voc + rpm feedback** (`mppt.hill_climb == 0`) —
  `vref = k × voc_est`, with the inner PI regulating to it fast and `k`
  adapted slowly from rotor speed. Needs no current measurement at all.
  See §3.3.
- **Hill-climbing** (`mppt.hill_climb == 1`) — dP/dV incremental conductance
  with the two-step escalation, as originally designed.

The switch is `mppt.i >= MPPT_TRACK_I_MIN`, derived from
`MPPT_TRACK_MIN_COUNTS` ADC counts rather than an absolute current, so it
follows the sense chain automatically. Force one mode with `MPPT_MODE`.

Why bother: on a 13.4 V / 120 mA panel the whole array spans 20 ADC counts,
and *every* tuning of the hill-climber topped out at 86.6% while plain
fractional-Voc held 96.8% dead flat. Above roughly 125 counts of operating
current, hill-climbing is the better of the two and takes over.

### 3.3 RPM feedback — how `k` finds the real MPP

Fractional-Voc is open-loop on a constant, and the constant is usually wrong.
The textbook 0.781 put the setpoint at 10.47 V on the bench panel against a
true Vmpp of 11.32 V.

Hill-climbing on measured *power* is not available here — 120 mA spans 20 ADC
counts, so dP is mostly quantisation. But the ESC has a much better signal
sitting unused: `e_com_time`, the electrical revolution period, is ~0.5% of
speed per LSB at 44–47 krpm, about ten times finer than one current count, and
averages over hundreds of revolutions.

For a fixed-pitch prop, load torque goes as ω² so shaft power goes as ω³, and
**maximum rpm is maximum shaft power**. On an aircraft that is not a proxy for
the objective, it *is* the objective — and it absorbs motor efficiency
variation, which panel-power MPPT ignores by construction. Note the sensitivity
that follows from the cube law: 44 krpm → 47 krpm is 6.8% in speed but **22% in
shaft power**.

So: perturb `k`, wait out the prop's mechanical time constant, average the
period, keep the direction if rpm improved and reverse if it did not. A tie
also reverses, which makes it dither one step either side of the peak instead
of drifting off it.

What is adapted is the *ratio*, not a voltage offset. The FOCV error is
`(k_true − k₀) × Voc`, which scales with Voc, so correcting `k` stays right as
irradiance and temperature move Voc; a fixed voltage trim would not.

Guards: throttle must hold still across a whole cycle, the motor must be spun
up, `e_com_time` must not be the 65408 pre-spin-up sentinel, and the PI must
have authority — if duty is railed, `vref` is not setting the operating point
and rpm says nothing about `k`. `k` is clamped to `MPPT_K_MIN_Q8`…`MPPT_K_MAX_Q8`
so no amount of bad rpm data can walk the setpoint anywhere dangerous.

Measured on the bench panel, 40 s: `k` 200 → 216, rpm 1532 → 1548,
**96.75% → 99.64%**. `k` is not stored in EEPROM; it re-learns in ~10 s of
steady throttle.

### 3.2 Voc measurement

1. **At boot, deferred** — the bus at power-up is Voc, but only once nothing
   is loading it. `mppt_init()` deliberately does *not* sample: AM32 plays its
   startup tunes by driving the motor coils, which drains Cbus, and the arming
   tune fires *later* than `mppt_init()` is even called. Instead the capture
   waits for a quiet window — no tone pending, no current, no applied duty,
   and the bus voltage stationary for `MPPT_BOOT_VOC_SETTLE_MS` (300 ms).
   "Stationary" is measured against the voltage at the *start* of the window,
   not tick to tick; see §6.14. It keeps retrying, so arming early only delays
   the capture rather than losing it. Result lands in `mppt.voc_boot`.
2. **Opportunistic** — when current is below `MPPT_VOC_I_TH` **and**
   `duty_applied` is below `MPPT_VOC_DUTY_TH`, IIR `voc_est` toward the
   measured bus voltage. Both conditions are required; see §6.5. In practice
   this fires every time the throttle comes down. Measured: `voc_est` holds
   13.39 V through a loaded run, and converges to 13.36 V against a true
   13.40 V within a 2 s throttle chop.

There is also a **periodic duty-0 sweep**, and it is **disabled by default**.
Do not enable it without reading §6.9 — on this hardware duty 0 is a brake,
not a coast.

`MPPT_VOC_NOMINAL` is only a fallback for the window before the first real
measurement.

---

## 4. Cost on EGAN_MPPT_L431

- **RAM:** 104 bytes (`mppt_t`) + 4 static bytes.
- **Flash:** ~2.5 kB (measured 3564 B of x86-64 `.text`; Thumb-2 is smaller).
- **CPU:** every division is by a compile-time constant and gets
  strength-reduced to multiply-high-plus-shift. Roughly 1% of an 80 MHz
  Cortex-M4. **Measure it** with a GPIO toggle before relying on that.
- No FPU use, no `float`, no `double`, no dynamic allocation.

---

## 5. Verification status

Run the harness with `test/mppt_sim/run.sh [seconds]`. It compiles the
**shipping** `Src/mppt.c` — no stubbed algorithm — against a PV +
bus-capacitor + BLDC + propeller plant, pulling every board constant from the
real `Inc/targets.h`.

Two array scales are covered, because the failure modes are completely
different at each and each one hid a bug the other could not show.

| Test | Result |
|---|---|
| `ARRAY=flight` (12 V / 3 A), 3.4 s bank + cloud profile | 96.72%, 0 brownouts, 0 collapses |
| `ARRAY=flight`, 60 s soak | 97.01%, 0 brownouts |
| `ARRAY=bench` (13.4 V / 120 mA), 4 s | 97.39% — rpm tracker has not converged yet |
| `ARRAY=bench`, 60 s soak | **99.71%**, k converged 200 → 216 |
| rpm tracker disabled, same run | 96.75% — the 3% is what the tracker buys |
| Steady rpm vs fixed k, 196…228 | peak at k=216, matches where the tracker lands |
| Throttle chop, Voc recovery | `voc_est` 13.36 V vs true 13.40 V, no corruption under load |
| `run.sh coast` — directed tests: coast/unload, boot Voc, rpm guards, 29 assertions | all pass, under UBSan/ASan |
| Cbus 100 µF … 1000 µF, flight scale | 95.8% throughout, 0 brownouts |
| Cbus 2200 µF | **fails** — 1 collapse, −1.32 A regeneration |
| `-fsanitize=undefined,address` | clean |
| `-Wall -Wundef -Wextra -Werror` (AM32's own flags) | clean, module and call sites |
| Call sites compiled against `main.c`'s real types, 4 boards | clean |
| int32 overflow audit, all arithmetic paths | worst case 79.3% of `INT32_MAX` |

**Not verified:** nothing has been cross-compiled with `arm-none-eabi-gcc` or
run on hardware. Build `make EGAN_MPPT_L431` yourself before trusting any of
the flash/RAM/cycle numbers above.

Keep Cbus in roughly the 100–1000 µF band. The 2200 µF failure is a real
stability bound, not a harness artefact: a large bus capacitor slows the panel
voltage's response to a duty change enough that the inner PI loses phase
margin.

---

## 6. Defects found and fixed during integration

### 6.1 `zero_crosses` type mismatch — periodic in-flight thrust collapse

`mppt.c` declared `extern uint16_t zero_crosses;`. In `Src/main.c:554` it is
`volatile uint32_t zero_crosses;`, and `zcfoundroutine()` increments it with no
upper bound. Separate translation units, so the linker never sees the
disagreement — it just reinterprets the object's bytes.

Consequence: the module saw only the low half-word, which at ~4800
zero-crossings/s wrapped to 0 roughly **every 13.7 seconds**. Each wrap dropped
the value back under `MPPT_STARTUP_ZC_COUNT`, which slams the PI output ceiling
from 2000 to `MPPT_STARTUP_DUTY_MAX` (400) and clamps the integrator to match.

Measured in simulation: duty **1494 → 400 in one tick**, the motor's back-EMF
then exceeding the panel voltage, and **2.7 A of regeneration into the array
(−66 W)** with the bus pushed above Voc — every 13.7 s, for ~30 ms at a time.

The shipped harness also declared it `uint16_t` **and** capped it at 65000,
which is why the original test never showed this. `test/mppt_sim/harness.c` now
uses the real types and lets the counter run free.

### 6.2 The control tick is 952 Hz, not 1000 Hz

AM32's "1 kHz" block is

```c
one_khz_loop_counter++;
if (one_khz_loop_counter > PID_LOOP_DIVIDER) { ...; one_khz_loop_counter = 0; }
```

which fires every `PID_LOOP_DIVIDER + 1` calls, not every `PID_LOOP_DIVIDER`.
At `LOOP_FREQUENCY_HZ 20000` / `PID_LOOP_DIVIDER 20` that is 20000/21 = **952
Hz**. The hard-coded `MPPT_TICK_HZ 1000` therefore put KI 5% high and stretched
every millisecond timeout in the module by the same 5%.

`MPPT_TICK_HZ` is now derived from the target's own constants, and
`MPPT_RECOVER_MS` / `MPPT_RECOVER_OK_MS` are converted to ticks rather than
being counted as if ticks were milliseconds.

### 6.3 Constants were physically unreachable on this board

`TARGET_VOLTAGE_DIVIDER 47` caps the measurable bus at 3300 mV × 4.7 =
**15.51 V**, but the module shipped with `MPPT_VOC_NOMINAL 2400` (24 V),
`MPPT_V_COLLAPSE 1400` and `MPPT_V_ABSOLUTE_MIN 1100`.

Dropped onto this board unchanged the result is not subtle: the collapse
threshold sits above the array's own Voc, the tracker enters `RECOVER` on the
first tick and never leaves, duty stays 0 and the motor never spins.
**0.00% tracking efficiency** in the harness.

`mppt.h` is now scaled for a ~12 V array, and a block of `_Static_assert`s at
the end of the header refuses to compile a configuration where:

- `MPPT_VOC_NOMINAL` exceeds what the divider can measure
- `MPPT_V_ABSOLUTE_MIN` ≥ `MPPT_V_COLLAPSE`
- the collapse threshold leaves no usable vref band below Voc
- the fractional-Voc seed sits below the collapse threshold
- `MPPT_AVG_TICKS` is outside `1..MPPT_PERIOD_TICKS`
- the tick rate falls below 500 Hz
- a recover timeout rounds to zero ticks
- `duty_q12 * MPPT_RECOVER_DECAY_Q8` would overflow int32

### 6.4 Hill-climbing on a quantised current signal

Symptom on hardware: "works kind of, but is rough and stutters a lot", on a
13.4 V / 120 mA panel.

`CURRENT_OFFSET 2500` puts zero current at ADC raw 3106, and one count is
5.9 mA at the true 136 mV/A. A 120 mA array therefore spans **20 ADC counts
end to end**. One count of current moves `dp = v*i` by 105 mW — against a
deadband of 50 mW. The tracker was reversing direction on quantisation.

Three things came out of it:

- `MPPT_DP_QUANT_MULT` — the power deadband now has a runtime floor of
  `3 × v × I_LSB`, computed from the live voltage. This alone took the bench
  panel from 77.6% to 90.7%.
- `MPPT_TRACK_MIN_COUNTS` — below 125 counts of operating current the module
  runs fractional-Voc instead of hill-climbing. 96.8% instead of 86.6%.
- `MPPT_ARRAY_ISC` — the array nameplate is now a declared input, and every
  current-domain threshold derives from it with static assertions.

### 6.5 `MPPT_VOC_I_TH` above the array's own Isc

Set to 0.20 A during the previous integration, on a panel whose Isc is 0.12 A.
`i < MPPT_VOC_I_TH` was therefore true even at short circuit, so the
open-circuit estimator ran while the panel was fully loaded and `voc_est`
converged on the *loaded* bus voltage. Since the `vref` clamp band is a
fraction of `voc_est`, the band collapsed downward and dragged the operating
point with it: 13.01 V → 7.75 V over one second, 73% of available power.

Fixed three ways: the threshold defaults to 5% of `MPPT_ARRAY_ISC` (floored at
the ADC resolution), a `_Static_assert` refuses to build if it is not
comfortably below Isc, and the estimator now requires low duty as well as low
current.

Note this particular failure needs the *true* 136 mV/A to bite. With
`MILLIVOLT_PER_AMP` at the as-built 13 the reported current is inflated ~10×
and the threshold is accidentally cleared — which is why the board reads
"rough" rather than "parks itself on the floor".

### 6.6 Duty rise-rate limit

Introduced by, and required by, the `last_duty_cycle` move in §2. With AM32's
ramp no longer pacing the MPPT, the Voc sweep's exit restored the integrator
*and* added a full proportional term — `v` is at Voc, so the error is large
and positive — stepping duty 0 → 1451 in one tick. Harmless on the 120 mA
bench panel; on the 3 A array it pulled 6.3 A out of Cbus and put the bus at
**4.6 V**. `MPPT_DUTY_RISE_MAX` caps the rise at 100 duty units per tick, with
anti-windup against that limit.

### 6.7 Telemetry gain vs control gain

`MILLIVOLT_PER_AMP` is deliberately set to 13 instead of the true 136 so DShot
telemetry, which reports whole amps, shows something on a sub-amp panel. The
MPPT now takes `MPPT_MILLIVOLT_PER_AMP` instead, so a display calibration
cannot put a control loop's thresholds into fictional amps.

The sim harness models this properly: `HW_MVA` is the hardware, targets.h is
the firmware's belief, and they are allowed to disagree.

### 6.9 The Voc sweep was braking the motor

Symptom on hardware: hard stutter every interval, the motor audibly and
physically braked rather than coasting.

"Duty 0" is not coasting on an AM32 ESC. `phaseAPWM()` in `phaseouts.c` puts
the low-side FET into alternate (PWM) mode whenever `eepromBuffer.comp_pwm` is
set, so the low side is driven complementary to the high side. At duty 0 that
means the low-side FETs are on ~100% of the time and the energised winding
pair is **shorted through them** — textbook synchronous braking. The sweep was
applying the brakes once per interval.

The energy cost was never the issue (0.75% at a 15 s interval). Braking a
propeller in flight is. The sweep now defaults to disabled and compiles out
entirely; boot Voc plus the opportunistic estimator replaces it.

Before concluding the sweep was needed, the alternative was tested: a slow
heavily-averaged hill-climber that would not need an accurate in-flight Voc at
all. One tuning looked excellent (98.7% at a 25-tick period), but it is a
knife-edge — 78% at 15 ticks, 98.7% at 25, 93% at 40, and 88.6% / 98.7% /
80.6% across Cbus 100 µF / 470 µF / 1000 µF. Fractional-Voc holds 96.8% flat
across all of it. Not adopted.

### 6.10 Opportunistic Voc gated on the wrong duty

The estimator tested `mppt.duty`, which in the gated state is deliberately set
to a conservative *ceiling* of `MPPT_STARTUP_DUTY_MAX` (400) rather than to
the applied duty. So the condition `duty < MPPT_VOC_DUTY_TH` was false exactly
when the throttle was down and the measurement was there for the taking. It
now tests `mppt.duty_applied`, the value actually handed to the PWM.

`voc_est` is also now clamped to `MPPT_ADC_FULL_SCALE_V` — no estimate above
what the divider can measure.

### 6.12 The undervoltage cut was braking too

Same root cause as §6.9, on the path where it matters most. `mppt_apply_duty()`
set `*duty_cycle = 0` for the `MPPT_V_ABSOLUTE_MIN` cut, and the tail of the
`RECOVER` exponential decay reached ~0 as well — both of which short the
winding through the low-side FETs.

On the safety path this is worse than rough. Braking during a bus collapse
dumps the rotor's kinetic energy into winding resistance as heat, so when the
bus recovers the motor is slow; low speed means low back-EMF, which means
re-applying duty draws a large surge, which collapses the bus again. That is
the limit cycle.

`mppt_coast()` now floats all six FETs with `allOff()` — the same primitive
AM32's own low-voltage cutoff uses — re-asserted every 20 kHz tick, because
`comStep()` restores the driven pin modes at each commutation and would
otherwise undo it within one electrical step.

Two things this cost getting right:

- **Coast is gated on `state == RECOVER`, not on low duty.** Gating on duty
  alone looks equivalent and is not: duty legitimately passes through zero on
  every spin-up, so a duty-only test floats the FETs exactly when the motor is
  trying to start. Measured on the bench panel: 97.4% → 95.8% and a visibly
  disturbed start.
- **The hard cut needed its own `allOff()`.** That branch returns early, before
  the shared one, so it was setting the coast flag and then leaving the winding
  shorted anyway. Caught by `test_coast.c`, not by the plant model.

Note the threshold matters for a physical reason: above roughly 2% duty the
complementary FETs are doing useful synchronous rectification and feeding the
collapsing bus. It is only at genuinely zero duty that the winding is a dead
short and the rotor's energy goes to heat for no return. Coasting earlier than
that would give away real energy.

### 6.13 The rpm tracker stalled short of the peak

First cut used `MPPT_K_STEP_Q8 = 1`. Near the peak the P–V curve is flat, so a
one-unit step changed rpm by 0.055% against a 0.1% deadband — every comparison
came back a tie, the tracker reversed every cycle, and it parked at k=203 while
the optimum was 216. **A tracker that stalls looks exactly like one that has
converged**, which is what makes this worth writing down.

The step has to beat the deadband. Measured, 40 s, true optimum k=216:

| step | result |
|---|---|
| 1 | k=203, 97.5% |
| 2 | k=210, 99.4% |
| 3 | k=212, 99.6% |
| **4** | **k=216, 99.6%** |
| 6 | overshoots, gives accuracy back to the dither |

Dropping the deadband to zero also converges, but only because the sim has no
rpm noise; the deadband is what makes it survive real jitter. Raising the step
is the robust fix.

### 6.14 Boot-Voc "settled" test defeated by a slow ramp

The settle test originally compared `v` against the *previous tick*. A slow
steady ramp whose per-tick step sits just under the threshold passes that test
indefinitely — the bus could climb by threshold × 300 ms, i.e. volts, and still
be declared settled, latching a Voc well below the real one.

It now anchors to the voltage at the start of the window, which bounds total
drift to `MPPT_BOOT_VOC_STABLE_DV` (0.10 V) across the whole 300 ms. Caught by
a directed test using a deliberately slow ramp; a fast ramp passes either
version and would have hidden it.

### 6.15 `reg_ticks` never incremented in FOCV mode

The "does the PI have authority?" counter was updated *after* the
fractional-Voc branch, which breaks out early. So in FOCV mode it stayed pinned
at 0 — and the rpm tracker gates on it, so the tracker would never have run at
all. Moved above the branch.

### 6.16 Dead code

`MPPT_REG_BAND` was defined and never used — the "does the PI have authority?"
gate is implemented purely as a duty-rail check. Removed. The `duty_applied`
static was written in three places and read nowhere; it is now
`mppt.duty_applied`, visible to telemetry.

---

## 7. Open items — bench these before flying

These are **not** fixed. They need hardware, not more simulation.

1. **The two safety thresholds are placeholders.** `MPPT_V_COLLAPSE 700` and
   `MPPT_V_ABSOLUTE_MIN 550` were chosen to be self-consistent with a 12 V
   array, *not* from this board's 3.3 V regulator dropout. With no battery on
   the bus they are the only thing keeping the MCU alive. Measure the dropout,
   add margin, set them, and size Cbus from:

   ```
   Cbus >= (I_load - I_sc) * MPPT_RECOVER_MS / (V_collapse - V_dropout)
   ```

2. **`MPPT_ARRAY_ISC` is set for the bench panel, not the flight array.**
   `Inc/targets.h` declares 0.12 A. Change it — and `MPPT_VOC_NOMINAL` — when
   the real array goes on, or the module will stay in fractional-Voc mode and
   leave a couple of percent on the table. `ARRAY=flight ./test/mppt_sim/run.sh`
   exercises the hill-climb path in the meantime.

3. **Zero-current calibration is unverified.** With `CURRENT_OFFSET 2500` and
   the true 136 mV/A, 1 mV of sense-amp offset drift moves the reading ~7.4 mA.
   `MPPT_VOC_I_TH` is now 20 mA on this board, which is under three ADC counts.
   Log the reading at true zero current and confirm it settles below that. The
   periodic sweep is the primary Voc source so this is no longer load-bearing,
   but it is still worth knowing.

4. **`voc_est` only moves when the throttle comes down.** With the periodic
   sweep disabled, the estimate is refreshed at boot and at every throttle
   chop, and is otherwise held. If the panel warms substantially during a long
   full-throttle run the estimate goes stale high — about -0.3%/°C — and the
   FOCV setpoint drifts above Vmpp with it. Per TI TIDA-010042 a setpoint
   within 2.5% of Vmpp still returns >99.5% of available power, so this is
   worth perhaps 1-2% in the worst case. Watch `mppt.voc_est` in telemetry
   across a long run before deciding it matters.

5. **Only ~24% of the current ADC range is usable.** The 2.5 V offset puts zero
   current at raw 3106, so positive current spans raw 3106–4095 and tops out at
   **5.85 A**. Negative current clamps to 0, meaning regeneration is invisible
   to the tracker. Check that 5.85 A is above your worst-case draw.

6. **`MPPT_VOLTAGE_OFFSET 6`** in the `EGAN_MPPT_L431` block is referenced
   nowhere in the tree. Left in place, but it does nothing.

7. **Right shifts of negative values.** The IIR filters rely on `>>` being
   arithmetic, which GCC guarantees but ISO C does not. It also biases the
   filter output low by under 40 mV / 40 mA. Harmless, but do not "clean up"
   the shifts into divisions without re-checking the tuning.

---

## 8. Bench-up order

1. **Power supply + resistor, no motor.** Confirm `mppt.v` tracks the real bus
   voltage and `mppt.voc_est` settles at the supply voltage. Confirm the
   current reading at true zero — that sets item 7.2 above.
2. **Panel, no prop.** Confirm the PI holds `v` at `vref`, and tune KP/KI per
   the recipe at the top of `Src/mppt.c`.
3. **Panel + prop, restrained.** Watch `mppt.collapse_events`. It should stay
   at 0. If it climbs, lower `MPPT_STARTUP_DUTY_MAX` before touching anything
   else. Instrument `mppt_1khz_update()` with a GPIO toggle and check the
   execution time on a scope.
4. **Fly it.**
