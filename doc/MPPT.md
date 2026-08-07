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
| End of `main()`, just before `while (1)` | `mppt_init()` | ADC DMA has been running through the whole startup tune, so `ADC_raw_volts` is live. Note init does *not* sample Voc — see §3.3 |
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
| `SEED` | Jumped to `k × Voc`, letting the inner PI settle |
| `TRACK` | Normal tracking — the β regulator, §3.2 |
| `RECOVER` | Bus collapse — shedding load. Safety path |

`mppt.state`, `mppt.vref`, `mppt.voc_est`, `mppt.voc_boot`, `mppt.duty_applied`,
`mppt.beta`, `mppt.beta_trim`, `mppt.beta_valid`, `mppt.i_zero_raw`,
`mppt.collapse_events` and `mppt.coast_events` are all safe to stream over AM32's serial telemetry while
commissioning.

### 3.1 Tracking modes

Two trackers, one compiled in per board, selected by `MPPT_TRACKER`:

| `MPPT_TRACKER` | Objective | Needs current? | Steady state |
|---|---|---|---|
| `MPPT_TRACKER_BETA` (default) | `β = ln(I/V) − c·V` regulated to its MPP value | **yes**, absolute | 0.000 V p-p |
| `MPPT_TRACKER_RPM` | P&O on `e_com_time`, adapting `k` | no | dithers ±0.21 V |

**Which board gets which is not a compiler decision.** `Inc/targets.h` ends
with `#ifndef MILLIVOLT_PER_AMP / #define MILLIVOLT_PER_AMP 20`, so a board
with no shunt still reports a current — a fabricated one — and the
preprocessor cannot tell it from a board that genuinely chose 20 mV/A.
136 of 254 board blocks are in that position. β drives the setpoint from
`ln(I/V)` and does not degrade gracefully on an invented current: it walks
`vref` into a clamp rail.

So `test/mppt_targets.sh` parses `targets.h` and asks whether the board's
*own* block defines `MILLIVOLT_PER_AMP`. 112 boards get β; 136 get the rpm
fallback. A board block may pin `MPPT_TRACKER` explicitly.

One further algorithm was implemented, measured and removed: dP/dV
incremental conductance with Hassani two-step escalation, ~86%. It infers
`dP/dV` from a *difference* of currents, and ~20 ADC counts makes that
quantisation noise. See §6.4.

**There is deliberately no "small array" versus "large array" split.** An
earlier version gated the algorithm on measured current, assuming those were
two different cases. They are not: a large array in low light draws the same
handful of ADC counts as a small array in full sun, so the gate did not
separate two kinds of hardware — it just flipped algorithms as the weather
changed, and into the worse one whenever the sun came out. Irradiance is the
axis that matters, and the harness varies that (`G=` in `run.sh`) rather than
swapping panel presets.

### 3.2 The beta method

Fractional-Voc is open-loop on a constant, and the constant is usually
wrong — the textbook 0.781 put the setpoint at 10.47 V on the bench panel
against a true Vmpp of 11.32 V. Hill-climbing fixes that but oscillates by
construction, which is what the rpm tracker did: ±0.21 V on `vref` and
±0.60 V on the bus, forever.

Beta is neither. Define

    β = ln(I/V) − c·V,     c = 1/Vt

β evaluated at the MPP is very nearly independent of irradiance, so rather
than searching for the peak you compute β from the present operating point
and drive `vref` until it matches the known MPP value. Measured on one panel:

| irradiance | β at MPP | tracking |
|---|---|---|
| 100% | −5022 | 99.90% |
| 60% | −5022 | 99.88% |
| 35% | −5023 | 99.71% |

That invariance — three parts in ten thousand across a 3:1 irradiance range
— is the whole reason to use it.

**Why not plain incremental conductance.** True INC solves `dI/dV = −I/V`,
which needs a *difference* of currents. On an array spanning 20 ADC counts
that difference is quantisation noise. β needs only *absolute* I and V, and
is dominated by the voltage term:

    dβ/dV = −1/V − c ≈ −0.09 − 1.33 = −1.42 per volt

so the precisely-measured `c·V` term carries the sensitivity while the noisy
`ln(I)` term contributes weakly. A 5% error in I moves the setpoint 36 mV.

**Steady-state movement:** 0.000 V peak-to-peak on both `vref` and the bus,
against 0.210 V / 0.596 V for the rpm tracker.

**The floor.** Below about 25% irradiance the current reading falls under two
ADC counts, β stops being computable, and the code falls back to plain
fractional-Voc while *holding* the trim it last learned (verified: the trim
survives a cloud edge unchanged). That floor is a property of the sense
chain, not the algorithm. See the crossover table in §7 item 7 — the rpm tracker still works down there
and is the better fallback if you can tolerate its dither.

**Calibration — two numbers.** `MPPT_CELLS` and `MPPT_ARRAY_ISC`. Everything
else derives: Voc, Vmpp, the ratio between them, the diode voltage, both
safety thresholds, and β's own target. See §3.4.

### 3.4 Describing the array

A typical RC solar wing is 10–16 SunPower back-contact cells in series, and
those cells are consistent enough that one number fixes the rest:

    per cell:  Voc 0.71 V,  Vmpp 0.62 V   →   Vmpp/Voc = 0.873

That ratio is far above the 0.78 textbook figure for ordinary crystalline
silicon — back-contact cells have an unusually high fill factor — and using
0.78 parks the setpoint about 12% below Vmpp. Solving
`Vmpp = Voc − Vt·ln(1 + Vmpp/Vt)` against the same two numbers gives a
per-cell diode voltage of 28.9 mV (ideality 1.13), which is exactly what β
needs for `c = 1/Vt`.

So the board block declares **`MPPT_CELLS`** and **`MPPT_ARRAY_ISC`**, and
`mppt.h` derives:

| Derived | From |
|---|---|
| `MPPT_VOC_NOMINAL` | cells × 0.71 V |
| `MPPT_K_FOCV_Q8` | 0.62/0.71 = 224/256 |
| `MPPT_BETA_VT` | cells × 28.9 mV |
| `MPPT_V_COLLAPSE` | max(0.62 × Voc, `MPPT_V_REG_MIN` + 0.60 V) |
| `MPPT_V_COLLAPSE_HYST` | max(Voc/20, 0.35 V) |
| β's target `beta_mpp` | computed at runtime from measured Voc and Isc |

**The safety thresholds have to scale.** They previously did not, and a fixed
7.00 V collapse threshold sat *above* Vmpp for any array of 12 cells or
fewer — which is not conservative, it is permanent `RECOVER`. Scaling from
Voc fixes that, floored by `MPPT_V_REG_MIN`, the one number that is about the
**board** rather than the panel: the lowest bus voltage at which its 3.3 V
rail still regulates. Measure it.

| cells | Voc | Vmpp | collapse | vref floor | Vt |
|---|---|---|---|---|---|
| 10 | 7.10 V | 6.21 V | 4.90 V | 5.25 V | 0.29 V |
| 12 | 8.52 V | 7.45 V | 5.28 V | 5.70 V | 0.34 V |
| 14 | 9.94 V | 8.69 V | 6.16 V | 6.65 V | 0.40 V |
| 16 | 11.36 V | 9.94 V | 7.04 V | 7.60 V | 0.46 V |
| 19 | 13.49 V | 11.80 V | 8.36 V | 9.03 V | 0.55 V |

**β's target is derived, not calibrated.** `beta_mpp = ln(Impp/Vmpp) −
Vmpp/Vt`, computed at init and again after the boot Voc capture using the
module's own integer `ln`. It has to be derived: it shifts with cell count
*and* with array current — doubling the array area moves it by ln(2) — so no
fixed default could be right for both a 10-cell and a 16-cell wing, or for a
0.5 A and a 3 A one. This also removes the hand-calibration step entirely.

Measured against a SunPower plant model, 60 s, β with its derived target:

| cells | 10 | 12 | 14 | 16 | 19 |
|---|---|---|---|---|---|
| tracking | 99.98% | 99.97% | 99.93% | 99.99% | 99.98% |

### 3.3 Voc measurement

1. **At boot, deferred** — the bus at power-up is Voc, but only once nothing
   is loading it. `mppt_init()` deliberately does *not* sample: AM32 plays its
   startup tunes by driving the motor coils, which drains Cbus, and the arming
   tune fires *later* than `mppt_init()` is even called. Instead the capture
   waits for a quiet window — no tone pending, no current, no applied duty,
   and the bus voltage stationary for `MPPT_BOOT_VOC_SETTLE_MS` (300 ms).
   "Stationary" is measured against the voltage at the *start* of the window,
   not tick to tick; see §6.12. It keeps retrying, so arming early only delays
   the capture rather than losing it. Result lands in `mppt.voc_boot`.
2. **Opportunistic** — when current is below `MPPT_VOC_I_TH` **and**
   `duty_applied` is below `MPPT_VOC_DUTY_TH`, IIR `voc_est` toward the
   measured bus voltage. Both conditions are required; see §6.5. In practice
   this fires every time the throttle comes down. Measured: `voc_est` holds
   13.39 V through a loaded run, and converges to 13.36 V against a true
   13.40 V within a 2 s throttle chop.

A periodic duty-0 Voc sweep was also implemented and **removed**: on this
hardware duty 0 is a brake, not a coast, so it braked the motor once per
interval. See §6.8.

`MPPT_VOC_NOMINAL` is only a fallback for the window before the first real
measurement.

---

## 4. Cost on EGAN_MPPT_L431

- **RAM:** 87 bytes total (`mppt_t` plus 3 static bytes).
- **Flash:** ~3.2 kB (measured 3312 B of x86-64 `.text`; Thumb-2 is smaller).
- **CPU:** every division is by a compile-time constant and gets
  strength-reduced to multiply-high-plus-shift. The only notable cost is an
  integer `ln` (CLZ plus a 17-entry interpolated table, ~20 cycles, no libm)
  twice per β update, and β updates at ~48 Hz. Well under 1% of an 80 MHz
  Cortex-M4. **Measure it** with a GPIO toggle before relying on that.
- No FPU use, no `float`, no `double`, no dynamic allocation.

---

## 5. Verification status

Run the harness with `test/mppt_sim/run.sh [seconds]`, and `G=0.35 ./run.sh`
for reduced irradiance. It compiles the **shipping** `Src/mppt.c` — no stubbed
algorithm — against a PV + bus-capacitor + BLDC + propeller plant, pulling
every board constant from the real `Inc/targets.h`.

One panel, with irradiance as the variable. There is no separate "small array"
case: in low light a large array draws the same few ADC counts as a small one
in full sun.

| Test | Result |
|---|---|
| β, full sun, 60 s | 99.91%, 0 brownouts, `vref` 0.000 V p-p |
| β at 60% / 35% irradiance | 99.89% / 99.72% |
| β below 25% irradiance | drops out, falls back to fixed-k holding its trim — see §7 item 7 |
| rpm P&O, same runs | 99.64% / 99.29% / 98.30%, but 0.210 V p-p dither |
| fixed k=0.781, same runs | 96.74% / 92.49% / 89.95% |
| β target constant across 3:1 irradiance | −5022 / −5022 / −5023 |
| Integer `ln` vs libm, x = 1…100000 | max error 0.011 → 7.6 mV of setpoint |
| Sun → deep shade → sun | trim held through the cloud, resumes on return |
| Cbus 100 µF … 1000 µF | 0 brownouts |
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

- A runtime quantisation floor on the power deadband (`3 × v × I_LSB`) took
  the panel from 77.6% to 90.7%.
- Falling back from hill-climbing to fractional-Voc took it to 96.8%.
- `MPPT_ARRAY_ISC` — the array nameplate is a declared input, and every
  current-domain threshold derives from it with static assertions.

The hill-climber itself was later removed outright; β superseded both fixes.

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

### 6.8 The Voc sweep was braking the motor

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

### 6.9 Opportunistic Voc gated on the wrong duty

The estimator tested `mppt.duty`, which in the gated state is deliberately set
to a conservative *ceiling* of `MPPT_STARTUP_DUTY_MAX` (400) rather than to
the applied duty. So the condition `duty < MPPT_VOC_DUTY_TH` was false exactly
when the throttle was down and the measurement was there for the taking. It
now tests `mppt.duty_applied`, the value actually handed to the PWM.

`voc_est` is also now clamped to `MPPT_ADC_FULL_SCALE_V` — no estimate above
what the divider can measure.

### 6.10 The undervoltage cut was braking too

Same root cause as §6.8, on the path where it matters most. `mppt_apply_duty()`
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
  shorted anyway. Caught by `test_directed.c`, not by the plant model.

Note the threshold matters for a physical reason: above roughly 2% duty the
complementary FETs are doing useful synchronous rectification and feeding the
collapsing bus. It is only at genuinely zero duty that the winding is a dead
short and the rotor's energy goes to heat for no return. Coasting earlier than
that would give away real energy.

### 6.11 The rpm tracker stalled short of the peak

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

### 6.12 Boot-Voc "settled" test defeated by a slow ramp

The settle test originally compared `v` against the *previous tick*. A slow
steady ramp whose per-tick step sits just under the threshold passes that test
indefinitely — the bus could climb by threshold × 300 ms, i.e. volts, and still
be declared settled, latching a Voc well below the real one.

It now anchors to the voltage at the start of the window, which bounds total
drift to `MPPT_BOOT_VOC_STABLE_DV` (0.10 V) across the whole 300 ms. Caught by
a directed test using a deliberately slow ramp; a fast ramp passes either
version and would have hidden it.

### 6.13 `reg_ticks` never incremented in FOCV mode

The "does the PI have authority?" counter was updated *after* the
fractional-Voc branch, which breaks out early. So in FOCV mode it stayed pinned
at 0 — and the rpm tracker gates on it, so the tracker would never have run at
all. Moved above the branch.

### 6.14 Removed outright

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

2. **`MPPT_BETA_MPP_Q8` and `MPPT_BETA_VT` are panel constants.** They are
   set for the 13.4 V / 120 mA panel. They do not need re-measuring per
   irradiance — that invariance is the point — but they *are* specific to a
   panel. Fit a different array and re-read `mppt.beta` at its best hand-found
   operating point. Pointing one panel's constant at another gives ~50%.

3. **The current-sense zero is now self-calibrated, but only at boot.** It is
   captured in the quiet window and held for the flight, so thermal drift of
   the sense amp is uncorrected. β tolerates about ±2 mV and rails past ±5 mV
   — and on this chain 1 mV is 7.4 mA, 6% of the array. Watch `mppt.i_zero_raw`
   against a cold and a warm board before trusting a long flight.

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

7. **β dies below ~25% irradiance, and the rpm tracker does not.** Measured
   over 60 s on one panel:

   | irradiance | β | rpm P&O | fixed k |
   |---|---|---|---|
   | 100% | **99.91%** | 99.64% | 96.74% |
   | 60% | **99.89%** | 99.29% | 92.49% |
   | 35% | **99.72%** | 98.30% | 89.95% |
   | 20% | 87.35% | **97.56%** | 87.35% |
   | 10% | 88.87% | **98.62%** | 88.87% |

   The crossover is where current falls under two ADC counts. β then falls
   back to fixed-k, which costs ~10%. The rpm tracker keeps working down
   there because it never needed current at all — at the price of the
   ±0.21 V dither. Using the rpm tracker as the low-light fallback instead
   of fixed-k is the obvious improvement and is not implemented.

8. **Right shifts of negative values.** The IIR filters rely on `>>` being
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
