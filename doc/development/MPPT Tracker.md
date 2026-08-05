# Proposal: Maximum Power Point Tracking (MPPT)

## Summary

Adds an optional input-voltage regulator that holds the supply rail at a configured
voltage by pulling the duty cycle back, plus an undervoltage cutoff that stops the
ESC dragging a collapsing rail down with it.

The intended use is running a BLDC motor directly from a solar panel or another
current-limited source. Such a source has a knee in its I-V curve: draw more than
it can supply and the voltage collapses, taking the ESC down with it. Holding the
input at a fixed voltage keeps the source near its maximum power point instead.

Entirely contained behind `ENABLE_MPPT`. Not defined by default, so every existing
target builds bit-identically. `Src/main.c` is the only code file touched.

## Enabling

Per-target defines in `Inc/targets.h`, exactly like `USE_SERIAL_TELEMETRY` or
`VOLTAGE_BASED_RAMP`. No Makefile or build system changes:

```c
#ifdef MY_SOLAR_BOARD_F421
#define FIRMWARE_NAME "Solar F421 "
#define FILE_NAME "MY_SOLAR_BOARD_F421"
...
#define ENABLE_MPPT
#define MPPT_VOLTAGE_OFFSET 6
#endif
```

| Define | Default | Purpose |
| --- | --- | --- |
| `ENABLE_MPPT` | undefined | Master switch. Nothing is compiled in without it. |
| `MPPT_VOLTAGE_OFFSET` | `5` | Bottom of the target range, in whole volts. |
| `MPPT_CUTOFF_MARGIN` | `250` | How far below the lowest target the hard cutoff sits, in 0.01 V. |
| `MPPT_CUTOFF_HYST` | `50` | Cutoff recovery margin, in 0.01 V. Must be less than `MPPT_CUTOFF_MARGIN`. |
| `MPPT_KI` | `400` | Power-on integrator gain, before the eeprom is read. The operating gain comes from `current_P`. |

All are `#ifndef`-guarded, so a target overrides only what it needs. Two
`_Static_assert`s enforce that the lowest selectable target clears the cutoff
recovery threshold, so a bad combination fails at compile time rather than
latching the ESC off in the field.

The offset is a board-level property, not a user preference — it depends on the
panel the hardware is built around — which is why it belongs in the target block.

## No EEPROM changes

The struct layout, `EEPROM_VERSION` and the config tool are all untouched. Two
existing fields are reinterpreted, but only when `ENABLE_MPPT` is defined:

| Field | Normal meaning | Under `ENABLE_MPPT` |
| --- | --- | --- |
| `low_cell_volt_cutoff` | per-cell cutoff, `raw + 250` | MPP target, `MPPT_VOLTAGE_OFFSET * 100 + raw` |
| `current_P` | current limiter P gain | tracker integrator gain, `raw * 4`. **0 disables the tracker.** |

The voltage scale counts 10 mV per bit, so the raw byte needs no scaling: one count
is 10 mV.

The byte holds 0-255, but the configuration tool clamps the field to 2.50-3.50 V
per cell, so only `raw` 0-100 is reachable. **The adjustable span is therefore
1.00 V starting at the offset**, not 2.55 V. Pick `MPPT_VOLTAGE_OFFSET` so the
panel's MPP falls inside that window, ideally near its middle: a 6.4 V panel wants
offset `6`, giving 6.00-7.00 V, with 6.40 V selected by entering 290.

`current_P = 0` switching the tracker off is deliberate: it makes the ESC behave as
a stock build without reflashing, which is the fastest way to answer "is this the
tracker or the motor?" during bring-up.

## Design

**Downward-only limiter.** A `mppt_duty_limit` clamps `duty_cycle_setpoint` in
`setInput()`, next to the existing current limit clamp. Throttle still sets the
ceiling; the tracker can only take power away, never add it.

**Pure integral, 1 kHz.** The controller runs in the existing `PID_LOOP_DIVIDER`
block of `tenKhzRoutine()`, alongside `currentPid` and `speedPid`. The accumulator
*is* the duty limit, scaled by 10000, so a standing error walks the limit until the
voltage reaches target. Integral action was chosen deliberately: a proportional
controller only produces output while error exists, so it would sit permanently
below target by an amount that varies with load — precisely the wrong behaviour for
holding a fixed point on an I-V curve.

**Two different filters, on purpose.** The tracker reads the raw conversion through
a 3-sample median; the cutoff reads the existing `battery_voltage` IIR. That is not
an inconsistency, it is the two jobs wanting opposite things:

- A *regulator* wants an unbiased estimate. `battery_voltage = (7 * battery_voltage
  + v) >> 3` truncates every iteration, so it reads up to 7 counts (0.07 V) low, and
  a tracker fed from it holds the panel that far off target at any gain. A median is
  an order statistic and carries no such bias.
- A *threshold* wants severity-proportional debouncing and does not care about
  0.07 V. The IIR gives exactly that for free: a total collapse crosses in ~2 ms, a
  marginal dip takes ~10 ms, and a brief inrush sag never crosses at all. A fixed
  sample counter would be strictly worse, delaying the severe case as much as the
  harmless one.

**Startup is exempt.** While `zero_crosses < 30` — the same window the existing
`min_startup_duty` clamp uses — the tracker neither integrates nor clamps. Without
this it winds the limit down in response to the startup sag and starves the very
ramp that would end that sag. Inside the window the loop is *frozen*, not released:
snapping the limit back to full there turns a brief desync into a relaxation
oscillator that presents as stuttering.

**Floored at `min_startup_duty`,** not `minimum_duty_cycle`. The latter is barely
above `DEAD_TIME`, low enough that winding down to it stalls the motor, and a
stalled motor draws *more* current, so the loop would wind into the stall rather
than out of it.

**Hard cutoff.** If the input falls to `MPPT_VOLTAGE_OFFSET * 100 -
MPPT_CUTOFF_MARGIN`, the bridge is killed rather than waiting for the integrator to
wind down. Recovers by itself once the voltage clears the hysteresis margin,
restarting through the normal startup ramp; `armed` is left alone. It also clears
whenever `!running`, which matters because sine startup drives the FETs through
`setPWMCompare()` and never reaches this code — a latched cutoff would otherwise
survive the whole sine ramp and zero the duty the instant normal commutation took
over, with no path to recover.

## Bring-up

The control loop is the easy part. Every problem encountered bringing this up on
real hardware was in the target definition, and all of them presented as the
tracker misbehaving.

**1. Verify the ADC channels first.** `Mcu/<mcu>/Src/ADC.c` assigns
`VOLTAGE_ADC_CHANNEL` to conversion rank 1 and `CURRENT_ADC_CHANNEL` to rank 2, so
getting them the wrong way round silently swaps the two readings. Symptom: telemetry
shows `Voltage 0.00` (reading the shunt at no load) and a current figure that scales
linearly with input voltage. Both fields must be checked — a stock build barely uses
voltage, so a target can appear to "work fine" with them swapped.

**2. Calibrate `TARGET_VOLTAGE_DIVIDER`.** There is a global fallback of `110`, so
omitting it fails silently. Compare the reported voltage against a meter with the
motor stopped:

```
TARGET_VOLTAGE_DIVIDER = 110 * (meter_volts / reported_volts)
```

Note that borrowing a dev-board target for its pin mapping does *not* mean the
divider matches — that is a separate property of the board.

**3. Do not use telemetry to validate the divider once the tracker is running.**
The loop regulates until its *own* reading equals the target, so it will report
exactly the target voltage whatever the divider is set to. The number is circular.
Only an external instrument tells you the true voltage.

**4. Distinguish a divider error from a sampling error.** Measure reported against
actual at two or three throttle settings. A constant ratio error is the divider. An
error that *changes with duty* is PWM ripple being sampled at a fixed phase — the
ADC converts once per 1 kHz loop off the same timer chain as the PWM, and if the
ratio is near-integer the sampling phase barely drifts, so no amount of filtering
recovers the true mean. Changing PWM frequency in the config tool is a cheap test.

**5. Then tune.** Set `current_P = 0` and confirm a clean baseline. Raise it from
10 upward (`KI = current_P * 4`) and stop below wherever hunting starts.

## Benefits

- Zero cost when disabled: no flash, no RAM, no eeprom, no config tool work, no
  build system changes.
- Zero steady-state error, and no load-dependent offset.
- Runtime off-switch for bisecting, without reflashing.
- Survives a source collapse without an ESC reset.
- Reuses existing structure (`tenKhzRoutine`, the duty clamp pattern from
  `use_current_limit_adjust`) rather than introducing a parallel control path.

## Limitations

Mostly consequences of not touching the eeprom, and should be weighed against that
choice.

- **Two features are disabled while `ENABLE_MPPT` is set.** The current limiter is
  forced off, because `current_P` no longer means what `currentPid` expects. The
  per-cell low voltage cutoff (mode 1) is forced off, because its threshold derives
  from a different decode of the same byte and lands far above the MPP target, which
  would disarm the ESC seconds into every run. Both are overridden through a runtime
  shadow, never written back to flash. Absolute cutoff (mode 2) is untouched and
  remains available as a backstop.
- **Config tool labels become misleading.** The fields still read as "low cell
  voltage cutoff" and "current P" while meaning something else.
- **The median is defeated at high RPM.** It rejects corruption only while at most
  one sample in three is bad. The ADC converts once per 1 kHz loop, so once the
  e-commutation rate outruns the sample rate, samples land at effectively random
  commutation phase and the noise stops being impulsive.
- **The tracker cannot rescue an impossible operating point.** If holding the target
  requires less duty than the motor needs to stay commutating, there is no solution
  and the system will hunt. That is physics, not tuning — it means the source cannot
  run that motor at that voltage.
- **`MPPT_VOLTAGE_OFFSET` must sit below the source's worst-case loaded voltage.**
  If open-circuit voltage never clears cutoff plus hysteresis, the cutoff latches
  off permanently.

## Status

Logic verified in a host harness against the real types and the actual integer
filter arithmetic, under `-Wall -Wextra -Werror`: target mapping, tracking to zero
error, load-step rejection, spike rejection, cutoff and recovery, cold start,
sine-exit latch clearing, startup exemption, and a no-op check with `ENABLE_MPPT`
undefined.

**Hardware validated** on an AT32F421 ESC against a current-limited bench supply:
with `MPPT_VOLTAGE_OFFSET 6` and the cell voltage field at maximum (350 → raw 100),
the input regulates to 7.00 V exactly, confirmed on the supply's own display rather
than the ESC's telemetry.

Not yet validated against a real panel, and not yet flown. Gains were characterised
against a simulated linear resistive source, which omits motor inductance, PWM
dynamics and ADC sample-and-hold, and is far gentler than a panel's I-V knee — so
expect the usable gain ceiling on a panel to be lower than simulation suggests.
`current_P` is the tuning knob.
