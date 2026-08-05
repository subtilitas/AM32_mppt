# Proposal: Maximum Power Point Tracking (MPPT)

## Summary

Adds an optional input-voltage regulator that holds the supply rail at a configured
voltage by pulling the duty cycle back, plus a fast undervoltage cutoff to protect
the MCU from brownout resets.

The intended use is running a BLDC motor directly from a solar panel or another
current-limited source. Such a source has a knee in its I-V curve: draw more than
it can supply and the voltage collapses, taking the ESC down with it. Holding the
input at a fixed voltage keeps the source near its maximum power point instead.

Entirely contained behind `ENABLE_MPPT`. Not defined by default, so every existing
target builds bit-identically.

## Enabling

Per-target defines in `Inc/targets.h`, exactly like `USE_SERIAL_TELEMETRY` or
`VOLTAGE_BASED_RAMP`. No Makefile or build system changes:

```c
#ifdef MY_SOLAR_BOARD_F051
#define FIRMWARE_NAME "Solar F051 "
#define FILE_NAME "MY_SOLAR_BOARD_F051"
...
#define ENABLE_MPPT
#define MPPT_VOLTAGE_OFFSET 5
#endif
```

| Define | Default | Purpose |
| --- | --- | --- |
| `ENABLE_MPPT` | undefined | Master switch. Nothing is compiled in without it. |
| `MPPT_VOLTAGE_OFFSET` | `5` | Bottom of the target range, in whole volts. Also the hard cutoff threshold. |
| `MPPT_KI` | `400` | Fallback integrator gain when the eeprom gain byte is zero. |
| `MPPT_CUTOFF_HYST` | `50` | Cutoff recovery margin, in 0.01 V. |

All three tunables are `#ifndef`-guarded, so a target only overrides what it needs.
Since the offset doubles as the brownout threshold it is a board-level property
rather than a user setting, which is why it belongs in the target block.

## No EEPROM changes

The struct layout, `EEPROM_VERSION` and the config tool are all untouched. Two
existing fields are reinterpreted, but only when `ENABLE_MPPT` is defined:

| Field | Normal meaning | Under `ENABLE_MPPT` |
| --- | --- | --- |
| `low_cell_volt_cutoff` | per-cell cutoff, `raw + 250` | MPP target, `MPPT_VOLTAGE_OFFSET * 100 + raw` |
| `current_P` | current limiter P gain | tracker integrator gain, `raw * 4` |

`battery_voltage` counts 10 mV per bit, so the raw byte needs no scaling: one count
is 10 mV. With the default offset the target range is 5.00 V to 7.55 V.

## Design

**Downward-only limiter.** A `mppt_duty_limit` clamps `duty_cycle_setpoint` in
`setInput()`, next to the existing current limit clamp. Throttle still sets the
ceiling; the tracker can only take power away, never add it.

**Pure integral, 1 kHz.** The controller runs in the existing `PID_LOOP_DIVIDER`
block of `tenKhzRoutine()`, alongside `currentPid` and `speedPid`. The accumulator
*is* the duty limit, scaled by 10000, so a standing error walks the limit until the
voltage reaches target. Integral action was chosen deliberately: a proportional
controller only produces output while error exists, so it would sit permanently
below target by an amount that varies with load, which is precisely the wrong
behaviour for tracking a fixed point on an I-V curve.

**Median-of-three on the raw ADC.** The tracker and the cutoff both read the
unfiltered conversion through a 3-sample median, not `battery_voltage`. Two
reasons:

- `battery_voltage = (7 * battery_voltage + v) >> 3` truncates on every iteration,
  so the reading sits up to 7 counts (0.07 V) below the true voltage. A tracker fed
  from it holds the panel that far off target, at any gain.
- A linear filter attenuates an impulse but smears it across the following samples;
  a median discards it. Against commutation spikes the IIR is close to useless and
  the median close to perfect, while costing one sample of delay instead of eight.

**Hard cutoff.** If the input reaches `MPPT_VOLTAGE_OFFSET`, the rail is collapsing
and the MCU is heading for a brownout reset, so the bridge is killed immediately
rather than waiting for the integrator to wind down. Recovers by itself once the
voltage clears the hysteresis margin, restarting through the normal startup ramp.
`armed` is left alone.

## Benefits

- Zero cost when disabled: no flash, no RAM, no eeprom, no config tool work, and
  no build system changes. `Src/main.c` is the only file touched.
- Zero steady-state error, and no load-dependent offset.
- Rejects isolated commutation spikes on the sense line entirely.
- Survives a source collapse without an ESC reset.
- Reuses existing structure (`tenKhzRoutine`, the duty clamp pattern from
  `use_current_limit_adjust`) rather than introducing a parallel control path.

## Limitations

These are consequences of not touching the eeprom, and should be weighed against
that choice.

- **Two features are disabled while `ENABLE_MPPT` is set.** The current limiter is
  forced off, because `current_P` no longer means what `currentPid` expects. The
  per-cell low voltage cutoff (mode 1) is forced off, because its threshold derives
  from a different decode of the same byte and lands far above the MPP target,
  which would disarm the ESC seconds into every run. Both are overridden through a
  runtime shadow, never written back to flash. Absolute cutoff (mode 2) still works
  and remains available as a backstop.
- **Config tool labels become misleading.** The fields still read as "low cell
  voltage cutoff" and "current P" while meaning something else.
- **The median is defeated at high RPM.** It rejects corruption only while at most
  one sample in three is bad. The ADC converts once per 1 kHz loop, so once the
  e-commutation rate outruns the sample rate, samples land at effectively random
  commutation phase and the noise stops being impulsive.
- **The cutoff can fire once on spin-up into a weak source**, before the integrator
  engages. It recovers and tracks normally, but expect a brief interruption.
- **`MPPT_VOLTAGE_OFFSET` must sit below the source's worst-case loaded voltage.**
  If open-circuit voltage never clears offset plus hysteresis, the cutoff latches
  off permanently.

## Status

Logic verified in a host harness against the real types and the actual integer
filter arithmetic, under `-Wall -Wextra -Werror`: mapping, tracking to zero error,
load-step rejection, spike rejection, cutoff and recovery, cold start, and a
no-op check with `ENABLE_MPPT` undefined.

Gains were chosen from a sweep against a simulated source, not from hardware. The
model is a linear resistive sag and omits motor inductance, PWM dynamics and ADC
sample-and-hold, so the real usable gain ceiling will be lower than simulation
suggests. **Not yet flown, and not yet validated against a real panel.** `MPPT_KI`
is the tuning knob if it hunts.
