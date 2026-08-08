/*
 * mppt.h - Maximum power point tracking for AM32 ESC firmware
 *
 * Target      : STM32L431 (Cortex-M4F @ 80 MHz), AM32 v2.x
 * Application : Direct-drive solar aircraft. PV array -> ESC -> BLDC.
 *               NO BATTERY ANYWHERE. The MCU logic supply shares the bus
 *               with the panel, so if the bus collapses the ESC browns out
 *               and reboots: thrust stops for as long as the restart takes,
 *               and a receiver on the same bus may glitch with it. Both
 *               recoverable, neither wanted. Avoiding the reset is what the
 *               safety thresholds below are for.
 *
 * Architecture (cascaded, all integer math, no FPU use):
 *
 *   tenKhzRoutine()          10/20 kHz   duty application + collapse clamp
 *     +-- 1 kHz block        ~950 Hz     sample, filter, PI voltage loop
 *           +-- tracker      ~48 Hz      beta = ln(I/V) - c*V, regulated
 *
 * The inner PI drives duty cycle to hold the PANEL voltage at vref. The
 * outer tracker moves vref. Decoupling this way keeps the fast loop clear
 * of the propeller's 0.5-2 s mechanical time constant.
 *
 * The outer loop is a REGULATOR, not a hill-climber: it computes where the
 * MPP is from the present operating point rather than searching for it, so
 * there is no perturbation and no steady-state dither. See the note at the
 * top of mppt.c for the three hill-climbing approaches that were tried and
 * removed, and why.
 *
 * ---------------------------------------------------------------------
 * ENABLING
 * ---------------------------------------------------------------------
 * This module is OFF unless the board's block in targets.h defines
 * USE_MPPT. When it is off, mppt.c compiles to nothing and the three call
 * sites in main.c preprocess away to (void)0, so every other AM32 target
 * builds byte-identical.
 *
 * targets.h MUST be included before this header. mppt.h includes it
 * itself for safety; targets.h is pure macros and is safe to re-include.
 *
 * References:
 *  [1] Microchip AN1521, "Practical Guide to Implementing Solar Panel MPPT
 *      Algorithms". -> the 1 kHz PI inner loop.
 *  [2] TI TIDA-010042, 400-W GaN MPPT Charge Controller Reference Design.
 *      -> "when Vpanel is within 97.5%-102.5% of Vmpp, the output power of
 *      the panel is above 99.5% of maximum power" - why a small residual
 *      setpoint error costs almost nothing.
 */

#pragma once

#include <stdint.h>
#include "targets.h"

#ifdef USE_MPPT

#define MPPT_MAX2(a, b) ((a) > (b) ? (a) : (b))

/* ===================================================================== */
/*  1. PANEL / ARRAY CONFIGURATION       <-- SET THESE FIRST             */
/* ===================================================================== */
/*
 * All voltages are in units of 10 mV  (i.e. volts * 100), matching AM32's
 * own battery_voltage scaling.  e.g. 12.00 V -> 1200
 * All currents are in units of 10 mA  (i.e. amps * 100), matching AM32's
 * actual_current scaling.             e.g.  3.50 A ->  350
 *
 * MEASURABLE RANGE. The bus voltage the firmware can see is bounded by the
 * board's own divider:
 *
 *     v_full_scale = 3300 mV * TARGET_VOLTAGE_DIVIDER / 100    [10 mV units]
 *
 * Past that the ADC rails at 0x0FFF and every voltage in this file becomes
 * meaningless. MPPT_ADC_FULL_SCALE_V below is that number, and the static
 * assertions at the bottom of this header check the configuration against
 * it at compile time. On EGAN_JUWI_L431 (divider 47) it is 15.51 V, which
 * is why this file is scaled for a ~12 V array and not the 24 V array the
 * module was originally written against.
 *
 * Every constant here is #ifndef-guarded, so a board block in targets.h can
 * override any of them without editing this file.
 */

/* ---------------------------------------------------------------------
 * THE ARRAY IS DESCRIBED BY ITS CELL COUNT, NOT BY LOOSE VOLTAGES
 *
 * A typical RC solar wing is 10-16 SunPower back-contact cells in series.
 * Those cells are well characterised and consistent, so one number - how
 * many are in series - fixes Voc, Vmpp, the ratio between them, and the
 * diode voltage the beta regulator needs. Deriving all four from it means
 * they cannot drift apart, which is what happened when they were four
 * independent constants.
 *
 * Per cell:  Voc 0.71 V,  Vmpp 0.62 V,  so Vmpp/Voc = 0.873.
 *
 * That ratio is much higher than the 0.78 textbook figure for ordinary
 * crystalline silicon - back-contact cells have an unusually high fill
 * factor. Using 0.78 on a SunPower array parks the setpoint about 12%
 * below Vmpp.
 *
 * The implied per-cell diode voltage follows from the same two numbers:
 * solving Vmpp = Voc - Vt*ln(1 + Vmpp/Vt) gives Vt = 28.9 mV, i.e. an
 * ideality factor of 1.13. Consistent, and it is what MPPT_BETA_VT needs.
 *
 *      cells    Voc      Vmpp     Vt(array)
 *        10    7.10 V   6.20 V     0.29 V
 *        12    8.52 V   7.44 V     0.35 V
 *        14    9.94 V   8.68 V     0.41 V
 *        16   11.36 V   9.92 V     0.46 V
 *
 * SET MPPT_CELLS FOR YOUR WING. Everything else follows. Any individual
 * constant can still be overridden in the board block if your cells are
 * not SunPower.
 * ------------------------------------------------------------------- */
#ifndef MPPT_CELLS
#define MPPT_CELLS                  12    /* series cells in the array */
#endif
#ifndef MPPT_CELL_VOC_MV
#define MPPT_CELL_VOC_MV           710
#endif
#ifndef MPPT_CELL_VMPP_MV
#define MPPT_CELL_VMPP_MV          620
#endif
/* Per-cell n*k*T/q implied by the Voc/Vmpp pair above. */
#ifndef MPPT_CELL_VT_MV
#define MPPT_CELL_VT_MV             29
#endif

/* Nameplate open-circuit voltage at STC, 10 mV units. Only a fallback
 * until the firmware measures Voc for real at power-up. */
#ifndef MPPT_VOC_NOMINAL
#define MPPT_VOC_NOMINAL   ((MPPT_CELLS * MPPT_CELL_VOC_MV) / 10)
#endif

/* Vmpp/Voc, Q8. Rounded, not truncated - the difference is a whole
 * count of k and about 40 mV of setpoint. */
#ifndef MPPT_K_FOCV_Q8
#define MPPT_K_FOCV_Q8 \
    (((MPPT_CELL_VMPP_MV * 256) + (MPPT_CELL_VOC_MV / 2)) / MPPT_CELL_VOC_MV)
#endif

/* Bounds on the ratio when the rpm tracker adapts it. Wide enough to hold
 * both ordinary crystalline silicon (0.78) and back-contact cells (0.873),
 * narrow enough that no amount of bad rpm data can walk the setpoint into
 * the short-circuit region or out to open circuit. */
#ifndef MPPT_K_MIN_Q8
#define MPPT_K_MIN_Q8             160    /* 0.625 */
#endif
#ifndef MPPT_K_MAX_Q8
#define MPPT_K_MAX_Q8             240    /* 0.938 */
#endif

/* Hard sanity band for vref, as a fraction of the Voc estimate, Q8.
 * The true MPP of a silicon panel is ALWAYS inside this window, at any
 * irradiance or temperature. Clamping here means a bad measurement can
 * never walk the tracker into the short-circuit region. */
#ifndef MPPT_VREF_MIN_FRAC_Q8
#define MPPT_VREF_MIN_FRAC_Q8     115     /* 0.449 * Voc */
#endif
#ifndef MPPT_VREF_MAX_FRAC_Q8
#define MPPT_VREF_MAX_FRAC_Q8     243     /* 0.949 * Voc */
#endif

/* ===================================================================== */
/*  2. SAFETY THRESHOLDS - avoiding a brownout reset                     */
/* ===================================================================== */
/*
 * With no battery on the bus there is nothing to hold it up when the load
 * outruns the panel. Let it fall far enough and the MCU browns out, the ESC
 * reboots, and thrust stops until it has restarted - a few hundred
 * milliseconds of dead prop, plus a possible receiver glitch if it shares
 * the bus. Recoverable on a glider-like airframe, but not something to do
 * repeatedly. MPPT_V_COLLAPSE must sit comfortably above the regulator
 * dropout plus the worst-case sag Cbus can ride through.
 *
 * Sizing rule of thumb: during a collapse the load pulls I_load and the
 * panel can only supply I_sc. Cbus must hold the bus above the LDO
 * dropout for the MPPT_RECOVER_MS it takes to unload:
 *
 *     Cbus >= (I_load - I_sc) * MPPT_RECOVER_MS / (V_collapse - V_dropout)
 *
 * *** THE VALUES BELOW ARE PLACEHOLDERS SCALED FOR A ~12 V ARRAY. ***
 * Set them from your board's actual 3.3 V regulator dropout plus margin
 * before the first flight. See BENCH-UP in doc/MPPT.md.
 */
/* THE ONE NUMBER THAT IS NOT ABOUT THE PANEL. The lowest bus voltage at
 * which this board's 3.3 V rail still regulates - LDO dropout plus margin.
 * It is a property of the ESC, not the array, so it does not scale with
 * cell count and it is the hard floor everything else sits above.
 *
 * *** MEASURE IT. *** It sets how far the bus may sag before the ESC
 * resets. 4.30 V suits a typical 3.3 V LDO;
 * a board with a buck may go lower, one with a high-dropout part may not. */
#ifndef MPPT_V_REG_MIN
#define MPPT_V_REG_MIN             430    /*  4.30 V */
#endif

/* Duty forced to 0 - the rail is about to drop out. */
#ifndef MPPT_V_ABSOLUTE_MIN
#define MPPT_V_ABSOLUTE_MIN   MPPT_V_REG_MIN
#endif

/* Enter RECOVER. Scales with the array so a big wing reacts early instead
 * of letting the bus fall most of the way to the regulator floor first,
 * but is floored so a 10-cell wing does not put it on top of the
 * absolute minimum.
 *
 * 0.62*Voc sits well below Vmpp (0.873*Voc) at every cell count, which
 * matters: a collapse threshold above Vmpp is not conservative, it is a
 * permanent RECOVER. The previous fixed 7.00 V did exactly that to any
 * array of 12 cells or fewer. */
#ifndef MPPT_V_COLLAPSE
#define MPPT_V_COLLAPSE \
    MPPT_MAX2((MPPT_VOC_NOMINAL * 62) / 100, MPPT_V_ABSOLUTE_MIN + 60)
#endif
#ifndef MPPT_V_COLLAPSE_HYST
#define MPPT_V_COLLAPSE_HYST  MPPT_MAX2(MPPT_VOC_NOMINAL / 20, 35)
#endif

/* How long to sit in RECOVER before giving up and fully unloading.
 * Prevents a collapse/recover limit cycle if the panel simply cannot
 * support even minimum duty (e.g. a cloud, or a stalled prop). */
#ifndef MPPT_RECOVER_MS
#define MPPT_RECOVER_MS            120
#endif

/* Consecutive milliseconds of healthy bus required to leave RECOVER. */
#ifndef MPPT_RECOVER_OK_MS
#define MPPT_RECOVER_OK_MS          10
#endif

/* Duty is multiplied by this each control tick while in RECOVER, Q8.
 * 208/256 = 0.8125 -> duty falls to 10% in ~11 ms. */
#ifndef MPPT_RECOVER_DECAY_Q8
#define MPPT_RECOVER_DECAY_Q8      208
#endif

/* ===================================================================== */
/*  3. LOOP RATES                                                        */
/* ===================================================================== */
/*
 * DERIVED, NOT ASSUMED. AM32's "1 kHz" block is
 *
 *     one_khz_loop_counter++;
 *     if (one_khz_loop_counter > PID_LOOP_DIVIDER) { ...; = 0; }
 *
 * which fires every (PID_LOOP_DIVIDER + 1) calls of tenKhzRoutine(), not
 * every PID_LOOP_DIVIDER. With the usual LOOP_FREQUENCY_HZ 20000 and
 * PID_LOOP_DIVIDER 20 the real rate is 20000/21 = 952 Hz, not 1000 Hz.
 * Hard-coding 1000 here put KI 5% high and stretched every millisecond
 * timeout in the module by the same 5%. Derive it instead so the module
 * follows whatever the target actually does.
 */
#define MPPT_TICK_HZ  (LOOP_FREQUENCY_HZ / (PID_LOOP_DIVIDER + 1))

/* Millisecond timeouts converted to control ticks. */
#define MPPT_MS_TO_TICKS(ms)  (((ms) * MPPT_TICK_HZ + 500) / 1000)
#define MPPT_RECOVER_TICKS    MPPT_MS_TO_TICKS(MPPT_RECOVER_MS)
#define MPPT_RECOVER_OK_TICKS MPPT_MS_TO_TICKS(MPPT_RECOVER_OK_MS)

/* ===================================================================== */
/*  4. INPUT FILTERING                                                   */
/* ===================================================================== */
/*
 * WHY WE DO NOT USE AM32's actual_current:
 *   getSmoothedCurrent() is a 50-tap boxcar at 1 kHz -> 50 ms window,
 *   25 ms group delay. That is 5x longer than our entire 5 ms tracker
 *   period; the tracker would be reacting to its own ancient history and
 *   would go unstable. battery_voltage is better (IIR a=1/8, ~8 ms) but
 *   still too slow.
 *
 *   So we build our own path from the RAW ADC registers, which AM32
 *   already refreshes at 1 kHz. No ADC or DMA reconfiguration needed.
 *
 * Filter is a 1-pole IIR: y += (x - y) >> SHIFT.
 *   SHIFT=2 -> a=1/4  -> tau ~ 3 ms @ 1 kHz
 *   SHIFT=3 -> a=1/8  -> tau ~ 7 ms @ 1 kHz
 *
 * NOTE: >> on a negative value is an arithmetic shift on GCC/ARM, which
 * this module relies on. It also biases the filter output low by up to
 * (2^SHIFT - 1) LSB, i.e. under 40 mV / 40 mA. Harmless here, but do not
 * "clean this up" into a division without re-checking the tuning.
 */
#ifndef MPPT_V_FILTER_SHIFT
#define MPPT_V_FILTER_SHIFT          2
#endif
#ifndef MPPT_I_FILTER_SHIFT
#define MPPT_I_FILTER_SHIFT          2
#endif

/* ===================================================================== */
/*  5. INNER PI VOLTAGE LOOP                                             */
/* ===================================================================== */
/*
 * Sign convention (this trips everyone up, and AN1521 calls it out):
 *   MORE duty -> motor draws MORE current -> panel is loaded HARDER
 *              -> panel voltage FALLS.
 * So dV/dD is NEGATIVE, and the controller is:
 *      error = v_measured - vref        (note the order!)
 *      duty += Kp*error + Ki*integral(error)
 * A positive error (panel above target = under-loaded) increases duty.
 */

/* Gains in human units. See mppt.c for the step-by-step tuning recipe.
 *
 * KI matters more than you would expect here, and for a non-obvious
 * reason: the duty required to hold a given panel voltage is not fixed,
 * it climbs continuously as the propeller spins up (back-EMF rises, so
 * duty must rise with it). The integrator is what follows that. Tuned too
 * low and the aircraft takes seconds to reach the MPP - the loop is not
 * unstable, just uselessly slow. Tuned here for ~60 Hz closed-loop
 * bandwidth, which is about the limit given ~1 kHz sampling plus AM32's
 * ~1 ms ADC staleness. */
#ifndef MPPT_KP_DUTY_PER_VOLT
#define MPPT_KP_DUTY_PER_VOLT      100    /* duty units (0..2000) per volt */
#endif
#ifndef MPPT_KI_DUTY_PER_VOLT_S
#define MPPT_KI_DUTY_PER_VOLT_S   2000    /* duty units per volt-second    */
#endif

/* Error clamp, in 10 mV. Bounds the gain math (prevents int32 overflow)
 * and acts as anti-windup. +-10.00 V. */
#ifndef MPPT_ERR_CLAMP
#define MPPT_ERR_CLAMP            1000
#endif

/* Integrator bounds, in AM32 duty units. The LIVE clamp in mppt.c is the
 * active output ceiling (2000, or MPPT_STARTUP_DUTY_MAX during spin-up) -
 * see the anti-windup note in mppt_pi_update(). MPPT_I_TERM_MAX is not
 * applied to the controller directly; it only feeds the int32-overflow
 * static assert at the bottom of this file, which sizes the worst-case
 * RECOVER decay multiply. */
#define MPPT_I_TERM_MAX           2000
#define MPPT_I_TERM_MIN              0

/* ---------------------------------------------------------------------
 * DUTY RISE-RATE LIMIT
 *
 * main.c captures last_duty_cycle BEFORE the MPPT ceiling is applied, so
 * AM32's ramp tracks its own intent and no longer paces the MPPT. That is
 * deliberate - otherwise every duty reduction reset the ramp and a 26 ms
 * Voc sweep cost 78 ms of recovery - but it means the MPPT is now the only
 * thing limiting how fast load is re-applied, and it has to do that job.
 *
 * It is not optional. Without it, the sweep exit restored the integrator
 * AND added a full proportional term (v is at Voc, so the error is large
 * and positive), stepping duty 0 -> 1451 in a single tick. On the 120 mA
 * bench panel that is harmless. On a 3 A array it drew 6.3 A out of Cbus
 * and put the bus on the floor at 4.6 V - well under any 3.3 V regulator,
 * so the ESC would have reset itself once per sweep.
 *
 * Sizing: the load step has to be slow enough that the panel's own current
 * can rise to meet it as the operating point walks down from Voc to Vmpp.
 * 100 units/tick moves full scale in ~21 ms.
 * ------------------------------------------------------------------- */
#ifndef MPPT_DUTY_RISE_MAX
#define MPPT_DUTY_RISE_MAX         100    /* duty units per control tick */
#endif

/* ---------------------------------------------------------------------
 * COASTING INSTEAD OF BRAKING ON THE UNLOAD PATH
 *
 * Commanding duty 0 does NOT unload the motor on an AM32 ESC. With
 * eepromBuffer.comp_pwm set - the normal case - phaseAPWM() puts the
 * low-side FET into complementary PWM, so at duty 0 the low side conducts
 * ~100% of the time and the energised winding pair is shorted through the
 * low-side FETs. That is synchronous braking.
 *
 * On the safety path that is actively harmful, not merely rough. Braking
 * during a bus collapse dumps the rotor's kinetic energy into the winding
 * resistance as heat, so by the time the bus recovers the motor is slow;
 * low speed means low back-EMF, which means re-applying duty draws a large
 * current surge, which collapses the bus again. That is the limit cycle,
 * and it turns one brief sag into a string of them.
 *
 * Coasting instead is better on both counts. No braking torque, so the
 * prop keeps its energy and its back-EMF. And with all six FETs off the
 * body diodes form a three-phase rectifier, so a motor turning faster than
 * the bus actively feeds current back INTO the collapsing bus rather than
 * burning it.
 *
 * Mechanism: allOff(), which is what AM32's own low-voltage cutoff uses.
 * It must be re-asserted every tick because comStep() re-drives the pin
 * modes at each commutation and would otherwise undo it within one
 * electrical step.
 *
 * The threshold is on mppt.duty - the MPPT's own demand - not on the
 * applied duty, so this never interferes with AM32's idle/brake_on_stop
 * behaviour when the pilot simply closes the throttle.
 * ------------------------------------------------------------------- */
#ifndef MPPT_COAST_DUTY_TH
#define MPPT_COAST_DUTY_TH          40    /* enter coast below 2% demand */
#endif
#ifndef MPPT_COAST_EXIT_DUTY
#define MPPT_COAST_EXIT_DUTY        90    /* leave it above 4.5%         */
#endif
_Static_assert(MPPT_COAST_EXIT_DUTY > MPPT_COAST_DUTY_TH,
    "coast thresholds need hysteresis or the output stage will chatter");

/* ---- "does the PI have authority?" gate ----
 * The tracker's whole premise is that a change in power was caused by its
 * own perturbation of vref. That premise is false whenever the PI is not
 * yet holding v at vref - during spin-up, after a collapse, or when the
 * panel simply cannot be loaded to vref at any duty. In those windows the
 * power is being driven by the propeller accelerating, and a hill-climber
 * will happily interpret that as "my last step was good, keep going" and
 * walk the reference clean off the MPP.
 *
 * So: only take tracker steps once the PI has had authority over the
 * operating point for MPPT_REG_TICKS consecutive ticks. Costs nothing, and
 * without it the tracker mis-tracks badly on every launch. */
#ifndef MPPT_REG_TICKS
#define MPPT_REG_TICKS               3
#endif

/* ===================================================================== */

/* ===================================================================== */
/*  6. BETA METHOD - the tracker                                         */
/* ===================================================================== */
/*
 *      beta = ln(I/V) - c*V          c = 1/Vt
 *
 * The useful property, and the reason this is worth the arithmetic: beta
 * evaluated at the MPP is very nearly INDEPENDENT OF IRRADIANCE. So instead
 * of searching for the peak, compute beta from the present operating point
 * and drive vref until beta equals its known MPP value. It is a regulator,
 * not a hill-climber - there is no perturbation, so there is no dither and
 * nothing to oscillate.
 *
 * WHY THIS ONE AND NOT PLAIN INCREMENTAL CONDUCTANCE
 *
 * True INC solves dI/dV = -I/V, which needs a DIFFERENCE of currents. On a
 * 120 mA array spanning 20 ADC counts, dI is essentially quantisation noise;
 * that is what defeated the power-domain tracker. beta needs only ABSOLUTE
 * I and V, which average cleanly, and it is dominated by the voltage term:
 *
 *      dbeta/dV = -1/V - c  ~= -0.09 - 1.33 = -1.42 per volt
 *
 * so the precisely-measured c*V term carries the sensitivity and the noisy
 * ln(I) term contributes weakly. A 5% error in I moves the setpoint 36 mV.
 *
 * Modelled against a panel WITH series resistance, which is what makes real
 * Vmpp/Voc differ from the ideal-diode value:
 *
 *      Rs      irradiance   fixed k=0.781    analytic(Voc)    beta
 *      0 ohm      full          96.6%           100.0%       100.0%
 *      5 ohm      full          99.3%            98.1%       100.0%
 *     10 ohm      full          99.9%            92.5%       100.0%
 *     10 ohm      35%           98.7%            99.7%        98.1%
 *
 * THE CATCH, AND IT IS A REAL ONE
 *
 * beta depends on ABSOLUTE current accuracy, and this board is brutal about
 * that: at 136 mV/A, one millivolt of sense-amp offset is 7.4 mA, which is
 * 6% of a 120 mA panel. Tolerance is about +-2 mV; past +-5 mV the reading
 * clamps at zero, ln blows up and the setpoint rails.
 *
 * No hand-set CURRENT_OFFSET is that good. So the zero is measured instead,
 * during the boot quiet window where the panel is unloaded and the motor is
 * stopped - see MPPT_BOOT_VOC_SETTLE_MS. That is what makes this viable.
 */

/* Vt for the ARRAY (not one cell): the diode thermal/ideality voltage,
 * n*k*T/q multiplied by the number of cells in series, in 10 mV units.
 * Sets c = 1/Vt. Calibrate from a measured I-V curve if you have one; the
 * default matches the 13.4 V bench panel. */
#ifndef MPPT_BETA_VT
#define MPPT_BETA_VT   ((MPPT_CELLS * MPPT_CELL_VT_MV) / 10)
#endif

/* beta at the MPP, Q8 - DERIVED, not a magic constant.
 *
 *      beta_mpp = ln(Impp/Vmpp) - Vmpp/Vt
 *
 * and every term on the right is already known: Vmpp is k * the MEASURED
 * Voc, Vt comes from the cell count, Impp is a fixed fraction of the
 * declared Isc. So mppt.c computes it at init and again after each boot
 * Voc capture, using the same integer ln the regulator uses.
 *
 * This has to be derived rather than defaulted. beta_mpp shifts with cell
 * count (through both Vmpp and Vt) AND with array current (through
 * ln(Impp)) - doubling the array area moves it by ln(2), which is 177 in
 * Q8. There is no single number that is right for 10 cells and 16, or for
 * a 0.5 A wing and a 3 A one. A fixed default would be wrong for almost
 * every panel it shipped to.
 *
 * Deriving it also removes the hand-calibration step: set MPPT_CELLS and
 * MPPT_ARRAY_ISC and the regulator has its target. Define
 * MPPT_BETA_MPP_Q8 to pin it anyway if you have measured better. */
#ifndef MPPT_BETA_MPP_Q8
#define MPPT_BETA_MPP_DERIVED        1
#endif

/* Hand beta's window over to the rpm tracker when the current channel stops
 * carrying information (beta_valid == 0, i.e. below MPPT_BETA_I_MIN, roughly
 * 20% irradiance). Costs a few hundred bytes of flash in a beta build for the
 * rpm tracker's code, and is worth 1.2 to 4.6 points of tracking efficiency in
 * the light where beta has nothing to work with. Set to 0 on a part that
 * cannot spare the flash - build_mppt_all.sh does that automatically. */
#ifndef MPPT_BETA_RPM_FALLBACK
#define MPPT_BETA_RPM_FALLBACK       1
#endif

/* Impp/Isc for a high-fill-factor back-contact cell. */
#ifndef MPPT_IMPP_FRAC_Q8
#define MPPT_IMPP_FRAC_Q8          243    /* 0.95 */
#endif

/* Regulator gain: 10 mV of vref per unit of beta error, Q8. Deliberately
 * slow - this closes a loop around the panel curve, and the inner PI plus
 * the propeller are inside it. */
#ifndef MPPT_BETA_GAIN_Q8
#define MPPT_BETA_GAIN_Q8            6
#endif

/* Ticks held in SEED before beta starts trimming: long enough for the inner
 * PI to settle at the fractional-Voc setpoint. */
#ifndef MPPT_SEED_TICKS
#define MPPT_SEED_TICKS             10
#endif

/* Control ticks between beta updates. */
#ifndef MPPT_BETA_PERIOD_TICKS
#define MPPT_BETA_PERIOD_TICKS      20
#endif

/* Below this current the ln() term is meaningless and beta is not usable;
 * fall back to plain fractional-Voc until there is a real reading. */
#ifndef MPPT_BETA_I_MIN
#define MPPT_BETA_I_MIN  ((((3 * MPPT_I_LSB_Q8) >> 8) + 1))
#endif

/* How far beta is allowed to pull vref away from the fractional-Voc seed.
 * Bounds any damage a bad current reading can do. */
#ifndef MPPT_BETA_TRIM_MAX
#define MPPT_BETA_TRIM_MAX         300    /* +-3.00 V */
#endif

/* ===================================================================== */
/*  6b. TRACKER SELECTION                                                */
/* ===================================================================== */
/*
 * Two outer loops, one compiled in at a time.
 *
 *   MPPT_TRACKER_BETA  beta = ln(I/V) - c*V, regulated. No perturbation,
 *                      no dither. Needs a REAL current sense chain.
 *   MPPT_TRACKER_RPM   perturb-and-observe on e_com_time, adapting k.
 *                      Needs no current measurement at all. Dithers by
 *                      construction, +-MPPT_K_STEP_Q8/256 of Voc.
 *
 * WHICH ONE, AND WHY IT IS NOT AUTOMATIC
 *
 * Inc/targets.h ends with a `#ifndef MILLIVOLT_PER_AMP / #define ... 20`
 * fallback, so a board with no shunt still reports a current - a fabricated
 * one. 136 of 254 board blocks are in that position, and the preprocessor
 * cannot tell them apart from a board that genuinely chose 20 mV/A.
 *
 * Beta drives the setpoint from ln(I/V) and does not degrade gracefully on
 * an invented current: it walks vref into a clamp rail. So the choice is
 * made outside the compiler, by test/mppt_targets.sh, which parses
 * targets.h and asks whether the board's OWN block defines
 * MILLIVOLT_PER_AMP. Boards that do get beta; boards that do not get rpm.
 *
 * A board block may of course pin it explicitly.
 */
#define MPPT_TRACKER_BETA            0
#define MPPT_TRACKER_RPM             1
#ifndef MPPT_TRACKER
#define MPPT_TRACKER  MPPT_TRACKER_BETA
#endif
#if MPPT_TRACKER != MPPT_TRACKER_BETA && MPPT_TRACKER != MPPT_TRACKER_RPM
#error "MPPT_TRACKER must be MPPT_TRACKER_BETA or MPPT_TRACKER_RPM"
#endif

/* ===================================================================== */
/*  6c. RPM FEEDBACK - fallback tracker, no current sense needed          */
/* ===================================================================== */
/*
 * WHY RPM AND NOT PANEL POWER
 *
 * Fractional-Voc is open-loop on a constant. It cannot find the peak; it
 * can only sit where MPPT_K_FOCV_Q8 says the peak ought to be. That is
 * fine until the constant is wrong, and on a real panel it usually is.
 *
 * The obvious fix is to hill-climb on measured power, and on this hardware
 * that does not work: a 120 mA array spans 20 ADC counts of current, so
 * dP is mostly quantisation. But the ESC has a
 * far better signal sitting unused. e_com_time is the electrical
 * revolution period in microseconds, so at 44-47 krpm one LSB is about
 * 0.5% of speed - ten times finer than one current count - and it can be
 * averaged over hundreds of revolutions.
 *
 * For a fixed-pitch prop the load torque goes as w^2, so shaft power goes
 * as w^3 and maximum rpm IS maximum shaft power. Two consequences worth
 * being explicit about:
 *
 *   - It is not merely a proxy for panel power. On an aircraft, thrust is
 *     the actual objective, and maximising rpm also absorbs variation in
 *     motor efficiency - which panel-power MPPT ignores by construction.
 *   - The 6.8% speed difference between 44 and 47 krpm is ~22% of shaft
 *     power. Small rpm errors are not small power errors.
 *
 * WHAT IS ADAPTED
 *
 * The ratio k, not a voltage offset. The FOCV error is (k_true - k0)*Voc,
 * which scales with Voc, so correcting k stays right as irradiance and
 * temperature move Voc around; a fixed voltage trim would not.
 *
 * The inner PI still regulates fast to whatever setpoint k produces. Only
 * k moves slowly, which is the whole point: perturbations have to outlast
 * the propeller's 0.5-2 s mechanical time constant before the rpm they
 * cause means anything.
 *
 * k is NOT written to EEPROM. It re-learns from MPPT_K_FOCV_Q8 within
 * ~10 s of steady throttle each flight.
 */

/* Selected by MPPT_TRACKER above, not by a switch of its own. */

/* Settling allowance after each perturbation, before rpm is believed.
 * MUST exceed the prop's mechanical time constant or the tracker measures
 * its own transient and wanders. */
#ifndef MPPT_RPM_SETTLE_MS
#define MPPT_RPM_SETTLE_MS         400
#endif

/* Averaging window after settling. */
#ifndef MPPT_RPM_AVG_MS
#define MPPT_RPM_AVG_MS            200
#endif

/* Step applied to k per decision.
 *
 * THE STEP MUST BEAT THE DEADBAND. This is the one that has to be got
 * right, and it is not obvious: near the peak the curve is flat, so a step
 * of 1 changed rpm by only 0.055% against a 0.1% deadband. Every
 * comparison came back a tie, the tracker reversed on every cycle, and it
 * parked at k=203 while the real optimum was 216 - 97.5% instead of 99.6%.
 * A tracker that stalls looks exactly like a tracker that has converged.
 *
 * Measured on the bench panel, 40 s, true optimum k=216:
 *      step=1 -> k=203, 97.5%     step=3 -> k=212, 99.6%
 *      step=2 -> k=210, 99.4%     step=4 -> k=216, 99.6%
 *
 * Going further does not help - step=6 overshoots to 212/224 and gives
 * back accuracy to the dither. The residual dither at step=4 is +-4/256 of
 * Voc, about +-0.21 V here, which the fixed-k sweep shows costs a few
 * tenths of a percent. That is the price of a deadband big enough to
 * ignore real rpm jitter, and it is worth paying. */
#ifndef MPPT_K_STEP_Q8
#define MPPT_K_STEP_Q8               4
#endif

/* Deadband on the comparison, per-mille of the accumulated period. Below
 * this the two measurements are a tie and the tracker reverses, which is
 * what makes it dither around the peak instead of drifting off it. */
#ifndef MPPT_RPM_DEADBAND_PERMILLE
#define MPPT_RPM_DEADBAND_PERMILLE   1    /* 0.1% */
#endif

/* Throttle must hold still across a whole measurement cycle, or the rpm
 * change was the pilot's doing and says nothing about the setpoint. */
#ifndef MPPT_RPM_INPUT_TOL
#define MPPT_RPM_INPUT_TOL          16    /* of 2047 */
#endif

/* Reject the startup sentinel (main.c sets e_com_time to 65408 before the
 * motor is turning) and anything implausible. */
#ifndef MPPT_RPM_ECT_MAX
#define MPPT_RPM_ECT_MAX         60000
#endif

#define MPPT_RPM_SETTLE_TICKS MPPT_MS_TO_TICKS(MPPT_RPM_SETTLE_MS)
#define MPPT_RPM_AVG_TICKS    MPPT_MS_TO_TICKS(MPPT_RPM_AVG_MS)

#define MPPT_RPM_PHASE_SETTLE        0
#define MPPT_RPM_PHASE_MEASURE       1

/* ---------------------------------------------------------------------
 * MANOEUVRE NOTE (aircraft-specific)
 *
 * Bank angle tilts the wing away from the sun, so irradiance follows
 * cos(phi - phi_sun). That is the headline 10:1 swing, but it is SLOW:
 * even at a 120 deg/s roll rate the irradiance moves well under 1% per
 * 5 ms tracker period, so ordinary hill-climbing rides it without ever
 * tripping the transient detector. Nothing extra is needed for it.
 *
 * The genuinely fast events are cloud edges and fuselage/prop shadow.
 * Those are handled by the safety paths, not the tracker: a big enough
 * drop pulls the bus under MPPT_V_COLLAPSE and the state machine falls
 * into RECOVER, then re-seeds vref from the fractional-Voc estimate on
 * the way back.
 *
 * ONE HARDWARE POINT, worth more than any firmware: wire the port and
 * starboard arrays in PARALLEL, not in series. In series, wing dihedral
 * makes one string weaker in a bank, its bypass diode conducts, and the
 * P-V curve grows a second local peak that no simple hill-climber can
 * escape. In parallel the currents just add and the curve stays
 * single-peaked, which is what keeps this tracker simple and fast.
 * ------------------------------------------------------------------- */

/* ===================================================================== */
/*  7. Voc ESTIMATION                                                    */
/* ===================================================================== */
/*
 * Conditions under which the measured bus voltage IS the open-circuit
 * voltage (unloaded panel).
 *
 * *** BOARD-SPECIFIC, AND THE DEFAULT IS MARGINAL ON A BIDIRECTIONAL
 * *** SENSE CHAIN. Set this to roughly 2-3% of array Isc: low enough that
 * the real MPP current never trips it even in deep shade, high enough to
 * catch the moment the tracker reaches Voc AND high enough to sit above
 * the sense chain's own offset uncertainty.
 *
 * On EGAN_JUWI_L431 the sense amp is bidirectional (CURRENT_OFFSET 2500,
 * MILLIVOLT_PER_AMP 136), so 1 mV of offset drift moves the reading by
 * 100/136 = 0.74 in these units, i.e. ~7.4 mA per mV. A 10 mV offset error
 * is therefore 0.074 A, which alone would swamp the original 0.08 A
 * threshold. Verify the reading at true zero current on the bench before
 * trusting Voc tracking, and raise this if it does not settle below it.
 */
/* 5% of Isc, but never below what the ADC can actually resolve. On a small
 * array 5% of Isc rounds to zero - 5% of 120 mA is 6 mA and one ADC count
 * is 5.9 mA - so the floor is what binds there. */
#ifndef MPPT_VOC_I_TH
#define MPPT_VOC_I_TH  MPPT_MAX2(MPPT_ARRAY_ISC / 20, \
                                 (((2 * MPPT_I_LSB_Q8) >> 8) + 1))
#endif
#ifndef MPPT_VOC_FILTER_SHIFT
#define MPPT_VOC_FILTER_SHIFT        3    /* IIR on the estimate, ~8 ms  */
#endif

/* Duty below which the panel counts as unloaded, for the opportunistic
 * estimator. Tested against mppt.duty_applied - the duty actually handed to
 * the PWM - NOT mppt.duty, which in the gated state is a conservative
 * CEILING of MPPT_STARTUP_DUTY_MAX and would fail this test forever exactly
 * when the throttle is down and the measurement is available.
 * The original code keyed on current ALONE and the comment
 * explained at length why the duty condition had been removed. Both
 * conditions are needed: current alone lets a small array - whose entire
 * Isc can sit below the threshold - drag voc_est down to the LOADED bus
 * voltage, which drops the vref clamp band with it and parks the tracker
 * on the floor. Reproduced on the 120 mA panel: voc_est walked 13.01 ->
 * 7.75 V over one second and the operating point went with it, 73% of
 * available power. The in-flight downward path is the sweep below, not
 * this estimator. */
#ifndef MPPT_VOC_DUTY_TH
#define MPPT_VOC_DUTY_TH            60    /* of 2000 */
#endif

/* ---------------------------------------------------------------------
 * BOOT Voc CAPTURE
 *
 * The bus voltage at power-up IS Voc, but only once nothing is loading it.
 * Sampling inside mppt_init() is too early: AM32 plays its startup tunes by
 * driving the motor coils, which pulls the bus down and drains Cbus, and
 * the arming tune (main.c:1362, playInputTune) fires LATER than mppt_init()
 * anyway. A single read at init would capture a sagged bus and set the
 * setpoint low for the whole flight.
 *
 * So the capture waits for a genuinely quiet window instead: no tone
 * pending, no current, no applied duty, and a bus voltage that has stopped
 * moving - that last one is what actually proves Cbus has refilled. It
 * keeps retrying until it gets one, so an early arm just delays it rather
 * than losing it.
 * ------------------------------------------------------------------- */
#ifndef MPPT_BOOT_VOC_SETTLE_MS
#define MPPT_BOOT_VOC_SETTLE_MS    300
#endif
/* Total bus movement tolerated across the whole settle window - measured
 * against the voltage at the start of it, not tick to tick. Tick-to-tick
 * differencing lets a slow steady ramp through indefinitely; anchoring
 * bounds the drift to this. */
#ifndef MPPT_BOOT_VOC_STABLE_DV
#define MPPT_BOOT_VOC_STABLE_DV     10    /* 0.10 V over 300 ms */
#endif
#define MPPT_BOOT_VOC_SETTLE_TICKS MPPT_MS_TO_TICKS(MPPT_BOOT_VOC_SETTLE_MS)

/* ===================================================================== */
/*  8. STARTUP                                                           */
/* ===================================================================== */
/* A cold panel has no back-EMF to push against. AM32's normal ramp will
 * happily walk duty up until the bus folds. We cap the duty ceiling during
 * spin-up and let the voltage PI, not the ramp, decide how fast to go. */
#ifndef MPPT_STARTUP_DUTY_MAX
#define MPPT_STARTUP_DUTY_MAX      400    /* of 2000 */
#endif
#ifndef MPPT_STARTUP_ZC_COUNT
#define MPPT_STARTUP_ZC_COUNT      150    /* zero-crossings to call it running */
#endif

/* ===================================================================== */
/*  9. ADC SCALING - must match your target's targets.h                  */
/* ===================================================================== */
/* AM32 uses a 12-bit right-aligned result on every MCU except the NXP
 * MCXA parts, which report 16-bit (see the #ifdef NXP block around
 * battery_voltage in main.c). Mirror that split here so the conversions
 * below stay bit-identical to AM32's own. */
#ifndef MPPT_ADC_FULL_SCALE
#ifdef NXP
#define MPPT_ADC_FULL_SCALE      65535
#else
#define MPPT_ADC_FULL_SCALE       4095    /* 12-bit */
#endif
#endif
#ifndef MPPT_ADC_VREF_MV
#define MPPT_ADC_VREF_MV          3300
#endif

/* Highest bus voltage this board can measure, in 10 mV units. Above this
 * the ADC rails and every threshold in this file is meaningless. */
#define MPPT_ADC_FULL_SCALE_V \
    ((MPPT_ADC_VREF_MV * TARGET_VOLTAGE_DIVIDER) / 100)

/* AM32 divides by the literal 41 (= 4095/100, rounded up) in its current
 * path, which makes the intermediate "millivolts * 100". Kept identical. */
/* AM32 divides by the literal 41 (= 4095/100, rounded up) in its current
 * path, which makes the intermediate "millivolts * 100". Kept identical.
 *
 * This is 12-bit ADC arithmetic. The 16-bit NXP MCXA parts stage it
 * differently and would read 16x low here, so they are rejected outright
 * below rather than shipped with silently wrong scaling. */
#ifndef MPPT_I_SCALE_DIV
#define MPPT_I_SCALE_DIV            41
#endif

#ifdef NXP
#error "MPPT does not support the 16-bit NXP MCXA ADC path - the current \
scaling in mppt_read_amps() assumes a 12-bit result. Untested, so refused \
rather than silently mis-scaled by 16x."
#endif

/* ---------------------------------------------------------------------
 * CURRENT GAIN, DECOUPLED FROM TELEMETRY
 *
 * The MPPT does its own current conversion and does NOT have to share
 * MILLIVOLT_PER_AMP with AM32's telemetry path.
 *
 * This matters on EGAN_JUWI_L431. DShot telemetry reports current in whole
 * amps, so a sub-amp bench panel reads as a flat 0 A; MILLIVOLT_PER_AMP is
 * deliberately mis-set (13 instead of the true 136) to inflate the reading
 * ~10x and make it visible. That is a fine thing to do to a display. It is
 * not a fine thing to do to a control loop: every absolute current
 * threshold in this file would then be expressed in fictional amps.
 *
 * So the board block sets MPPT_MILLIVOLT_PER_AMP to the TRUE sense-chain
 * gain and the two paths go their separate ways.
 * ------------------------------------------------------------------- */
#ifndef MPPT_MILLIVOLT_PER_AMP
#define MPPT_MILLIVOLT_PER_AMP  MILLIVOLT_PER_AMP
#endif

/* One ADC count of current, in 10 mA units, Q8 (value/256).
 * EGAN_JUWI_L431 at the true 136 mV/A: 151/256 = 0.59, i.e. 5.9 mA. */
#define MPPT_I_LSB_Q8 \
    ((MPPT_ADC_VREF_MV * 256) / (MPPT_I_SCALE_DIV * MPPT_MILLIVOLT_PER_AMP))

/* Array nameplate current at STC, in 10 mA units.
 *
 * Impp is the knob because it is the number people have: cell vendors and
 * panel datasheets quote the maximum power point, and on a wing you size
 * the array by the current you want at the MPP, not by what it does into a
 * short. Isc follows from the fill factor above and remains the reference
 * every current-domain threshold is checked against - it is what tells the
 * module whether its own measurements mean anything.
 *
 * Define MPPT_ARRAY_ISC directly to override if Isc is what you measured;
 * the board block on EGAN_JUWI_L431 does exactly that for a 120 mA bench
 * panel. Defining either one alone is enough. */
#ifndef MPPT_ARRAY_IMPP
#define MPPT_ARRAY_IMPP            600    /* 6.00 A at the maximum power point */
#endif

#ifndef MPPT_ARRAY_ISC
#define MPPT_ARRAY_ISC \
    (((int32_t)MPPT_ARRAY_IMPP << 8) / MPPT_IMPP_FRAC_Q8)
#endif

/* How many ADC counts the whole array spans. Below ~40 the current channel
 * carries almost no information and hill-climbing is pointless. */
#define MPPT_ARRAY_ISC_COUNTS \
    (((int32_t)MPPT_ARRAY_ISC << 8) / MPPT_I_LSB_Q8)

/* ===================================================================== */
/* 10. CONFIGURATION SANITY CHECKS (compile time)                        */
/* ===================================================================== */
/*
 * These exist because the original constants were written for a 24 V array
 * and silently produced an unreachable configuration when dropped onto a
 * board whose divider tops out at 15.5 V. Nothing here costs a byte of
 * flash; it just refuses to build a configuration that cannot work.
 */
_Static_assert(MPPT_VOC_NOMINAL < MPPT_ADC_FULL_SCALE_V,
    "MPPT_VOC_NOMINAL is above what TARGET_VOLTAGE_DIVIDER can measure");
_Static_assert(MPPT_V_ABSOLUTE_MIN < MPPT_V_COLLAPSE,
    "MPPT_V_ABSOLUTE_MIN must be below MPPT_V_COLLAPSE");
_Static_assert(MPPT_V_COLLAPSE + MPPT_V_COLLAPSE_HYST
                   < ((MPPT_VOC_NOMINAL * MPPT_VREF_MAX_FRAC_Q8) >> 8),
    "collapse threshold leaves no usable vref band below Voc");
_Static_assert((MPPT_VOC_NOMINAL * MPPT_K_FOCV_Q8 >> 8) > MPPT_V_COLLAPSE,
    "fractional-Voc seed sits below the collapse threshold");
/* The rpm tracker clamps k to [MPPT_K_MIN_Q8, MPPT_K_MAX_Q8]. A seed
 * outside that band (possible if a board block overrides the per-cell
 * voltages) would jump to the rail on the first track decision. */
_Static_assert(MPPT_K_FOCV_Q8 >= MPPT_K_MIN_Q8
                   && MPPT_K_FOCV_Q8 <= MPPT_K_MAX_Q8,
    "MPPT_K_FOCV_Q8 seed is outside the rpm tracker's clamp band");
_Static_assert(MPPT_TICK_HZ >= 500,
    "control tick below 500 Hz - the PI gains and deadbands do not hold");
_Static_assert(MPPT_RECOVER_OK_TICKS >= 1 && MPPT_RECOVER_TICKS >= 1,
    "recover timeouts round to zero ticks at this LOOP_FREQUENCY_HZ");
/* Tightest integer margin in the module by a wide margin: the RECOVER
 * decay multiply sits at 79% of int32 with the stock constants
 * (2000<<12) * 208 = 1.70e9. Raising either factor overflows it. */
_Static_assert((int64_t)((int64_t)MPPT_I_TERM_MAX << 12) * MPPT_RECOVER_DECAY_Q8
                   < 2147483647LL,
    "duty_q12 * MPPT_RECOVER_DECAY_Q8 overflows int32");
/* The open-circuit detector must be able to distinguish "unloaded" from
 * "at the MPP". If the threshold is at or above the array's own Isc it is
 * true even at short circuit, and voc_est tracks the loaded bus voltage
 * down instead. This is the check that would have caught MPPT_VOC_I_TH
 * being set to 0.20 A on a 0.12 A panel. */
_Static_assert(MPPT_VOC_I_TH < (MPPT_ARRAY_ISC / 2),
    "MPPT_VOC_I_TH is not comfortably below MPPT_ARRAY_ISC");
_Static_assert(MPPT_VOC_I_TH >= 1,
    "MPPT_VOC_I_TH rounds to zero - set MPPT_ARRAY_ISC or override it");
/* Sub-count thresholds are meaningless: the reading cannot land there. */
_Static_assert(MPPT_ARRAY_ISC_COUNTS >= 8,
    "array Isc spans under 8 ADC counts - check MPPT_MILLIVOLT_PER_AMP");

/* ===================================================================== */
/* 11. PUBLIC API                                                        */
/* ===================================================================== */

typedef enum {
    MPPT_STATE_OFF     = 0,   /* disarmed / not running                  */
    MPPT_STATE_SEED    = 1,   /* seeding vref from fractional Voc        */
    MPPT_STATE_TRACK   = 2,   /* normal tracking (FOCV or hill-climb)    */
    MPPT_STATE_RECOVER = 3    /* bus collapse - unloading, SAFETY PATH   */
} mppt_state_t;

typedef struct {
    mppt_state_t state;

    /* Filtered measurements (10 mV / 10 mA units) */
    int32_t  v;                 /* panel/bus voltage                     */
    int32_t  i;                 /* bus current                           */
    int32_t  i_q8;              /* filter state, Q8 - see mppt.c          */
    int32_t  p;                 /* v*i, in 0.1 mW units                  */

    /* Control */
    int32_t  vref;              /* tracker output = PI setpoint, 10 mV   */
    int32_t  voc_est;           /* open-circuit voltage estimate, 10 mV  */
    int32_t  duty_q12;          /* PI output, AM32 duty units << 12      */
    int32_t  duty_q12_prev;     /* for the rise-rate limit               */
    uint16_t duty;              /* PI output, AM32 duty units 0..2000    */
    uint16_t duty_applied;      /* duty actually handed to the PWM       */

    int32_t  i_term_q12;

    /* Tracker state. k is moved by the rpm tracker, beta_trim by the beta
     * regulator; only one is active, and mppt_focv_seed() applies both so
     * the seed is correct either way. */
    int32_t  k_focv_q8;

    /* RPM perturb-and-observe */
    int32_t  rpm_acc;
    int32_t  rpm_acc_last;
    uint16_t rpm_n;
    uint16_t rpm_ticks;
    uint16_t rpm_input_ref;
    uint16_t rpm_steps;         /* decisions taken - telemetry             */
    uint8_t  rpm_phase;
    uint8_t  rpm_have_last;
    int8_t   rpm_dir;

    /* Beta method */
    int32_t  beta;              /* live beta, Q8 - telemetry / calibration */
    int32_t  beta_mpp;          /* the target it is regulated to, Q8       */
    int32_t  beta_trim;         /* offset from the FOCV seed, 10 mV        */
    uint16_t beta_ticks;
    uint8_t  beta_valid;        /* 1 = current usable, regulator running   */

    /* Boot Voc capture */
    int32_t  voc_boot;          /* what the settled boot read gave, 10 mV */
    int32_t  i_zero_raw;        /* measured sense-amp zero, raw ADC counts */
    uint8_t  i_zero_done;
    int32_t  quiet_v_last;
    uint16_t quiet_ticks;
    uint8_t  boot_voc_done;

    /* Diagnostics - safe to stream over telemetry */
    uint8_t  coasting;          /* 1 = FETs floated, motor freewheeling   */
    uint16_t coast_events;
    uint16_t recover_ticks;
    uint16_t collapse_events;
} mppt_t;

extern mppt_t mppt;

/* Call once from main(), after initCorePeripherals() and after the first
 * ADC conversion has completed, before the while(1). */
void mppt_init(void);

/* Call from the existing 1 kHz block inside tenKhzRoutine(), i.e. inside
 *     if (one_khz_loop_counter > PID_LOOP_DIVIDER) { ... }
 * Reads ADC_raw_volts / ADC_raw_current directly. */
void mppt_1khz_update(void);

/* Call from tenKhzRoutine() immediately before
 *     adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1;
 * Applies the MPPT duty ceiling. Reductions are applied IMMEDIATELY
 * (bypassing AM32's slew limiter) so a collapse can be caught in one tick;
 * increases still respect AM32's ramp.
 *
 * The pointer is volatile-qualified because main.c's duty_cycle is
 * `volatile uint16_t` (main.c:566). Taking a plain uint16_t* here compiles
 * fine on its own but fails the tree's -Werror on -Wdiscarded-qualifiers at
 * the call site. */
void mppt_apply_duty(volatile uint16_t *duty_cycle);

/* True when the tracker is actively controlling duty. */
static inline uint8_t mppt_is_active(void)
{
    return (mppt.state == MPPT_STATE_TRACK) ||
           (mppt.state == MPPT_STATE_SEED)  ||
           (mppt.state == MPPT_STATE_RECOVER);
}

#else  /* !USE_MPPT - preprocess the call sites away entirely */

#define mppt_init()            ((void)0)
#define mppt_1khz_update()     ((void)0)
#define mppt_apply_duty(p)     ((void)0)
#define mppt_is_active()       (0)

#endif /* USE_MPPT */
