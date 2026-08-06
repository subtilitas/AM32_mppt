/*
 * mppt.h - Fast MPPT tracker for AM32 ESC firmware
 *
 * Target      : STM32L431 (Cortex-M4F @ 80 MHz), AM32 v2.x
 * Application : Direct-drive solar aircraft. PV array -> ESC -> BLDC.
 *               NO BATTERY ANYWHERE. The MCU logic supply is on the same
 *               bus as the panel, so bus collapse = loss of control.
 *
 * Architecture (cascaded, all integer math, no FPU use):
 *
 *   tenKhzRoutine()          10/20 kHz   duty application + collapse clamp
 *     +-- 1 kHz block        ~1 kHz      sample, filter, PI voltage loop
 *           +-- tracker      ~200 Hz     dP/dV incremental conductance
 *                                        with Hassani two-step escalation
 *
 * The inner PI drives duty cycle to hold the PANEL voltage at vref.
 * The outer tracker moves vref. Decoupling this way lets the tracker run
 * ~200 Hz instead of being stuck behind the propeller's ~0.5-2 s
 * mechanical time constant.
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
 *  [1] Hassani, Maamoun, Tadrist, Nesba, "A New High Speed and Accurate
 *      FPGA-based MPPT Method for Photovoltaic Systems", IJPEDS Vol.8 No.3,
 *      2017. -> the two-step (DminStep/DmaxStep) automatic switcher.
 *  [2] Microchip AN1521, "Practical Guide to Implementing Solar Panel MPPT
 *      Algorithms". -> 1 kHz PI inner loop, MPPT_AVERAGE sample averaging,
 *      and the finding that the dI/dV form of incr. conductance is too
 *      noise-sensitive on real hardware while the dP/dV form works.
 *  [3] TI TIDA-010042, 400-W GaN MPPT Charge Controller Reference Design.
 *      -> "when Vpanel is within 97.5%-102.5% of Vmpp, the output power of
 *      the panel is above 99.5% of maximum power". This is why the
 *      deadbands below are deliberately generous.
 */

#pragma once

#include <stdint.h>
#include "targets.h"

#ifdef USE_MPPT

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
 * it at compile time. On EGAN_MPPT_L431 (divider 47) it is 15.51 V, which
 * is why this file is scaled for a ~12 V array and not the 24 V array the
 * module was originally written against.
 *
 * Every constant here is #ifndef-guarded, so a board block in targets.h can
 * override any of them without editing this file.
 */

/* Nameplate open-circuit voltage of the array at STC. Used only as a
 * fallback until the firmware measures Voc for real at power-up. */
#ifndef MPPT_VOC_NOMINAL
#define MPPT_VOC_NOMINAL          1200    /* 12.00 V */
#endif

/* Vmpp/Voc ratio, Q8 (value/256). Crystalline silicon is 0.76-0.80.
 * 200/256 = 0.781. This is the STARTING value only - see the rpm tracker in
 * section 6b, which adapts it in flight. It still sets where the tracker
 * begins, so a good guess converges faster.
 *
 * Be aware that 0.781 is a textbook figure and real panels vary widely. For
 * the single-diode model of the 13.4 V / 120 mA bench panel the true ratio
 * is 0.845, and running at 0.781 puts the setpoint at 10.47 V against a
 * Vmpp of 11.32 V. That is the error the rpm tracker exists to remove. */
#ifndef MPPT_K_FOCV_Q8
#define MPPT_K_FOCV_Q8            200
#endif

/* Bounds on the adapted ratio. No silicon panel has its MPP outside this,
 * so the tracker cannot walk the setpoint anywhere dangerous however badly
 * the rpm signal misbehaves. */
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
/*  2. SAFETY THRESHOLDS      *** THESE KEEP THE MCU ALIVE ***           */
/* ===================================================================== */
/*
 * With no battery, if the bus collapses the MCU browns out and you lose
 * the aircraft. MPPT_V_COLLAPSE must sit comfortably above the regulator
 * dropout + the worst-case sag that Cbus can ride through.
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
#ifndef MPPT_V_COLLAPSE
#define MPPT_V_COLLAPSE            700    /*  7.00 V - enter RECOVER      */
#endif
#ifndef MPPT_V_COLLAPSE_HYST
#define MPPT_V_COLLAPSE_HYST        75    /*  0.75 V - exit hysteresis    */
#endif
#ifndef MPPT_V_ABSOLUTE_MIN
#define MPPT_V_ABSOLUTE_MIN        550    /*  5.50 V - duty forced to 0   */
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

/* Tracker decision period, in control ticks.
 *   5 -> ~190 Hz tracker at a 952 Hz tick.
 * Lower is faster but the inner PI must have settled. Do not set below
 * the inner loop's ~2% settling time (see tuning notes in mppt.c). */
#ifndef MPPT_PERIOD_TICKS
#define MPPT_PERIOD_TICKS            5
#endif

/* Number of ticks averaged immediately BEFORE each decision. Per AN1521,
 * the samples right after a perturbation contain the settling transient
 * and must be discarded; only the last MPPT_AVG_TICKS are used.
 * Must be >= 1 and <= MPPT_PERIOD_TICKS. */
#ifndef MPPT_AVG_TICKS
#define MPPT_AVG_TICKS               2
#endif

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

/* Integrator clamp, in AM32 duty units. */
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
 * and put the bus on the floor at 4.6 V - a brownout, and with no battery
 * that is the aircraft.
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
 * current surge, which collapses the bus again. That is the limit cycle.
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
#ifndef MPPT_COAST_ENABLE
#define MPPT_COAST_ENABLE            1
#endif
#ifndef MPPT_COAST_DUTY_TH
#define MPPT_COAST_DUTY_TH          40    /* enter coast below 2% demand */
#endif
#ifndef MPPT_COAST_EXIT_DUTY
#define MPPT_COAST_EXIT_DUTY        90    /* leave it above 4.5%         */
#endif
_Static_assert(MPPT_COAST_EXIT_DUTY > MPPT_COAST_DUTY_TH,
    "coast thresholds need hysteresis or the output stage will chatter");

/* ---- tracker slot phases (see mppt.c) ---- */
#define MPPT_PHASE_DRIFT             0
#define MPPT_PHASE_RESPONSE          1

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
/*  6. TRACKER (dP/dV incremental conductance + two-step escalation)     */
/* ===================================================================== */

/* Perturbation steps applied to vref, in 10 mV.
 * MPPT_STEP_MIN sets the steady-state ripple around the MPP.
 * Per TI [3], +-2.5% of Vmpp still yields >99.5% of available power, so
 * for a 12 V array (Vmpp ~9.4 V) anything under ~235 (2.35 V) is "free".
 * We use far less than that; the limit is really ADC noise. */
#ifndef MPPT_STEP_MIN
#define MPPT_STEP_MIN                8    /* 0.08 V */
#endif
#ifndef MPPT_STEP_MAX
#define MPPT_STEP_MAX               40    /* 0.40 V */
#endif

/* Hassani two-step switcher [1]: after this many consecutive
 * same-direction MIN steps, escalate to MAX. Any direction reversal drops
 * straight back to MIN. Counter-based and therefore SCALE-FREE - it works
 * identically at 5 W and at 50 W, which matters when a banking wing swings
 * irradiance 10:1. */
#ifndef MPPT_ESCALATE_COUNT
#define MPPT_ESCALATE_COUNT          4
#endif

/* MPP lock deadbands. Inside both -> hold vref still (this is what kills
 * the steady-state dither that plain P&O always has).
 *
 * The power deadband is RELATIVE (per-mille of present power) with an
 * absolute floor, for the same reason the transient threshold is relative:
 * a fixed 0.5 W deadband is invisible at full sun and swallows the entire
 * signal in deep shade. */
#ifndef MPPT_DP_DEADBAND_PERMILLE
#define MPPT_DP_DEADBAND_PERMILLE    8    /* 0.8% of present power  */
#endif
#ifndef MPPT_DP_DEADBAND_MIN
#define MPPT_DP_DEADBAND_MIN       500    /* floor: 50 mW, 0.1 mW units */
#endif

/* QUANTISATION FLOOR ON THE DEADBAND. This is the one that bit us.
 *
 * dp is computed as v*i, so one ADC count of current shows up as a step of
 * v * MPPT_I_LSB in dp whether or not anything real happened. If the
 * deadband is smaller than that step, the tracker reverses direction on
 * pure quantisation and hunts forever - which is exactly what a small
 * panel on a 5.9 mA/count sense chain looks like.
 *
 * Measured on the 13.4 V / 120 mA bench panel (20 ADC counts end to end):
 * one count of current produced 6.2x the deadband, and tracking efficiency
 * sat at 77.6%. Raising the deadband above the quantisation step took it
 * to 90.7% with no other change.
 *
 * The floor is therefore computed at runtime from the live voltage rather
 * than being a fixed number, so it stays correct as the operating point
 * moves. Set the multiplier to 0 to disable. */
#ifndef MPPT_DP_QUANT_MULT
#define MPPT_DP_QUANT_MULT           3    /* deadband >= 3 counts of dp */
#endif
#ifndef MPPT_DV_DEADBAND
#define MPPT_DV_DEADBAND             3    /* 0.03 V, in 10 mV       */
#endif

/* Irradiance-transient detection, RELATIVE (percent of present current).
 * Absolute amp thresholds are useless here: 0.5 A is a rounding error at
 * full sun and the entire output in deep shade. If |di| exceeds this
 * fraction of i, the power change is attributed to irradiance rather than
 * to our perturbation, and the tracker holds direction instead of
 * spuriously reversing. */
#ifndef MPPT_I_TRANSIENT_PCT
#define MPPT_I_TRANSIENT_PCT        18    /* 18% */
#endif

/* If a transient is large enough, abandon hill-climbing entirely and
 * re-seed from the fractional-Voc estimate. This is the only way to
 * recover from a 10:1 swing in tens of milliseconds. */
#ifndef MPPT_I_RESEED_PCT
#define MPPT_I_RESEED_PCT           60    /* 60% */
#endif

/* ---------------------------------------------------------------------
 * HILL-CLIMB GATE - when is the current channel worth listening to?
 *
 * The tracker infers dP/dV from a current measurement. If the operating
 * current is only a handful of ADC counts, dP is dominated by quantisation
 * and hill-climbing does worse than not trying: on the 120 mA bench panel
 * every tuning of the hill-climber topped out at 86.6%, while plain
 * fractional-Voc - which needs no current measurement at all - held 96.8%
 * dead flat.
 *
 * So: hill-climb only above MPPT_TRACK_MIN_COUNTS of current, and run pure
 * fractional-Voc below it. The threshold is in ADC counts, not amps, so it
 * follows the sense chain automatically.
 *
 * 125 counts is where one count equals the 0.8% relative power deadband.
 * ------------------------------------------------------------------- */
#ifndef MPPT_TRACK_MIN_COUNTS
#define MPPT_TRACK_MIN_COUNTS      125
#endif
#define MPPT_TRACK_I_MIN \
    ((((int32_t)MPPT_TRACK_MIN_COUNTS * MPPT_I_LSB_Q8) >> 8) + 1)

/* Set to force one mode regardless of measured current. Useful for bringing
 * a new array up: MPPT_MODE_FOCV is smooth and cannot mis-track. */
#define MPPT_MODE_AUTO               0
#define MPPT_MODE_FOCV               1
#define MPPT_MODE_HILLCLIMB          2
#ifndef MPPT_MODE
#define MPPT_MODE            MPPT_MODE_AUTO
#endif

/* ===================================================================== */
/*  6b. RPM FEEDBACK - slow adaptation of the FOCV ratio                 */
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
 * dP is mostly quantisation (see MPPT_DP_QUANT_MULT). But the ESC has a
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

/* Set to 0 to compile the rpm tracker out and run plain fixed-k FOCV. */
#ifndef MPPT_RPM_TRACK
#define MPPT_RPM_TRACK               1
#endif

/* Settling allowance after each perturbation, before rpm is believed.
 * MUST exceed the prop's mechanical time constant or the tracker measures
 * its own transient and wanders - the same trap as MPPT_PERIOD_TICKS in
 * the power-domain tracker, but three orders of magnitude slower. */
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
 * The genuinely fast events are cloud edges and fuselage/prop shadow,
 * which is exactly what MPPT_I_TRANSIENT_PCT and MPPT_I_RESEED_PCT above
 * are for: hold direction on a moderate step, jump to the fractional-Voc
 * estimate on a big one.
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
 * On EGAN_MPPT_L431 the sense amp is bidirectional (CURRENT_OFFSET 2500,
 * MILLIVOLT_PER_AMP 136), so 1 mV of offset drift moves the reading by
 * 100/136 = 0.74 in these units, i.e. ~7.4 mA per mV. A 10 mV offset error
 * is therefore 0.074 A, which alone would swamp the original 0.08 A
 * threshold. Verify the reading at true zero current on the bench before
 * trusting Voc tracking, and raise this if it does not settle below it.
 */
/* 5% of Isc, but never below what the ADC can actually resolve. On a small
 * array 5% of Isc rounds to zero - 5% of 120 mA is 6 mA and one ADC count
 * is 5.9 mA - so the floor is what binds there. */
#define MPPT_MAX2(a, b) ((a) > (b) ? (a) : (b))
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

/* ---------------------------------------------------------------------
 * PERIODIC Voc SWEEP
 *
 * Force duty to 0 briefly and read the true open-circuit voltage. This is
 * the only Voc source that does not depend on the current sensor at all,
 * which is what makes it usable on an array whose whole Isc is 20 ADC
 * counts wide.
 *
 * The sweep ends as soon as the bus stops rising, so it costs only as long
 * as the panel needs to charge Cbus: ~0.5 ms on a 3 A array, ~12 ms on the
 * 120 mA bench panel (C*dV/I = 470uF * 2.9 V / 0.12 A).
 *
 * Set MPPT_VOC_SWEEP_INTERVAL_MS to 0 to disable.
 * ------------------------------------------------------------------- */
/* *** DISABLED BY DEFAULT, AND THINK HARD BEFORE ENABLING IT. ***
 *
 * "Duty 0" is not coasting on this hardware. AM32 drives complementary PWM
 * whenever eepromBuffer.comp_pwm is set (see phaseAPWM() in phaseouts.c), so
 * at duty 0 the low-side FETs are on ~100% of the time and the energised
 * winding pair is shorted through them. That is synchronous BRAKING. On the
 * bench it is unmistakable: the motor is actively braked once per interval
 * and the whole aircraft stutters.
 *
 * The energy cost was never the problem. Measured over a 60 s flight-scale
 * run: 2 s -> 93.3%, 5 s -> 95.1%, 15 s -> 96.3%, off -> 97.0%. Losing 0.75%
 * would be fine. Braking the propeller every 15 seconds is not.
 *
 * Boot Voc plus the opportunistic estimator above covers it instead. Voc is
 * measured properly at power-up - panel unloaded, motor stopped, no braking
 * possible - and refreshed every time the throttle comes down. Thermal drift
 * is about -0.3%/degC, and per TI TIDA-010042 a setpoint within 2.5% of Vmpp
 * still returns >99.5% of available power, so a stale estimate is cheap.
 *
 * Only enable this if comp_pwm is OFF on your setup, in which case duty 0
 * really is a coast. Even then the body diodes will rectify into the bus if
 * back-EMF exceeds bus voltage, which would read as a falsely high Voc. */
#ifndef MPPT_VOC_SWEEP_INTERVAL_MS
#define MPPT_VOC_SWEEP_INTERVAL_MS   0    /* 0 = disabled */
#endif
#ifndef MPPT_VOC_SWEEP_MAX_MS
#define MPPT_VOC_SWEEP_MAX_MS       40    /* hard timeout */
#endif
#ifndef MPPT_VOC_SWEEP_SETTLE_DV
#define MPPT_VOC_SWEEP_SETTLE_DV     3    /* 0.03 V/tick = stopped rising */
#endif
#ifndef MPPT_VOC_SWEEP_SETTLE_TICKS
#define MPPT_VOC_SWEEP_SETTLE_TICKS  3
#endif
#define MPPT_VOC_SWEEP_INTERVAL_TICKS MPPT_MS_TO_TICKS(MPPT_VOC_SWEEP_INTERVAL_MS)
#define MPPT_VOC_SWEEP_MAX_TICKS      MPPT_MS_TO_TICKS(MPPT_VOC_SWEEP_MAX_MS)

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
#ifndef MPPT_I_SCALE_DIV
#define MPPT_I_SCALE_DIV            41
#endif

/* ---------------------------------------------------------------------
 * CURRENT GAIN, DECOUPLED FROM TELEMETRY
 *
 * The MPPT does its own current conversion and does NOT have to share
 * MILLIVOLT_PER_AMP with AM32's telemetry path.
 *
 * This matters on EGAN_MPPT_L431. DShot telemetry reports current in whole
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
 * EGAN_MPPT_L431 at the true 136 mV/A: 151/256 = 0.59, i.e. 5.9 mA. */
#define MPPT_I_LSB_Q8 \
    ((MPPT_ADC_VREF_MV * 256) / (MPPT_I_SCALE_DIV * MPPT_MILLIVOLT_PER_AMP))

/* Array nameplate short-circuit current at STC, in 10 mA units. This is
 * the reference every current-domain threshold is checked against - it is
 * what tells the module whether its own measurements mean anything. */
#ifndef MPPT_ARRAY_ISC
#define MPPT_ARRAY_ISC             300    /* 3.00 A */
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
_Static_assert(MPPT_AVG_TICKS >= 1 && MPPT_AVG_TICKS <= MPPT_PERIOD_TICKS,
    "MPPT_AVG_TICKS must be in 1..MPPT_PERIOD_TICKS");
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
    MPPT_STATE_RECOVER = 3,   /* bus collapse - unloading, SAFETY PATH   */
    MPPT_STATE_VOC     = 4    /* duty 0, measuring open-circuit voltage  */
} mppt_state_t;

typedef struct {
    mppt_state_t state;

    /* Filtered measurements (10 mV / 10 mA units) */
    int32_t  v;                 /* panel/bus voltage                     */
    int32_t  i;                 /* bus current                           */
    int32_t  p;                 /* v*i, in 0.1 mW units                  */

    /* Control */
    int32_t  vref;              /* tracker output = PI setpoint, 10 mV   */
    int32_t  voc_est;           /* open-circuit voltage estimate, 10 mV  */
    int32_t  duty_q12;          /* PI output, AM32 duty units << 12      */
    int32_t  duty_q12_prev;     /* for the rise-rate limit               */
    uint16_t duty;              /* PI output, AM32 duty units 0..2000    */
    uint16_t duty_applied;      /* duty actually handed to the PWM       */

    /* Tracker internals */
    int32_t  i_term_q12;
    int32_t  p_last;
    int32_t  v_last;
    int32_t  i_last;
    int32_t  step;
    int32_t  dp_drift;          /* manoeuvre-driven dP, subtracted out    */
    int8_t   dir;
    uint8_t  phase;             /* MPPT_PHASE_DRIFT / _RESPONSE           */
    uint8_t  railed;            /* last step was swallowed by the clamp   */
    uint8_t  same_dir_count;

    /* Averaging accumulators */
    int32_t  v_acc;
    int32_t  i_acc;
    uint8_t  acc_n;
    uint8_t  tick;

    /* Boot Voc capture */
    int32_t  voc_boot;          /* what the settled boot read gave, 10 mV */
    int32_t  quiet_v_last;
    uint16_t quiet_ticks;
    uint8_t  boot_voc_done;

    /* RPM feedback - slow adaptation of the FOCV ratio */
    int32_t  k_focv_q8;         /* adapted Vmpp/Voc, Q8. THE learned value */
    int32_t  rpm_acc;           /* sum of e_com_time over the window      */
    int32_t  rpm_acc_last;
    uint16_t rpm_n;
    uint16_t rpm_ticks;
    uint16_t rpm_input_ref;     /* throttle at the start of the cycle     */
    uint16_t rpm_steps;         /* decisions taken - telemetry            */
    uint8_t  rpm_phase;
    uint8_t  rpm_have_last;
    int8_t   rpm_dir;

    /* Voc sweep */
    uint16_t since_sweep;       /* ticks since the last sweep             */
    uint16_t sweep_ticks;       /* ticks spent in the current sweep       */
    int32_t  sweep_v_last;      /* for the "stopped rising" test          */
    int32_t  sweep_i_term;      /* integrator parked across the sweep     */
    uint8_t  sweep_settle_n;

    /* Diagnostics - safe to stream over telemetry */
    uint8_t  hill_climb;        /* 1 = climbing, 0 = fractional-Voc only  */
    uint8_t  coasting;          /* 1 = FETs floated, motor freewheeling   */
    uint16_t coast_events;
    uint16_t recover_ticks;
    uint16_t collapse_events;
    uint16_t reseed_events;
    uint16_t voc_sweeps;
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
           (mppt.state == MPPT_STATE_VOC)   ||
           (mppt.state == MPPT_STATE_RECOVER);
}

#else  /* !USE_MPPT - preprocess the call sites away entirely */

#define mppt_init()            ((void)0)
#define mppt_1khz_update()     ((void)0)
#define mppt_apply_duty(p)     ((void)0)
#define mppt_is_active()       (0)

#endif /* USE_MPPT */
