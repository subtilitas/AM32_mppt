/*
 * mppt.c - Maximum power point tracking for AM32 ESC firmware
 * See mppt.h for architecture, references and configuration.
 *
 * ---------------------------------------------------------------------
 * ALGORITHM SELECTION - how it ended up here
 * ---------------------------------------------------------------------
 * Default is the BETA METHOD: beta = ln(I/V) - c*V, regulated to its known
 * MPP value. A regulator, not a hill-climber, so nothing perturbs and
 * nothing dithers. Full detail in mppt.h section 6a.
 *
 * Three alternatives were implemented, measured against the same plant, and
 * REMOVED. Recorded here so nobody re-derives them:
 *
 *  - Incremental conductance (dP/dV, Hassani two-step). The textbook
 *    choice, and unusable here. It infers dP/dV from a DIFFERENCE of
 *    currents, and this board's array spans about 20 ADC counts, so that
 *    difference is very nearly pure quantisation. Every tuning of it
 *    topped out around 86% while doing nothing at all held 97%.
 *
 *  - Fractional Voc. vref = k * Voc, open-loop on a constant. Cheap and
 *    unconditionally stable, but the constant is usually wrong: the
 *    textbook 0.781 put the setpoint at 10.47 V against a real Vmpp of
 *    11.32 V. It survives as the seed, the transient fallback, and what
 *    beta degrades into when the current reading dies.
 *
 *  - RPM perturb-and-observe on e_com_time. Reached 99.6% and needs no
 *    current measurement at all, so it is the only one that still works
 *    below ~25% irradiance where beta goes blind. Removed because it
 *    oscillates by construction: +-0.21 V on vref, forever. That is the
 *    dither beta exists to eliminate. The measured crossover is in
 *    doc/MPPT.md section 7, item 7, if you ever want it back.
 *
 * ---------------------------------------------------------------------
 * PI TUNING RECIPE (do this on the bench, prop off, panel or PSU+resistor)
 * ---------------------------------------------------------------------
 *
 * 1. Set MPPT_KI_DUTY_PER_VOLT_S = 0. Set KP = 4.
 * 2. Command a small vref step (e.g. 1 V) and log v. Raise KP until you
 *    see ~10% overshoot, then halve it. That is your KP.
 * 3. Raise KI until the steady-state error closes in ~5-10 ms without
 *    adding overshoot. Rule of thumb: KI ~ 20 * KP.
 * 4. Measure the 2% settling time of that step response. Any outer loop
 *    must be slower than it - measuring your own transient is how a
 *    tracker talks itself into walking the wrong way.
 * 5. Only then reconnect the prop.
 */

#include "targets.h"
#include "mppt.h"

#ifdef USE_MPPT

#include "common.h"
/* For allOff(). Included rather than hand-declared on purpose - a
 * hand-written extern is exactly how the zero_crosses type mismatch got in.
 * This is the per-MCU header (Mcu/<mcu>/Inc/phaseouts.h); every target
 * provides it with the same allOff() signature. */
#include "phaseouts.h"

/* --------------------------------------------------------------------- */
/* AM32 globals we consume. All are plain (non-static) globals in main.c. */
/*                                                                        */
/* EVERY TYPE HERE MUST MATCH ITS DEFINITION IN Src/main.c EXACTLY.       */
/* These are separate translation units, so the linker will NOT catch a   */
/* mismatch - it just silently reinterprets the object's bytes.           */
/*                                                                        */
/* This bit up. zero_crosses is `volatile uint32_t zero_crosses;` at      */
/* main.c:554 and is incremented without an upper bound in                */
/* zcfoundroutine(). Declaring it uint16_t here read only the low half    */
/* word, so at ~4800 zero-crossings/s the value this module saw wrapped   */
/* to 0 roughly every 13.7 s. Each wrap dropped it back under             */
/* MPPT_STARTUP_ZC_COUNT, which slammed the PI output ceiling from 2000   */
/* to MPPT_STARTUP_DUTY_MAX and clamped the integrator to match. Measured */
/* in simulation: duty 1494 -> 400 in one tick, the motor's back-EMF then */
/* exceeding the panel voltage, and 2.7 A of REGENERATION into the array  */
/* (-66 W) with the bus pushed above Voc, every 13.7 s, for ~30 ms at a   */
/* time. The sim harness had also declared it uint16_t, which is why the  */
/* shipped test never showed it.                                          */
/* --------------------------------------------------------------------- */
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t VOLTAGE_DIVIDER;
extern volatile char armed;
extern uint8_t  running;
extern uint16_t input;
extern char     prop_brake_active;
extern volatile uint32_t zero_crosses;

mppt_t mppt;

/* Consecutive ticks the bus has been healthy while in RECOVER. */
static uint8_t  recover_ok_ticks;
/* Ticks spent in SEED before beta takes over. */
static uint8_t  seed_ticks;
/* Consecutive ticks the inner PI has been holding v at vref. */
static uint8_t  reg_ticks;

/* --------------------------------------------------------------------- */
/* Small helpers                                                          */
/* --------------------------------------------------------------------- */

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

/* Enter or leave the coasting (all FETs floated) state. See the long note in
 * mppt.h - on this hardware duty 0 is a brake, not a coast. */
static inline void mppt_coast(uint8_t on)
{
    if (on && !mppt.coasting) {
        if (mppt.coast_events < 0xFFFFu) mppt.coast_events++;
    }
    mppt.coasting = on;
}

static inline int32_t clamp32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* --------------------------------------------------------------------- */
/* ADC conversion                                                         */
/* --------------------------------------------------------------------- */
/*
 * These reproduce AM32's own scaling exactly, so any calibration you have
 * already done in targets.h carries over unchanged.
 *
 * OVERFLOW NOTE - the staging matters. Do NOT "optimise" this into
 *   (raw * VREF_MV * VOLTAGE_DIVIDER) / (FULL_SCALE * 100)
 * because 4095 * 3300 * 480 = 6.49e9 overflows int32. Dividing by
 * FULL_SCALE first keeps every intermediate under 1.6e6.
 */
static inline int32_t mppt_read_volts(void)
{
    int32_t raw = (int32_t)ADC_raw_volts;
    int32_t mv  = (raw * MPPT_ADC_VREF_MV) / MPPT_ADC_FULL_SCALE;   /* <= 3300 */
    return (mv * (int32_t)VOLTAGE_DIVIDER) / 100;                   /* 10 mV units */
}

/* MPPT_I_SCALE_DIV and MPPT_MILLIVOLT_PER_AMP live in mppt.h - the latter
 * because the control loop must not inherit a gain that was deliberately
 * mis-set to make DShot telemetry readable. See the comment there. */
/*
 * The zero reference is MEASURED, not taken from CURRENT_OFFSET, whenever a
 * boot capture has succeeded.
 *
 * This matters far more than it looks. At MPPT_MILLIVOLT_PER_AMP = 136, one
 * millivolt of sense-amp offset is 7.4 mA - 6% of a 120 mA array. The beta
 * regulator tolerates about +-2 mV and rails past +-5 mV, and no
 * hand-entered constant survives temperature and part-to-part spread at
 * that resolution. The boot quiet window gives the reading for free: panel
 * unloaded, motor stopped, so whatever the ADC says then IS zero amps.
 */
static inline int32_t mppt_read_amps(void)
{
    int32_t raw = (int32_t)ADC_raw_current;
    int32_t x;

    if (mppt.i_zero_done) {
        raw -= mppt.i_zero_raw;
        if (raw < 0) raw = 0;
        x = (raw * MPPT_ADC_VREF_MV) / MPPT_I_SCALE_DIV;
    } else {
        x = (raw * MPPT_ADC_VREF_MV) / MPPT_I_SCALE_DIV;            /* <= 3.3e5 */
        x -= ((int32_t)CURRENT_OFFSET * 100);
    }
    x /= (int32_t)MPPT_MILLIVOLT_PER_AMP;                           /* 10 mA units */
    return (x < 0) ? 0 : x;
}

#if MPPT_TRACKER == MPPT_TRACKER_BETA
/* --------------------------------------------------------------------- */
/* Integer natural log, Q8                                                */
/* --------------------------------------------------------------------- */
/*
 * ln(x) for x >= 1, to about 0.004 absolute. CLZ gives the exponent (one
 * instruction on Cortex-M), a 17-entry table with linear interpolation
 * gives the mantissa, then scale log2 -> ln.
 *
 * Accuracy needed is modest: beta uses ln(I) - ln(V), and dbeta/dV is about
 * -1.42 per volt, so even 0.03 of log error is only ~20 mV of setpoint.
 * No float, no libm, ~20 cycles.
 */
/* 256 * log2(1 + n/16), n = 0..16. The last entry MUST be 256, since
 * log2(2) = 1; an off-the-end value here skews every mantissa and cost
 * 0.037 of log error - 26 mV of setpoint - before it was spotted by
 * checking the whole function against libm rather than eyeballing it. */
static const uint16_t mppt_log2_tab[17] = {
      0,  22,  44,  63,  82, 100, 118, 134, 150, 165, 179, 193, 207, 220,
    232, 244, 256
};

static int32_t mppt_ln_q8(uint32_t x)
{
    int32_t e, log2_q8;
    uint32_t m, idx, frac;

    if (x == 0) return -32768;                 /* clamped, caller guards */

    e = 31 - (int32_t)__builtin_clz(x);        /* floor(log2 x) */

    /* Normalise to 8 fractional bits below the leading 1. */
    m = (e >= 8) ? (x >> (e - 8)) : (x << (8 - e));
    m &= 0xFF;                                 /* mantissa, 0..255 */

    idx  = m >> 4;                             /* 0..15 */
    frac = m & 0x0F;
    log2_q8 = (e << 8)
            + (int32_t)mppt_log2_tab[idx]
            + (((int32_t)mppt_log2_tab[idx + 1] - (int32_t)mppt_log2_tab[idx])
               * (int32_t)frac >> 4);

    /* * ln(2), 0.693147 in Q16 = 45426 */
    return (log2_q8 * 45426) >> 16;
}

/* Compute the beta target from the array nameplate and the best Voc we
 * have. Called at init and again whenever the boot capture lands a real
 * Voc, so the target tracks the actual panel rather than a guess. */
static void mppt_beta_target(void)
{
#ifdef MPPT_BETA_MPP_DERIVED
    int32_t vmpp = (mppt.voc_est * mppt.k_focv_q8) >> 8;
    int32_t impp = ((int32_t)MPPT_ARRAY_ISC * MPPT_IMPP_FRAC_Q8) >> 8;

    if (vmpp > 0 && impp > 0) {
        mppt.beta_mpp = mppt_ln_q8((uint32_t)impp) - mppt_ln_q8((uint32_t)vmpp)
                        - ((vmpp << 8) / MPPT_BETA_VT);
    }
#else
    mppt.beta_mpp = MPPT_BETA_MPP_Q8;
#endif
}

#endif /* MPPT_TRACKER_BETA */

/* --------------------------------------------------------------------- */
/* Reference clamping                                                     */
/* --------------------------------------------------------------------- */
/*
 * The MPP of a silicon panel always lies between roughly 0.45*Voc and
 * 0.95*Voc across every irradiance and temperature it will ever see.
 * Clamping here is cheap insurance: no sequence of bad measurements can
 * walk the tracker down into the constant-current region where the bus
 * collapses, or up to open circuit where thrust goes to zero.
 */
static inline int32_t mppt_clamp_vref(int32_t vref)
{
    int32_t lo = (mppt.voc_est * MPPT_VREF_MIN_FRAC_Q8) >> 8;
    int32_t hi = (mppt.voc_est * MPPT_VREF_MAX_FRAC_Q8) >> 8;

    if (lo < MPPT_V_COLLAPSE + MPPT_V_COLLAPSE_HYST) {
        lo = MPPT_V_COLLAPSE + MPPT_V_COLLAPSE_HYST;
    }
    if (hi < lo) hi = lo;
    return clamp32(vref, lo, hi);
}

/* The fractional-Voc setpoint: the ratio (fixed under beta, adapted by
 * the rpm tracker) plus beta's learned trim. Only one of the two moves in
 * a given build, so applying both is always correct. Everything that seeds or re-seeds goes through
 * here, so the trim is never thrown away by a transient. */
static inline int32_t mppt_focv_seed(void)
{
    return mppt_clamp_vref(((mppt.voc_est * mppt.k_focv_q8) >> 8)
                           + mppt.beta_trim);
}

/* --------------------------------------------------------------------- */
/* init                                                                   */
/* --------------------------------------------------------------------- */

void mppt_init(void)
{
    uint8_t k;

    mppt.state          = MPPT_STATE_OFF;
    mppt.voc_est        = MPPT_VOC_NOMINAL;
    mppt.k_focv_q8      = MPPT_K_FOCV_Q8;
    mppt.rpm_acc        = 0;
    mppt.rpm_acc_last   = 0;
    mppt.rpm_n          = 0;
    mppt.rpm_ticks      = 0;
    mppt.rpm_input_ref  = 0;
    mppt.rpm_steps      = 0;
    mppt.rpm_phase      = MPPT_RPM_PHASE_SETTLE;
    mppt.rpm_have_last  = 0;
    mppt.rpm_dir        = +1;
    mppt.beta           = 0;
    mppt.beta_mpp       = 0;
    mppt.beta_trim      = 0;
    mppt.beta_ticks     = 0;
    mppt.beta_valid     = 0;
    mppt.vref           = mppt_focv_seed();
    mppt.voc_boot       = 0;
    mppt.i_zero_raw     = 0;
    mppt.i_zero_done    = 0;
    mppt.quiet_v_last   = 0;
    mppt.quiet_ticks    = 0;
    mppt.boot_voc_done  = 0;
    mppt.duty_q12       = 0;
    mppt.duty_q12_prev  = 0;
    mppt.duty           = 0;
    mppt.i_term_q12     = 0;
    mppt.recover_ticks  = 0;
    mppt.duty_applied   = 0;
    mppt.coasting       = 0;
    mppt.coast_events   = 0;
    mppt.collapse_events = 0;

    recover_ok_ticks = 0;
    seed_ticks       = 0;
    reg_ticks        = 0;

    /* Prime the input filters so the first control tick is not a step.
     *
     * Deliberately NOT capturing Voc here. AM32 plays its startup tunes by
     * driving the motor coils, which pulls the bus down and drains Cbus,
     * and the arming tune fires later than this function is even called.
     * A read now would latch a sagged bus and hold the setpoint low for
     * the whole flight. The capture waits for a settled quiet window
     * instead - see the boot-Voc block in mppt_1khz_update(). */
    mppt.v = mppt_read_volts();
    mppt.i = mppt_read_amps();
    mppt.i_q8 = mppt.i << 8;
    for (k = 0; k < 8; k++) {
        mppt.v += (mppt_read_volts() - mppt.v) >> MPPT_V_FILTER_SHIFT;
        mppt.i_q8 += ((mppt_read_amps() << 8) - mppt.i_q8) >> MPPT_I_FILTER_SHIFT;
        mppt.i = mppt.i_q8 >> 8;
    }
    mppt.quiet_v_last = mppt.v;

#if MPPT_TRACKER == MPPT_TRACKER_BETA
    mppt_beta_target();
#endif
}

/* --------------------------------------------------------------------- */
/* RPM feedback: slow perturb-and-observe on the FOCV ratio                */
/* --------------------------------------------------------------------- */
/*
 * Objective is e_com_time, the electrical revolution period in microseconds.
 * SMALLER IS FASTER, so this minimises rather than maximises - easy to get
 * backwards. See the long rationale in mppt.h section 6b.
 *
 * Cycle: perturb k -> wait out the prop's mechanical time constant -> average
 * the period -> compare against the previous average -> keep the direction if
 * it improved, reverse if it did not. Reversing on a tie is deliberate: it
 * makes the tracker dither one step either side of the peak rather than
 * drift off it, and one step is 1/256 of Voc, about 52 mV.
 *
 * Everything here is guarded on the measurement being meaningful. Throttle
 * must not have moved, the motor must be spun up, and the PI must actually
 * have authority - if duty is railed, vref is not setting the operating
 * point and the rpm tells us nothing about k.
 */
#if MPPT_TRACKER == MPPT_TRACKER_RPM
static void mppt_rpm_track(uint8_t regulating)
{
    int32_t ect = (int32_t)e_com_time;
    int32_t diff, deadband;
    uint16_t in_now = input;
    int32_t in_delta;

    /* ---- is this cycle trustworthy at all? ---- */
    in_delta = (int32_t)in_now - (int32_t)mppt.rpm_input_ref;
    if (in_delta < 0) in_delta = -in_delta;

    if (!regulating || ect <= 0 || ect >= MPPT_RPM_ECT_MAX ||
        zero_crosses < MPPT_STARTUP_ZC_COUNT ||
        in_delta > MPPT_RPM_INPUT_TOL) {
        /* Abandon the cycle and resync. Note rpm_have_last is cleared: the
         * stored average was taken under conditions that no longer apply,
         * and comparing across that boundary is how a hill-climber talks
         * itself into walking the wrong way. k itself is kept - it is a
         * learned property of the panel, not of this cycle. */
        mppt.rpm_phase     = MPPT_RPM_PHASE_SETTLE;
        mppt.rpm_ticks     = 0;
        mppt.rpm_acc       = 0;
        mppt.rpm_n         = 0;
        mppt.rpm_have_last = 0;
        mppt.rpm_input_ref = in_now;
        return;
    }

    if (mppt.rpm_phase == MPPT_RPM_PHASE_SETTLE) {
        if (++mppt.rpm_ticks >= MPPT_RPM_SETTLE_TICKS) {
            mppt.rpm_phase = MPPT_RPM_PHASE_MEASURE;
            mppt.rpm_ticks = 0;
            mppt.rpm_acc   = 0;
            mppt.rpm_n     = 0;
        }
        return;
    }

    /* ---- measure ---- */
    mppt.rpm_acc += ect;
    mppt.rpm_n++;
    if (mppt.rpm_n < MPPT_RPM_AVG_TICKS) return;

    /* Compare SUMS, not averages: same sample count both times, so the sum
     * keeps resolution that integer division would throw away. */
    if (mppt.rpm_have_last) {
        deadband = (mppt.rpm_acc / 1000) * MPPT_RPM_DEADBAND_PERMILLE;
        if (deadband < 1) deadband = 1;

        diff = mppt.rpm_acc - mppt.rpm_acc_last;   /* negative = faster */

        if (diff < -deadband) {
            /* Faster than last time - the last move was good, keep going. */
        } else {
            /* Slower, or a tie. Turn around. */
            mppt.rpm_dir = (int8_t)-mppt.rpm_dir;
        }
    }

    mppt.rpm_acc_last  = mppt.rpm_acc;
    mppt.rpm_have_last = 1;

    /* Apply the next perturbation. */
    mppt.k_focv_q8 = clamp32(mppt.k_focv_q8 + (int32_t)mppt.rpm_dir * MPPT_K_STEP_Q8,
                             MPPT_K_MIN_Q8, MPPT_K_MAX_Q8);
    if (mppt.rpm_steps < 0xFFFFu) mppt.rpm_steps++;

    mppt.rpm_phase     = MPPT_RPM_PHASE_SETTLE;
    mppt.rpm_ticks     = 0;
    mppt.rpm_input_ref = in_now;
}
#endif /* MPPT_TRACKER_RPM */

#if MPPT_TRACKER == MPPT_TRACKER_BETA
/* --------------------------------------------------------------------- */
/* Beta method: a pure I-V regulator, no perturbation                      */
/* --------------------------------------------------------------------- */
/*
 *      beta = ln(I/V) - c*V,   c = 1/Vt
 *
 * beta at the MPP is nearly irradiance-invariant, so this drives vref until
 * the measured beta matches that value. dbeta/dV is negative, so a beta
 * ABOVE target means the panel is being held below Vmpp and vref must rise.
 * Getting that sign backwards walks the setpoint straight into the
 * short-circuit region, which is why it is spelled out here.
 *
 * Output is a bounded trim on the fractional-Voc seed rather than an
 * absolute setpoint: FOCV remains the skeleton, so a bad current reading
 * can only pull the operating point MPPT_BETA_TRIM_MAX away from a
 * position that is already sane.
 */
static void mppt_beta_update(void)
{
    int32_t beta, err, dv;

    if (mppt.i < MPPT_BETA_I_MIN || mppt.v <= 0) {
        /* Nothing usable to compute from. Hold the trim - it is the
         * learned part - but stop integrating on garbage. */
        mppt.beta_valid = 0;
        return;
    }

    /* ln(I/V) = ln(I) - ln(V). Both are raw integer units; the unit scaling
     * is a constant offset that lives inside MPPT_BETA_MPP_Q8. */
    beta = mppt_ln_q8((uint32_t)mppt.i) - mppt_ln_q8((uint32_t)mppt.v);

    /* - c*V, with c = 1/Vt. V is in 10 mV and Vt in 10 mV, so V/Vt is the
     * ratio directly; << 8 for Q8. */
    beta -= ((int32_t)mppt.v << 8) / MPPT_BETA_VT;

    mppt.beta       = beta;
    mppt.beta_valid = 1;

    err = beta - mppt.beta_mpp;         /* > 0  =>  below Vmpp  =>  raise */
    dv  = (err * MPPT_BETA_GAIN_Q8) >> 8;

    mppt.beta_trim = clamp32(mppt.beta_trim + dv,
                             -MPPT_BETA_TRIM_MAX, MPPT_BETA_TRIM_MAX);
}
#endif /* MPPT_TRACKER_BETA */

/* --------------------------------------------------------------------- */
/* Inner PI voltage loop (1 kHz)                                          */
/* --------------------------------------------------------------------- */
/*
 * error = v - vref   (NOT vref - v)
 * Increasing duty loads the panel harder, which pulls its voltage DOWN.
 * So a positive error (measured above target => under-loaded) must
 * INCREASE duty. AN1521 flags this same sign reversal.
 */
static void mppt_pi_update(void)
{
    int32_t err, p_q12, i_inc, out_max, out_max_q12;

    /* ---- effective output limit for THIS tick ----
     * A cold panel has no back-EMF to push against, so duty is capped
     * during spin-up. Critically, this cap must be visible to the
     * anti-windup logic below - see the comment there. */
    out_max = 2000;
    if (zero_crosses < MPPT_STARTUP_ZC_COUNT) {
        out_max = MPPT_STARTUP_DUTY_MAX;
    }
    out_max_q12 = out_max << 12;

    err = clamp32(mppt.v - mppt.vref, -MPPT_ERR_CLAMP, MPPT_ERR_CLAMP);

    /* Q12 fixed point. The magic 41 is 4096/100 (error is in 10 mV, gains
     * are per volt). Worst case with the clamp above:
     *   KP: 500 * 1000 * 41 = 2.05e7   -- 100x margin on int32
     *   KI: 500 * 1000 * 41 = 2.05e7   -- then /1000                    */
    p_q12 = (int32_t)MPPT_KP_DUTY_PER_VOLT   * err * 41;
    i_inc = ((int32_t)MPPT_KI_DUTY_PER_VOLT_S * err * 41) / MPPT_TICK_HZ;

    mppt.i_term_q12 += i_inc;

    /* ---- anti-windup, against the ACTIVE limit ----
     * This nearly killed the aircraft in simulation. The clamp used to be
     * a fixed 2000 while the spin-up ceiling was applied afterwards as a
     * separate cap on the output. So during the 130 ms of spin-up the
     * integrator happily wound up to 1867 behind a ceiling of 400 - and
     * the instant zero_crosses passed the threshold and the ceiling
     * lifted, duty stepped 400 -> 1867 in ONE tick. Bus current hit
     * 15.6 A and the bus fell to 4.9 V: a guaranteed brownout with no
     * battery to hold it up.
     *
     * The fix is not a bigger capacitor. It is that an integrator must
     * always be limited by the constraint that is actually binding. */
    mppt.i_term_q12 = clamp32(mppt.i_term_q12,
                              (int32_t)MPPT_I_TERM_MIN << 12,
                              out_max_q12);

    mppt.duty_q12 = clamp32(p_q12 + mppt.i_term_q12, 0, out_max_q12);

    /* Conditional integration: if the output is railed and the error would
     * push it further into the rail, undo this tick's integration. Cheaper
     * than back-calculation, and it matters here because the rail is hit
     * constantly in panel-limited flight. */
    if ((mppt.duty_q12 <= 0       && err < 0) ||
        (mppt.duty_q12 >= out_max_q12 && err > 0)) {
        mppt.i_term_q12 -= i_inc;
    }

    mppt.duty = (uint16_t)(mppt.duty_q12 >> 12);
}

/* --------------------------------------------------------------------- */
/* Main 1 kHz entry point                                                 */
/* --------------------------------------------------------------------- */

void mppt_1khz_update(void)
{
    int32_t v_raw, i_raw;
    uint8_t gated;

    /* ---- 1. sample + low-latency filter ------------------------------
     * Built from the RAW ADC registers, not from actual_current: AM32's
     * getSmoothedCurrent() is a 50-tap boxcar (50 ms window, 25 ms group
     * delay), which is 5x our whole tracker period. */
    v_raw = mppt_read_volts();
    i_raw = mppt_read_amps();

    mppt.v += (v_raw - mppt.v) >> MPPT_V_FILTER_SHIFT;

    /* Current is filtered in Q8, NOT in whole 10 mA units.
     *
     * y += (x - y) >> 2 has a truncation dead zone: it stops moving while
     * 0 <= x - y <= 3, so the output settles anywhere up to 3 units BELOW
     * the input. At a few hundred units of current that is under 1%. On this
     * bench panel, where the whole array is ~11 units, it is up to 27% -
     * measured 8 against a true 11.25 - and any method needing absolute
     * current accuracy is then hopeless. Keeping 8 fractional bits of
     * filter state drops the dead zone to 1/256 of a unit.
     *
     * The voltage filter is left alone: v is ~1100 units, so the same dead
     * zone is under 0.03 V and the PI does not care. */
    mppt.i_q8 += ((i_raw << 8) - mppt.i_q8) >> MPPT_I_FILTER_SHIFT;
    mppt.i     = mppt.i_q8 >> 8;

    /* ---- 1b. boot Voc capture -----------------------------------------
     * Runs before the gating return below, so it works while disarmed -
     * which is exactly when the good measurement is available.
     *
     * Four conditions, and the last is the one that matters: the bus must
     * have STOPPED MOVING. No tone, no current and no duty say nothing is
     * loading the panel right now; only a stationary voltage proves Cbus
     * has actually finished refilling after the tunes. Keeps retrying
     * until it gets a clean window, so arming early only delays it. */
    if (!mppt.boot_voc_done) {
        int32_t dv;

        /* Anchor to the voltage at the START of the window, not to the
         * previous tick. Tick-to-tick differencing looks equivalent and is
         * not: a slow steady ramp whose per-tick step is just under the
         * threshold passes it forever, so the bus could climb by
         * threshold * 300 ms worth - volts - and still be called settled.
         * Anchoring bounds the TOTAL drift across the window instead. */
        if (mppt.quiet_ticks == 0) mppt.quiet_v_last = mppt.v;
        dv = mppt.v - mppt.quiet_v_last;
        if (dv < 0) dv = -dv;

        /* Deliberately NOT testing mppt.i here. That reading depends on the
         * very zero point this window exists to measure, so gating on it
         * would be circular - a board whose CURRENT_OFFSET is wrong enough
         * to matter would never open the window that fixes it. "Motor
         * stopped and no duty applied" already means no motor current, and
         * the stability test catches anything else loading the bus. */
        if (play_tone_flag == 0 && !running && mppt.duty_applied == 0 &&
            dv < MPPT_BOOT_VOC_STABLE_DV) {
            if (++mppt.quiet_ticks >= MPPT_BOOT_VOC_SETTLE_TICKS) {
                if (mppt.v > MPPT_V_COLLAPSE) {
                    /* Panel unloaded and motor stopped, so this ADC reading
                     * IS zero amps. Worth more than any compile-time
                     * constant: 1 mV of offset here is 7.4 mA, 6% of the
                     * bench array. */
                    mppt.i_zero_raw    = (int32_t)ADC_raw_current;
                    mppt.i_zero_done   = 1;
                    mppt.voc_boot      = mppt.v;
                    mppt.voc_est       = mppt.v;
                    mppt.vref          = mppt_focv_seed();
                    mppt.boot_voc_done = 1;
                    /* Re-prime the current filter through the new zero. */
                    mppt.i             = mppt_read_amps();
                    mppt.i_q8          = mppt.i << 8;
#if MPPT_TRACKER == MPPT_TRACKER_BETA
                    /* Real Voc now, so the beta target can stop guessing. */
                    mppt_beta_target();
#endif
                    mppt.i_q8          = mppt.i << 8;
                }
                mppt.quiet_ticks = 0;
            }
        } else {
            mppt.quiet_ticks = 0;
        }
    }

    /* ---- 2. opportunistic Voc estimate --------------------------------
     * When duty and current are both near zero the panel is effectively
     * open-circuit, so the bus voltage IS Voc. Free measurement, no pilot
     * cell and no load-disconnect switch. */
    if (mppt.i < MPPT_VOC_I_TH && mppt.duty_applied < MPPT_VOC_DUTY_TH) {
        /* Panel is unloaded, so v IS Voc.
         *
         * BOTH conditions are required. An earlier version keyed on current
         * alone, reasoning that a duty condition would only ever learn Voc
         * on the ground and leave the estimate pinned high in flight. The
         * reasoning is sound; the cure was worse. On an array whose entire
         * Isc sits near the threshold, "current is low" is true while fully
         * loaded, and voc_est then converges on the LOADED bus voltage.
         * Since the vref clamp band is a fraction of voc_est, the band
         * collapses downward and drags the operating point with it -
         * measured on the 120 mA bench panel, voc_est walked 13.01 V ->
         * 7.75 V over one second and power fell to 73% of available.
         *
         * The in-flight downward path is the periodic sweep instead, which
         * needs no current measurement at all. */
        if (mppt.voc_est == 0) mppt.voc_est = mppt.v;
        else mppt.voc_est += (mppt.v - mppt.voc_est) >> MPPT_VOC_FILTER_SHIFT;
        if (mppt.voc_est < MPPT_V_COLLAPSE) mppt.voc_est = MPPT_VOC_NOMINAL;
    } else if (mppt.v > mppt.voc_est) {
        /* Current is flowing at a voltage above the estimate, which is
         * physically impossible - the estimate was stale-low. Snap it up. */
        mppt.voc_est = mppt.v;
    }

    /* Never believe a Voc the board cannot measure. Above full scale the ADC
     * rails and every threshold derived from voc_est becomes meaningless. */
    if (mppt.voc_est > MPPT_ADC_FULL_SCALE_V) {
        mppt.voc_est = MPPT_ADC_FULL_SCALE_V;
    }

    /* ---- 3. gating ---------------------------------------------------- */
    gated = (!armed) || (!running) || prop_brake_active || (input < 48);
    if (gated) {
        mppt.state      = MPPT_STATE_OFF;
        /* NOT zero. This is a ceiling, not a command - AM32 still decides
         * the actual duty. Leaving a conservative cap in place means the
         * one-tick window between AM32 setting running=1 and the MPPT
         * taking over cannot ramp into a bus collapse on a cold panel. */
        mppt.duty       = MPPT_STARTUP_DUTY_MAX;
        mppt.duty_q12   = 0;
        mppt.duty_q12_prev = 0;
        mppt.i_term_q12 = 0;
        seed_ticks      = 0;
        reg_ticks       = 0;
        return;
    }

    if (mppt.state == MPPT_STATE_OFF) {
        mppt.state      = MPPT_STATE_SEED;
        mppt.vref       = mppt_focv_seed();
        mppt.i_term_q12 = 0;
        seed_ticks      = 0;
    }

    /* ---- 4. SAFETY: absolute floor ------------------------------------
     * Below this the LDO is about to drop out. Unload completely, now.
     * This check is deliberately ahead of everything else. */
    if (mppt.v < MPPT_V_ABSOLUTE_MIN) {
        mppt.state      = MPPT_STATE_RECOVER;
        mppt.duty       = 0;
        mppt.duty_q12   = 0;
        mppt.duty_q12_prev = 0;
        mppt.i_term_q12 = 0;
        mppt.recover_ticks = MPPT_RECOVER_TICKS; /* force the full-unload path */
        return;
    }

    /* ---- 5. SAFETY: collapse detection --------------------------------- */
    if (mppt.state != MPPT_STATE_RECOVER && mppt.v < MPPT_V_COLLAPSE) {
        mppt.state         = MPPT_STATE_RECOVER;
        mppt.recover_ticks = 0;
        recover_ok_ticks   = 0;
        mppt.i_term_q12  = 0;
        mppt.collapse_events++;
    }

    /* ---- 6. state machine ---------------------------------------------- */
    switch (mppt.state) {

    case MPPT_STATE_RECOVER:
        /* Bleed duty off fast. We drive duty directly here and skip the PI
         * entirely - the PI's job is regulation, and this is not a
         * regulation problem, it is a "get off the panel" problem. */
        if (mppt.recover_ticks < 0xFFFFu) mppt.recover_ticks++;

        if (mppt.recover_ticks >= MPPT_RECOVER_TICKS) {
            /* Been stuck too long: the panel genuinely cannot support even
             * this load (cloud, or a stalled prop). Unload completely and
             * let the bus come all the way back, rather than sitting in a
             * collapse/recover limit cycle. */
            mppt.duty     = 0;
            mppt.duty_q12 = 0;
        } else {
            mppt.duty_q12 = (mppt.duty_q12 * MPPT_RECOVER_DECAY_Q8) >> 8;
            mppt.duty     = (uint16_t)(mppt.duty_q12 >> 12);
        }

        mppt.i_term_q12 = 0;
        mppt.vref       = mppt_focv_seed();

        if (mppt.v > (MPPT_V_COLLAPSE + MPPT_V_COLLAPSE_HYST)) {
            if (++recover_ok_ticks >= MPPT_RECOVER_OK_TICKS) {
                mppt.state         = MPPT_STATE_SEED;
                mppt.recover_ticks = 0;
                seed_ticks         = 0;
                /* Restart the integrator from the duty we survived at, so
                 * the PI does not have to climb from zero again. */
                mppt.i_term_q12 = mppt.duty_q12;
            }
        } else {
            recover_ok_ticks = 0;
        }
        break;


    case MPPT_STATE_SEED:
        /* Hold the fractional-Voc setpoint for a few ticks and let the inner
         * PI settle before beta starts trimming around it. */
        mppt.vref = mppt_focv_seed();
        mppt_pi_update();
        if (++seed_ticks >= MPPT_SEED_TICKS) {
            mppt.state = MPPT_STATE_TRACK;
        }
        break;

    case MPPT_STATE_TRACK:
        /* Does the PI have authority over the operating point? If duty is
         * railed, vref is not setting the operating point and nothing
         * measured here says anything about where the MPP is. */
        if (mppt.duty_q12 > 0 && mppt.duty_q12 < ((int32_t)2000 << 12)) {
            if (reg_ticks < 255) reg_ticks++;
        } else {
            reg_ticks = 0;
        }

#if MPPT_TRACKER == MPPT_TRACKER_BETA
        /* Pure I-V regulation, on a slow sub-multiple of the control tick.
         * Nothing here perturbs, so there is nothing for ADC noise to act
         * on and no steady-state dither. The loop closes around the panel
         * curve with the inner PI inside it. */
        if (++mppt.beta_ticks >= MPPT_BETA_PERIOD_TICKS) {
            mppt.beta_ticks = 0;
            if (reg_ticks >= MPPT_REG_TICKS) mppt_beta_update();
            else                             mppt.beta_valid = 0;
        }
#else
        /* Fallback for boards with no usable current sense: perturb k and
         * keep whichever direction raises rpm. Dithers by construction -
         * that is the price of needing no current measurement at all. */
        mppt_rpm_track((uint8_t)(reg_ticks >= MPPT_REG_TICKS));
#endif

        mppt.vref = mppt_focv_seed();     /* seed + beta's learned trim */
        mppt.p    = mppt.v * mppt.i;      /* telemetry only */
        mppt_pi_update();
        break;

    case MPPT_STATE_OFF:
    default:
        mppt.duty     = 0;
        mppt.duty_q12 = 0;
        break;
    }

    /* ---- 7. duty rise-rate limit --------------------------------------
     * The only thing pacing how fast load is re-applied - AM32's ramp now
     * tracks its own intent, not ours (see mppt.h). Reductions are never
     * limited: shedding load is the safety direction and must stay
     * instant. */
    {
        int32_t rise_max = (int32_t)MPPT_DUTY_RISE_MAX << 12;

        if (mppt.duty_q12 > mppt.duty_q12_prev + rise_max) {
            mppt.duty_q12 = mppt.duty_q12_prev + rise_max;

            /* Anti-windup against the slew limit. Exactly the lesson the
             * spin-up ceiling taught: an integrator must be bounded by
             * whichever constraint is actually binding, or it winds up
             * behind the limit and steps the moment the limit lifts. */
            if (mppt.i_term_q12 > mppt.duty_q12) {
                mppt.i_term_q12 = mppt.duty_q12;
            }
        }
        mppt.duty_q12_prev = mppt.duty_q12;
        mppt.duty          = (uint16_t)(mppt.duty_q12 >> 12);
    }
}

/* --------------------------------------------------------------------- */
/* Duty application (called at the full tenKhzRoutine rate)               */
/* --------------------------------------------------------------------- */
/*
 * Asymmetric on purpose:
 *   - REDUCTIONS are applied immediately, bypassing AM32's slew limiter.
 *     A collapse has to be caught in one tick; a ramp rate of 2 duty units
 *     per step would take ~100 ms to shed load, which is far too slow when
 *     the MCU's own supply is on that bus.
 *   - INCREASES are left to AM32's existing ramp, so we keep its desync
 *     and startup protection intact.
 */
void mppt_apply_duty(volatile uint16_t *duty_cycle)
{
    /* Last line of defence, and it runs unconditionally - no state check,
     * no arming check. If the bus is this low the MCU's own supply is
     * about to drop out, and nothing else matters.
     *
     * This is deliberately outside the state machine. An earlier version
     * returned early when state == OFF, which left AM32's startup ramp
     * completely uncapped: in simulation the bus went to 6.8 V during
     * spin-up on a cold panel, which on real hardware means the ESC
     * brown-out resets before the motor has even spun up. */
    /* Read the ADC register directly rather than mppt.v. mppt.v is the
     * IIR-filtered value, and that filter's ~3 ms group delay defeats the
     * guard: in simulation it fired late enough that the bus reached
     * 9.6 V against an 11.0 V threshold during cold-panel spin-up. The
     * filter exists to give the TRACKER a clean signal; the emergency path
     * wants the freshest number available, noise and all. */
    if (mppt_read_volts() < MPPT_V_ABSOLUTE_MIN) {
        *duty_cycle        = 0;
        mppt.duty_applied  = 0;
        mppt.duty          = 0;
        mppt.duty_q12      = 0;
        mppt.duty_q12_prev = 0;
        mppt.i_term_q12    = 0;
        /* Coast, and actually float the FETs on this same tick. Setting the
         * flag alone is not enough - this branch returns before the shared
         * allOff() below, so the emergency path would have zeroed duty and
         * then left the winding shorted through the low side. That is the
         * exact braking this whole mechanism exists to prevent, on the one
         * path where it matters most. */
        mppt_coast(1);
        allOff();
        return;
    }

    if (*duty_cycle > mppt.duty) {
        *duty_cycle = mppt.duty;          /* immediate reduction */
    }

    /* ---- coast rather than brake when UNLOADING -----------------------
     * Two conditions, and the state one is not optional. Gating on low duty
     * alone looks equivalent and is not: duty legitimately passes through
     * zero on the way up during every spin-up, so a duty-only test floats
     * the FETs exactly when the motor is trying to start and it never does.
     * Measured cost of getting this wrong on the bench panel: 97.4% -> 95.8%
     * and a visibly disturbed start.
     *
     * So coast only when the MPPT is deliberately shedding load - RECOVER,
     * or the absolute-minimum cut above - and the demand has fallen far
     * enough that there is nothing left to give away. Above roughly 2% duty
     * the complementary FETs are still doing useful synchronous
     * rectification and feeding the bus; it is only at genuinely zero duty
     * that the winding is a dead short and the rotor's energy goes to heat
     * for no return. */
    if (mppt.coasting) {
        if (mppt.state != MPPT_STATE_RECOVER ||
            mppt.duty >= MPPT_COAST_EXIT_DUTY) {
            mppt_coast(0);
        }
    } else if (mppt.state == MPPT_STATE_RECOVER &&
               mppt.duty < MPPT_COAST_DUTY_TH) {
        mppt_coast(1);
    }

    if (mppt.coasting) {
        *duty_cycle = 0;
        /* Re-assert every tick: comStep() restores the driven pin modes at
         * each commutation and would undo this within one electrical step. */
        allOff();
    }

    mppt.duty_applied = *duty_cycle;
}

#else  /* !USE_MPPT */

/* Nothing at all is emitted for this target. The call sites in main.c are
 * preprocessed to (void)0 by the macros at the bottom of mppt.h, so no
 * stub functions are needed and no flash or RAM is used. ISO C wants at
 * least one declaration in a translation unit. */
typedef int mppt_translation_unit_not_empty_t;

#endif /* USE_MPPT */
