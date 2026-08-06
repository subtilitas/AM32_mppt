/*
 * mppt.c - Fast MPPT tracker for AM32 ESC firmware
 * See mppt.h for architecture, references and configuration.
 *
 * ---------------------------------------------------------------------
 * ALGORITHM SELECTION - why this one
 * ---------------------------------------------------------------------
 * Incremental conductance in dP/dV form, with Hassani's counter-based
 * two-step escalation and a fractional-Voc re-seed.
 *
 *  - dP/dV, not dI/dV. AN1521 implemented both on real hardware and found
 *    the dI/dV form's sign flipped erratically on filtered-but-real ADC
 *    noise, which "confused the algorithm and caused it to get stuck".
 *    The dP/dV form worked. We are noisier than they were (a commutating
 *    BLDC on the same shunt), so this is not a close call.
 *
 *  - Incremental conductance, not plain P&O. INC can LOCK at the MPP
 *    (dP/dV ~ 0 -> hold), while P&O dithers forever by construction and
 *    gives back 1-2% of available power. INC also recovers correctly when
 *    the operating point moved for a reason other than our perturbation,
 *    which on a banking wing is most of the time.
 *
 *  - Two-step escalation, not a magnitude-proportional step. The step is
 *    chosen by a COUNTER of consecutive same-direction moves, so it is
 *    scale-free. A step calibrated in watts is correct at exactly one
 *    irradiance; with a 10:1 swing it is either glacial or unstable at
 *    the ends. The counter behaves identically at 5 W and 50 W.
 *
 *  - Fractional Voc for re-seeding, not for tracking. Pure FOCV is only
 *    ~95-98% accurate, so it is a bad tracker. But it converges in ONE
 *    sample, which makes it the right tool for the two cases hill-climbing
 *    handles worst: cold start, and a step change too large to walk to.
 *    Critically, on this airframe Voc is free to measure - at duty 0 the
 *    bus voltage IS Voc, no pilot cell or load-disconnect switch needed.
 *
 * ---------------------------------------------------------------------
 * PI TUNING RECIPE (do this on the bench, prop off, panel or PSU+resistor)
 * ---------------------------------------------------------------------
 * 1. Set MPPT_KI_DUTY_PER_VOLT_S = 0. Set KP = 4.
 * 2. Command a small vref step (e.g. 1 V) and log v. Raise KP until you
 *    see ~10% overshoot, then halve it. That is your KP.
 * 3. Raise KI until the steady-state error closes in ~5-10 ms without
 *    adding overshoot. Rule of thumb: KI ~ 20 * KP.
 * 4. Measure the 2% settling time of that step response. Set
 *    MPPT_PERIOD_TICKS >= settling_time_ms. If settling is 5 ms, the
 *    tracker runs at 200 Hz. Going faster than settling means you are
 *    measuring your own transient, and the tracker will wander.
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
/* Ticks spent in SEED before handing over to the hill-climber. */
static uint8_t  seed_ticks;
/* Consecutive ticks the inner PI has been holding v at vref. */
static uint8_t  reg_ticks;
/* Last value of mppt.hill_climb, to detect the FOCV <-> climb transition. */
static uint8_t  hc_last;

/* --------------------------------------------------------------------- */
/* Small helpers                                                          */
/* --------------------------------------------------------------------- */

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

/* Enter or leave the coasting (all FETs floated) state. See the long note in
 * mppt.h - on this hardware duty 0 is a brake, not a coast. */
static inline void mppt_coast(uint8_t on)
{
#if MPPT_COAST_ENABLE
    if (on && !mppt.coasting) {
        if (mppt.coast_events < 0xFFFFu) mppt.coast_events++;
    }
    mppt.coasting = on;
#else
    (void)on;
    mppt.coasting = 0;
#endif
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
static inline int32_t mppt_read_amps(void)
{
    int32_t raw = (int32_t)ADC_raw_current;
    int32_t x   = (raw * MPPT_ADC_VREF_MV) / MPPT_I_SCALE_DIV;      /* <= 3.3e5 */
    x -= ((int32_t)CURRENT_OFFSET * 100);
    x /= (int32_t)MPPT_MILLIVOLT_PER_AMP;                           /* 10 mA units */
    return (x < 0) ? 0 : x;
}

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

/* The fractional-Voc setpoint, using the ADAPTED ratio rather than the
 * compile-time one. mppt.k_focv_q8 starts at MPPT_K_FOCV_Q8 and is moved by
 * the rpm tracker; everything that seeds or re-seeds goes through here, so
 * the learned correction is never thrown away by a transient. */
static inline int32_t mppt_focv_seed(void)
{
    return mppt_clamp_vref((mppt.voc_est * mppt.k_focv_q8) >> 8);
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
    mppt.vref           = mppt_focv_seed();
    mppt.voc_boot       = 0;
    mppt.quiet_v_last   = 0;
    mppt.quiet_ticks    = 0;
    mppt.boot_voc_done  = 0;
    mppt.rpm_acc        = 0;
    mppt.rpm_acc_last   = 0;
    mppt.rpm_n          = 0;
    mppt.rpm_ticks      = 0;
    mppt.rpm_input_ref  = 0;
    mppt.rpm_steps      = 0;
    mppt.rpm_phase      = MPPT_RPM_PHASE_SETTLE;
    mppt.rpm_have_last  = 0;
    mppt.rpm_dir        = +1;
    mppt.duty_q12       = 0;
    mppt.duty_q12_prev  = 0;
    mppt.duty           = 0;
    mppt.i_term_q12     = 0;
    mppt.p_last         = 0;
    mppt.v_last         = 0;
    mppt.i_last         = 0;
    mppt.step           = MPPT_STEP_MIN;
    mppt.dp_drift       = 0;
    mppt.phase          = MPPT_PHASE_DRIFT;
    mppt.railed         = 0;
    mppt.dir            = +1;
    mppt.same_dir_count = 0;
    mppt.v_acc          = 0;
    mppt.i_acc          = 0;
    mppt.acc_n          = 0;
    mppt.tick           = 0;
    mppt.recover_ticks  = 0;
    mppt.duty_applied   = 0;
    mppt.since_sweep    = 0;
    mppt.sweep_ticks    = 0;
    mppt.sweep_v_last   = 0;
    mppt.sweep_i_term   = 0;
    mppt.sweep_settle_n = 0;
    mppt.hill_climb     = 0;
    mppt.coasting       = 0;
    mppt.coast_events   = 0;
    mppt.collapse_events = 0;
    mppt.reseed_events   = 0;
    mppt.voc_sweeps      = 0;

    recover_ok_ticks = 0;
    seed_ticks       = 0;
    reg_ticks        = 0;
    hc_last          = 0;

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
    for (k = 0; k < 8; k++) {
        mppt.v += (mppt_read_volts() - mppt.v) >> MPPT_V_FILTER_SHIFT;
        mppt.i += (mppt_read_amps()  - mppt.i) >> MPPT_I_FILTER_SHIFT;
    }
    mppt.quiet_v_last = mppt.v;
}

/* --------------------------------------------------------------------- */
/* Tracker: dP/dV incremental conductance + Hassani two-step escalation    */
/* --------------------------------------------------------------------- */
/*
 * Called once every MPPT_PERIOD_TICKS ms, alternating between two slots:
 *
 *   DRIFT slot     vref is held still, so whatever the power did over this
 *                  slot was NOT caused by us. Record it, then apply the
 *                  perturbation decided at the end of the last response.
 *   RESPONSE slot  Measure the power change, subtract the drift recorded
 *                  above, and what remains is our perturbation's true
 *                  effect. Decide the next direction from that.
 *
 * So a perturbation lands every 2*MPPT_PERIOD_TICKS ms. At the default 5,
 * that is a 100 Hz perturbation rate on a 1 kHz sample stream.
 *
 * WHY THE ALTERNATION IS WORTH HALVING THE RATE (Sera's dP-P&O):
 * On an aircraft the power is almost never stationary - rolling out of a
 * bank raises irradiance continuously, and the propeller's own inertia
 * makes power drift for a second after any change. A plain hill-climber
 * sees power rise after every step, concludes every step was good,
 * escalates, and walks the reference clean off the peak into the
 * open-circuit region where the motor regenerates into the panel.
 * Measured in simulation before this was added: 50-67% tracking
 * efficiency through a roll-out, and -5 W drawn from a panel with 5.9 W
 * available. After: >98%.
 */

static void mppt_tracker_step(int32_t v_avg, int32_t i_avg, uint8_t usable)
{
    int32_t p, dp, dv, di, dp_deadband, i_thresh, vref_before;
    int8_t  dir;

    p  = v_avg * i_avg;             /* 0.1 mW units. 60V*100A -> 6e7, safe */
    dp = p     - mppt.p_last;
    dv = v_avg - mppt.v_last;
    di = i_avg - mppt.i_last;

    mppt.p = p;

    /* ---- is the measurement usable at all? ----
     * Only one condition, deliberately: the PI must have authority. If duty
     * is railed at 0 or 2000 the panel is not being held at vref by us, so
     * nothing we measure says anything about which side of the MPP we are
     * on. Resync the phase and bank the samples.
     *
     * An earlier version also gated on |v - vref| being small. That looked
     * reasonable and was actively harmful: it fired constantly (only 55 of
     * an expected 340 decisions survived in a 3.4 s run), desynchronised
     * the drift/response alternation, and made tracking efficiency swing
     * between 77% and 98% on tiny parameter changes. One condition, and
     * let the drift subtraction do the work it is there for. */
    if (!usable) {
        mppt.phase          = MPPT_PHASE_DRIFT;
        mppt.same_dir_count = 0;
        mppt.step           = MPPT_STEP_MIN;
        goto store;
    }

    if (mppt.phase == MPPT_PHASE_DRIFT) {
        mppt.dp_drift = dp;
        mppt.phase    = MPPT_PHASE_RESPONSE;

        /* Apply the perturbation and remember whether vref actually moved.
         * If the sanity clamp swallowed it we are pinned against a rail,
         * and the response slot must not read "no change" as "we are at
         * the MPP" - that latches the tracker against the rail for the
         * rest of the flight. */
        vref_before   = mppt.vref;
        mppt.vref     = mppt_clamp_vref(mppt.vref + (int32_t)mppt.dir * mppt.step);
        mppt.railed   = (uint8_t)(mppt.vref == vref_before);
        goto store;
    }

    /* ================= RESPONSE SLOT ================= */
    mppt.phase = MPPT_PHASE_DRIFT;

    /* Pinned against a clamp: the only useful information is "turn around". */
    if (mppt.railed) {
        mppt.dir            = (int8_t)-mppt.dir;
        mppt.step           = MPPT_STEP_MIN;
        mppt.same_dir_count = 0;
        goto store;
    }

    dp -= mppt.dp_drift;            /* strip the manoeuvre / inertia drift */

    /* ---- irradiance-transient detection, RELATIVE to present current ----
     * Absolute amp thresholds break at the extremes: 0.5 A is noise at full
     * sun and is the whole output in deep shade. Percentages hold up. */
    i_thresh = mppt.i_last;
    if (i_thresh < 10) i_thresh = 10;          /* floor: 0.10 A */

    if (iabs32(di) * 100 > (int32_t)MPPT_I_RESEED_PCT * i_thresh) {
        /* Cloud edge or similar - far too big to walk to. Jump straight to
         * the fractional-Voc estimate and start hill-climbing from there. */
        mppt.vref           = mppt_focv_seed();
        mppt.step           = MPPT_STEP_MIN;
        mppt.same_dir_count = 0;
        mppt.reseed_events++;
        goto store;
    }

    if (iabs32(di) * 100 > (int32_t)MPPT_I_TRANSIENT_PCT * i_thresh) {
        /* Moderate transient: the drift was not linear across the two
         * slots, so the subtraction did not fully clean dp. Hold course
         * rather than reverse on bad information. */
        goto store;
    }

    /* ---- did the loop actually respond? ----
     * If the commanded step produced no measurable voltage change the inner
     * PI has not settled yet. Skip the decision - do NOT read it as
     * "dP/dV = 0, we must be at the peak". */
    if (iabs32(dv) < MPPT_DV_DEADBAND) {
        goto store;
    }

    /* ---- MPP lock ----
     * Deadband is relative (per-mille of present power) with an absolute
     * floor, so it stays meaningful across a 10:1 irradiance swing.
     * Per TI TIDA-010042, holding within +-2.5% of Vmpp already returns
     * >99.5% of available power, so locking early costs almost nothing and
     * buys a lot of stability. */
    dp_deadband = (p / 1000) * MPPT_DP_DEADBAND_PERMILLE;
    if (dp_deadband < MPPT_DP_DEADBAND_MIN) dp_deadband = MPPT_DP_DEADBAND_MIN;

    /* ---- quantisation floor ----
     * One ADC count of current moves dp by v * I_LSB whether or not
     * anything physical happened. A deadband below that makes the tracker
     * reverse on quantisation and hunt forever. On the 120 mA bench panel
     * one count was 6.2x the relative deadband and tracking sat at 77.6%;
     * this floor alone took it to 90.7%. Computed from the live voltage so
     * it stays right as the operating point moves. */
    {
        int32_t dp_quant = (v_avg * MPPT_I_LSB_Q8 * MPPT_DP_QUANT_MULT) >> 8;
        if (dp_deadband < dp_quant) dp_deadband = dp_quant;
    }

    if (iabs32(dp) < dp_deadband) {
        mppt.step           = MPPT_STEP_MIN;
        mppt.same_dir_count = 0;
        goto store;
    }

    /* ---- direction: sign(dP/dV), computed without a division ----
     * dP/dV > 0  =>  we are LEFT of the MPP  =>  raise vref (unload).
     * dP/dV < 0  =>  we are RIGHT of the MPP =>  lower vref (load more). */
    dir = ((dp > 0) == (dv > 0)) ? (int8_t)+1 : (int8_t)-1;

    /* ---- Hassani two-step switcher ----
     * MIN step while hunting; escalate to MAX after ESCALATE_COUNT moves in
     * the same direction (we are clearly far from the peak); drop straight
     * back to MIN the moment we overshoot and reverse. Counter-based, so it
     * behaves identically at 5 W and at 50 W. */
    if (dir == mppt.dir) {
        if (mppt.same_dir_count < 255) mppt.same_dir_count++;
        if (mppt.same_dir_count >= MPPT_ESCALATE_COUNT) {
            mppt.step = MPPT_STEP_MAX;
        }
    } else {
        mppt.step           = MPPT_STEP_MIN;
        mppt.same_dir_count = 0;
    }

    /* Direction only. The step itself is applied at the start of the next
     * drift slot, so that slot measures pure drift. */
    mppt.dir = dir;

store:
    mppt.p_last = p;
    mppt.v_last = v_avg;
    mppt.i_last = i_avg;
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
#if MPPT_RPM_TRACK
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
#endif /* MPPT_RPM_TRACK */

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
    mppt.i += (i_raw - mppt.i) >> MPPT_I_FILTER_SHIFT;

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

        if (play_tone_flag == 0 && !running && mppt.duty_applied == 0 &&
            mppt.i < MPPT_VOC_I_TH && dv < MPPT_BOOT_VOC_STABLE_DV) {
            if (++mppt.quiet_ticks >= MPPT_BOOT_VOC_SETTLE_TICKS) {
                if (mppt.v > MPPT_V_COLLAPSE) {
                    mppt.voc_boot      = mppt.v;
                    mppt.voc_est       = mppt.v;
                    mppt.vref          = mppt_focv_seed();
                    mppt.boot_voc_done = 1;
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
        mppt.tick       = 0;
        mppt.acc_n      = 0;
        mppt.v_acc      = 0;
        mppt.i_acc      = 0;
        seed_ticks      = 0;
        reg_ticks       = 0;
        mppt.since_sweep    = 0;
        mppt.sweep_ticks    = 0;
        mppt.sweep_settle_n = 0;
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

#if MPPT_VOC_SWEEP_INTERVAL_TICKS > 0
    case MPPT_STATE_VOC:
        /* ---- periodic open-circuit measurement ----
         * Duty 0, wait for the bus to stop rising, take v as Voc. This is
         * the only Voc source independent of the current sensor, which is
         * what makes it usable on an array spanning 20 ADC counts.
         *
         * Inherently safe: unloading the panel can only raise the bus. The
         * absolute-minimum guard above still runs first regardless.
         *
         * Ends on "stopped rising" rather than a fixed delay, so it costs
         * only as long as the panel needs to charge Cbus - sub-millisecond
         * on a healthy array, ~12 ms on the 120 mA bench panel. */
        mppt.duty       = 0;
        mppt.duty_q12   = 0;
        mppt.i_term_q12 = 0;
        mppt.sweep_ticks++;

        if (iabs32(mppt.v - mppt.sweep_v_last) < MPPT_VOC_SWEEP_SETTLE_DV) {
            mppt.sweep_settle_n++;
        } else {
            mppt.sweep_settle_n = 0;
        }
        mppt.sweep_v_last = mppt.v;

        if (mppt.sweep_settle_n >= MPPT_VOC_SWEEP_SETTLE_TICKS ||
            mppt.sweep_ticks    >= MPPT_VOC_SWEEP_MAX_TICKS) {
            /* Direct assignment, not the IIR: this is a real measurement,
             * not an inference. */
            if (mppt.v > MPPT_V_COLLAPSE) mppt.voc_est = mppt.v;
            mppt.voc_sweeps++;
            mppt.since_sweep = 0;
            mppt.state       = MPPT_STATE_SEED;
            seed_ticks       = 0;
            /* Hand the integrator back what it had, so the PI does not have
             * to climb from zero and AM32's ramp is the only thing limiting
             * how fast thrust returns. */
            mppt.i_term_q12  = mppt.sweep_i_term;
            mppt.vref        = mppt_focv_seed();
        }
        break;
#endif /* MPPT_VOC_SWEEP_INTERVAL_TICKS > 0 */

    case MPPT_STATE_SEED:
        /* Hold the fractional-Voc setpoint for a few ticks and let the PI
         * settle before handing over to the hill-climber. */
        mppt.vref = mppt_focv_seed();
        mppt_pi_update();
        if (++seed_ticks >= (MPPT_PERIOD_TICKS * 2)) {
            mppt.state          = MPPT_STATE_TRACK;
            mppt.v_last         = mppt.v;
            mppt.i_last         = mppt.i;
            mppt.p_last         = mppt.v * mppt.i;
            mppt.step           = MPPT_STEP_MIN;
            mppt.same_dir_count = 0;
            mppt.dir            = -1;   /* first move: load up slightly */
            mppt.phase          = MPPT_PHASE_DRIFT;
            mppt.railed         = 0;
            mppt.dp_drift       = 0;
            mppt.tick           = 0;
            mppt.acc_n          = 0;
            mppt.v_acc          = 0;
            mppt.i_acc          = 0;
        }
        break;

    case MPPT_STATE_TRACK:
        mppt.tick++;
        if (mppt.since_sweep < 0xFFFFu) mppt.since_sweep++;

        /* ---- time for a Voc sweep? ---- */
#if MPPT_VOC_SWEEP_INTERVAL_TICKS > 0
        if (mppt.since_sweep >= MPPT_VOC_SWEEP_INTERVAL_TICKS) {
            mppt.state          = MPPT_STATE_VOC;
            mppt.sweep_ticks    = 0;
            mppt.sweep_settle_n = 0;
            mppt.sweep_v_last   = mppt.v;
            mppt.sweep_i_term   = mppt.i_term_q12;
            break;
        }
#endif

        /* ---- is the current channel worth listening to? ----
         * dP/dV is inferred from current. Below MPPT_TRACK_I_MIN one ADC
         * count is a large fraction of the reading and dp is mostly
         * quantisation, so hill-climbing does actively worse than not
         * trying: on the 120 mA bench panel every hill-climb tuning topped
         * out at 86.6% while plain fractional-Voc held 96.8% dead flat.
         * Fall back to FOCV and let the inner PI do the work. */
#if MPPT_MODE == MPPT_MODE_FOCV
        mppt.hill_climb = 0;
#elif MPPT_MODE == MPPT_MODE_HILLCLIMB
        mppt.hill_climb = 1;
#else
        mppt.hill_climb = (uint8_t)(mppt.i >= MPPT_TRACK_I_MIN);
#endif

        /* Mode flipped? The hill-climber's p_last/v_last/i_last are from
         * before the FOCV interval and comparing against them would make
         * the first decision pure noise. Resync and start clean. */
        if (mppt.hill_climb != hc_last) {
            hc_last             = mppt.hill_climb;
            mppt.v_last         = mppt.v;
            mppt.i_last         = mppt.i;
            mppt.p_last         = mppt.v * mppt.i;
            mppt.step           = MPPT_STEP_MIN;
            mppt.same_dir_count = 0;
            mppt.dir            = -1;
            mppt.phase          = MPPT_PHASE_DRIFT;
            mppt.railed         = 0;
            mppt.dp_drift       = 0;
            mppt.tick           = 0;
            mppt.acc_n          = 0;
            mppt.v_acc          = 0;
            mppt.i_acc          = 0;
        }

        /* Does the PI have authority over the operating point?
         * Must be evaluated BEFORE the fractional-Voc branch below, not
         * after: that branch breaks out early, so leaving this where it
         * used to sit left reg_ticks pinned at 0 in FOCV mode and the rpm
         * tracker - which gates on it - would never have run at all. */
        if (mppt.duty_q12 > 0 && mppt.duty_q12 < ((int32_t)2000 << 12)) {
            if (reg_ticks < 255) reg_ticks++;
        } else {
            reg_ticks = 0;
        }

        if (!mppt.hill_climb) {
            /* Fractional-Voc: vref is k*Voc, refreshed each tick so it
             * follows both Voc and the rpm tracker's correction to k. The
             * PI regulates to it fast; only k moves slowly. Nothing here
             * perturbs on the fast path, so ADC noise has nothing to act
             * on - the perturbation lives in the rpm tracker at ~1.7 Hz. */
#if MPPT_RPM_TRACK
            mppt_rpm_track((uint8_t)(reg_ticks >= MPPT_REG_TICKS));
#endif
            mppt.vref  = mppt_focv_seed();
            mppt.tick  = 0;
            mppt.acc_n = 0;
            mppt.v_acc = 0;
            mppt.i_acc = 0;
            mppt.p     = mppt.v * mppt.i;
            mppt_pi_update();
            break;
        }

        /* Average only the LAST few samples before a decision. The samples
         * right after a perturbation are the inner loop's settling
         * transient and must not be fed to the tracker (AN1521). */
        if (mppt.tick > (MPPT_PERIOD_TICKS - MPPT_AVG_TICKS)) {
            mppt.v_acc += mppt.v;
            mppt.i_acc += mppt.i;
            mppt.acc_n++;
        }

        if (mppt.tick >= MPPT_PERIOD_TICKS) {
            /* Divide by the CONSTANT, not by acc_n. Every other division in
             * this module is by a compile-time constant, which the compiler
             * strength-reduces to a multiply-high plus shift; a division by
             * a runtime variable would be the only real SDIV in the whole
             * hot path. Requiring a full accumulator also throws away
             * decisions built on partial data. */
            if (mppt.acc_n == MPPT_AVG_TICKS) {
                mppt_tracker_step(mppt.v_acc / MPPT_AVG_TICKS,
                                  mppt.i_acc / MPPT_AVG_TICKS,
                                  (uint8_t)(reg_ticks >= MPPT_REG_TICKS));
            }
            mppt.tick  = 0;
            mppt.acc_n = 0;
            mppt.v_acc = 0;
            mppt.i_acc = 0;
        }

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
     * spin-up on a cold panel, which on real hardware is a brownout and a
     * lost aircraft. */
    /* Read the ADC register directly rather than mppt.v. mppt.v is the
     * IIR-filtered value, and that filter's ~3 ms group delay is fatal
     * here: in simulation the guard fired late enough that the bus reached
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
