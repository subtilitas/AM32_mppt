/*
 * Directed tests for the parts of mppt.c the plant model cannot reach.
 *
 * The full sim in sim.c is a closed loop, which is exactly what makes it a
 * poor tool for some of this. It cannot reliably drive the module into
 * RECOVER - a spinning motor regenerates through the complementary FETs and
 * props the bus up, and if Cbus is made small enough to actually collapse it
 * does so in well under one 1 ms control tick, so the ADC never sees it. Nor
 * can it exercise the boot-Voc window, because it arms almost immediately.
 * Rather than contrive scenarios, this drives the ADC inputs directly and
 * asserts on the state machine.
 *
 * Covers:
 *   - coast / unload path (duty 0 is a BRAKE on this hardware, not a coast)

 *     convention, or the rpm fallback and its guards
 *   - current-sense zero self-calibration
 *
 * Run with:  ./run.sh test
 */
#include <stdio.h>
#include <stdint.h>
#include "targets.h"
#include "mppt.h"

extern uint16_t ADC_raw_volts, ADC_raw_current, VOLTAGE_DIVIDER, input;
extern volatile uint32_t zero_crosses;
extern volatile char armed;
extern uint8_t running;
extern char prop_brake_active;
extern int h_all_off;
extern char play_tone_flag;
extern int e_com_time;

static int fails = 0;

static void ok(const char *what, int cond)
{
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}

/* exact inverses of AM32's scaling, with this board's constants */
static uint16_t adc_v(double volts)
{
    double r = volts * 100.0 * 4095.0 / (3300.0 * (TARGET_VOLTAGE_DIVIDER / 100.0));
    if (r < 0) r = 0;
    if (r > 4095) r = 4095;
    return (uint16_t)(r + 0.5);
}
static uint16_t adc_i(double amps)
{
    double r = (amps * 100.0 * MPPT_MILLIVOLT_PER_AMP + CURRENT_OFFSET * 100.0)
               * 41.0 / 3300.0;
    if (r < 0) r = 0;
    if (r > 4095) r = 4095;
    return (uint16_t)(r + 0.5);
}

/* one control tick plus the 20 kHz duty application that follows it */
static uint16_t step(double bus_v, double bus_i, uint16_t am32_duty)
{
    uint16_t d = am32_duty;
    ADC_raw_volts   = adc_v(bus_v);
    ADC_raw_current = adc_i(bus_i);
    mppt_1khz_update();
    h_all_off = 0;
    mppt_apply_duty(&d);
    return d;
}

int main(void)
{
    int i, all_off_ticks;

    VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER;
    armed = 1; running = 1; input = 2047; prop_brake_active = 0;
    zero_crosses = 5000;                      /* well past spin-up */
    ADC_raw_volts = adc_v(13.4); ADC_raw_current = adc_i(0.0);
    mppt_init();

    printf("coast path\n");

    /* 1. healthy bus, normal running */
    for (i = 0; i < 400; i++) step(10.5, 0.11, 1200);
    ok("healthy running does not coast", !mppt.coasting);
    ok("healthy running drives the FETs", h_all_off == 0);

    /* 2. spin-up: duty legitimately passes through zero */
    mppt_init();
    zero_crosses = 0;
    armed = 1; running = 1; input = 2047;
    all_off_ticks = 0;
    for (i = 0; i < 200; i++) {
        step(13.0, 0.01, (uint16_t)(i * 5));  /* AM32 ramping up from 0 */
        if (h_all_off) all_off_ticks++;
    }
    ok("spin-up never coasts despite duty passing through 0",
       all_off_ticks == 0 && !mppt.coasting);

    /* 3. bus collapse -> RECOVER -> coast once duty has decayed */
    mppt_init();
    zero_crosses = 5000;
    armed = 1; running = 1; input = 2047;
    for (i = 0; i < 400; i++) step(10.5, 0.11, 1200);
    ok("settled before the collapse", mppt.state == MPPT_STATE_TRACK);

    for (i = 0; i < 200; i++) step(6.5, 0.02, 1200);   /* below V_COLLAPSE */
    ok("collapse entered RECOVER", mppt.state == MPPT_STATE_RECOVER);
    ok("collapse counted", mppt.collapse_events > 0);
    ok("RECOVER coasts once duty decays", mppt.coasting);
    ok("coast counted", mppt.coast_events > 0);

    /* 4. allOff() re-asserted every tick, not just on entry */
    all_off_ticks = 0;
    for (i = 0; i < 50; i++) { step(6.5, 0.02, 1200); if (h_all_off) all_off_ticks++; }
    ok("allOff() re-asserted on every coasting tick", all_off_ticks == 50);

    /* and the duty handed to the PWM is zero throughout */
    ok("duty forced to 0 while coasting", step(6.5, 0.02, 1200) == 0);

    /* 5. bus recovers -> coast released, FETs driven again */
    for (i = 0; i < 600; i++) step(12.5, 0.05, 1200);
    ok("coast released after recovery", !mppt.coasting);
    ok("FETs driven again after recovery", h_all_off == 0);
    ok("left RECOVER after recovery", mppt.state != MPPT_STATE_RECOVER);

    /* 6. hard absolute-minimum cut coasts immediately */
    {
        uint16_t d = 1200;
        ADC_raw_volts = adc_v(MPPT_V_ABSOLUTE_MIN / 100.0 - 0.5);
        h_all_off = 0;
        mppt_apply_duty(&d);
        ok("absolute-min cut zeroes duty", d == 0);
        ok("absolute-min cut coasts on the same tick", mppt.coasting && h_all_off);
    }

    /* ------------------------------------------------------------------
     * boot Voc capture
     *
     * The plant sim cannot exercise this: it arms almost immediately, so
     * the quiet window never opens. Drive it directly instead.
     * ---------------------------------------------------------------- */
    printf("\nboot Voc capture\n");
    {
        const int settle = MPPT_BOOT_VOC_SETTLE_TICKS;

        armed = 0; running = 0; input = 0; zero_crosses = 0;
        play_tone_flag = 0;
        ADC_raw_volts = adc_v(13.40); ADC_raw_current = adc_i(0.0);
        mppt_init();
        ok("nothing captured at init", !mppt.boot_voc_done && mppt.voc_boot == 0);
        ok("falls back to the nameplate until then",
           mppt.voc_est == MPPT_VOC_NOMINAL);

        /* a tune is playing - must not sample, however quiet the bus looks */
        play_tone_flag = 1;
        for (i = 0; i < settle * 3; i++) step(13.40, 0.0, 0);
        ok("does not sample while a tone is pending", !mppt.boot_voc_done);

        /* Tune done, but Cbus is still refilling. Deliberately a SLOW ramp -
         * 4 V over ~1.5 settle windows - because a fast one is easy to
         * reject and a slow one is what defeats tick-to-tick differencing. */
        play_tone_flag = 0;
        for (i = 0; i < settle * 3 / 2; i++)
            step(9.0 + i * (4.0 / (settle * 3.0 / 2.0)), 0.0, 0);
        ok("does not sample while the bus is still rising", !mppt.boot_voc_done);

        /* quiet and settled, but not yet for long enough */
        for (i = 0; i < settle - 4; i++) step(13.40, 0.0, 0);
        ok("does not sample before the settle time has elapsed",
           !mppt.boot_voc_done);

        /* ...and now it should latch */
        for (i = 0; i < 8; i++) step(13.40, 0.0, 0);
        ok("captures once quiet and settled", mppt.boot_voc_done);
        ok("captured the right voltage",
           mppt.voc_boot > 1320 && mppt.voc_boot < 1360);
        ok("setpoint follows the capture",
           mppt.vref > ((1320 * MPPT_K_FOCV_Q8) >> 8) &&
           mppt.vref < ((1370 * MPPT_K_FOCV_Q8) >> 8));

        /* an arming tune afterwards must not disturb the captured value */
        {
            int32_t held = mppt.voc_boot;
            play_tone_flag = 2;
            for (i = 0; i < settle * 2; i++) step(9.5, 0.0, 0);
            play_tone_flag = 0;
            ok("a later arming tune does not overwrite it",
               mppt.voc_boot == held && mppt.boot_voc_done);
        }
    }

#if MPPT_TRACKER == MPPT_TRACKER_BETA
    /* ------------------------------------------------------------------
     * beta regulator
     * ---------------------------------------------------------------- */
    printf("\nbeta regulator\n");
    {
        int32_t t1, t2;

        armed = 1; running = 1; input = 2047; zero_crosses = 5000;
        play_tone_flag = 0; e_com_time = 200;
        mppt_init();
        /* Only briefly: this test drives the ADCs directly with no plant, so
         * v does not respond to duty and the regulator has no way to
         * converge - left running it winds the trim until duty rails and the
         * authority gate (correctly) marks beta invalid. Check it while the
         * PI still has authority. */
        for (i = 0; i < 40; i++) step(10.5, 0.11, 1200);
        ok("computes a beta when current is usable",
           mppt.beta_valid && mppt.beta < 0);

        /* ...and that the authority gate does exactly that once duty rails */
        for (i = 0; i < 2000; i++) step(10.5, 0.11, 1200);
        ok("marks beta invalid once the PI loses authority", !mppt.beta_valid);

        /* Sign convention. beta below target means the panel is being held
         * ABOVE Vmpp, so the trim must come DOWN. Getting this backwards
         * walks the setpoint into the short-circuit region. */
        t1 = mppt.beta_trim;
        for (i = 0; i < 2000; i++) step(12.6, 0.05, 1200);  /* high V, low I */
        t2 = mppt.beta_trim;
        ok("held above Vmpp, the trim decreases", t2 < t1);

        t1 = mppt.beta_trim;
        for (i = 0; i < 2000; i++) step(8.5, 0.118, 1200);  /* low V, high I */
        t2 = mppt.beta_trim;
        ok("held below Vmpp, the trim increases", t2 > t1);

        /* Low light: current falls under the resolvable floor. The trim is
         * the learned part and must survive, or every cloud would throw the
         * calibration away. */
        t1 = mppt.beta_trim;
        for (i = 0; i < 4000; i++) step(11.0, 0.000, 1200);
        ok("goes invalid when current is unusable", !mppt.beta_valid);
        ok("holds the learned trim through low light", mppt.beta_trim == t1);

        /* and it can never pull the setpoint arbitrarily far */
        for (i = 0; i < 60000; i++) step(8.0, 0.119, 1200);
        ok("trim stays inside its bound",
           mppt.beta_trim <= MPPT_BETA_TRIM_MAX &&
           mppt.beta_trim >= -MPPT_BETA_TRIM_MAX);

        /* zero current must not reach ln(0) */
        for (i = 0; i < 200; i++) step(11.0, 0.0, 1200);
        ok("no ln(0) at zero current", !mppt.beta_valid);
    }

#else
    /* ------------------------------------------------------------------
     * rpm tracker (the no-current-sense fallback)
     * ---------------------------------------------------------------- */
    printf("\nrpm tracker\n");
    {
        int32_t k0;

        armed = 1; running = 1; input = 2047; zero_crosses = 5000;
        play_tone_flag = 0; e_com_time = 200;
        mppt_init();
        ok("starts from the compile-time ratio",
           mppt.k_focv_q8 == MPPT_K_FOCV_Q8);

        /* AM32 reports 65408 until the motor is turning. Adapting on that
         * would move k on a number that is not a speed at all. */
        k0 = mppt.k_focv_q8; mppt.rpm_steps = 0;
        e_com_time = 65408;
        for (i = 0; i < 4000; i++) step(10.5, 0.11, 1200);
        ok("rejects the 65408 pre-spin-up sentinel",
           mppt.rpm_steps == 0 && mppt.k_focv_q8 == k0);

        /* A moving throttle changes rpm for reasons that have nothing to do
         * with the setpoint; comparing across it walks k the wrong way. */
        e_com_time = 200; mppt.rpm_steps = 0;
        for (i = 0; i < 4000; i++) {
            input = (uint16_t)(1500 + (i % 400));
            step(10.5, 0.11, 1200);
        }
        ok("does not adapt while the throttle is moving", mppt.rpm_steps == 0);

        input = 2047; mppt.rpm_steps = 0;
        for (i = 0; i < 4000; i++) step(10.5, 0.11, 1200);
        ok("adapts again once the throttle settles", mppt.rpm_steps > 0);

        /* However badly the rpm signal misbehaves, k cannot leave the band
         * that every silicon panel's MPP lives in. */
        for (i = 0; i < 40000; i++) { e_com_time = 200 + (i % 3); step(10.5, 0.11, 1200); }
        ok("k stays inside the sanity bounds",
           mppt.k_focv_q8 >= MPPT_K_MIN_Q8 && mppt.k_focv_q8 <= MPPT_K_MAX_Q8);
    }
#endif

    /* ------------------------------------------------------------------
     * current-sense zero self-calibration
     * ---------------------------------------------------------------- */
    printf("\ncurrent-sense zero\n");
    {
        const int settle = MPPT_BOOT_VOC_SETTLE_TICKS;
        armed = 0; running = 0; input = 0; zero_crosses = 0; play_tone_flag = 0;
        ADC_raw_volts = adc_v(13.40); ADC_raw_current = adc_i(0.0);
        mppt_init();
        ok("uses CURRENT_OFFSET until calibrated", !mppt.i_zero_done);
        for (i = 0; i < settle + 8; i++) step(13.40, 0.0, 0);
        ok("captures the zero in the quiet window", mppt.i_zero_done);
        ok("captured zero matches the sensor midpoint",
           mppt.i_zero_raw > adc_i(0.0) - 3 && mppt.i_zero_raw < adc_i(0.0) + 3);
        /* and a real current still reads correctly through the new zero */
        for (i = 0; i < 60; i++) step(11.0, 0.100, 400);
        ok("current reads sanely after calibration",
           mppt.i >= 7 && mppt.i <= 13);
    }

    printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
