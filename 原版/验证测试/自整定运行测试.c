/*
 * Repeatable C89 runtime tests for the actual PortableTune implementation.
 * Old Windows compilers may not resolve a UTF-8 Chinese include filename.
 * Force-include the implementation on the compiler command line, for example:
 * tcc -DPORTABLE_TUNE_IMPLEMENTATION -include <module-header> -run <this-file>
 * This test does not access hardware, Flash, serial ports, or installed files.
 */
#include <stdio.h>
#include <math.h>
#include <float.h>

#ifndef PORTABLE_TUNE_H
#error Force_include_the_actual_module_header_with_PORTABLE_TUNE_IMPLEMENTATION
#endif

#define TEST_PI 3.14159265358979323846
#define TEST_MASK 0xFFFFFFFFUL

static int tests_total = 0;
static int tests_failed = 0;

static float absf_test(float x)
{
    return x < 0.0f ? -x : x;
}

static int near_test(float actual, float expected, float tolerance)
{
    return actual == actual && absf_test(actual - expected) <= tolerance;
}

static float quantized_output(float value)
{
    return (float)floor((double)value + 0.5);
}

static float nan_test(void)
{
    volatile float zero;
    zero = 0.0f;
    return zero / zero;
}

static void finish_test(const char *name, int ok)
{
    tests_total++;
    if (!ok) tests_failed++;
    printf("%s %02d %s\n", ok ? "PASS" : "FAIL", tests_total, name);
}

static int safe_failure(const PortableTune *t, unsigned char reason)
{
    return t->state == PORTABLE_TUNE_FAILED && t->reason == reason
        && t->output == 0.0f && t->result.valid == 0;
}

static void print_result(const char *name, const PortableTune *t)
{
    printf("  %s: state=%u reason=%s cycles=%u stored=%u"
           " Kp=%.7g Ki=%.7g Kd=%.7g Ku=%.7g Pu=%.6g A=%.6g h=%.6g\n",
           name, (unsigned)t->state, portable_tune_reason(t->reason),
           (unsigned)t->total_cycles, (unsigned)t->stored,
           (double)t->result.kp, (double)t->result.ki,
           (double)t->result.kd, (double)t->result.ku,
           (double)t->result.period_s, (double)t->result.amplitude,
           (double)t->result.output_half_span);
}

static void fast_config(PortableTuneConfig *c)
{
    portable_tune_defaults(c);
    c->bias = 50.3f;
    c->relay_step = 10.1f;
    c->settle_ms = 1000UL;
    c->settle_timeout_ms = 5000UL;
    c->min_half_ms = 500UL;
    c->half_timeout_ms = 20000UL;
    c->min_period_ms = 2000UL;
    c->max_period_ms = 20000UL;
    c->max_sample_gap_ms = 500UL;
    c->total_timeout_ms = 120000UL;
    c->actuator_grace_ms = 100UL;
    c->discard_cycles = 1;
    c->required_cycles = 3;
    c->max_cycles = 9;
}

/* A known waveform tests timing/extraction, not a physical plant model. */
static void run_wave(PortableTune *t, const PortableTuneConfig *c,
                     float amplitude, unsigned long period_ms,
                     unsigned long start_ms)
{
    unsigned long elapsed, now, wave_begin;
    float y, applied;
    int wave_started;
    portable_tune_reset(t);
    portable_tune_start(t, c, start_ms);
    wave_begin = 0UL;
    wave_started = 0;
    for (elapsed = 0UL; elapsed < 130000UL; elapsed += 100UL) {
        if (!portable_tune_active(t)) break;
        if (t->state != PORTABLE_TUNE_SETTLING && !wave_started) {
            wave_started = 1;
            wave_begin = elapsed;
        }
        y = c->target;
        if (wave_started) {
            y += amplitude * (float)sin(2.0 * TEST_PI
                * (double)(elapsed - wave_begin) / (double)period_ms);
        }
        applied = quantized_output(t->output);
        now = (start_ms + elapsed) & TEST_MASK;
        portable_tune_update(t, y, applied, now, 1);
    }
}

static void test_defaults_and_config(void)
{
    PortableTuneConfig c;
    PortableTune t;
    PortableTuneResult r;
    int ok;
    portable_tune_defaults(&c);
    portable_tune_reset(&t);
    ok = portable_tune_config_valid(&c) && t.state == PORTABLE_TUNE_IDLE
        && !portable_tune_active(&t) && !portable_tune_get_result(&t, &r)
        && c.td_factor == 0.0f;
    finish_test("default configuration and reset", ok);

    c.relay_step = 0.0f;
    ok = !portable_tune_start(&t, &c, 0UL)
        && safe_failure(&t, PORTABLE_TUNE_BAD_CONFIG);
    finish_test("invalid configuration rejects start safely", ok);

    portable_tune_defaults(&c);
    c.target = nan_test();
    finish_test("NaN configuration is rejected", !portable_tune_config_valid(&c));

    ok = portable_tune_active((PortableTune *)0) == 0
        && portable_tune_start((PortableTune *)0, &c, 0UL) == 0
        && portable_tune_tick((PortableTune *)0, 0UL, 1) == PORTABLE_TUNE_IDLE
        && portable_tune_update((PortableTune *)0, 1.0f, 0.0f, 0UL, 1)
            == PORTABLE_TUNE_IDLE;
    finish_test("null-pointer API guards", ok);
}

static void test_known_waveforms(void)
{
    PortableTuneConfig c;
    PortableTune t;
    PortableTuneResult r;
    float expected_ku, expected_kp;
    int ok;
    fast_config(&c);
    run_wave(&t, &c, 2.0f, 10000UL, 0UL);
    print_result("known PI waveform with integer actuator", &t);
    expected_ku = (float)(40.0 / (TEST_PI * 2.0));
    expected_kp = c.kp_factor * expected_ku;
    ok = portable_tune_get_result(&t, &r)
        && near_test(r.period_s, 10.0f, 0.11f)
        && near_test(r.amplitude, 2.0f, 0.01f)
        && near_test(r.output_half_span, 10.0f, 0.01f)
        && near_test(r.ku, expected_ku, 0.01f)
        && near_test(r.kp, expected_kp, 0.01f)
        && near_test(r.ki, expected_kp / 10.0f, 0.002f)
        && r.kd == 0.0f && t.output == 0.0f;
    finish_test("PI extraction uses measured actuator span and seconds", ok);

    portable_tune_tick(&t, 60000UL, 0);
    finish_test("interlock revokes a previously obtained candidate",
        safe_failure(&t, PORTABLE_TUNE_INTERLOCK)
        && !portable_tune_get_result(&t, &r));

    c.kp_factor = 0.45f;
    c.ti_factor = 0.5f;
    c.td_factor = 0.125f;
    run_wave(&t, &c, 2.0f, 10000UL, 0UL);
    print_result("known optional PID waveform", &t);
    expected_kp = c.kp_factor * expected_ku;
    ok = portable_tune_get_result(&t, &r)
        && near_test(r.kp, expected_kp, 0.01f)
        && near_test(r.ki, expected_kp / 5.0f, 0.002f)
        && near_test(r.kd, expected_kp * 1.25f, 0.02f);
    finish_test("optional PID coefficients use continuous-time Ki/Kd", ok);

    portable_tune_cancel(&t);
    finish_test("cancel revokes candidate and returns zero output",
        t.state == PORTABLE_TUNE_CANCELLED
        && t.reason == PORTABLE_TUNE_USER_CANCEL
        && t.output == 0.0f && !portable_tune_get_result(&t, &r));
}

static void test_physical_heater(void)
{
    PortableTuneConfig c;
    PortableTune t;
    PortableTuneResult r;
    float delay[20];
    float y, applied, delayed, ymin, ymax;
    unsigned long now;
    int i, slot, ok, saw_high, saw_low;
    portable_tune_defaults(&c);
    portable_tune_reset(&t);
    portable_tune_start(&t, &c, 0UL);
    for (i = 0; i < 20; i++) delay[i] = c.bias;
    slot = 0;
    y = 21.0f;
    ymin = y;
    ymax = y;
    saw_high = 0;
    saw_low = 0;
    /* y'=((21 + delayed_power)-y)/80; delay=10s; sample=0.5s.
     * The bias 79% produces a 100 degree equilibrium from 21 ambient.
     * No fabricated temperature waveform or direct state mutation is used.
     */
    for (now = 0UL; now < 1800000UL && portable_tune_active(&t); now += 500UL) {
        applied = quantized_output(t.output);
        if (now > 0UL) {
            delayed = delay[slot];
            delay[slot] = applied;
            slot = (slot + 1) % 20;
            y += (21.0f + delayed - y) * (0.5f / 80.0f);
        }
        if (y < ymin) ymin = y;
        if (y > ymax) ymax = y;
        portable_tune_update(&t, y, applied, now, 1);
        if (t.state == PORTABLE_TUNE_HIGH) saw_high = 1;
        if (t.state == PORTABLE_TUNE_LOW) saw_low = 1;
    }
    print_result("FOPDT heater: ambient21 K1 tau80 deadtime10", &t);
    printf("  simulated seconds=%.1f input range=[%.5g,%.5g]\n",
           (double)now / 1000.0, (double)ymin, (double)ymax);
    ok = portable_tune_get_result(&t, &r) && saw_high && saw_low
        && r.kp > 0.0f && r.ki > 0.0f && r.kd == 0.0f
        && r.amplitude >= c.min_amplitude && r.amplitude <= c.max_amplitude
        && r.amplitude_spread <= c.amplitude_tolerance
        && r.period_spread <= c.period_tolerance
        && r.center_error <= c.center_tolerance && t.output == 0.0f;
    finish_test("default settling and real first-order-plus-deadtime heater", ok);
}

static void test_faults(void)
{
    PortableTuneConfig c;
    PortableTune t;
    unsigned long now;

    fast_config(&c);
    portable_tune_start(&t, &c, 0UL);
    portable_tune_cancel(&t);
    finish_test("cancel an active experiment",
        t.state == PORTABLE_TUNE_CANCELLED && t.output == 0.0f
        && t.result.valid == 0);

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, nan_test(), c.bias, 0UL, 1);
    finish_test("NaN measurement stops safely", safe_failure(&t, PORTABLE_TUNE_BAD_SAMPLE));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, c.target, nan_test(), 0UL, 1);
    finish_test("NaN applied output stops safely", safe_failure(&t, PORTABLE_TUNE_BAD_SAMPLE));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, c.target, c.bias, 0UL, 1);
    portable_tune_tick(&t, c.max_sample_gap_ms + 1UL, 1);
    finish_test("sensor silence detected by tick", safe_failure(&t, PORTABLE_TUNE_NO_SAMPLE));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_tick(&t, 0UL, 0);
    finish_test("interlock cancels active experiment", safe_failure(&t, PORTABLE_TUNE_INTERLOCK));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, c.target, 0.0f, 200UL, 1);
    finish_test("unapplied actuator command after grace is rejected",
        safe_failure(&t, PORTABLE_TUNE_OUTPUT_MISMATCH));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_tick(&t, c.actuator_grace_ms + 1UL, 1);
    finish_test("missing initial actuator confirmation expires independently",
        safe_failure(&t, PORTABLE_TUNE_OUTPUT_MISMATCH));

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, c.target, 0.0f, 50UL, 1);
    portable_tune_update(&t, c.target, c.bias, 100UL, 1);
    portable_tune_tick(&t, 150UL, 1);
    finish_test("actuator confirmation inside grace clears pending command",
        t.state == PORTABLE_TUNE_SETTLING && t.reason == PORTABLE_TUNE_OK);

    portable_tune_start(&t, &c, 0UL);
    portable_tune_update(&t, c.input_max, c.bias, 0UL, 1);
    finish_test("measurement at upper safety limit is rejected",
        safe_failure(&t, PORTABLE_TUNE_OUT_OF_RANGE));

    portable_tune_start(&t, &c, 0UL);
    for (now = 0UL; now <= 5000UL && portable_tune_active(&t); now += 100UL)
        portable_tune_update(&t, 90.0f, t.output, now, 1);
    finish_test("settling timeout with fresh samples",
        safe_failure(&t, PORTABLE_TUNE_SETTLE_TIMEOUT));

    c.settle_ms = 3000UL;
    c.settle_span = 0.02f;
    portable_tune_start(&t, &c, 0UL);
    for (now = 0UL; now <= 5000UL && portable_tune_active(&t); now += 100UL)
        portable_tune_update(&t, 99.9f + 0.02f * (float)now / 1000.0f,
            t.output, now, 1);
    finish_test("slow drift cannot satisfy the settling window",
        safe_failure(&t, PORTABLE_TUNE_SETTLE_TIMEOUT));

    fast_config(&c);
    c.half_timeout_ms = 2000UL;
    portable_tune_start(&t, &c, 0UL);
    for (now = 0UL; now <= 10000UL && portable_tune_active(&t); now += 100UL)
        portable_tune_update(&t, 100.0f, t.output, now, 1);
    finish_test("no threshold crossing triggers half-cycle timeout",
        safe_failure(&t, PORTABLE_TUNE_HALF_TIMEOUT));

    fast_config(&c);
    c.settle_timeout_ms = 2000UL;
    c.total_timeout_ms = 3000UL;
    portable_tune_start(&t, &c, 0UL);
    for (now = 0UL; now <= 4000UL && portable_tune_active(&t); now += 100UL)
        portable_tune_update(&t, 100.0f, t.output, now, 1);
    finish_test("total experiment timeout with fresh samples",
        safe_failure(&t, PORTABLE_TUNE_TOTAL_TIMEOUT));

    fast_config(&c);
    run_wave(&t, &c, 0.4f, 10000UL, 0UL);
    print_result("insufficient oscillation amplitude", &t);
    finish_test("insufficient oscillation fails without invented gains",
        safe_failure(&t, PORTABLE_TUNE_POOR_OSCILLATION));

    run_wave(&t, &c, 6.0f, 10000UL, 0UL);
    finish_test("excessive peak-to-peak amplitude is rejected",
        safe_failure(&t, PORTABLE_TUNE_POOR_OSCILLATION));

    c.max_period_ms = 8000UL;
    run_wave(&t, &c, 2.0f, 10000UL, 0UL);
    finish_test("oscillation outside configured period range is rejected",
        safe_failure(&t, PORTABLE_TUNE_POOR_OSCILLATION));

    fast_config(&c);
    c.kp_max = 0.1f;
    run_wave(&t, &c, 2.0f, 10000UL, 0UL);
    finish_test("candidate outside configured gain range is rejected",
        safe_failure(&t, PORTABLE_TUNE_GAIN_RANGE));
}

static void test_clock_and_instances(void)
{
    PortableTuneConfig c, other;
    PortableTune t, a, b;
    unsigned long now, elapsed, wave_a, wave_b;
    int began_a, began_b;
    float ya, yb;

    fast_config(&c);
    portable_tune_start(&t, &c, 1000UL);
    portable_tune_update(&t, c.target, c.bias, 1000UL, 1);
    portable_tune_update(&t, c.target, c.bias, 1100UL, 1);
    portable_tune_tick(&t, 1099UL, 1);
    finish_test("reversed millisecond clock stops safely", safe_failure(&t, PORTABLE_TUNE_CLOCK));

    run_wave(&t, &c, 2.0f, 10000UL, 0xFFFFFF00UL);
    finish_test("natural 32-bit clock rollover preserves full experiment",
        t.state == PORTABLE_TUNE_CANDIDATE && t.result.valid
        && near_test(t.result.period_s, 10.0f, 0.11f));

    other = c;
    other.target = 60.0f;
    other.bias = 40.3f;
    other.relay_step = 5.1f;
    portable_tune_start(&a, &c, 0UL);
    portable_tune_start(&b, &other, 0UL);
    wave_a = 0UL;
    wave_b = 0UL;
    began_a = 0;
    began_b = 0;
    for (elapsed = 0UL; elapsed < 120000UL; elapsed += 100UL) {
        now = elapsed;
        if (!portable_tune_active(&a) && !portable_tune_active(&b)) break;
        if (a.state != PORTABLE_TUNE_SETTLING && !began_a) {
            began_a = 1;
            wave_a = elapsed;
        }
        if (b.state != PORTABLE_TUNE_SETTLING && !began_b) {
            began_b = 1;
            wave_b = elapsed;
        }
        ya = c.target;
        yb = other.target;
        if (began_a) ya += 2.0f * (float)sin(2.0 * TEST_PI
            * (double)(elapsed - wave_a) / 10000.0);
        if (began_b) yb += 1.2f * (float)sin(2.0 * TEST_PI
            * (double)(elapsed - wave_b) / 12000.0);
        if (portable_tune_active(&a))
            portable_tune_update(&a, ya, quantized_output(a.output), now, 1);
        if (portable_tune_active(&b))
            portable_tune_update(&b, yb, quantized_output(b.output), now, 1);
    }
    print_result("independent instance A", &a);
    print_result("independent instance B", &b);
    finish_test("two interleaved instances keep independent states and results",
        a.state == PORTABLE_TUNE_CANDIDATE && b.state == PORTABLE_TUNE_CANDIDATE
        && near_test(a.result.period_s, 10.0f, 0.11f)
        && near_test(b.result.period_s, 12.0f, 0.11f)
        && near_test(a.result.output_half_span, 10.0f, 0.01f)
        && near_test(b.result.output_half_span, 5.0f, 0.01f));
}

int main(void)
{
    printf("PortableTune actual-C runtime regression, version=%d\n", PORTABLE_TUNE_VERSION);
    test_defaults_and_config();
    test_known_waveforms();
    test_physical_heater();
    test_faults();
    test_clock_and_instances();
    printf("RESULT total=%d passed=%d failed=%d\n",
        tests_total, tests_total - tests_failed, tests_failed);
    return tests_failed ? 1 : 0;
}
