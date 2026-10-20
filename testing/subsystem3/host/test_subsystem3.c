/* Regression tests use the actual C implementation, not a Python model. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <tk/tkernel.h>
#include "car_config.h"
#include "ir_sensor.h"
#include "line_follow.h"
#include "barcode.h"
#include "subsystem3_test.h"
#include "app_tests.h"
#include "fakes.h"

static unsigned g_checks;
static unsigned g_failures;
#define CHECK(expr) do { g_checks++; if (!(expr)) { \
    g_failures++; printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

#if IR_BLACK_IS_HIGH
#define WHITE (300u)
#define BLACK (3600u)
#else
#define WHITE (3600u)
#define BLACK (300u)
#endif

/* Independent fixtures transcribed from the supplied course sample and
 * standard patterns, rather than fetched from the implementation's table. */
static uint32_t const g_photo[29] = {
    15u,47u,15u,15u,48u,15u,47u,15u,15u,15u,47u,15u,15u,16u,14u,
    48u,14u,16u,45u,17u,15u,46u,15u,15u,47u,15u,47u,15u,15u
};
static char const * const g_patterns[] = {
    "100001001", "001001001", "101001000", "000011001", "011010000"
};

static void make_frame(uint32_t *widths, unsigned symbol, uint32_t narrow)
{
    char const *patterns[3] = {"010010100", g_patterns[symbol], "010010100"};
    uint32_t pos = 0u;
    for (unsigned c = 0u; c < 3u; c++)
    {
        if (c != 0u) { widths[pos++] = narrow; }
        for (unsigned bit = 0u; bit < 9u; bit++)
        {
            widths[pos++] = narrow * ((patterns[c][bit] == '1') ? 3u : 1u);
        }
    }
}

static void test_decoder(void)
{
    char text[9] = {0};
    uint32_t widths[29];
    uint32_t reversed[29];
    CHECK(barcode_decode_frame(g_photo, 29u, text, sizeof(text)) == 1);
    CHECK(strcmp(text, "A") == 0);
    for (unsigned symbol = 0u; symbol < 5u; symbol++)
    {
        for (uint32_t speed = 100u; speed <= 300u; speed += 10u)
        {
            make_frame(widths, symbol, 3000000u / speed);
            for (unsigned i = 0u; i < 29u; i++)
            {
                /* Deterministic +-10% width distortion, including gaps. */
                widths[i] = widths[i] * (90u + (i * 7u) % 21u) / 100u;
                reversed[28u - i] = widths[i];
            }
            CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) == 1);
            CHECK(text[0] == ((symbol < 4u) ? (char) ('A' + symbol) : 'Z'));
            CHECK(barcode_decode_frame(reversed, 29u, text, sizeof(text)) == 1);
            CHECK(text[0] == ((symbol < 4u) ? (char) ('A' + symbol) : 'Z'));
        }
    }
    CHECK(barcode_decode_to_command('A') == NAV_LEFT);
    CHECK(barcode_decode_to_command('B') == NAV_RIGHT);
    CHECK(barcode_decode_to_command('C') == NAV_STRAIGHT);
    CHECK(barcode_decode_to_command('D') == NAV_UTURN);
    CHECK(barcode_decode_to_command('Z') == NAV_NONE);
    CHECK(barcode_decode_frame(NULL, 29u, text, sizeof(text)) < 0);
    CHECK(text[0] == '\0');
    CHECK(barcode_decode_frame(g_photo, UINT32_MAX, text, sizeof(text)) < 0);
    CHECK(barcode_decode_frame(g_photo, 28u, text, sizeof(text)) < 0);
    CHECK(barcode_decode_frame(g_photo, 29u, text, 1u) < 0);
    make_frame(widths, 0u, 10000u);
    widths[9] = 0u;
    CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) < 0);
    widths[9] = 250000u;
    CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) < 0);
    make_frame(widths, 0u, 10000u);
    widths[5] = 0u;
    CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) < 0);
    for (unsigned i = 0u; i < 29u; i++) { widths[i] = 10000u; }
    CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) < 0);
    memcpy(widths, g_photo, sizeof(widths));
    memcpy(&widths[10], g_photo, 9u * sizeof(uint32_t)); /* *** */
    CHECK(barcode_decode_frame(widths, 29u, text, sizeof(text)) < 0);
}

static void test_line(void)
{
    line_follow_t line;
    line_follow_init(&line);
    CHECK(line_follow_classify(&line, .5f, .5f, 0.0f, 20u) == LINE_ON);
    CHECK(line_follow_steer(&line, .9f, .1f, .02f) > 0.0f);
    line_follow_init(&line);
    CHECK(line_follow_steer(&line, .1f, .9f, .02f) < 0.0f);
    CHECK(line_follow_classify(&line, 0.0f, 0.0f, 0.0f, 149u) == LINE_ON);
    CHECK(line_follow_classify(&line, 0.0f, 0.0f, 0.0f, 1u) == LINE_LOST);
    CHECK(fabsf(line_follow_steer(&line, .9f, .1f, .02f)) < .001f);
    CHECK(line_follow_classify(&line, 0.0f, 0.0f, 0.0f, UINT32_MAX) == LINE_LOST);
    CHECK(line.lost_ms == 150u);
    CHECK(line_follow_classify(&line, .5f, .5f, 0.0f, 20u) == LINE_ON);
    CHECK(line_follow_classify(&line, .9f, .9f, 9.0f, 20u) == LINE_ON);
    CHECK(line_follow_classify(&line, .9f, .9f, 1.0f, 20u) == LINE_JUNCTION);
    CHECK(line_follow_classify(&line, .5f, .5f, 0.0f, 20u) == LINE_ON);
    CHECK(line_follow_classify(&line, .9f, .9f, 0.0f, 20u) == LINE_ON);
    CHECK(line_follow_classify(&line, NAN, .5f, 0.0f, 20u) == LINE_LOST);
    CHECK(fabsf(line_follow_steer(&line, NAN, .5f, .02f)) < .001f);
    CHECK(fabsf(line_follow_speed(180.0f, 1.0f) - 90.0f) < .001f);
    CHECK(fabsf(line_follow_speed(180.0f, NAN)) < .001f);
    CHECK(fabsf(line_follow_speed(-10.0f, 0.0f)) < .001f);
    line_follow_init(&line);
    CHECK(fabsf(line_follow_steer(&line, .5f, .5f, INFINITY)) < .001f);
    line_follow_set_gains(&line, NAN, 0.0f, 0.0f);
    CHECK(isfinite(line.pid.gain_p));
}

static void test_calibration(void)
{
    ir_sensor_snapshot_t sample;
    ir_sensor_calibration_t old;
    ir_sensor_calibration_t current;
    g_fake_init_error = -5;
    CHECK(ir_sensor_init() == -5);
    g_fake_init_error = 0;
    CHECK(ir_sensor_init() == 0);
    CHECK(ir_sensor_init() == 0);
    ir_sensor_get_calibration(&current);
    CHECK(!current.b_valid);
    fake_sample(WHITE, WHITE, WHITE, 100u, false);
    ir_sensor_calib_begin();
    fake_sample(WHITE, WHITE, WHITE, 100u, false);
    fake_sample(BLACK, BLACK, WHITE, 100u, false);
    CHECK(!ir_sensor_calib_end()); /* third sensor must also pass */
    ir_sensor_get_calibration(&current);
    CHECK(!current.b_valid);
    ir_sensor_calib_begin();
    fake_sample(WHITE, WHITE, WHITE, 100u, false);
    fake_sample(BLACK, BLACK, BLACK, 100u, false);
    CHECK(ir_sensor_calib_end());
    ir_sensor_get_calibration(&old);
    CHECK(old.b_valid);
    fake_sample(WHITE, WHITE, WHITE, 100u, false);
    ir_sensor_read(&sample);
    CHECK(sample.norm[0] < .01f && sample.norm[1] < .01f && sample.norm[2] < .01f);
    fake_sample(BLACK, BLACK, BLACK, 100u, false);
    ir_sensor_read(&sample);
    CHECK(sample.norm[0] > .99f && sample.norm[1] > .99f && sample.norm[2] > .99f);
    ir_sensor_calib_begin();
    fake_sample(BLACK, BLACK, BLACK, 100u, false);
    CHECK(!ir_sensor_calib_end());
    ir_sensor_get_calibration(&current);
    CHECK(memcmp(old.white, current.white, sizeof(old.white)) == 0);
    CHECK(memcmp(old.black, current.black, sizeof(old.black)) == 0);
    ir_sensor_calib_begin();
    fake_sample(WHITE, WHITE, WHITE, 100u, false);
    ir_sensor_calib_cancel();
    ir_sensor_get_calibration(&current);
    CHECK(memcmp(old.black, current.black, sizeof(old.black)) == 0);
}

static void stream_frame(unsigned symbol, uint32_t narrow_ms, bool reverse)
{
    uint32_t widths[29];
    make_frame(widths, symbol, narrow_ms);
    fake_sample(WHITE, WHITE, WHITE, 300u, true);
    for (unsigned i = 0u; i < 29u; i++)
    {
        uint16_t level = (i % 2u == 0u) ? BLACK : WHITE;
        fake_sample(WHITE, WHITE, level, widths[reverse ? 28u-i : i], true);
    }
    /* Includes the real 10 ms task period, below the old 250 ms wait. */
    fake_sample(WHITE, WHITE, WHITE, 15u, true);
}

static void test_stream(void)
{
    barcode_event_t event;
    barcode_stats_t before;
    barcode_stats_t after;
    g_fake_init_error = -5;
    CHECK(barcode_init() == -5);
    g_fake_init_error = 0;
    CHECK(barcode_init() == 0);
    CHECK(barcode_init() == 0);
    CHECK(g_fake_tasks == 1u);
    CHECK(g_fake_message_size == 6u);
    barcode_set_enabled(true);
    for (unsigned symbol = 0u; symbol < 5u; symbol++)
    {
        for (uint32_t narrow = 10u; narrow <= 30u; narrow += 10u)
        {
            for (unsigned reverse = 0u; reverse < 2u; reverse++)
            {
                stream_frame(symbol, narrow, reverse != 0u);
                CHECK(barcode_get_event(&event, 0));
                CHECK(event.text[0] == ((symbol < 4u) ? (char) ('A'+symbol) : 'Z'));
                CHECK(event.cmd == barcode_decode_to_command(event.text[0]));
                fake_sample(WHITE, WHITE, WHITE, 300u, true);
                CHECK(!barcode_get_event(&event, 0)); /* no duplicate */
            }
        }
    }
    /* Truncated capture and recovery. */
    fake_sample(WHITE, WHITE, BLACK, 10u, true);
    fake_sample(WHITE, WHITE, WHITE, 300u, true);
    CHECK(!barcode_get_event(&event, 0));
    stream_frame(1u, 10u, false);
    CHECK(barcode_get_event(&event, 0) && event.cmd == NAV_RIGHT);

    /* A full task queue must never block capture or silently lose evidence. */
    barcode_get_stats(&before);
    for (unsigned i = 0u; i < 5u; i++) { stream_frame(0u, 10u, false); }
    barcode_get_stats(&after);
    CHECK(after.dropped == before.dropped + 1u);
    for (unsigned i = 0u; i < 4u; i++) { CHECK(barcode_get_event(&event, 0)); }
    CHECK(!barcode_get_event(&event, 0));

    /* ISR ring overflow invalidates the complete in-flight frame. */
    barcode_get_stats(&before);
    for (unsigned i = 0u; i < 80u; i++)
    {
        fake_sample(WHITE, WHITE, (i % 2u == 0u) ? BLACK : WHITE, 20u, false);
    }
    barcode_poll();
    barcode_get_stats(&after);
    CHECK(after.overruns > before.overruns);
    CHECK(!barcode_get_event(&event, 0));
    stream_frame(3u, 10u, true);
    CHECK(barcode_get_event(&event, 0) && event.cmd == NAV_UTURN);

    barcode_set_enabled(false);
    stream_frame(0u, 10u, false);
    CHECK(!barcode_get_event(&event, 0));
    barcode_set_enabled(true);
    fake_set_time(UINT32_MAX - 400000u);
    stream_frame(2u, 10u, false); /* timer wrap during the frame */
    CHECK(barcode_get_event(&event, 0) && event.cmd == NAV_STRAIGHT);
}

int main(void)
{
    test_decoder();
    test_line();
    test_calibration();
    test_stream();
    /* STOP exits either real bench routine without any motion dependency. */
    gb_fake_stop = true;
#if APP_MODE == APP_MODE_TEST_IR
    subsystem3_test_ir();
#else
    subsystem3_test_barcode();
#endif
    CHECK(app_tests_start() == 0);
    printf("%u checks, %u failures\n", g_checks, g_failures);
    printf("edge ring: 512 -> 192 bytes; queued event: %zu -> %u bytes\n",
           sizeof(barcode_event_t), g_fake_message_size);
    return (g_failures == 0u) ? 0 : 1;
}
