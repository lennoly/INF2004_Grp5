/** @file subsystem3_test.c
 *
 * @brief Standalone Buddy 3 bench tests. All movement is by hand.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "hal.h"
#include "ir_sensor.h"
#include "line_follow.h"
#include "barcode.h"
#include "app_tests.h"
#include "subsystem3_test.h"

#define SAMPLE_MS        (20u)
#define PRINT_MS         (100u)
#define TEST_DURATION_MS (60000u)
#define CALIBRATION_MS   (5000u)
#define PERMILLE         (1000.0f)
#define SAMPLE_SECONDS   (0.02f)

#if (APP_MODE == APP_MODE_TEST_IR) || (APP_MODE == APP_MODE_TEST_BARCODE)
static bool calibrate(void);

#endif

#if APP_MODE == APP_MODE_TEST_IR
/*!
 * @brief Show line classifier and steering output without commanding motors.
 *
 * Hand travel is not measured, so distance is zero: both_black reports a
 * junction candidate; the 10 mm junction confirmation is tested on the host.
 */
void
subsystem3_test_ir (void)
{
    ir_sensor_snapshot_t ir_snap;
    line_follow_t        follower;
    uint32_t             elapsed      = 0u;
    float32_t            steer        = 0.0f;
    bool                 b_both_black = false;

    if (calibrate())
    {
        line_follow_init(&follower);
        /* Cast: the logger uses kernel byte strings. */
        (void) tm_printf((UB const *) "# ms,rawL,rawR,rawB,nL,nR,nB,error,"
                                      "state,both_black,steer_mm_s\n");

        for (elapsed = 0u; elapsed < TEST_DURATION_MS; elapsed += SAMPLE_MS)
        {
            if (!hal_gpio_get(PIN_BTN_STOP))
            {
                break;
            }

            ir_sensor_read(&ir_snap);
            (void) line_follow_classify(&follower, ir_snap.norm[IR_LEFT],
                                        ir_snap.norm[IR_RIGHT], 0.0f,
                                        SAMPLE_MS);
            steer = line_follow_steer(&follower, ir_snap.norm[IR_LEFT],
                                      ir_snap.norm[IR_RIGHT], SAMPLE_SECONDS);
            b_both_black = (ir_snap.norm[IR_LEFT] > LINE_JUNCTION_LEVEL)
                           && (ir_snap.norm[IR_RIGHT] > LINE_JUNCTION_LEVEL);

            if (0u == (elapsed % PRINT_MS))
            {
                /* Casts: elapsed <= 60000, ADC <= 4095, state <= 2. */
                (void) tm_printf(
                    (UB const *) "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    (INT) elapsed, (INT) ir_snap.raw[IR_LEFT],
                    (INT) ir_snap.raw[IR_RIGHT], (INT) ir_snap.raw[IR_BARCODE],
                    app_tests_print_int(ir_snap.norm[IR_LEFT] * PERMILLE),
                    app_tests_print_int(ir_snap.norm[IR_RIGHT] * PERMILLE),
                    app_tests_print_int(ir_snap.norm[IR_BARCODE] * PERMILLE),
                    app_tests_print_int(follower.error * PERMILLE),
                    (INT) follower.state, (INT) b_both_black,
                    app_tests_print_int(steer));
            }

            (void) tk_dly_tsk(SAMPLE_MS);
        }
    }
}
#endif

#if APP_MODE == APP_MODE_TEST_BARCODE
/*!
 * @brief Calibrate, then read hand-moved barcodes for 60 seconds or until STOP.
 */
void
subsystem3_test_barcode (void)
{
    barcode_stats_t before;
    barcode_stats_t after;
    barcode_event_t event;
    uint32_t        elapsed = 0u;

    barcode_set_enabled(false);

    if (calibrate())
    {
        barcode_get_stats(&before);
        barcode_set_enabled(true);
        /* Cast: tm_printf uses kernel byte strings. */
        (void) tm_printf((UB const *) "# Move any *A*..*Z* under barcode "
                                      "sensor, both directions. Leave white "
                                      "for >= 250 ms between passes.\n");

        for (elapsed = 0u; elapsed < TEST_DURATION_MS; elapsed += SAMPLE_MS)
        {
            if (!hal_gpio_get(PIN_BTN_STOP))
            {
                break;
            }

            while (barcode_get_event(&event, TMO_POL))
            {
                /* Cast: kernel UINT holds the full millisecond timestamp. */
                (void) tm_printf((UB const *) "barcode,%u,%s,%s\n",
                                 (UINT) event.time_ms, event.text,
                                 barcode_decode_command_name(event.cmd));
            }

            (void) tk_dly_tsk(SAMPLE_MS);
        }

        barcode_set_enabled(false);
        barcode_get_stats(&after);
        /* Casts: uint32_t counters match kernel UINT. */
        (void) tm_printf((UB const *) "# frames=%u decoded=%u errors=%u "
                                      "overruns=%u dropped=%u\n",
                         (UINT) (after.frames - before.frames),
                         (UINT) (after.decoded - before.decoded),
                         (UINT) (after.errors - before.errors),
                         (UINT) (after.overruns - before.overruns),
                         (UINT) (after.dropped - before.dropped));
    }
}
#endif

#if (APP_MODE == APP_MODE_TEST_IR) || (APP_MODE == APP_MODE_TEST_BARCODE)
/*!
 * @brief Capture all sensors over black and white; STOP aborts the test.
 *
 * @return true only when all three channels pass measured calibration.
 */
static bool
calibrate (void)
{
    uint32_t                elapsed = 0u;
    uint32_t                idx     = 0u;
    ir_sensor_calibration_t calibration;
    bool                    b_ok      = false;
    bool                    b_stopped = false;

    /* Cast: tm_printf uses the kernel ASCII byte-string type. */
    (void) tm_printf(
        (UB const *) "# Motors OFF. Sweep ALL THREE sensors over "
                     "black and white for 5 seconds. STOP aborts.\n");
    ir_sensor_calib_begin();

    for (elapsed = 0u; elapsed < CALIBRATION_MS; elapsed += SAMPLE_MS)
    {
        if (!hal_gpio_get(PIN_BTN_STOP))
        {
            ir_sensor_calib_cancel();
            b_stopped = true;
            break;
        }

        (void) tk_dly_tsk(SAMPLE_MS);
    }

    if (!b_stopped)
    {
        b_ok = ir_sensor_calib_end();
        ir_sensor_get_calibration(&calibration);

        for (idx = 0u; idx < IR_COUNT; idx++)
        {
            /* Casts: channel indices and 12-bit ADC values fit INT. */
            (void) tm_printf((UB const *) "# calibration channel=%d white=%d "
                                          "black=%d min=%d max=%d\n",
                             (INT) idx, (INT) calibration.white[idx],
                             (INT) calibration.black[idx],
                             (INT) calibration.minimum[idx],
                             (INT) calibration.maximum[idx]);
        }

        (void) tm_printf((UB const *) "# calibration %s (required span >= 2048 "
                                      "on every channel)\n",
                         b_ok ? "PASS"
                              : "FAIL - adjust height/light, then retry");
    }

    return (b_ok);
}
#endif

/*** end of file ***/
