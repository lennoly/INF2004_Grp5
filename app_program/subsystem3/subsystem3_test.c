/** @file subsystem3_test.c
 *
 * @brief Buddy 3 test programs: line sensors (TEST_IR) and barcode reading
 *        (TEST_BARCODE).  Normalised values are printed in permille.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "motion.h"
#include "ir_sensor.h"
#include "line_follow.h"
#include "barcode.h"
#include "app_tests.h"
#include "subsystem3_test.h"

#define PRINT_MS         (100u)
#define TEST_DURATION_MS (30000u) /* streaming tests run for 30 s     */
#define CALIBRATION_MS   (5000u)
#define SLOW_TIMEOUT_MS  (20000)
#define PERMILLE         (1000.0f)
#define BARCODE_RUN_MM   (1000.0f)

#if APP_MODE == APP_MODE_TEST_IR
/*!
 * @brief Buddy 3 line-sensor test: a 5 s manual calibration (slide the car
 *        across the line), then raw and normalised values, line error and
 *        state at 10 Hz.  Normalised values are printed in permille.
 */
void
subsystem3_test_ir (void)
{
    ir_sensor_snapshot_t ir_snap;
    line_follow_t        follower;
    uint32_t             elapsed_ms = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# calibrating 5 s: sweep sensors over "
                                  "line+floor\n");
    ir_sensor_calib_begin();
    (void) tk_dly_tsk(CALIBRATION_MS);
    (void) tm_printf((UB const *) "# calibration %s\n",
                     ir_sensor_calib_end() ? "OK" : "FAILED (low contrast)");
    line_follow_init(&follower);
    (void) tm_printf((UB const *) "# rawL,rawR,rawB,nL,nR,nB,err,state\n");

    for (elapsed_ms = 0u; elapsed_ms < TEST_DURATION_MS; elapsed_ms += PRINT_MS)
    {
        ir_sensor_read(&ir_snap);
        (void) line_follow_classify(&follower, ir_snap.norm[IR_LEFT],
                                    ir_snap.norm[IR_RIGHT], 0.0f, PRINT_MS);

        /* Casts: raw values are 12-bit ADC counts and the enumeration
           value is 0..2, so both fit INT. */
        (void) tm_printf(
            (UB const *) "%d,%d,%d,%d,%d,%d,%d,%d\n",
            (INT) ir_snap.raw[IR_LEFT], (INT) ir_snap.raw[IR_RIGHT],
            (INT) ir_snap.raw[IR_BARCODE],
            app_tests_print_int(ir_snap.norm[IR_LEFT] * PERMILLE),
            app_tests_print_int(ir_snap.norm[IR_RIGHT] * PERMILLE),
            app_tests_print_int(ir_snap.norm[IR_BARCODE] * PERMILLE),
            app_tests_print_int((ir_snap.norm[IR_LEFT] - ir_snap.norm[IR_RIGHT])
                                * PERMILLE),
            (INT) follower.state);
        (void) tk_dly_tsk(PRINT_MS);
    }
}
#endif

#if APP_MODE == APP_MODE_TEST_BARCODE
/*!
 * @brief Buddy 3 barcode test: drive slowly and straight over a barcode
 *        (pushing the car by hand also works); barcode.c prints each
 *        decode.
 */
void
subsystem3_test_barcode (void)
{
    barcode_stats_t bc_stats;

    motion_move_forward(BARCODE_RUN_MM, SPEED_SLOW_MM_S);
    (void) motion_wait_done(SLOW_TIMEOUT_MS);
    barcode_get_stats(&bc_stats);

    /* Casts: tm_printf() takes the kernel's UB string type (the literal is
       ASCII); the counters are 32-bit unsigned like UINT. */
    (void) tm_printf((UB const *) "# frames=%u decoded=%u errors=%u "
                                  "last='%s'\n",
                     (UINT) bc_stats.frames, (UINT) bc_stats.decoded,
                     (UINT) bc_stats.errors, bc_stats.last.text);
}
#endif

/*** end of file ***/
