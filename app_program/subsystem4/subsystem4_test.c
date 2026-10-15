/** @file subsystem4_test.c
 *
 * @brief Buddy 4 test program: tilt, rates, hump height and motion events
 *        (TEST_IMU).  Angles are printed in tenths of a degree.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h" /* APP_MODE_* names: without it every #if is true */
#include "imu.h"
#include "terrain.h"
#include "app_tests.h"
#include "subsystem4_test.h"

#define PRINT_MS         (100u)
#define TEST_DURATION_MS (30000u) /* streaming tests run for 30 s     */
#define TENTHS           (10.0f)

#if APP_MODE == APP_MODE_TEST_IMU
/*!
 * @brief Buddy 4 IMU test: push or drive the car over a hump; prints tilt
 *        (tenths of a degree), rates, hump height and motion events.
 */
void
subsystem4_test_imu (void)
{
    imu_status_t imu_state;
    uint32_t     elapsed_ms = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# ok,pitch,roll,yaw,encRate,magRate,h,"
                                  "peak,max,n,event,impacts\n");

    for (elapsed_ms = 0u; elapsed_ms < TEST_DURATION_MS; elapsed_ms += PRINT_MS)
    {
        imu_get_status(&imu_state);

        /* Casts: the counters are 32-bit unsigned like UINT. */
        (void) tm_printf((UB const *) "%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%s,%u\n",
                         imu_state.b_ok ? 1 : 0,
                         app_tests_print_int(imu_state.pitch_deg * TENTHS),
                         app_tests_print_int(imu_state.roll_deg * TENTHS),
                         app_tests_print_int(imu_state.heading_deg * TENTHS),
                         app_tests_print_int(imu_state.enc_rate_dps),
                         app_tests_print_int(imu_state.mag_rate_dps),
                         app_tests_print_int(imu_state.hump.height_mm),
                         app_tests_print_int(imu_state.hump.peak_mm),
                         app_tests_print_int(imu_state.hump.max_peak_mm),
                         (UINT) imu_state.hump.humps,
                         terrain_event_name(imu_state.event),
                         (UINT) imu_state.impacts);
        (void) tk_dly_tsk(PRINT_MS);
    }
}
#endif

/*** end of file ***/
