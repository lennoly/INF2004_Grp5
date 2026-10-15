/** @file subsystem5_test.c
 *
 * @brief Buddy 5 test program: full coarse-to-fine scan, obstacle profile
 *        and bypass plan (TEST_ULTRASONIC).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h" /* APP_MODE_* names: without it every #if is true */
#include "obstacle.h"
#include "avoidance.h"
#include "app_tests.h"
#include "subsystem5_test.h"

#if APP_MODE == APP_MODE_TEST_ULTRASONIC
/*!
 * @brief Buddy 5 test: full coarse-to-fine scan, profile and plan.
 */
void
subsystem5_test_ultrasonic (void)
{
    avoidance_profile_t profile;
    avoidance_plan_t    plan;

    /* Casts in the messages: tm_printf() takes the kernel's UB string type
       (the literals are ASCII); distances, angles and clearances are at
       most a few thousand, so they fit INT. */
    (void) tm_printf((UB const *) "# front=%d mm\n", (INT) obstacle_front_mm());
    (void) obstacle_scan(&profile);
    plan = avoidance_plan(&profile, 0u);
    (void) tm_printf(
        (UB const *) "# found=%d closest=%d@%d edges L=%d R=%d width=%d "
                     "clear L=%d R=%d -> %s offset=%d\n",
        profile.b_found ? 1 : 0, (INT) profile.closest_mm,
        (INT) profile.closest_angle, app_tests_print_int(profile.left_edge_mm),
        app_tests_print_int(profile.right_edge_mm),
        app_tests_print_int(profile.width_mm), (INT) profile.clear_left_mm,
        (INT) profile.clear_right_mm, avoidance_action_name(plan.action),
        app_tests_print_int(plan.offset_mm));
}
#endif

/*** end of file ***/
