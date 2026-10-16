/** @file app_tests.c
 *
 * @brief Shared runner for the per-subsystem test programs (see
 *        app_tests.h).  The START button (GP20) runs or repeats the
 *        selected test, which lives in subsystemN/subsystemN_test.c.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "encoder.h"
#include "motion.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "telemetry.h"
#include "subsystem2_test.h"
#include "subsystem3_test.h"
#include "subsystem4_test.h"
#include "subsystem5_test.h"
#include "app_tests.h"

#if (APP_MODE != APP_MODE_MISSION) && (APP_MODE != APP_MODE_TEST_TELEMETRY)
#define TEST_TASK_BUILD (1)
#else
#define TEST_TASK_BUILD (0)
#endif

#define TASK_PRIORITY (11)
#define TASK_STACK    (3072)
#define PRINT_MS      (100u)
#define RELEASE_MS    (500u) /* time to let go of the car        */

#if TEST_TASK_BUILD
static void test_task(INT stacd, void * p_exinf);
static void wait_start_button(void);
static void run_test(void);
#endif

/*!
 * @brief Initialise only what the selected test needs and start its task.
 *
 * @return E_OK or a kernel error code; always E_OK in mission and
 *         telemetry builds, which have no separate test task.
 */
int32_t
app_tests_start (void)
{
    int32_t ercd = E_OK;

#if TEST_TASK_BUILD
    T_CTSK ctsk   = {0};
    ID     h_task = 0;

#if APP_MODE == APP_MODE_TEST_MOTOR
    (void) encoder_init();
    (void) motion_init();
#elif APP_MODE == APP_MODE_TEST_MOTION
    (void) encoder_init();
    (void) motion_init();
#elif APP_MODE == APP_MODE_TEST_WHEELS
    /* No motion task: it would rewrite the motor pins every cycle. */
    (void) encoder_init();
#elif APP_MODE == APP_MODE_TEST_IR
    (void) ir_sensor_init();
#elif APP_MODE == APP_MODE_TEST_BARCODE
    (void) encoder_init();
    (void) motion_init();
    (void) ir_sensor_init();
    (void) barcode_init();
#elif APP_MODE == APP_MODE_TEST_IMU
    (void) encoder_init();
    (void) motion_init();
    (void) subsystem4_test_init(); /* I2C scan, then imu_init() */
#elif APP_MODE == APP_MODE_TEST_ULTRASONIC
    (void) obstacle_init();
#else
#error "Unknown APP_MODE"
#endif

    /* Commands (pid=, ff=, line=, rate=, stop ...) work in every test
       build over the USB / UART console; telemetry streams on "rate=". */
    (void) telemetry_init(false);

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       test_task(). */
    ctsk.task    = (FP) test_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);
    ercd         = (h_task >= E_OK) ? tk_sta_tsk(h_task, 0) : h_task;
#endif

    return (ercd);
}

/*!
 * @brief Round a measurement for printing with %d (used by every
 *        subsystemN_test.c).
 *
 * @param[in] value Measurement (well inside the int32_t range).
 *
 * @return Nearest integer.
 */
INT
app_tests_print_int (float32_t value)
{
    return (car_math_round(value));
}

#if TEST_TASK_BUILD
/*!
 * @brief Test task: run the selected test each time START is pressed.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
test_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;
    hal_gpio_init_in(PIN_BTN_START, true);

    for (;;)
    {
        wait_start_button();
        run_test();

        /* Cast: tm_printf() takes the kernel's UB string type; the literal
           is ASCII. */
        (void) tm_printf((UB const *) "# test done\n");
    }
}

/*!
 * @brief Wait until START (active low) is pressed, then give the user
 *        time to let go of the car.
 */
static void
wait_start_button (void)
{
    /* Cast: tm_printf() takes the kernel's UB string type; the literal is
       ASCII. */
    (void) tm_printf((UB const *) "# press START (GP20)\n");

    while (hal_gpio_get(PIN_BTN_START))
    {
        (void) tk_dly_tsk(PRINT_MS);
    }

    (void) tk_dly_tsk(RELEASE_MS);
}

/*!
 * @brief Run the test of the selected APP_MODE (in its subsystem's folder).
 */
static void
run_test (void)
{
#if APP_MODE == APP_MODE_TEST_MOTOR
    subsystem2_test_motor();
#elif APP_MODE == APP_MODE_TEST_MOTION
    subsystem2_test_motion();
#elif APP_MODE == APP_MODE_TEST_WHEELS
    subsystem2_test_wheels();
#elif APP_MODE == APP_MODE_TEST_IR
    subsystem3_test_ir();
#elif APP_MODE == APP_MODE_TEST_BARCODE
    subsystem3_test_barcode();
#elif APP_MODE == APP_MODE_TEST_IMU
    subsystem4_test_imu();
#else
    subsystem5_test_ultrasonic();
#endif
}
#endif

/*** end of file ***/
