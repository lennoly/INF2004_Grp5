/** @file app_main.c
 *
 * @brief PicoCar entry point (usermain) for micro T-Kernel 3.0.
 *
 * Task map (lower number = higher priority):
 *   4  cyw43     template WiFi / lwIP service (and MQTT client hook)
 *   5  motion    50 Hz speed PID (cyclic handler wake-up)       Buddy 2
 *   6  vehicle   mission state machine                          integration
 *   7  barcode   frame and decode the barcode edge stream       Buddy 3
 *   8  imu       50 Hz tilt, hump and motion events             Buddy 4
 *   9  sonar     front distance monitor                         Buddy 5
 *  10  telemetry 5 Hz JSON, heartbeat and commands              Buddy 1
 *  10  blink     template liveness LED (GP16)
 *  11  test      standalone test programs (APP_MODE=TEST_xxx)
 * ISRs: IO_BANK0 (encoders, echo) and TIMER alarm 3 (1 kHz IR sampler).
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include "usb_console_compat.h"
#include "demo_tasks.h"
#include "car_config.h"
#include "encoder.h"
#include "motion.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "telemetry.h"
#include "vehicle.h"
#include "app_tests.h"
#include "app_main.h"

#define USB_WAIT_POLLS (50)
#define USB_WAIT_MS    (100u)
#define USB_ENUMERATED (2u) /* tm_usb_state() once the host has configured */
#define BLINK_PRIORITY (10)
#define BLINK_STACK    (1024)

static void report_init(char const * p_name, int32_t ercd);
static void start_blink(void);

/*!
 * @brief Application entry point, called once by the kernel's initial
 *        task.  Starts every subsystem and then sleeps for ever.
 *
 * @return Never returns in practice (returning would shut the system
 *         down).
 */
INT
usermain (void)
{
    INT attempt = 0;

    start_blink();

    /* Let USB-CDC enumerate so that the boot messages are not lost. */
    for (attempt = 0;
         (attempt < USB_WAIT_POLLS) && (tm_usb_state() < USB_ENUMERATED);
         attempt++)
    {
        (void) tk_dly_tsk(USB_WAIT_MS);
    }

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; every literal here is ASCII. */
    (void) tm_printf((UB const *) "\n=== PicoCar on uT-Kernel 3.0 "
                                  "(APP_MODE %d) ===\n",
                     APP_MODE);

#if (APP_MODE == APP_MODE_MISSION) || (APP_MODE == APP_MODE_TEST_TELEMETRY)
    /* Keep the car still: the IMU calibrates for 2 s after imu_init(). */
    report_init("encoder", encoder_init());
    report_init("motion", motion_init());
    report_init("ir", ir_sensor_init());
    report_init("barcode", barcode_init());
    report_init("imu", imu_init());
    report_init("obstacle", obstacle_init());
    report_init("vehicle", vehicle_init());
    report_init("telemetry", telemetry_init(true));

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    (void) tm_printf((UB const *) "START=GP20  STOP=GP21 (STOP while idle "
                                  "= calibrate)\nType help on this console "
                                  "for tuning commands.\n");
#else
    report_init("test", app_tests_start());
#endif

    (void) tk_slp_tsk(TMO_FEVR);

    return (0);
}

/*!
 * @brief Print one initialisation result so that wiring faults are
 *        obvious at boot.
 *
 * @param[in] p_name Subsystem name.
 * @param[in] ercd   Result of its init function (E_OK or an error).
 */
static void
report_init (char const * p_name, int32_t ercd)
{
    /* Casts: tm_printf() takes the kernel's UB string type (the literal is
       ASCII); ercd is a kernel error code, which fits in INT. */
    (void) tm_printf((UB const *) "[init] %-10s %s (%d)\n", p_name,
                     (ercd >= E_OK) ? "OK" : "FAILED", (INT) ercd);
}

/*!
 * @brief Start the template's liveness LED task.
 */
static void
start_blink (void)
{
    T_CTSK ctsk   = {0};
    ID     h_task = 0;

    gpio_set_pin(BOARD_LED_PIN, GPIO_MODE_OUT);
    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       blink_task(). */
    ctsk.task    = (FP) blink_task;
    ctsk.itskpri = BLINK_PRIORITY;
    ctsk.stksz   = BLINK_STACK;
    h_task       = tk_cre_tsk(&ctsk);

    if (h_task >= E_OK)
    {
        (void) tk_sta_tsk(h_task, 0);
    }
}

/*** end of file ***/
