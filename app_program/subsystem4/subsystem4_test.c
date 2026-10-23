/** @file subsystem4_test.c
 *
 * @brief Buddy 4 test programs: tilt, rates, hump height and motion events
 *        (TEST_IMU; angles in tenths of a degree), and impact detection
 *        by driving into a wall (TEST_CRASH).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h" /* APP_MODE_* names: without it every #if is true */
#include "hal.h"
#include "imu.h"
#include "motion.h"
#include "terrain.h"
#include "app_tests.h"
#include "subsystem4_test.h"

#define PRINT_MS         (100u)
#define TEST_DURATION_MS (30000u) /* streaming tests run for 30 s     */
#define TENTHS           (10.0f)
#define I2C_FIRST_ADDR   (0x08u) /* 7-bit addresses a device can use */
#define I2C_LAST_ADDR    (0x77u)
#define SCAN_MAX_FOUND   (16u)
#define ID_REG_COUNT     (3u) /* chip-ID registers read per device */

/* TEST_CRASH: start 20-30 cm from a wall, facing it. */
#define CRASH_TRIALS        (10u)
/* Fixed duty, no speed PID: at low speed the PID hunted (stuttered). */
#define CRASH_DUTY_PCT      (70.0f)  /* drive at the wall               */
#define CRASH_BACK_PCT      (-50.0f) /* reverse, also fixed duty        */
#define CRASH_MAX_MS        (6000u)  /* give up if it never gets there  */
#define CRASH_RUN_MM        (600.0f) /* longest drive towards the wall  */
#define CRASH_BACK_MM       (150.0f) /* reverse after each try          */
#define CRASH_START_MM      (50.0f)  /* first part of a drive: start   */
#define CRASH_MOVED_MM      (20.0f)  /* rolling: now watch for a stop   */
#define CRASH_BLOCKED_MS    (500u)   /* no travel this long: at the wall*/
#define CRASH_POLL_MS       (20)
#define CRASH_SETTLE_MS     (600u)   /* past the impact hold-off        */

#if (APP_MODE == APP_MODE_TEST_IMU) || (APP_MODE == APP_MODE_TEST_CRASH)
typedef struct
{
    bool      b_hit;         /* an impact past CRASH_START_MM ended it */
    float32_t travel_mm;     /* driven before it stopped             */
    float32_t speed_mm_s;    /* speed on the sample before the stop  */
    uint32_t  start_impacts; /* impacts in the first CRASH_START_MM  */
    float32_t start_jerk_mg; /* peak jerk while starting             */
    float32_t wall_jerk_mg;  /* peak jerk after that (the wall)      */
} crash_result_t;

typedef struct
{
    float32_t back_mm;  /* distance reversed            */
    uint32_t  impacts;  /* impacts while reversing      */
    float32_t jerk_mg;  /* peak jerk while reversing    */
    float32_t stop_mg;  /* peak jerk braking to a stop  */
} crash_back_t;

/* Boot scan result, printed again when START is pressed. */
static uint8_t  g_found[SCAN_MAX_FOUND];
static uint8_t  g_ids[SCAN_MAX_FOUND][ID_REG_COUNT];

/* ID registers: 0x00 ADXL345 (0xE5), 0x0D MMA845x (0x1A / 0x2A / 0x3A),
   0x0F ST WHO_AM_I (LSM303D / LSM9DS0 0x49, LSM303AGR 0x33). */
static uint8_t const g_id_regs[ID_REG_COUNT] = {0x00u, 0x0Du, 0x0Fu};
static uint32_t g_found_count = 0u;
static bool     gb_sda_high   = false;
static bool     gb_scl_high   = false;
static int32_t  g_imu_ercd    = E_OK;

static void print_scan(void);
static void drive_to_wall(crash_result_t * p_result);
static void back_off(crash_back_t * p_back);

/*!
 * @brief Scan I2C0 (the IMU's GP4 / GP5) for devices, then start the IMU,
 *        so a wiring fault or a different chip shows up in the log.  Runs
 *        once at boot, before the IMU task starts using the bus.
 *
 * @return imu_init() result.
 */
int32_t
subsystem4_test_init (void)
{
    uint8_t  byte = 0u;
    uint32_t addr = 0u;
    uint32_t idx  = 0u;
    uint32_t reg  = 0u;

    (void) hal_i2c_init(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_HZ);

    /* Both lines idle high (pull-ups).  A line held low means a short or
       a miswired module, and the scan would never finish. */
    gb_sda_high = hal_gpio_get(PIN_IMU_SDA);
    gb_scl_high = hal_gpio_get(PIN_IMU_SCL);

    for (addr = I2C_FIRST_ADDR;
         gb_sda_high && gb_scl_high && (addr <= I2C_LAST_ADDR); addr++)
    {
        /* A 1-byte read changes nothing in a device; only an ACK answers.
           Cast: addr <= 0x77 fits uint8_t. */
        if ((1 == hal_i2c_read((uint8_t) addr, &byte, 1u))
            && (g_found_count < SCAN_MAX_FOUND))
        {
            g_found[g_found_count] = (uint8_t) addr;
            g_found_count++;
        }
    }

    /* Read each device's ID registers (0xFF = no answer). */
    for (idx = 0u; idx < g_found_count; idx++)
    {
        for (reg = 0u; reg < ID_REG_COUNT; reg++)
        {
            g_ids[idx][reg] = 0xFFu;

            if (1 == hal_i2c_write(g_found[idx], &g_id_regs[reg], 1u, true))
            {
                (void) hal_i2c_read(g_found[idx], &g_ids[idx][reg], 1u);
            }
        }
    }

    g_imu_ercd = imu_init();
    print_scan();

    return (g_imu_ercd);
}

/*!
 * @brief Buddy 4 IMU test: push the car forward by hand (over a hump if
 *        one is placed ahead); prints tilt (tenths of a degree), rates,
 *        hump height, motion events and the wheel readings for 30 s.
 */
void
subsystem4_test_imu (void)
{
    imu_status_t    imu_state;
    motion_status_t motion;
    uint32_t        elapsed_ms = 0u;

    print_scan();

    /* Start each run flat with no impacts: tilting the car to press START
       must not leave a hump open (the mission does the same at START). */
    imu_reset_run();

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type; the literals are ASCII. */
    (void) tm_printf((UB const *) "# ok,pitch,roll,yaw,encRate,magRate,h,"
                                  "peak,max,n,event,impacts,"
                                  "tgtL,vL,pwmL,tgtR,vR,pwmR,odo,"
                                  "ticksL,ticksR\n");

    for (elapsed_ms = 0u; elapsed_ms < TEST_DURATION_MS; elapsed_ms += PRINT_MS)
    {
        imu_get_status(&imu_state);
        motion_get_status(&motion);

        /* Casts: the counters are 32-bit unsigned like UINT. */
        (void) tm_printf((UB const *) "%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%s,%u,",
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

        /* Wheel columns: target and measured speed (mm/s), duty (%),
           odometer (mm) and raw tick counts.  Casts: as above (UB string,
           32-bit counters as UINT). */
        (void) tm_printf((UB const *) "%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
                         app_tests_print_int(motion.target_l_mm_s),
                         app_tests_print_int(motion.speed_l_mm_s),
                         app_tests_print_int(motion.pwm_l_pct),
                         app_tests_print_int(motion.target_r_mm_s),
                         app_tests_print_int(motion.speed_r_mm_s),
                         app_tests_print_int(motion.pwm_r_pct),
                         app_tests_print_int(motion.odo_mm),
                         (UINT) motion.ticks_l, (UINT) motion.ticks_r);
        (void) tk_dly_tsk(PRINT_MS);
    }
}

/*!
 * @brief Buddy 4 impact test: drive at the wall CRASH_TRIALS times,
 *        reversing after each try.  Each row: WALL (impact at the wall) or
 *        MISS (stopped by the wall, no impact), the impacts and peak impact
 *        jerk (mg, terrain_impact_jerk()) while starting, at the wall,
 *        while reversing and braking after it.  IMPACT_JERK_MG belongs
 *        above the start, reverse and stop jerks and below the wall jerks.
 */
void
subsystem4_test_crash (void)
{
    crash_result_t result;
    crash_back_t   back;
    uint32_t       trial = 0u;
    uint32_t       walls = 0u;

    print_scan();
    imu_reset_run();

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type (the literals are ASCII); the counters fit UINT. */
    (void) tm_printf((UB const *) "# trial,result,travel,speed,startImpacts,"
                                  "startJerk,wallJerk,back,backImpacts,"
                                  "backJerk,stopJerk\n");

    for (trial = 1u; trial <= CRASH_TRIALS; trial++)
    {
        drive_to_wall(&result);
        back_off(&back);
        walls += result.b_hit ? 1u : 0u;

        (void) tm_printf((UB const *) "%u,%s,%d,%d,%u,%d,%d,", (UINT) trial,
                         result.b_hit ? "WALL" : "MISS",
                         app_tests_print_int(result.travel_mm),
                         app_tests_print_int(result.speed_mm_s),
                         (UINT) result.start_impacts,
                         app_tests_print_int(result.start_jerk_mg),
                         app_tests_print_int(result.wall_jerk_mg));
        (void) tm_printf((UB const *) "%d,%u,%d,%d\n",
                         app_tests_print_int(back.back_mm),
                         (UINT) back.impacts,
                         app_tests_print_int(back.jerk_mg),
                         app_tests_print_int(back.stop_mg));
    }

    (void) tm_printf((UB const *) "# walls %u of %u\n", (UINT) walls,
                     (UINT) CRASH_TRIALS);
}

/*!
 * @brief Drive towards the wall at CRASH_DUTY_PCT until an impact past
 *        CRASH_START_MM (one sooner is the motor start, counted apart), until
 *        the car stops moving (against the wall), or for CRASH_RUN_MM or
 *        CRASH_MAX_MS; then stop.
 *
 * @param[out] p_result How the drive ended.
 */
static void
drive_to_wall (crash_result_t * p_result)
{
    imu_status_t    imu_state;
    motion_status_t motion;
    uint32_t        last_impacts = 0u;
    uint32_t        still_ms     = 0u;
    uint32_t        elapsed_ms   = 0u;
    float32_t       start_mm     = 0.0f;
    float32_t       last_mm      = 0.0f;
    float32_t       jerk         = 0.0f;
    bool            b_starting   = true;

    imu_get_status(&imu_state);
    motion_get_status(&motion);
    last_impacts             = imu_state.impacts;
    start_mm                 = motion.odo_mm;
    last_mm                  = motion.odo_mm;
    p_result->b_hit          = false;
    p_result->speed_mm_s     = 0.0f;
    p_result->start_impacts  = 0u;
    p_result->start_jerk_mg  = 0.0f;
    p_result->wall_jerk_mg   = 0.0f;
    (void) imu_take_peak_jerk();
    motion_set_open_loop(CRASH_DUTY_PCT, CRASH_DUTY_PCT);

    while ((!p_result->b_hit) && (still_ms < CRASH_BLOCKED_MS)
           && ((motion.odo_mm - start_mm) < CRASH_RUN_MM)
           && (elapsed_ms < CRASH_MAX_MS))
    {
        (void) tk_dly_tsk(CRASH_POLL_MS);
        elapsed_ms += (uint32_t) CRASH_POLL_MS;
        imu_get_status(&imu_state);
        jerk = imu_take_peak_jerk();

        /* Speed before this sample: the hit sample has already slowed. */
        p_result->speed_mm_s = 0.5f * (motion.speed_l_mm_s + motion.speed_r_mm_s);
        motion_get_status(&motion);
        b_starting = ((motion.odo_mm - start_mm) < CRASH_START_MM);

        if (b_starting)
        {
            p_result->start_impacts += imu_state.impacts - last_impacts;
            p_result->start_jerk_mg = (jerk > p_result->start_jerk_mg)
                                          ? jerk
                                          : p_result->start_jerk_mg;
        }
        else
        {
            p_result->b_hit        = (imu_state.impacts != last_impacts);
            p_result->wall_jerk_mg = (jerk > p_result->wall_jerk_mg)
                                         ? jerk
                                         : p_result->wall_jerk_mg;
        }

        last_impacts = imu_state.impacts;

        /* Count still time only once rolling, so the start is not a stop. */
        still_ms = ((motion.odo_mm > last_mm)
                    || ((motion.odo_mm - start_mm) < CRASH_MOVED_MM))
                       ? 0u
                       : (still_ms + (uint32_t) CRASH_POLL_MS);
        last_mm  = motion.odo_mm;
    }

    motion_stop();
    p_result->travel_mm = motion.odo_mm - start_mm;
}

/*!
 * @brief Let the car settle, back CRASH_BACK_MM away from the wall at a
 *        fixed duty (the speed PID stuttered) and pause.
 *
 * @param[out] p_back Distance, impacts and peak jerk while reversing (a
 *                    motor start, not a wall) and braking to a stop.
 */
static void
back_off (crash_back_t * p_back)
{
    imu_status_t    imu_state;
    motion_status_t motion;
    uint32_t        start_impacts = 0u;
    uint32_t        elapsed_ms    = 0u;
    float32_t       start_mm      = 0.0f;

    (void) tk_dly_tsk(CRASH_SETTLE_MS);
    imu_get_status(&imu_state);
    motion_get_status(&motion);
    start_impacts = imu_state.impacts;
    start_mm      = motion.odo_mm;
    (void) imu_take_peak_jerk();
    motion_set_open_loop(CRASH_BACK_PCT, CRASH_BACK_PCT);

    /* The odometer counts down: its sign follows the duty. */
    while (((start_mm - motion.odo_mm) < CRASH_BACK_MM)
           && (elapsed_ms < CRASH_MAX_MS))
    {
        (void) tk_dly_tsk(CRASH_POLL_MS);
        elapsed_ms += (uint32_t) CRASH_POLL_MS;
        motion_get_status(&motion);
    }

    p_back->jerk_mg = imu_take_peak_jerk();
    motion_stop();
    p_back->back_mm = start_mm - motion.odo_mm;
    (void) tk_dly_tsk(CRASH_SETTLE_MS);
    imu_get_status(&imu_state);
    p_back->impacts = imu_state.impacts - start_impacts;
    p_back->stop_mg = imu_take_peak_jerk();
}

/*!
 * @brief Print the boot scan and the IMU start-up result.
 */
static void
print_scan (void)
{
    uint32_t idx = 0u;

    /* Casts in the messages: tm_printf() takes the kernel's UB string
       type (the literals are ASCII); the values fit UINT and INT. */
    (void) tm_printf((UB const *) "[i2c] SDA GP%u=%u SCL GP%u=%u (idle should "
                                  "be 1 and 1)\n",
                     (UINT) PIN_IMU_SDA, gb_sda_high ? 1u : 0u,
                     (UINT) PIN_IMU_SCL, gb_scl_high ? 1u : 0u);

    if (!(gb_sda_high && gb_scl_high))
    {
        (void) tm_printf((UB const *) "[i2c] a line is held low: check the "
                                      "wiring for a short; scan skipped\n");
    }
    else
    {
        (void) tm_printf((UB const *) "[i2c] devices found:");

        for (idx = 0u; idx < g_found_count; idx++)
        {
            (void) tm_printf((UB const *) " 0x%02X", (UINT) g_found[idx]);
        }

        (void) tm_printf((UB const *) "%s\n",
                         (0u == g_found_count) ? " none (not connected or "
                                                 "not powered)"
                                               : "");
    }

    /* Casts: tm_printf() takes the kernel's UB string type; the bytes fit
       UINT. */
    for (idx = 0u; idx < g_found_count; idx++)
    {
        (void) tm_printf((UB const *) "[i2c] 0x%02X id: reg00=0x%02X "
                                      "reg0D=0x%02X reg0F=0x%02X\n",
                         (UINT) g_found[idx], (UINT) g_ids[idx][0],
                         (UINT) g_ids[idx][1], (UINT) g_ids[idx][2]);
    }

    /* Casts: UB string type as above; the error code fits INT. */
    (void) tm_printf((UB const *) "[i2c] the LSM303D answers at 0x1D "
                                  "(WHO_AM_I reg0F=0x49)\n");
    (void) tm_printf((UB const *) "[init] imu        %s (%d)\n",
                     (g_imu_ercd >= E_OK) ? "OK" : "FAILED", (INT) g_imu_ercd);
}
#endif

/*** end of file ***/
