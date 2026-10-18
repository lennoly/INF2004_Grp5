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
#include "hal.h"
#include "imu.h"
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

#if APP_MODE == APP_MODE_TEST_IMU
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
 * @brief Buddy 4 IMU test: push or drive the car over a hump; prints tilt
 *        (tenths of a degree), rates, hump height and motion events.
 */
void
subsystem4_test_imu (void)
{
    imu_status_t imu_state;
    uint32_t     elapsed_ms = 0u;

    print_scan();

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
