/** @file imu.c
 *
 * @brief Buddy 4 - LSM303DLHC driver and terrain monitoring task.
 *
 * Register values from the LSM303DLHC datasheet (DocID018771):
 *   CTRL_REG1_A 0x20 = 0x57 : 100 Hz, normal mode, X/Y/Z enabled
 *   CTRL_REG4_A 0x23 = 0x88 : BDU, +-2 g, high resolution (1 mg/LSB)
 *   CRA_REG_M   0x00 = 0x18 : 75 Hz magnetometer
 *   CRB_REG_M   0x01 = 0x20 : +-1.3 gauss
 *   MR_REG_M    0x02 = 0x00 : continuous conversion
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "motion.h"
#include "imu.h"

#define TASK_PRIORITY  (8)
#define TASK_STACK     (3072)
#define ADDR_ACCEL     (0x19u)
#define ADDR_MAG       (0x1Eu)
#define REG_CTRL1_A    (0x20u)
#define REG_CTRL4_A    (0x23u)
#define REG_OUT_X_L_A  (0x28u)
#define REG_CRA_M      (0x00u)
#define REG_CRB_M      (0x01u)
#define REG_MR_M       (0x02u)
#define REG_OUT_X_H_M  (0x03u)
#define AUTO_INCREMENT (0x80u)
#define VAL_CTRL1_A    (0x57u)
#define VAL_CTRL4_A    (0x88u)
#define VAL_CRA_M      (0x18u)
#define VAL_CRB_M      (0x20u)
#define VAL_MR_M       (0x00u)
#define HR_DIVISOR     (16) /* 12-bit left-justified data          */
#define SIGN_BIT_16    (0x8000u)
#define WRAP_16        (0x10000)
#define IMU_AXES       (3u)
#define AXIS_X         (0u)
#define AXIS_Y         (1u)
#define AXIS_Z         (2u)
#define REG_BYTES      (6u) /* X, Y, Z as 16-bit pairs             */
#define WRITE_BYTES    (2u) /* register address + value            */
#define BYTE_SHIFT     (8u)
#define ONE_G_MG       (1000.0f)
#define MM_S2_PER_MG   (9.80665f)
#define ACCEL_LPF      (0.30f) /* weight of the new sample            */
#define DV_LPF         (0.30f)
#define RATE_LPF       (0.20f)
#define MS_PER_S       (1000.0f)
#define FULL_CIRCLE    (360.0f)
#define HALF_CIRCLE    (180.0f)
#define WRAP_MAX_DEG   (1.0e6f)
#define MAG_RAW_MAX    (32767.0f)
#define MAG_RAW_MIN    (-32768.0f)

/* Cast: the period is a small number of milliseconds, which float32_t
   holds exactly. */
#define DT_S ((float32_t) IMU_PERIOD_MS / MS_PER_S)

/* Values from the previous 50 Hz step, kept by the IMU task. */
typedef struct
{
    float32_t speed;           /* mean wheel speed, mm/s     */
    float32_t odo;             /* odometer, mm               */
    float32_t accel[IMU_AXES]; /* raw acceleration, mg       */
    uint32_t  holdoff_ms;      /* impact detector hold-off   */
} imu_history_t;

/* Published snapshot: written by the IMU task, read by other tasks. */
static volatile imu_status_t g_st;

/* Shared with the vehicle task (calibration and run reset): volatile. */
static volatile float32_t g_mag_min[IMU_AXES];
static volatile float32_t g_mag_max[IMU_AXES];
static volatile float32_t g_mag_off[IMU_AXES] = {0.0f, 0.0f, 0.0f};
static volatile bool      gb_mag_cal          = false;
static volatile bool      gb_reset_run        = false;

/* IMU-task private working state (never touched by other tasks). */
static imu_status_t   g_work;
static terrain_hump_t g_hump;
static float32_t      g_bias[IMU_AXES] = {0.0f, 0.0f, 0.0f};

static bool      transfer_ok(int32_t result, uint32_t expected);
static bool      reg_write(uint8_t addr, uint8_t reg, uint8_t value);
static bool      reg_read(uint8_t addr, uint8_t reg, uint8_t * p_dst,
                          uint32_t count);
static int32_t   to_signed16(uint32_t raw16);
static int32_t   pair_le(uint8_t const * p_bytes);
static int32_t   pair_be(uint8_t const * p_bytes);
static bool      read_sensors(float32_t * p_acc, float32_t * p_mag);
static void      calibrate_accel(void);
static void      mag_track(float32_t const * p_mag);
static float32_t wrap180(float32_t angle_deg);
static float32_t finite_or(float32_t value, float32_t fallback);
static void      publish(void);
static void      update_tilt(float32_t const * p_acc, float32_t speed,
                             imu_history_t const * p_hist);
static void update_heading(float32_t const * p_mag, float32_t turn_rate_dps);
static bool detect_impact(float32_t const * p_acc, imu_history_t * p_hist);
static void imu_step(imu_history_t * p_hist);
static void imu_task(INT stacd, void * p_exinf);

/*!
 * @brief Configure the sensor and start the task (the car must be still
 *        for the 2 s accelerometer calibration).
 *
 * @return E_OK, E_IO if the sensor did not answer, or a tk_cre_tsk()
 *         error.
 */
int32_t
imu_init (void)
{
    T_CTSK  ctsk   = {0};
    ID      h_task = 0;
    bool    b_ok   = false;
    int32_t ercd   = E_OK;

    g_st.az_mg = ONE_G_MG;
    (void) hal_i2c_init(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_HZ);
    b_ok = (reg_write(ADDR_ACCEL, REG_CTRL1_A, VAL_CTRL1_A))
           && (reg_write(ADDR_ACCEL, REG_CTRL4_A, VAL_CTRL4_A))
           && (reg_write(ADDR_MAG, REG_CRA_M, VAL_CRA_M))
           && (reg_write(ADDR_MAG, REG_CRB_M, VAL_CRB_M))
           && (reg_write(ADDR_MAG, REG_MR_M, VAL_MR_M));
    g_st.b_ok = b_ok;

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       imu_task(). */
    ctsk.task    = (FP) imu_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);

    if (h_task >= E_OK)
    {
        (void) tk_sta_tsk(h_task, 0);
        ercd = b_ok ? E_OK : E_IO;
    }
    else
    {
        ercd = h_task;
    }

    return (ercd);
}

/*!
 * @brief Consistent copy of the latest IMU status.
 *
 * @param[out] p_out Status.
 */
void
imu_get_status (imu_status_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_st;
    EI(imask);
}

/*!
 * @brief Start hard-iron calibration (the vehicle task spins the car).
 */
void
imu_mag_calib_begin (void)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < IMU_AXES; idx++)
    {
        g_mag_min[idx] = MAG_RAW_MAX;
        g_mag_max[idx] = MAG_RAW_MIN;
    }

    gb_mag_cal = true;
}

/*!
 * @brief Stop hard-iron calibration and apply the centre of each axis's
 *        range as its offset.
 */
void
imu_mag_calib_end (void)
{
    uint32_t idx = 0u;

    gb_mag_cal = false;

    for (idx = 0u; idx < IMU_AXES; idx++)
    {
        if (g_mag_max[idx] > g_mag_min[idx])
        {
            g_mag_off[idx] = 0.5f * (g_mag_max[idx] + g_mag_min[idx]);
        }
    }
}

/*!
 * @brief Clear the hump statistics at the start of a mission run.
 */
void
imu_reset_run (void)
{
    gb_reset_run = true;
}

/*!
 * @brief Check an I2C transfer result.
 *
 * @param[in] result   Byte count, or a negative error.
 * @param[in] expected Bytes that should have been transferred.
 *
 * @return true if exactly the expected number of bytes was transferred.
 */
static bool
transfer_ok (int32_t result, uint32_t expected)
{
    /* Cast: result is not negative when it is converted. */
    return ((result >= 0) && (expected == (uint32_t) result));
}

/*!
 * @brief Write one register.
 *
 * @param[in] addr  7-bit I2C address.
 * @param[in] reg   Register.
 * @param[in] value Value.
 *
 * @return true on ACK.
 */
static bool
reg_write (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[WRITE_BYTES];

    buf[0] = reg;
    buf[1] = value;

    return (
        transfer_ok(hal_i2c_write(addr, buf, WRITE_BYTES, false), WRITE_BYTES));
}

/*!
 * @brief Burst read starting at a register.
 *
 * @param[in]  addr  7-bit I2C address.
 * @param[in]  reg   First register (with AUTO_INCREMENT if needed).
 * @param[out] p_dst Destination.
 * @param[in]  count Bytes to read.
 *
 * @return true on success.
 */
static bool
reg_read (uint8_t addr, uint8_t reg, uint8_t * p_dst, uint32_t count)
{
    bool b_ok = transfer_ok(hal_i2c_write(addr, &reg, 1u, true), 1u);

    if (b_ok)
    {
        b_ok = transfer_ok(hal_i2c_read(addr, p_dst, count), count);
    }

    return (b_ok);
}

/*!
 * @brief Two's-complement 16-bit register value to a signed integer, using
 *        only unsigned bit operations (Rule 5.3.b) and no
 *        implementation-defined narrowing conversion.
 *
 * @param[in] raw16 Register pair assembled as an unsigned value 0..0xFFFF.
 *
 * @return The value in -32768..32767.
 */
static int32_t
to_signed16 (uint32_t raw16)
{
    /* Cast: raw16 <= 0xFFFF, which int32_t always represents exactly. */
    int32_t result = (int32_t) raw16;

    if (0u != (raw16 & SIGN_BIT_16))
    {
        result -= WRAP_16;
    }

    return (result);
}

/*!
 * @brief Little-endian byte pair (accelerometer) to a signed value.
 *
 * @param[in] p_bytes Low byte, then high byte.
 *
 * @return The value in -32768..32767.
 */
static int32_t
pair_le (uint8_t const * p_bytes)
{
    /* Casts: widening a byte to uint32_t keeps its value and keeps a
       promoted signed int out of the bitwise operations. */
    return (to_signed16(((uint32_t) p_bytes[1] << BYTE_SHIFT)
                        | (uint32_t) p_bytes[0]));
}

/*!
 * @brief Big-endian byte pair (magnetometer) to a signed value.
 *
 * @param[in] p_bytes High byte, then low byte.
 *
 * @return The value in -32768..32767.
 */
static int32_t
pair_be (uint8_t const * p_bytes)
{
    /* Casts: as in pair_le(). */
    return (to_signed16(((uint32_t) p_bytes[0] << BYTE_SHIFT)
                        | (uint32_t) p_bytes[1]));
}

/*!
 * @brief Read the acceleration (mg) and magnetic field (raw counts).
 *
 * @param[out] p_acc Acceleration X, Y, Z.
 * @param[out] p_mag Magnetic field X, Y, Z.
 *
 * @return true if both sensors answered.
 */
static bool
read_sensors (float32_t * p_acc, float32_t * p_mag)
{
    uint8_t raw[REG_BYTES];
    bool    b_ok = false;

    b_ok = reg_read(ADDR_ACCEL, REG_OUT_X_L_A | AUTO_INCREMENT, raw, REG_BYTES);

    if (b_ok)
    {
        /* Casts: |value| <= 2048, exactly representable in float32_t. */
        p_acc[AXIS_X] = (float32_t) (pair_le(&raw[0]) / HR_DIVISOR);
        p_acc[AXIS_Y] = (float32_t) (pair_le(&raw[2]) / HR_DIVISOR);
        p_acc[AXIS_Z] = (float32_t) (pair_le(&raw[4]) / HR_DIVISOR);
        b_ok          = reg_read(ADDR_MAG, REG_OUT_X_H_M, raw, REG_BYTES);
    }

    if (b_ok)
    {
        /* The magnetometer order is X, Z, Y, big-endian.
           Casts: |value| <= 32768, exactly representable in float32_t. */
        p_mag[AXIS_X] = (float32_t) pair_be(&raw[0]);
        p_mag[AXIS_Z] = (float32_t) pair_be(&raw[2]);
        p_mag[AXIS_Y] = (float32_t) pair_be(&raw[4]);
    }

    return (b_ok);
}

/*!
 * @brief Average IMU_CALIB_SAMPLES readings with the car standing still on
 *        a flat floor (expected: 0, 0 and 1 g).
 */
static void
calibrate_accel (void)
{
    float32_t acc[IMU_AXES];
    float32_t mag[IMU_AXES];
    float32_t sum[IMU_AXES] = {0.0f, 0.0f, 0.0f};
    uint32_t  count         = 0u;
    uint32_t  idx           = 0u;
    uint32_t  axis          = 0u;
    float32_t samples       = 0.0f;

    for (idx = 0u; idx < IMU_CALIB_SAMPLES; idx++)
    {
        if (read_sensors(acc, mag))
        {
            sum[AXIS_X] += acc[AXIS_X];
            sum[AXIS_Y] += acc[AXIS_Y];
            sum[AXIS_Z] += acc[AXIS_Z];
            count++;
        }

        (void) tk_dly_tsk(IMU_PERIOD_MS);
    }

    if (count > 0u)
    {
        /* Cast: count <= IMU_CALIB_SAMPLES, a small integer. */
        samples = (float32_t) count;

        for (axis = 0u; axis < IMU_AXES; axis++)
        {
            g_bias[axis] = sum[axis] / samples;
        }

        g_bias[AXIS_Z] -= ONE_G_MG;
    }
}

/*!
 * @brief Hard-iron tracking while the car spins (see vehicle.c).
 *
 * @param[in] p_mag Magnetic field X, Y, Z (raw counts).
 */
static void
mag_track (float32_t const * p_mag)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < IMU_AXES; idx++)
    {
        g_mag_min[idx] =
            (p_mag[idx] < g_mag_min[idx]) ? p_mag[idx] : g_mag_min[idx];
        g_mag_max[idx] =
            (p_mag[idx] > g_mag_max[idx]) ? p_mag[idx] : g_mag_max[idx];
    }
}

/*!
 * @brief Wrap an angle difference into (-180, 180].
 *
 * @param[in] angle_deg Angle in degrees, |angle_deg| <= 1e6.  Larger or
 *                      non-finite input gives 0 (repeated subtraction
 *                      would never end there).
 *
 * @return The wrapped angle.
 */
static float32_t
wrap180 (float32_t angle_deg)
{
    float32_t angle = 0.0f;

    if ((car_math_is_finite(angle_deg))
        && (car_math_abs(angle_deg) <= WRAP_MAX_DEG))
    {
        /* Remove whole turns in one step, then fix up the rounding.
           Cast: |turns| < 2800, exact in float32_t. */
        angle = angle_deg
                - ((float32_t) car_math_round(angle_deg / FULL_CIRCLE)
                   * FULL_CIRCLE);

        while (angle > HALF_CIRCLE)
        {
            angle -= FULL_CIRCLE;
        }

        while (angle <= -HALF_CIRCLE)
        {
            angle += FULL_CIRCLE;
        }
    }

    return (angle);
}

/*!
 * @brief Replace a non-finite result by the previous value (Rule
 *        5.4.b.v).
 *
 * @param[in] value    New result of a floating-point calculation.
 * @param[in] fallback Last good value.
 *
 * @return value if it is finite, otherwise fallback.
 */
static float32_t
finite_or (float32_t value, float32_t fallback)
{
    return (car_math_is_finite(value) ? value : fallback);
}

/*!
 * @brief Publish the working copy for the other tasks (short DI section).
 */
static void
publish (void)
{
    UINT imask = 0u;

    DI(imask);
    g_st = g_work;
    EI(imask);
}

/*!
 * @brief Filter the acceleration and compute pitch and roll.
 *
 * The car's own longitudinal acceleration (from the encoders) is removed
 * before the tilt is computed, so braking does not look like a slope.
 *
 * @param[in] p_acc  Acceleration X, Y, Z in mg.
 * @param[in] speed  Mean wheel speed in mm/s.
 * @param[in] p_hist Previous step.
 */
static void
update_tilt (float32_t const * p_acc, float32_t speed,
             imu_history_t const * p_hist)
{
    float32_t ax_corr    = 0.0f;
    float32_t lateral_mg = 0.0f;

    g_work.ax_mg += ACCEL_LPF
                    * ((IMU_FORWARD_SIGN * (p_acc[AXIS_X] - g_bias[AXIS_X]))
                       - g_work.ax_mg);
    g_work.ay_mg += ACCEL_LPF
                    * ((IMU_FORWARD_SIGN * (p_acc[AXIS_Y] - g_bias[AXIS_Y]))
                       - g_work.ay_mg);
    g_work.az_mg +=
        ACCEL_LPF * ((p_acc[AXIS_Z] - g_bias[AXIS_Z]) - g_work.az_mg);

    g_work.accel_mm_s2 +=
        DV_LPF * (((speed - p_hist->speed) / DT_S) - g_work.accel_mm_s2);
    g_work.accel_mm_s2 = finite_or(g_work.accel_mm_s2, 0.0f);

    ax_corr    = g_work.ax_mg - (g_work.accel_mm_s2 / MM_S2_PER_MG);
    lateral_mg = car_math_sqrt((g_work.ay_mg * g_work.ay_mg)
                               + (g_work.az_mg * g_work.az_mg));

    g_work.pitch_deg =
        finite_or(car_math_atan2(ax_corr, lateral_mg) * CAR_MATH_DEG_PER_RAD,
                  g_work.pitch_deg);
    g_work.roll_deg = finite_or(car_math_atan2(g_work.ay_mg, g_work.az_mg)
                                    * CAR_MATH_DEG_PER_RAD,
                                g_work.roll_deg);
}

/*!
 * @brief Compass heading and its rate of change.
 *
 * @param[in] p_mag         Magnetic field X, Y, Z (raw counts).
 * @param[in] turn_rate_dps Turn rate from the wheel encoders.
 */
static void
update_heading (float32_t const * p_mag, float32_t turn_rate_dps)
{
    float32_t heading = 0.0f;

    if (gb_mag_cal)
    {
        mag_track(p_mag);
    }

    heading = car_math_atan2(p_mag[AXIS_Y] - g_mag_off[AXIS_Y],
                             p_mag[AXIS_X] - g_mag_off[AXIS_X])
              * CAR_MATH_DEG_PER_RAD;
    heading = finite_or(heading, g_work.heading_deg);
    heading = (heading < 0.0f) ? (heading + FULL_CIRCLE) : heading;

    g_work.mag_rate_dps += RATE_LPF
                           * ((wrap180(heading - g_work.heading_deg) / DT_S)
                              - g_work.mag_rate_dps);
    g_work.heading_deg  = heading;
    g_work.enc_rate_dps = turn_rate_dps;
}

/*!
 * @brief Impact = sudden change of the acceleration vector (jerk), with a
 *        hold-off so one bump counts once.
 *
 * @param[in]     p_acc  Acceleration X, Y, Z in mg.
 * @param[in,out] p_hist Previous step (hold-off timer updated).
 *
 * @return true if an impact was detected.
 */
static bool
detect_impact (float32_t const * p_acc, imu_history_t * p_hist)
{
    float32_t delta_x  = p_acc[AXIS_X] - p_hist->accel[AXIS_X];
    float32_t delta_y  = p_acc[AXIS_Y] - p_hist->accel[AXIS_Y];
    float32_t delta_z  = p_acc[AXIS_Z] - p_hist->accel[AXIS_Z];
    float32_t jerk     = 0.0f;
    bool      b_impact = false;

    jerk               = car_math_sqrt((delta_x * delta_x) + (delta_y * delta_y)
                                       + (delta_z * delta_z));
    p_hist->holdoff_ms = (p_hist->holdoff_ms > IMU_PERIOD_MS)
                             ? (p_hist->holdoff_ms - IMU_PERIOD_MS)
                             : 0u;
    b_impact           = (jerk > IMPACT_JERK_MG) && (0u == p_hist->holdoff_ms);

    if (b_impact)
    {
        g_work.impacts++;
        p_hist->holdoff_ms = IMPACT_HOLDOFF_MS;
    }

    return (b_impact);
}

/*!
 * @brief One 50 Hz processing step on the task-private working copy.
 *
 * @param[in,out] p_hist Values from the previous step (updated).
 */
static void
imu_step (imu_history_t * p_hist)
{
    motion_status_t motion;
    float32_t       acc[IMU_AXES];
    float32_t       mag[IMU_AXES];
    float32_t       speed    = 0.0f;
    bool            b_impact = false;
    uint32_t        axis     = 0u;

    if (!read_sensors(acc, mag))
    {
        g_work.i2c_errors++;
    }
    else
    {
        motion_get_status(&motion);
        speed = 0.5f * (motion.speed_l_mm_s + motion.speed_r_mm_s);

        if (gb_reset_run)
        {
            terrain_hump_init(&g_hump);
            g_work.impacts = 0u;
            gb_reset_run   = false;
        }

        update_tilt(acc, speed, p_hist);
        update_heading(mag, motion.turn_rate_dps);
        b_impact = detect_impact(acc, p_hist);

        terrain_hump_update(&g_hump, g_work.pitch_deg,
                            motion.odo_mm - p_hist->odo);
        g_work.hump = g_hump;
        g_work.event =
            terrain_classify(speed, g_work.accel_mm_s2, motion.turn_rate_dps,
                             g_hump.state, b_impact);

        p_hist->speed = speed;
        p_hist->odo   = motion.odo_mm;

        for (axis = 0u; axis < IMU_AXES; axis++)
        {
            p_hist->accel[axis] = acc[axis];
        }
    }

    publish();
}

/*!
 * @brief IMU task: calibrate, then run the 50 Hz processing step.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
imu_task (INT stacd, void * p_exinf)
{
    imu_history_t history = {0.0f, 0.0f, {0.0f, 0.0f, ONE_G_MG}, 0u};
    UINT          imask   = 0u;

    (void) stacd;
    (void) p_exinf;

    /* Take over the fields imu_init() set, then own the working copy. */
    DI(imask);
    g_work = g_st;
    EI(imask);

    terrain_hump_init(&g_hump);
    calibrate_accel();
    publish();

    for (;;)
    {
        imu_step(&history);
        (void) tk_dly_tsk(IMU_PERIOD_MS);
    }
}

/*** end of file ***/
