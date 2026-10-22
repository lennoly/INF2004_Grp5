/** @file packed.c
 *
 * @brief Pack / unpack functions for the shared data (see packed.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "car_math.h"
#include "bitpack.h"
#include "packed.h"

#define TENTHS       (10.0f)
#define PERMILLE     (1000.0f)
#define UNIT         (1.0f)
#define FULL_TURN    (360.0f)
#define US_PER_TICK  (1000u) /* IR sampler: 1 kHz */
#define HALF_TICK_US (500u)
#define MS_MASK      (0xFFFFu)
#define CLASS_MASK   (0x3u)
#define RANGE_SHIFT  (2u)
#define TIME_SHIFT   (16u)
#define NAV_MASK     (0x7u)
#define SYMBOL_SHIFT (3u)

/* Code 39 data characters, in index order (43 symbols). */
static char const g_code39[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

#define CODE39_COUNT (sizeof(g_code39) - 1u)

/* Vehicle state-change reasons, 5-bit codes.  Index 0 is any other text. */
static char const * const g_reasons[] = {
    "other",
    "boot",
    "start",
    "stop command",
    "spin 360",
    "calibrated",
    "IR contrast too low",
    "line lost",
    "line reacquired",
    "line not found / end",
    "nav done",
    "nav: line not found",
    "obstacle ahead / impact",
    "path clear",
    "obstacle bypassed",
    "bypass: line not found",
    "path blocked",
    "LEFT",
    "RIGHT",
    "STRAIGHT",
    "UTURN",
    "fault: motor stall",
    "fault: encoder",
    "fault: IMU",
    "fault: sonar no echo",
    "sonar stale",
    "watchdog reset"};

#define REASON_COUNT (sizeof(g_reasons) / sizeof(g_reasons[0]))

static uint32_t  s_field(float32_t value, float32_t scale, uint32_t bits);
static uint32_t  u_field(float32_t value, float32_t scale, uint32_t bits);
static float32_t s_value(uint32_t field, uint32_t bits, float32_t scale);
static uint32_t  sat_count(uint32_t value);
static float32_t wrap_180(float32_t angle_deg);

/*!
 * @brief Pack the motion snapshot (motion task, once per cycle).
 *
 * @param[in]  p_in  Full-width status.
 * @param[out] p_out Packed snapshot.
 */
void
packed_motion_pack (motion_status_t const * p_in, packed_motion_t * p_out)
{
    p_out->b_busy    = p_in->b_busy ? 1u : 0u;
    p_out->b_last_ok = p_in->b_last_ok ? 1u : 0u;
    p_out->speed_l   = s_field(p_in->speed_l_mm_s, UNIT, PACKED_SPEED_BITS);
    p_out->speed_r   = s_field(p_in->speed_r_mm_s, UNIT, PACKED_SPEED_BITS);
    p_out->pwm_l     = s_field(p_in->pwm_l_pct, UNIT, PACKED_PWM_BITS);
    p_out->pwm_r     = s_field(p_in->pwm_r_pct, UNIT, PACKED_PWM_BITS);
    p_out->target_l  = s_field(p_in->target_l_mm_s, UNIT, PACKED_SPEED_BITS);
    p_out->target_r  = s_field(p_in->target_r_mm_s, UNIT, PACKED_SPEED_BITS);
    p_out->heading =
        s_field(wrap_180(p_in->heading_deg), TENTHS, PACKED_ANGLE_S_BITS);
    p_out->turn_rate = s_field(p_in->turn_rate_dps, TENTHS, PACKED_TURN_BITS);
    p_out->odo       = s_field(p_in->odo_mm, TENTHS, PACKED_DIST_BITS);
    p_out->dist_l    = s_field(p_in->dist_l_mm, TENTHS, PACKED_DIST_BITS);
    p_out->dist_r    = s_field(p_in->dist_r_mm, TENTHS, PACKED_DIST_BITS);
    p_out->ticks_l   = p_in->ticks_l;
    p_out->ticks_r   = p_in->ticks_r;
}

/*!
 * @brief Unpack a motion snapshot into a reader's private copy.
 *
 * @param[in]  p_in  Packed snapshot.
 * @param[out] p_out Status in engineering units.
 */
void
packed_motion_unpack (packed_motion_t const * p_in, motion_status_t * p_out)
{
    p_out->b_busy        = (0u != p_in->b_busy);
    p_out->b_last_ok     = (0u != p_in->b_last_ok);
    p_out->speed_l_mm_s  = s_value(p_in->speed_l, PACKED_SPEED_BITS, UNIT);
    p_out->speed_r_mm_s  = s_value(p_in->speed_r, PACKED_SPEED_BITS, UNIT);
    p_out->target_l_mm_s = s_value(p_in->target_l, PACKED_SPEED_BITS, UNIT);
    p_out->target_r_mm_s = s_value(p_in->target_r, PACKED_SPEED_BITS, UNIT);
    p_out->pwm_l_pct     = s_value(p_in->pwm_l, PACKED_PWM_BITS, UNIT);
    p_out->pwm_r_pct     = s_value(p_in->pwm_r, PACKED_PWM_BITS, UNIT);
    p_out->ticks_l       = p_in->ticks_l;
    p_out->ticks_r       = p_in->ticks_r;
    p_out->dist_l_mm     = s_value(p_in->dist_l, PACKED_DIST_BITS, TENTHS);
    p_out->dist_r_mm     = s_value(p_in->dist_r, PACKED_DIST_BITS, TENTHS);
    p_out->odo_mm        = s_value(p_in->odo, PACKED_DIST_BITS, TENTHS);
    p_out->heading_deg = s_value(p_in->heading, PACKED_ANGLE_S_BITS, TENTHS);
    p_out->turn_rate_dps = s_value(p_in->turn_rate, PACKED_TURN_BITS, TENTHS);
}

/*!
 * @brief Pack the IMU snapshot (imu task, once per cycle).
 *
 * @param[in]  p_in  Full-width status (the tracker keeps its accumulators).
 * @param[in]  seq   Good-sample counter; the low 8 bits are kept.
 * @param[out] p_out Packed snapshot.
 */
void
packed_imu_pack (imu_status_t const * p_in, uint32_t seq, packed_imu_t * p_out)
{
    /* Casts: enumeration constants are small and non-negative; the masks
       keep them inside their fields. */
    p_out->b_ok       = p_in->b_ok ? 1u : 0u;
    p_out->event      = (uint32_t) p_in->event & bitpack_max_u(3u);
    p_out->hump_state = (uint32_t) p_in->hump.state & bitpack_max_u(2u);
    p_out->humps      = sat_count(p_in->hump.humps);
    p_out->impacts    = sat_count(p_in->impacts);
    p_out->i2c_errors = sat_count(p_in->i2c_errors);
    p_out->pitch      = s_field(p_in->pitch_deg, TENTHS, PACKED_TILT_BITS);
    p_out->roll       = s_field(p_in->roll_deg, TENTHS, PACKED_TILT_BITS);
    p_out->seq        = seq & bitpack_max_u(PACKED_COUNT_BITS);
    p_out->heading    = u_field(p_in->heading_deg, TENTHS, PACKED_HEADING_BITS);
    p_out->mag_rate   = s_field(p_in->mag_rate_dps, TENTHS, PACKED_RATE_BITS);
    p_out->enc_rate   = s_field(p_in->enc_rate_dps, TENTHS, PACKED_RATE_BITS);
    p_out->accel      = s_field(p_in->accel_mm_s2, UNIT, PACKED_ACCEL_BITS);
    p_out->ax         = s_field(p_in->ax_mg, UNIT, PACKED_MG_BITS);
    p_out->ay         = s_field(p_in->ay_mg, UNIT, PACKED_MG_BITS);
    p_out->az         = s_field(p_in->az_mg, UNIT, PACKED_MG_BITS);
    p_out->height = s_field(p_in->hump.height_mm, TENTHS, PACKED_HEIGHT_BITS);
    p_out->peak   = u_field(p_in->hump.peak_mm, TENTHS, PACKED_PEAK_BITS);
    p_out->max_peak = u_field(p_in->hump.max_peak_mm, TENTHS, PACKED_PEAK_BITS);
}

/*!
 * @brief Unpack an IMU snapshot.  The hump tracker's private distances
 *        (active, level, pre-roll) are not shared and read as 0.
 *
 * @param[in]  p_in  Packed snapshot.
 * @param[out] p_out Status in engineering units.
 */
void
packed_imu_unpack (packed_imu_t const * p_in, imu_status_t * p_out)
{
    /* Casts: each field holds a valid enumeration value (packed above). */
    p_out->b_ok             = (0u != p_in->b_ok);
    p_out->event            = (terrain_event_t) p_in->event;
    p_out->hump.state       = (terrain_hump_state_t) p_in->hump_state;
    p_out->hump.humps       = p_in->humps;
    p_out->impacts          = p_in->impacts;
    p_out->i2c_errors       = p_in->i2c_errors;
    p_out->pitch_deg        = s_value(p_in->pitch, PACKED_TILT_BITS, TENTHS);
    p_out->roll_deg         = s_value(p_in->roll, PACKED_TILT_BITS, TENTHS);
    /* Cast: the field is below 2^12, exact in float32_t. */
    p_out->heading_deg      = (float32_t) p_in->heading / TENTHS;
    p_out->mag_rate_dps     = s_value(p_in->mag_rate, PACKED_RATE_BITS, TENTHS);
    p_out->enc_rate_dps     = s_value(p_in->enc_rate, PACKED_RATE_BITS, TENTHS);
    p_out->accel_mm_s2      = s_value(p_in->accel, PACKED_ACCEL_BITS, UNIT);
    p_out->ax_mg            = s_value(p_in->ax, PACKED_MG_BITS, UNIT);
    p_out->ay_mg            = s_value(p_in->ay, PACKED_MG_BITS, UNIT);
    p_out->az_mg            = s_value(p_in->az, PACKED_MG_BITS, UNIT);
    p_out->hump.height_mm   = s_value(p_in->height, PACKED_HEIGHT_BITS, TENTHS);
    /* Casts: the fields are below 2^11, exact in float32_t. */
    p_out->hump.peak_mm     = (float32_t) p_in->peak / TENTHS;
    p_out->hump.max_peak_mm = (float32_t) p_in->max_peak / TENTHS;
    p_out->hump.active_mm   = 0.0f;
    p_out->hump.level_mm    = 0.0f;
    p_out->hump.preroll_mm  = 0.0f;
}

/*!
 * @brief The IMU's good-sample counter (freshness check, risk R1).
 *
 * @param[in] p_in Packed snapshot.
 *
 * @return 0..255; it changes on every good sample.
 */
uint32_t
packed_imu_seq (packed_imu_t const * p_in)
{
    return (p_in->seq);
}

/*!
 * @brief Pack the vehicle status (vehicle task).
 *
 * @param[in]  p_in  Full-width status.
 * @param[out] p_out Packed status.
 */
void
packed_vehicle_pack (vehicle_status_t const * p_in, packed_vehicle_t * p_out)
{
    /* Casts: enumeration constants are small and non-negative; the masks
       keep them inside their fields. */
    p_out->state       = (uint32_t) p_in->state & bitpack_max_u(3u);
    p_out->line_state  = (uint32_t) p_in->line_state & bitpack_max_u(2u);
    p_out->pending_nav = (uint32_t) p_in->pending_nav & bitpack_max_u(3u);
    p_out->last_nav    = (uint32_t) p_in->last_nav & bitpack_max_u(3u);
    p_out->last_action = (uint32_t) p_in->last_action & bitpack_max_u(3u);
    p_out->reason      = packed_reason_code(p_in->p_reason);
    p_out->state_seq   = p_in->state_seq & bitpack_max_u(PACKED_COUNT_BITS);
    p_out->barcodes    = sat_count(p_in->barcodes);
    p_out->obstacles   = sat_count(p_in->obstacles_passed);
    p_out->cruise      = bitpack_clamp_u(p_in->cruise_mm_s, PACKED_CRUISE_BITS);
    p_out->line_error =
        s_field(p_in->line_error, PERMILLE, PACKED_ERROR_BITS);
    p_out->front  = packed_range(p_in->front_mm);
    p_out->run_ms = (p_in->run_ms > bitpack_max_u(PACKED_RUN_BITS))
                        ? bitpack_max_u(PACKED_RUN_BITS)
                        : p_in->run_ms;
}

/*!
 * @brief Unpack the vehicle status.
 *
 * @param[in]  p_in  Packed status.
 * @param[out] p_out Status (p_reason points into the reason table).
 */
void
packed_vehicle_unpack (packed_vehicle_t const * p_in, vehicle_status_t * p_out)
{
    /* Casts: each field holds a valid enumeration value (packed above);
       the cruise speed is below 2^9 and fits int32_t. */
    p_out->state            = (vehicle_state_t) p_in->state;
    p_out->line_state       = (line_follow_state_t) p_in->line_state;
    p_out->pending_nav      = (barcode_decode_nav_t) p_in->pending_nav;
    p_out->last_nav         = (barcode_decode_nav_t) p_in->last_nav;
    p_out->last_action      = (avoidance_action_t) p_in->last_action;
    p_out->p_reason         = packed_reason_text(p_in->reason);
    p_out->state_seq        = p_in->state_seq;
    p_out->barcodes         = p_in->barcodes;
    p_out->obstacles_passed = p_in->obstacles;
    p_out->cruise_mm_s      = (int32_t) p_in->cruise;
    p_out->line_error = s_value(p_in->line_error, PACKED_ERROR_BITS, PERMILLE);
    p_out->front_mm   = packed_range_mm(p_in->front);
    p_out->run_ms     = p_in->run_ms;
}

/*!
 * @brief Pack an obstacle profile (after each scan).
 *
 * @param[in]  p_in  Profile from avoidance_profile().
 * @param[out] p_out Packed profile.
 */
void
packed_profile_pack (avoidance_profile_t const * p_in, packed_profile_t * p_out)
{
    p_out->b_found     = p_in->b_found ? 1u : 0u;
    p_out->closest     = packed_range(p_in->closest_mm);
    p_out->angle = bitpack_clamp_u(p_in->closest_angle, PACKED_SERVO_BITS);
    p_out->left_edge = s_field(p_in->left_edge_mm, UNIT, PACKED_LATERAL_BITS);
    p_out->right_edge =
        s_field(p_in->right_edge_mm, UNIT, PACKED_LATERAL_BITS);
    p_out->width = u_field(p_in->width_mm, UNIT, PACKED_RANGE_BITS);
    p_out->clear_left =
        bitpack_clamp_u(p_in->clear_left_mm, PACKED_RANGE_BITS);
    p_out->centre = s_field(p_in->centre_mm, UNIT, PACKED_LATERAL_BITS);
    p_out->clear_right =
        bitpack_clamp_u(p_in->clear_right_mm, PACKED_RANGE_BITS);
}

/*!
 * @brief Unpack an obstacle profile.
 *
 * @param[in]  p_in  Packed profile.
 * @param[out] p_out Profile in mm and degrees.
 */
void
packed_profile_unpack (packed_profile_t const *  p_in,
                       avoidance_profile_t * p_out)
{
    /* Casts: the unsigned fields are below 2^12, so they fit int32_t and
       are exact in float32_t. */
    p_out->b_found        = (0u != p_in->b_found);
    p_out->closest_mm     = packed_range_mm(p_in->closest);
    p_out->closest_angle  = (int32_t) p_in->angle;
    p_out->left_edge_mm   = s_value(p_in->left_edge, PACKED_LATERAL_BITS, UNIT);
    p_out->right_edge_mm =
        s_value(p_in->right_edge, PACKED_LATERAL_BITS, UNIT);
    p_out->width_mm       = (float32_t) p_in->width;
    p_out->centre_mm      = s_value(p_in->centre, PACKED_LATERAL_BITS, UNIT);
    p_out->clear_left_mm  = (int32_t) p_in->clear_left;
    p_out->clear_right_mm = (int32_t) p_in->clear_right;
}

/*!
 * @brief Build the front sonar word (sonar monitor task).
 *
 * @param[in] cls     Ping class.
 * @param[in] dist_mm Distance, or ULTRASONIC_NO_ECHO.
 * @param[in] now_ms  Kernel milliseconds when the ping finished.
 *
 * @return The word to store with a single write.
 */
packed_front_t
packed_front_make (ultrasonic_class_t cls, int32_t dist_mm, uint32_t now_ms)
{
    /* Cast: enumeration constants are small and non-negative. */
    return (((uint32_t) cls & CLASS_MASK)
            | (packed_range(dist_mm) << RANGE_SHIFT)
            | ((now_ms & MS_MASK) << TIME_SHIFT));
}

/*!
 * @brief Front distance from the sonar word.
 *
 * @param[in] front Sonar word.
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
packed_front_mm (packed_front_t front)
{
    return (packed_range_mm((front >> RANGE_SHIFT)
                            & bitpack_max_u(PACKED_RANGE_BITS)));
}

/*!
 * @brief Ping class from the sonar word.
 *
 * @param[in] front Sonar word.
 *
 * @return VALID, OUT_OF_RANGE or FAULT.
 */
ultrasonic_class_t
packed_front_class (packed_front_t front)
{
    /* Cast: the two bits hold a class written by packed_front_make(). */
    return ((ultrasonic_class_t) (front & CLASS_MASK));
}

/*!
 * @brief Age of the reading (for the stale-reading check).
 *
 * @param[in] front  Sonar word.
 * @param[in] now_ms Kernel milliseconds now.
 *
 * @return Age in ms, 0..65535 (wrap-safe).
 */
uint32_t
packed_front_age_ms (packed_front_t front, uint32_t now_ms)
{
    return ((now_ms - (front >> TIME_SHIFT)) & MS_MASK);
}

/*!
 * @brief Build one bar / space element (IR sampler ISR).
 *
 * @param[in] b_bar    true for a black bar.
 * @param[in] width_us Element width in microseconds.
 *
 * @return The element; widths over 254 ms read as PACKED_WIDTH_OVER.
 */
packed_ir_element_t
packed_ir_element_make (bool b_bar, uint32_t width_us)
{
    uint32_t ticks = (width_us / US_PER_TICK)
                     + (((width_us % US_PER_TICK) >= HALF_TICK_US) ? 1u : 0u);

    ticks = (ticks > PACKED_WIDTH_OVER) ? PACKED_WIDTH_OVER : ticks;

    /* Cast: 1 + 8 bits, below 2^9. */
    return ((packed_ir_element_t) ((b_bar ? 1u : 0u) | (ticks << 1u)));
}

/*!
 * @brief Whether an element is a bar.
 *
 * @param[in] element Packed element.
 *
 * @return true for a black bar, false for a white space.
 */
bool
packed_ir_element_bar (packed_ir_element_t element)
{
    return (0u != (element & 1u));
}

/*!
 * @brief Element width in sampler ticks (1 ms).
 *
 * @param[in] element Packed element.
 *
 * @return 0..254, or PACKED_WIDTH_OVER.
 */
uint32_t
packed_ir_element_ticks (packed_ir_element_t element)
{
    /* Cast: widen the 16-bit element before shifting. */
    return (((uint32_t) element >> 1u) & PACKED_WIDTH_OVER);
}

/*!
 * @brief Build a barcode message for the vehicle task.
 *
 * @param[in] cmd    Navigation command.
 * @param[in] symbol First decoded character.
 *
 * @return The 2-byte message.
 */
packed_barcode_t
packed_barcode_make (barcode_decode_nav_t cmd, char symbol)
{
    /* Casts: the command is a small non-negative enumeration constant and
       the result is below 2^9. */
    return ((packed_barcode_t) (((uint32_t) cmd & NAV_MASK)
                                | (packed_symbol_index(symbol)
                                   << SYMBOL_SHIFT)));
}

/*!
 * @brief Navigation command of a barcode message.
 *
 * @param[in] message Packed message.
 *
 * @return The command.
 */
barcode_decode_nav_t
packed_barcode_cmd (packed_barcode_t message)
{
    /* Cast: the three bits hold a command written by packed_barcode_make. */
    return ((barcode_decode_nav_t) (message & NAV_MASK));
}

/*!
 * @brief Symbol of a barcode message.
 *
 * @param[in] message Packed message.
 *
 * @return The character, or '?' if none.
 */
char
packed_barcode_symbol (packed_barcode_t message)
{
    /* Cast: widen the 16-bit message before shifting. */
    return (packed_symbol_char(((uint32_t) message >> SYMBOL_SHIFT)
                               & bitpack_max_u(PACKED_SYMBOL_BITS)));
}

/*!
 * @brief Code 39 index of a character.
 *
 * @param[in] symbol Character.
 *
 * @return 0..42, or PACKED_SYMBOL_NONE.
 */
uint32_t
packed_symbol_index (char symbol)
{
    uint32_t index  = 0u;
    uint32_t result = PACKED_SYMBOL_NONE;

    for (index = 0u; index < CODE39_COUNT; index++)
    {
        if ((PACKED_SYMBOL_NONE == result) && (g_code39[index] == symbol))
        {
            result = index;
        }
    }

    return (result);
}

/*!
 * @brief Character of a Code 39 index.
 *
 * @param[in] index Index.
 *
 * @return The character, or '?' if out of range.
 */
char
packed_symbol_char (uint32_t index)
{
    return ((index < CODE39_COUNT) ? g_code39[index] : '?');
}

/*!
 * @brief 5-bit code of a vehicle state-change reason.
 *
 * @param[in] p_reason Reason text (may be NULL).
 *
 * @return Table index, or 0 ("other") for text not in the table.
 */
uint32_t
packed_reason_code (char const * p_reason)
{
    uint32_t index  = 0u;
    uint32_t result = 0u;

    for (index = 1u; (NULL != p_reason) && (index < REASON_COUNT); index++)
    {
        if ((0u == result) && (0 == strcmp(p_reason, g_reasons[index])))
        {
            result = index;
        }
    }

    return (result);
}

/*!
 * @brief Text of a reason code.
 *
 * @param[in] code Reason code.
 *
 * @return The text ("other" for 0 or an unknown code).
 */
char const *
packed_reason_text (uint32_t code)
{
    return ((code < REASON_COUNT) ? g_reasons[code] : g_reasons[0]);
}

/*!
 * @brief Encode a distance into a 12-bit range field.
 *
 * @param[in] dist_mm Distance in mm, or a negative "no echo" value.
 *
 * @return 0..4094 mm, or PACKED_RANGE_NONE.
 */
uint32_t
packed_range (int32_t dist_mm)
{
    uint32_t result = PACKED_RANGE_NONE;

    if (dist_mm >= 0)
    {
        result = bitpack_clamp_u(dist_mm, PACKED_RANGE_BITS);
        result = (result >= PACKED_RANGE_NONE) ? (PACKED_RANGE_NONE - 1u)
                                               : result;
    }

    return (result);
}

/*!
 * @brief Decode a 12-bit range field.
 *
 * @param[in] field Range field.
 *
 * @return Distance in mm, or ULTRASONIC_NO_ECHO.
 */
int32_t
packed_range_mm (uint32_t field)
{
    /* Cast: the field is below 2^12, so it fits int32_t. */
    return ((PACKED_RANGE_NONE == (field & PACKED_RANGE_NONE))
                ? ULTRASONIC_NO_ECHO
                : (int32_t) (field & PACKED_RANGE_NONE));
}

/*!
 * @brief Signed measurement to an n-bit two's complement field.
 *
 * @param[in] value Measurement.
 * @param[in] scale Steps per unit.
 * @param[in] bits  Field width.
 *
 * @return Field bits (clamped).
 */
static uint32_t
s_field (float32_t value, float32_t scale, uint32_t bits)
{
    /* Cast: bits <= 24, so the half range fits int32_t. */
    int32_t high = (int32_t) bitpack_max_u(bits - 1u);

    return (bitpack_to_field_s(bitpack_quantise(value, scale, -high - 1, high),
                               bits));
}

/*!
 * @brief Non-negative measurement to an n-bit unsigned field.
 *
 * @param[in] value Measurement (negative values store 0).
 * @param[in] scale Steps per unit.
 * @param[in] bits  Field width.
 *
 * @return Field bits (clamped).
 */
static uint32_t
u_field (float32_t value, float32_t scale, uint32_t bits)
{
    /* Casts: bits <= 24, so the maximum fits int32_t; the quantised value
       is in 0..max, so the conversion keeps it. */
    int32_t high = (int32_t) bitpack_max_u(bits);

    return ((uint32_t) bitpack_quantise(value, scale, 0, high));
}

/*!
 * @brief n-bit two's complement field to a measurement.
 *
 * @param[in] field Field bits.
 * @param[in] bits  Field width.
 * @param[in] scale Steps per unit.
 *
 * @return Measurement in engineering units.
 */
static float32_t
s_value (uint32_t field, uint32_t bits, float32_t scale)
{
    /* Cast: the value is below 2^24 in magnitude, exact in float32_t. */
    return ((float32_t) bitpack_from_field_s(field, bits) / scale);
}

/*!
 * @brief Saturate a counter into 8 bits.
 *
 * @param[in] value Counter.
 *
 * @return 0..255.
 */
static uint32_t
sat_count (uint32_t value)
{
    uint32_t max = bitpack_max_u(PACKED_COUNT_BITS);

    return ((value > max) ? max : value);
}

/*!
 * @brief Wrap an accumulated heading to -180..+180 degrees.
 *
 * @param[in] angle_deg Heading (any size).
 *
 * @return Equivalent angle in -180..+180.
 */
static float32_t
wrap_180 (float32_t angle_deg)
{
    /* Cast: whole turns, far below 2^24 for any real run. */
    float32_t turns = (float32_t) car_math_round(angle_deg / FULL_TURN);

    return (car_math_is_finite(angle_deg) ? (angle_deg - (turns * FULL_TURN))
                                          : 0.0f);
}

/*** end of file ***/
