/** @file frames.c
 *
 * @brief Buddy 1 - compact binary MQTT frames (see frames.h).
 *
 * Pure functions (no kernel calls): tests/host builds every frame, decodes
 * it again and checks each field at its range limits.  Every frame
 * function returns the frame length in bytes, or 0 if the buffer is too
 * small.
 */

#include <stdint.h>
#include <stdbool.h>
#include "car_math.h"
#include "bitpack.h"
#include "packed.h"
#include "frames.h"

#define TYPE_BITS     (4u)
#define VERSION_BITS  (4u)
#define TENTHS        (10.0f)
#define PERMILLE      (1000.0f)
#define MILLI         (1000.0f) /* gains: 3 decimals, as the text format */
#define UNIT          (1.0f)
#define FULL_TURN     (360.0f)
#define TICKS_BITS    (16u) /* low 16 bits of each encoder counter     */
#define ODO_BITS      (20u) /* mm, signed, +-524 m                     */
#define IR_BITS       (10u) /* permille 0..1000                        */
#define UP_BITS       (20u) /* uptime, s (12 days)                     */
#define SEQ_BITS      (16u)
#define WIDE_COUNT    (16u) /* published / dropped / received          */
#define LINK_BITS     (3u)
#define WIFI_BITS     (3u)
#define WHY_BITS      (4u)
#define IP_BITS       (32u)
#define FCOUNT_BITS   (4u) /* per-fault counts, saturating            */
#define REQ_ID_BITS   (16u)
#define KIND_BITS     (4u)
#define ERROR_BITS    (4u)
#define GAIN_BITS     (20u) /* 0..1000.000 in 0.001 steps              */
#define OFFSET_BITS   (17u) /* 0..100.000 in 0.001 steps               */
#define RATE_BITS     (13u) /* 0..5000 ms                              */
#define NAV_ARG_BITS  (3u)  /* nav= 1..4                               */
#define JERK_BITS     (12u) /* impact strength, mg, saturating at 4095 */
#define WIN_BITS      (16u) /* ms                                      */
#define LOAD_BITS     (7u)  /* %                                       */
#define US_BITS       (16u) /* us, saturating                          */
#define ENUM3_BITS    (3u)
#define ENUM2_BITS    (2u)
#define ONE_BIT       (1u)
#define FRAME_STATUS_ON (1u)
#define PID_GAINS     (3u)
#define LINE_GAINS    (3u)

static void      put_header(bitpack_writer_t * p_writer, uint8_t * p_buf,
                            uint32_t size, frames_type_t type);
static uint32_t  finish(bitpack_writer_t const * p_writer);
static void      put_value_s(bitpack_writer_t * p_writer, float32_t value,
                             float32_t scale, uint32_t bits);
static void      put_value_u(bitpack_writer_t * p_writer, float32_t value,
                             float32_t scale, uint32_t bits);
static void      put_count(bitpack_writer_t * p_writer, uint32_t value,
                           uint32_t bits);
static void      put_enum(bitpack_writer_t * p_writer, uint32_t value,
                          uint32_t bits);
static float32_t get_gain(bitpack_reader_t * p_reader, uint32_t bits);
static void      put_telemetry_motion(bitpack_writer_t * p_writer,
                                      frames_telemetry_t const * p_in);
static void      put_telemetry_sensors(bitpack_writer_t * p_writer,
                                       frames_telemetry_t const * p_in);

/*!
 * @brief Telemetry frame (43 bytes; field list in DATA_REPRESENTATION.md).
 *
 * @param[out] p_buf Frame buffer.
 * @param[in]  size  Buffer size.
 * @param[in]  p_in  Snapshot values.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_telemetry (uint8_t * p_buf, uint32_t size,
                  frames_telemetry_t const * p_in)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_TELEMETRY);
    put_count(&writer, p_in->veh.run_ms, PACKED_RUN_BITS);
    /* Casts: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) p_in->veh.state, ENUM3_BITS);
    put_enum(&writer, (uint32_t) p_in->veh.line_state, ENUM2_BITS);
    put_enum(&writer, (uint32_t) p_in->veh.last_nav, ENUM3_BITS);
    put_enum(&writer, (uint32_t) p_in->veh.pending_nav, ENUM3_BITS);
    put_telemetry_motion(&writer, p_in);
    put_telemetry_sensors(&writer, p_in);
    put_count(&writer, p_in->faults, FAULT_BITS);

    return (finish(&writer));
}

/*!
 * @brief Heartbeat frame (26 bytes).
 *
 * @param[out] p_buf Frame buffer.
 * @param[in]  size  Buffer size.
 * @param[in]  p_in  Heartbeat values.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_heartbeat (uint8_t * p_buf, uint32_t size,
                  frames_heartbeat_t const * p_in)
{
    bitpack_writer_t writer;
    uint32_t         idx = 0u;

    put_header(&writer, p_buf, size, FRAME_HEARTBEAT);
    put_count(&writer, p_in->up_s, UP_BITS);
    bitpack_put_u(&writer, p_in->seq, SEQ_BITS); /* wraps */
    put_enum(&writer, p_in->state, ENUM3_BITS);
    put_enum(&writer, p_in->link, LINK_BITS);
    bitpack_put_u(&writer, p_in->published, WIDE_COUNT); /* wrap: deltas */
    bitpack_put_u(&writer, p_in->dropped, WIDE_COUNT);
    bitpack_put_u(&writer, p_in->commands, WIDE_COUNT);
    put_count(&writer, p_in->reconnects, PACKED_COUNT_BITS);
    put_count(&writer, p_in->rejoins, PACKED_COUNT_BITS);
    put_count(&writer, p_in->wifi_restarts, PACKED_COUNT_BITS);
    put_enum(&writer, p_in->wifi_code, WIFI_BITS);
    bitpack_put_u(&writer, p_in->address, IP_BITS);
    put_count(&writer, p_in->attempts, PACKED_COUNT_BITS);
    put_enum(&writer, p_in->why_code, WHY_BITS);
    bitpack_put_u(&writer, p_in->b_imu_ok ? 1u : 0u, ONE_BIT);
    put_count(&writer, p_in->i2c_errors, PACKED_COUNT_BITS);
    put_count(&writer, p_in->faults, FAULT_BITS);

    /* Cast: FAULT_COUNT is a small positive enumeration constant. */
    for (idx = 0u; idx < (uint32_t) FAULT_COUNT; idx++)
    {
        put_count(&writer, p_in->fault_counts[idx], FCOUNT_BITS);
    }

    return (finish(&writer));
}

/*!
 * @brief State-change event (2 bytes).
 *
 * @param[out] p_buf       Frame buffer.
 * @param[in]  size        Buffer size.
 * @param[in]  state       New state.
 * @param[in]  reason_code packed_reason_code() of the reason.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_state (uint8_t * p_buf, uint32_t size, vehicle_state_t state,
              uint32_t reason_code)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_STATE);
    /* Cast: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) state, ENUM3_BITS);
    put_enum(&writer, reason_code, PACKED_REASON_BITS);

    return (finish(&writer));
}

/*!
 * @brief Barcode event (3 bytes).
 *
 * @param[out] p_buf  Frame buffer.
 * @param[in]  size   Buffer size.
 * @param[in]  symbol Decoded character.
 * @param[in]  cmd    Navigation command.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_barcode (uint8_t * p_buf, uint32_t size, char symbol,
                barcode_decode_nav_t cmd)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_BARCODE);
    put_enum(&writer, packed_symbol_index(symbol), PACKED_SYMBOL_BITS);
    /* Cast: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) cmd, ENUM3_BITS);

    return (finish(&writer));
}

/*!
 * @brief Hump event (5 bytes).
 *
 * @param[out] p_buf   Frame buffer.
 * @param[in]  size    Buffer size.
 * @param[in]  humps   Humps counted this run.
 * @param[in]  peak_mm Peak of the hump just finished.
 * @param[in]  max_mm  Highest peak of the run.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_hump (uint8_t * p_buf, uint32_t size, uint32_t humps,
             float32_t peak_mm, float32_t max_mm)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_HUMP);
    put_count(&writer, humps, PACKED_COUNT_BITS);
    put_value_u(&writer, peak_mm, TENTHS, PACKED_PEAK_BITS);
    put_value_u(&writer, max_mm, TENTHS, PACKED_PEAK_BITS);

    return (finish(&writer));
}

/*!
 * @brief Obstacle event after a scan (11 bytes).
 *
 * @param[out] p_buf  Frame buffer.
 * @param[in]  size   Buffer size.
 * @param[in]  p_prof Profile.
 * @param[in]  action Chosen action.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_obstacle (uint8_t * p_buf, uint32_t size,
                 avoidance_profile_t const * p_prof, avoidance_action_t action)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_OBSTACLE);
    bitpack_put_u(&writer, p_prof->b_found ? 1u : 0u, ONE_BIT);
    bitpack_put_u(&writer, packed_range(p_prof->closest_mm), PACKED_RANGE_BITS);
    put_count(&writer,
              bitpack_clamp_u(p_prof->closest_angle, PACKED_SERVO_BITS),
              PACKED_SERVO_BITS);
    put_value_u(&writer, p_prof->width_mm, UNIT, PACKED_RANGE_BITS);
    put_value_s(&writer, p_prof->centre_mm, UNIT, PACKED_LATERAL_BITS);
    put_count(&writer,
              bitpack_clamp_u(p_prof->clear_left_mm, PACKED_RANGE_BITS),
              PACKED_RANGE_BITS);
    put_count(&writer,
              bitpack_clamp_u(p_prof->clear_right_mm, PACKED_RANGE_BITS),
              PACKED_RANGE_BITS);
    /* Cast: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) action, ENUM3_BITS);

    return (finish(&writer));
}

/*!
 * @brief Impact event (2 bytes).
 *
 * @param[out] p_buf   Frame buffer.
 * @param[in]  size    Buffer size.
 * @param[in]  impacts Impacts counted this run.
 * @param[in]  jerk_mg Strength: the largest jerk around the impact, mg.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_impact (uint8_t * p_buf, uint32_t size, uint32_t impacts,
               float32_t jerk_mg)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_IMPACT);
    put_count(&writer, impacts, PACKED_COUNT_BITS);
    put_value_u(&writer, jerk_mg, UNIT, JERK_BITS);

    return (finish(&writer));
}

/*!
 * @brief Command reply (4 bytes).
 *
 * @param[out] p_buf  Frame buffer.
 * @param[in]  size   Buffer size.
 * @param[in]  req_id Request id of the command (low 16 bits are sent).
 * @param[in]  kind   Command, or COMMAND_INVALID.
 * @param[in]  error  FRAMES_OK or the reason it was refused.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_reply (uint8_t * p_buf, uint32_t size, uint32_t req_id,
              command_id_t kind, frames_error_t error)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_REPLY);
    bitpack_put_u(&writer, req_id, REQ_ID_BITS);
    /* Casts: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) kind, KIND_BITS);
    put_enum(&writer, (uint32_t) error, ERROR_BITS);

    return (finish(&writer));
}

/*!
 * @brief Current gains (26 bytes; 3 decimals, as the text commands).
 *
 * @param[out] p_buf Frame buffer.
 * @param[in]  size  Buffer size.
 * @param[in]  p_in  Gains.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_gains (uint8_t * p_buf, uint32_t size, frames_gains_t const * p_in)
{
    bitpack_writer_t writer;
    uint32_t         idx = 0u;

    put_header(&writer, p_buf, size, FRAME_GAINS);

    for (idx = 0u; idx < PID_GAINS; idx++)
    {
        put_value_u(&writer, p_in->pid[idx], MILLI, GAIN_BITS);
    }

    put_value_u(&writer, p_in->ff[0], MILLI, GAIN_BITS);
    put_value_u(&writer, p_in->ff[1], MILLI, OFFSET_BITS);

    for (idx = 0u; idx < LINE_GAINS; idx++)
    {
        put_value_u(&writer, p_in->line[idx], MILLI, GAIN_BITS);
    }

    put_count(&writer, bitpack_clamp_u(p_in->speed_mm_s, PACKED_CRUISE_BITS),
              PACKED_CRUISE_BITS);
    put_count(&writer, p_in->rate_ms, RATE_BITS);

    return (finish(&writer));
}

/*!
 * @brief Timing window summary (25 bytes).
 *
 * @param[out] p_buf Frame buffer.
 * @param[in]  size  Buffer size.
 * @param[in]  p_in  Timing values.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_timing (uint8_t * p_buf, uint32_t size, frames_timing_t const * p_in)
{
    bitpack_writer_t writer;
    uint32_t         idx = 0u;

    put_header(&writer, p_buf, size, FRAME_TIMING);
    put_count(&writer, p_in->window_ms, WIN_BITS);
    put_count(&writer, p_in->load_pct, LOAD_BITS);
    put_count(&writer, p_in->jitter_max_us, US_BITS);
    put_count(&writer, p_in->periods, WIN_BITS);

    for (idx = 0u; idx < FRAMES_TIMING_SLOTS; idx++)
    {
        put_count(&writer, p_in->max_us[idx], US_BITS);
    }

    return (finish(&writer));
}

/*!
 * @brief Online / offline status (2 bytes, retained; also the last will).
 *
 * @param[out] p_buf    Frame buffer.
 * @param[in]  size     Buffer size.
 * @param[in]  b_online true for online.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_status (uint8_t * p_buf, uint32_t size, bool b_online)
{
    bitpack_writer_t writer;

    put_header(&writer, p_buf, size, FRAME_STATUS);
    bitpack_put_u(&writer, b_online ? FRAME_STATUS_ON : 0u, ONE_BIT);

    return (finish(&writer));
}

/*!
 * @brief Encode a command (for the host tests and tools; the car only
 *        decodes them).
 *
 * @param[out] p_buf Frame buffer.
 * @param[in]  size  Buffer size.
 * @param[in]  p_cmd Valid command.
 *
 * @return Frame length, or 0 if it did not fit.
 */
uint32_t
frames_command (uint8_t * p_buf, uint32_t size, command_t const * p_cmd)
{
    bitpack_writer_t writer;
    uint32_t         idx = 0u;

    put_header(&writer, p_buf, size, FRAME_COMMAND);
    /* Cast: enumeration constants are small and non-negative. */
    put_enum(&writer, (uint32_t) p_cmd->kind, KIND_BITS);
    bitpack_put_u(&writer, p_cmd->req_id, REQ_ID_BITS);

    switch (p_cmd->kind)
    {
        case COMMAND_SPEED:
            put_value_u(&writer, p_cmd->arg[0], UNIT, PACKED_CRUISE_BITS);
        break;

        case COMMAND_RATE:
            put_value_u(&writer, p_cmd->arg[0], UNIT, RATE_BITS);
        break;

        case COMMAND_NAV:
            put_value_u(&writer, p_cmd->arg[0], UNIT, NAV_ARG_BITS);
        break;

        case COMMAND_MAP:
            put_value_u(&writer, p_cmd->arg[0], UNIT, NAV_ARG_BITS);
            put_value_u(&writer, p_cmd->arg[1], UNIT, PACKED_SYMBOL_BITS);
        break;

        case COMMAND_FF:
            put_value_u(&writer, p_cmd->arg[0], MILLI, GAIN_BITS);
            put_value_u(&writer, p_cmd->arg[1], MILLI, OFFSET_BITS);
        break;

        case COMMAND_PID:
        case COMMAND_LINE:
            for (idx = 0u; idx < PID_GAINS; idx++)
            {
                put_value_u(&writer, p_cmd->arg[idx], MILLI, GAIN_BITS);
            }
        break;

        default:
            /* No values. */
        break;
    }

    return (finish(&writer));
}

/*!
 * @brief Decode and range-check a command frame received on MQTT.
 *
 * @param[in]  p_buf Received bytes.
 * @param[in]  len   Number of bytes.
 * @param[out] p_cmd Decoded command (kind COMMAND_INVALID if rejected).
 *
 * @return true if it is a valid version-1 command.
 */
bool
frames_parse_command (uint8_t const * p_buf, uint32_t len, command_t * p_cmd)
{
    bitpack_reader_t reader;
    uint32_t         kind = 0u;
    uint32_t         idx  = 0u;
    bool             b_ok = false;

    bitpack_reader_init(&reader, p_buf, len);

    /* Cast: enumeration constants are small and non-negative. */
    b_ok = ((uint32_t) FRAME_COMMAND == bitpack_get_u(&reader, TYPE_BITS))
           && (FRAMES_VERSION == bitpack_get_u(&reader, VERSION_BITS));
    kind          = bitpack_get_u(&reader, KIND_BITS);
    p_cmd->req_id = bitpack_get_u(&reader, REQ_ID_BITS);
    /* Cast: checked against COMMAND_INVALID below by command_check(). */
    p_cmd->kind   = (kind < (uint32_t) COMMAND_INVALID) ? (command_id_t) kind
                                                        : COMMAND_INVALID;
    p_cmd->n_args = command_arg_count(p_cmd->kind);

    switch (p_cmd->kind)
    {
        case COMMAND_SPEED:
        case COMMAND_RATE:
            /* Cast: below 2^13, exact in float32_t. */
            p_cmd->arg[0] = (float32_t) bitpack_get_u(
                &reader, (COMMAND_SPEED == p_cmd->kind) ? PACKED_CRUISE_BITS
                                                         : RATE_BITS);
        break;

        case COMMAND_NAV:
            /* Cast: below 2^3, exact in float32_t. */
            p_cmd->arg[0] = (float32_t) bitpack_get_u(&reader, NAV_ARG_BITS);
        break;

        case COMMAND_MAP:
            /* Casts: below 2^6, exact in float32_t. */
            p_cmd->arg[0] = (float32_t) bitpack_get_u(&reader, NAV_ARG_BITS);
            p_cmd->arg[1] =
                (float32_t) bitpack_get_u(&reader, PACKED_SYMBOL_BITS);
        break;

        case COMMAND_FF:
            p_cmd->arg[0] = get_gain(&reader, GAIN_BITS);
            p_cmd->arg[1] = get_gain(&reader, OFFSET_BITS);
        break;

        case COMMAND_PID:
        case COMMAND_LINE:
            for (idx = 0u; idx < PID_GAINS; idx++)
            {
                p_cmd->arg[idx] = get_gain(&reader, GAIN_BITS);
            }
        break;

        default:
            /* No values. */
        break;
    }

    b_ok = b_ok && (!reader.b_overflow) && (command_check(p_cmd));

    if (!b_ok)
    {
        p_cmd->kind = COMMAND_INVALID;
    }

    return (b_ok);
}

/*!
 * @brief Motion, line and barcode part of the telemetry frame.
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     p_in     Snapshot values.
 */
static void
put_telemetry_motion (bitpack_writer_t * p_writer,
                      frames_telemetry_t const * p_in)
{
    motion_status_t const * p_mot = &p_in->motion;
    uint32_t                idx   = 0u;
    float32_t               turns = 0.0f;

    /* Cast: whole turns, far below 2^24 for any real run. */
    turns = (float32_t) car_math_round(p_mot->heading_deg / FULL_TURN);

    put_value_s(p_writer, p_mot->speed_l_mm_s, UNIT, PACKED_SPEED_BITS);
    put_value_s(p_writer, p_mot->speed_r_mm_s, UNIT, PACKED_SPEED_BITS);
    put_value_s(p_writer, p_mot->target_l_mm_s, UNIT, PACKED_SPEED_BITS);
    put_value_s(p_writer, p_mot->target_r_mm_s, UNIT, PACKED_SPEED_BITS);
    put_value_s(p_writer, p_mot->pwm_l_pct, UNIT, PACKED_PWM_BITS);
    put_value_s(p_writer, p_mot->pwm_r_pct, UNIT, PACKED_PWM_BITS);
    bitpack_put_u(p_writer, p_mot->ticks_l, TICKS_BITS); /* wraps: deltas */
    bitpack_put_u(p_writer, p_mot->ticks_r, TICKS_BITS);
    put_value_s(p_writer, p_mot->odo_mm, UNIT, ODO_BITS);
    put_value_s(p_writer, p_mot->heading_deg - (turns * FULL_TURN), TENTHS,
                PACKED_ANGLE_S_BITS);

    for (idx = 0u; idx < FRAMES_IR_COUNT; idx++)
    {
        put_value_u(p_writer, p_in->ir_norm[idx], PERMILLE, IR_BITS);
    }

    put_value_s(p_writer, p_in->veh.line_error, PERMILLE, PACKED_ERROR_BITS);
    put_count(p_writer, p_in->veh.barcodes, PACKED_COUNT_BITS);
    put_enum(p_writer, packed_symbol_index(p_in->barcode), PACKED_SYMBOL_BITS);
}

/*!
 * @brief IMU, hump and obstacle part of the telemetry frame.
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     p_in     Snapshot values.
 */
static void
put_telemetry_sensors (bitpack_writer_t * p_writer,
                       frames_telemetry_t const * p_in)
{
    imu_status_t const * p_imu = &p_in->imu;

    put_value_s(p_writer, p_imu->pitch_deg, TENTHS, PACKED_TILT_BITS);
    put_value_s(p_writer, p_imu->roll_deg, TENTHS, PACKED_TILT_BITS);
    put_value_u(p_writer, p_imu->heading_deg, TENTHS, PACKED_HEADING_BITS);
    put_value_s(p_writer, p_imu->enc_rate_dps, TENTHS, PACKED_RATE_BITS);
    put_value_s(p_writer, p_imu->hump.height_mm, TENTHS, PACKED_HEIGHT_BITS);
    put_value_u(p_writer, p_imu->hump.peak_mm, TENTHS, PACKED_PEAK_BITS);
    put_value_u(p_writer, p_imu->hump.max_peak_mm, TENTHS, PACKED_PEAK_BITS);
    put_count(p_writer, p_imu->hump.humps, PACKED_COUNT_BITS);
    /* Casts: enumeration constants are small and non-negative. */
    put_enum(p_writer, (uint32_t) p_imu->event, ENUM3_BITS);
    bitpack_put_u(p_writer, packed_range(p_in->veh.front_mm),
                  PACKED_RANGE_BITS);
    put_enum(p_writer, (uint32_t) p_in->veh.last_action, ENUM3_BITS);
    put_count(p_writer, p_in->veh.obstacles_passed, PACKED_COUNT_BITS);
}

/*!
 * @brief Start a frame: clear the buffer and write the header byte.
 *
 * @param[out] p_writer Frame writer.
 * @param[out] p_buf    Frame buffer.
 * @param[in]  size     Buffer size.
 * @param[in]  type     Frame type.
 */
static void
put_header (bitpack_writer_t * p_writer, uint8_t * p_buf, uint32_t size,
            frames_type_t type)
{
    bitpack_writer_init(p_writer, p_buf, size);
    /* Cast: enumeration constants are small and non-negative. */
    bitpack_put_u(p_writer, (uint32_t) type, TYPE_BITS);
    bitpack_put_u(p_writer, FRAMES_VERSION, VERSION_BITS);
}

/*!
 * @brief Frame length, or 0 if any field did not fit.
 *
 * @param[in] p_writer Frame writer.
 *
 * @return Bytes used.
 */
static uint32_t
finish (bitpack_writer_t const * p_writer)
{
    return (p_writer->b_overflow ? 0u : bitpack_writer_bytes(p_writer));
}

/*!
 * @brief Append a signed measurement, scaled and clamped.
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     value    Measurement.
 * @param[in]     scale    Steps per unit.
 * @param[in]     bits     Field width.
 */
static void
put_value_s (bitpack_writer_t * p_writer, float32_t value, float32_t scale,
             uint32_t bits)
{
    /* Cast: bits <= 24, so the half range fits int32_t. */
    int32_t high = (int32_t) bitpack_max_u(bits - 1u);

    bitpack_put_s(p_writer, bitpack_quantise(value, scale, -high - 1, high),
                  bits);
}

/*!
 * @brief Append a non-negative measurement, scaled and clamped.
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     value    Measurement (negative values send 0).
 * @param[in]     scale    Steps per unit.
 * @param[in]     bits     Field width.
 */
static void
put_value_u (bitpack_writer_t * p_writer, float32_t value, float32_t scale,
             uint32_t bits)
{
    /* Casts: bits <= 20, so the maximum fits int32_t; the quantised value
       is in 0..max, so the conversion keeps it. */
    int32_t high = (int32_t) bitpack_max_u(bits);

    bitpack_put_u(p_writer, (uint32_t) bitpack_quantise(value, scale, 0, high),
                  bits);
}

/*!
 * @brief Append a counter, saturating at the field's maximum.
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     value    Counter.
 * @param[in]     bits     Field width.
 */
static void
put_count (bitpack_writer_t * p_writer, uint32_t value, uint32_t bits)
{
    uint32_t max = bitpack_max_u(bits);

    bitpack_put_u(p_writer, (value > max) ? max : value, bits);
}

/*!
 * @brief Append an enumeration value (masked into its field).
 *
 * @param[in,out] p_writer Frame writer.
 * @param[in]     value    Enumeration value.
 * @param[in]     bits     Field width.
 */
static void
put_enum (bitpack_writer_t * p_writer, uint32_t value, uint32_t bits)
{
    bitpack_put_u(p_writer, value, bits);
}

/*!
 * @brief Read a gain sent in 0.001 steps.
 *
 * @param[in,out] p_reader Frame reader.
 * @param[in]     bits     Field width.
 *
 * @return The gain.
 */
static float32_t
get_gain (bitpack_reader_t * p_reader, uint32_t bits)
{
    /* Cast: below 2^20, exact in float32_t. */
    return ((float32_t) bitpack_get_u(p_reader, bits) / MILLI);
}

/*** end of file ***/
