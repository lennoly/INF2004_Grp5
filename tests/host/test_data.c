/** @file test_data.c
 *
 * @brief Host tests for the minimum-bit data representation (Change
 *        Requirement 1): every pack / unpack pair at its range limits,
 *        saturation instead of wrapping, the wire bit order, the length of
 *        every MQTT frame, and the command frames in both directions.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bitpack.h"
#include "packed.h"
#include "frames.h"
#include "mqtt_bridge.h"
#include "test_data.h"

#define TOL_TENTH (0.051f) /* one 0.1 step, rounded */
#define TOL_UNIT  (0.51f)
#define TOL_MILLI (0.00051f)

static void (*gp_check)(bool b_ok, char const * p_what) = NULL;

static bool near(float32_t a_value, float32_t b_value, float32_t tolerance);
static void test_bitpack(void);
static void test_sizes(void);
static void test_motion(void);
static void test_imu(void);
static void test_vehicle(void);
static void test_small_words(void);
static void test_frame_lengths(void);
static void test_command_frames(void);

/*!
 * @brief Run every data-representation test.
 *
 * @param[in] p_check The test runner's check function.
 */
void
test_data (void (*p_check)(bool b_ok, char const * p_what))
{
    gp_check = p_check;
    test_bitpack();
    test_sizes();
    test_motion();
    test_imu();
    test_vehicle();
    test_small_words();
    test_frame_lengths();
    test_command_frames();
    (void) printf("data: packed snapshots and binary MQTT frames OK\n");
}

static bool
near (float32_t a_value, float32_t b_value, float32_t tolerance)
{
    return (fabsf(a_value - b_value) <= tolerance);
}

static void
test_bitpack (void)
{
    uint8_t          buf[8];
    bitpack_writer_t writer;
    bitpack_reader_t reader;

    gp_check((1u == bitpack_max_u(1u)) && (0xFFFFFFFFu == bitpack_max_u(32u)),
             "bitpack: field maxima");
    gp_check((0u == bitpack_clamp_u(-5, 8u)) && (255u == bitpack_clamp_u(300, 8u)),
             "bitpack: unsigned fields saturate, never wrap");
    gp_check((1023 == bitpack_clamp_s(5000, 11u))
                 && (-1024 == bitpack_clamp_s(-5000, 11u)),
             "bitpack: signed fields saturate");
    gp_check((-1024 == bitpack_from_field_s(bitpack_to_field_s(-1024, 11u), 11u))
                 && (-1 == bitpack_from_field_s(bitpack_to_field_s(-1, 11u), 11u))
                 && (0 == bitpack_from_field_s(bitpack_to_field_s(0, 11u), 11u))
                 && (1023
                     == bitpack_from_field_s(bitpack_to_field_s(1023, 11u), 11u)),
             "bitpack: two's complement round trip at the limits");
    gp_check((2047 == bitpack_quantise(1e9f, 10.0f, -2048, 2047))
                 && (0 == bitpack_quantise(NAN, 10.0f, 0, 10)),
             "bitpack: quantise clamps before converting");

    bitpack_writer_init(&writer, buf, sizeof(buf));
    bitpack_put_u(&writer, 1u, 1u);
    bitpack_put_u(&writer, 5u, 3u);
    bitpack_put_s(&writer, -300, 11u);
    bitpack_put_u(&writer, 0xBEEFu, 16u);
    bitpack_put_u(&writer, 0xDEADBEEFu, 32u);
    gp_check((!writer.b_overflow) && (8u == bitpack_writer_bytes(&writer)),
             "bitpack: 63 bits fill 8 bytes");
    gp_check((0x0Bu == (buf[0] & 0x0Fu)), "bitpack: least significant bit first");
    bitpack_put_u(&writer, 3u, 2u);
    gp_check(writer.b_overflow, "bitpack: writing past the buffer is flagged");

    bitpack_reader_init(&reader, buf, sizeof(buf));
    gp_check((1u == bitpack_get_u(&reader, 1u)) && (5u == bitpack_get_u(&reader, 3u))
                 && (-300 == bitpack_get_s(&reader, 11u))
                 && (0xBEEFu == bitpack_get_u(&reader, 16u))
                 && (0xDEADBEEFu == bitpack_get_u(&reader, 32u))
                 && (!reader.b_overflow),
             "bitpack: every field reads back");
    (void) bitpack_get_u(&reader, 2u);
    gp_check(reader.b_overflow, "bitpack: reading past a short frame is flagged");
}

static void
test_sizes (void)
{
    gp_check(32u == sizeof(packed_motion_t), "size: motion snapshot 32 B (was 56)");
    gp_check(28u == sizeof(packed_imu_t), "size: IMU snapshot 28 B (was 84)");
    gp_check(16u == sizeof(packed_vehicle_t), "size: vehicle status 16 B (was 48)");
    gp_check(16u == sizeof(packed_profile_t), "size: obstacle profile 16 B (was 36)");
    gp_check((2u == sizeof(packed_ir_element_t)) && (2u == sizeof(packed_barcode_t))
                 && (4u == sizeof(packed_front_t)),
             "size: IR element 2 B, barcode message 2 B, front word 4 B");
    gp_check(FRAMES_MAX_BYTES <= MQTT_BRIDGE_PAYLOAD_MAX,
             "size: every frame fits one TX slot");
}

static void
test_motion (void)
{
    motion_status_t in;
    motion_status_t out;
    packed_motion_t packed;

    memset(&in, 0, sizeof(in));
    in.b_busy        = true;
    in.b_last_ok     = false;
    in.speed_l_mm_s  = 123.4f;
    in.speed_r_mm_s  = -600.0f;
    in.target_l_mm_s = 5000.0f; /* beyond the field: saturates */
    in.target_r_mm_s = -180.0f;
    in.pwm_l_pct     = -100.0f;
    in.pwm_r_pct     = 55.4f;
    in.heading_deg   = 370.0f; /* accumulated: wraps to +10 */
    in.turn_rate_dps = -12.34f;
    in.odo_mm        = 12345.67f;
    in.dist_l_mm     = -50.0f;
    in.dist_r_mm     = 12400.0f;
    in.ticks_l       = 0xFFFFFFF0u;
    in.ticks_r       = 7u;
    packed_motion_pack(&in, &packed);
    packed_motion_unpack(&packed, &out);
    gp_check(out.b_busy && (!out.b_last_ok), "motion: flags");
    gp_check(near(out.speed_l_mm_s, 123.4f, TOL_UNIT)
                 && near(out.speed_r_mm_s, -600.0f, TOL_UNIT)
                 && near(out.target_l_mm_s, 1023.0f, TOL_UNIT)
                 && near(out.target_r_mm_s, -180.0f, TOL_UNIT),
             "motion: speeds to 1 mm/s, out-of-range saturates");
    gp_check(near(out.pwm_l_pct, -100.0f, TOL_UNIT) && near(out.pwm_r_pct, 55.0f, TOL_UNIT),
             "motion: duty to 1 %");
    gp_check(near(out.heading_deg, 10.0f, TOL_TENTH)
                 && near(out.turn_rate_dps, -12.3f, TOL_TENTH),
             "motion: heading wrapped to +-180, rates to 0.1 deg");
    gp_check(near(out.odo_mm, 12345.7f, TOL_TENTH) && near(out.dist_l_mm, -50.0f, TOL_TENTH)
                 && near(out.dist_r_mm, 12400.0f, TOL_TENTH),
             "motion: distances to 0.1 mm");
    gp_check((0xFFFFFFF0u == out.ticks_l) && (7u == out.ticks_r),
             "motion: ISR counters stay full 32 bits");
}

static void
test_imu (void)
{
    imu_status_t in;
    imu_status_t out;
    packed_imu_t packed;

    memset(&in, 0, sizeof(in));
    in.b_ok             = true;
    in.event            = EVT_CLIMBING;
    in.hump.state       = HUMP_CLIMBING;
    in.hump.humps       = 300u; /* saturates at 255 */
    in.hump.height_mm   = -12.34f;
    in.hump.peak_mm     = 40.04f;
    in.hump.max_peak_mm = 300.0f; /* saturates at 204.7 */
    in.impacts          = 3u;
    in.i2c_errors       = 9u;
    in.pitch_deg        = 12.34f;
    in.roll_deg         = -90.0f;
    in.heading_deg      = 359.94f;
    in.mag_rate_dps     = 250.0f;
    in.enc_rate_dps     = -0.5f; /* small drift stays visible */
    in.accel_mm_s2      = -2500.0f;
    in.ax_mg            = 1999.0f;
    in.ay_mg            = -3000.0f; /* saturates at -2048 */
    in.az_mg            = 1000.0f;
    packed_imu_pack(&in, 257u, &packed);
    packed_imu_unpack(&packed, &out);
    gp_check(out.b_ok && (EVT_CLIMBING == out.event) && (HUMP_CLIMBING == out.hump.state)
                 && (255u == out.hump.humps) && (3u == out.impacts)
                 && (9u == out.i2c_errors) && (1u == packed_imu_seq(&packed)),
             "imu: flags, enumerations, saturating counters, 8-bit sequence");
    gp_check(near(out.pitch_deg, 12.3f, TOL_TENTH) && near(out.roll_deg, -90.0f, TOL_TENTH)
                 && near(out.heading_deg, 359.9f, TOL_TENTH)
                 && near(out.mag_rate_dps, 250.0f, TOL_TENTH)
                 && near(out.enc_rate_dps, -0.5f, TOL_TENTH),
             "imu: angles and rates to 0.1 deg");
    gp_check(near(out.hump.height_mm, -12.3f, TOL_TENTH)
                 && near(out.hump.peak_mm, 40.0f, TOL_TENTH)
                 && near(out.hump.max_peak_mm, 204.7f, TOL_TENTH),
             "imu: hump heights to 0.1 mm, saturating");
    gp_check(near(out.accel_mm_s2, -2500.0f, TOL_UNIT) && near(out.ax_mg, 1999.0f, TOL_UNIT)
                 && near(out.ay_mg, -2048.0f, TOL_UNIT) && near(out.az_mg, 1000.0f, TOL_UNIT),
             "imu: accelerations to 1 mg, saturating");
}

static void
test_vehicle (void)
{
    vehicle_status_t     in;
    vehicle_status_t     out;
    packed_vehicle_t     packed;
    avoidance_profile_t  prof_in;
    avoidance_profile_t  prof_out;
    packed_profile_t     prof_packed;

    memset(&in, 0, sizeof(in));
    in.state            = VS_STOPPED;
    in.state_seq        = 258u;
    in.line_state       = LINE_JUNCTION;
    in.line_error       = -0.7531f;
    in.pending_nav      = NAV_UTURN;
    in.last_nav         = NAV_LEFT;
    in.barcodes         = 4u;
    in.last_action      = AVOID_REVERSE;
    in.obstacles_passed = 999u;
    in.front_mm         = ULTRASONIC_NO_ECHO;
    in.cruise_mm_s      = 300;
    in.run_ms           = 123456u;
    in.p_reason         = "path blocked";
    packed_vehicle_pack(&in, &packed);
    packed_vehicle_unpack(&packed, &out);
    gp_check((VS_STOPPED == out.state) && (2u == out.state_seq)
                 && (LINE_JUNCTION == out.line_state) && (NAV_UTURN == out.pending_nav)
                 && (NAV_LEFT == out.last_nav) && (AVOID_REVERSE == out.last_action),
             "vehicle: state machine fields in 2-3 bits");
    gp_check((4u == out.barcodes) && (255u == out.obstacles_passed)
                 && (300 == out.cruise_mm_s) && (123456u == out.run_ms)
                 && (ULTRASONIC_NO_ECHO == out.front_mm)
                 && near(out.line_error, -0.753f, TOL_MILLI),
             "vehicle: counters, cruise, run time, no-echo marker, error");
    gp_check(0 == strcmp(out.p_reason, "path blocked"),
             "vehicle: reason text travels as a 5-bit code");
    in.p_reason = "a reason nobody listed";
    packed_vehicle_pack(&in, &packed);
    packed_vehicle_unpack(&packed, &out);
    gp_check(0 == strcmp(out.p_reason, "other"), "vehicle: unknown reason is other");

    memset(&prof_in, 0, sizeof(prof_in));
    prof_in.b_found        = true;
    prof_in.closest_mm     = 196;
    prof_in.closest_angle  = 135;
    prof_in.left_edge_mm   = -188.0f;
    prof_in.right_edge_mm  = -52.0f;
    prof_in.width_mm       = 137.0f;
    prof_in.centre_mm      = -120.0f;
    prof_in.clear_left_mm  = 1000;
    prof_in.clear_right_mm = 5000; /* saturates */
    packed_profile_pack(&prof_in, &prof_packed);
    packed_profile_unpack(&prof_packed, &prof_out);
    gp_check(prof_out.b_found && (196 == prof_out.closest_mm) && (135 == prof_out.closest_angle)
                 && near(prof_out.left_edge_mm, -188.0f, TOL_UNIT)
                 && near(prof_out.right_edge_mm, -52.0f, TOL_UNIT)
                 && near(prof_out.width_mm, 137.0f, TOL_UNIT)
                 && near(prof_out.centre_mm, -120.0f, TOL_UNIT)
                 && (1000 == prof_out.clear_left_mm) && (4095 == prof_out.clear_right_mm),
             "profile: every field in mm, saturating");
}

static void
test_small_words (void)
{
    packed_front_t      front   = packed_front_make(ULTRASONIC_VALID, 250, 1000u);
    packed_ir_element_t element = packed_ir_element_make(true, 16700u);

    gp_check((250 == packed_front_mm(front))
                 && (ULTRASONIC_VALID == packed_front_class(front))
                 && (100u == packed_front_age_ms(front, 1100u)),
             "front word: distance, class and age");
    front = packed_front_make(ULTRASONIC_OUT_OF_RANGE, ULTRASONIC_NO_ECHO, 65530u);
    gp_check((ULTRASONIC_NO_ECHO == packed_front_mm(front))
                 && (ULTRASONIC_OUT_OF_RANGE == packed_front_class(front))
                 && (16u == packed_front_age_ms(front, 65546u)),
             "front word: no-echo marker, age across the 16-bit wrap");
    gp_check((ULTRASONIC_NO_ECHO == packed_front_mm(PACKED_FRONT_NONE))
                 && (4094 == packed_front_mm(packed_front_make(ULTRASONIC_VALID, 9999, 0u))),
             "front word: no reading yet; long ranges saturate below the marker");
    gp_check(packed_ir_element_bar(element) && (17u == packed_ir_element_ticks(element)),
             "IR element: bar flag and width rounded to 1 ms ticks");
    element = packed_ir_element_make(false, 400000u);
    gp_check((!packed_ir_element_bar(element))
                 && (PACKED_WIDTH_OVER == packed_ir_element_ticks(element)),
             "IR element: over-range width is the 255 marker");
    gp_check((NAV_LEFT == packed_barcode_cmd(packed_barcode_make(NAV_LEFT, 'A')))
                 && ('A' == packed_barcode_symbol(packed_barcode_make(NAV_LEFT, 'A')))
                 && ('?' == packed_barcode_symbol(packed_barcode_make(NAV_NONE, '#'))),
             "barcode message: command and Code 39 symbol in 9 bits");
    gp_check((42u == packed_symbol_index('%')) && ('Z' == packed_symbol_char(35u)),
             "Code 39 index table");
}

static void
test_frame_lengths (void)
{
    uint8_t             buf[FRAMES_MAX_BYTES];
    frames_telemetry_t  tel;
    frames_heartbeat_t  hb;
    frames_gains_t      gains;
    frames_timing_t     timing;
    avoidance_profile_t prof;

    memset(&tel, 0, sizeof(tel));
    memset(&hb, 0, sizeof(hb));
    memset(&gains, 0, sizeof(gains));
    memset(&timing, 0, sizeof(timing));
    memset(&prof, 0, sizeof(prof));
    tel.barcode = 'A';
    tel.veh.state = VS_LINE_FOLLOW;
    tel.veh.run_ms = 0x00ABCDEFu;
    gp_check(43u == frames_telemetry(buf, sizeof(buf), &tel),
             "frame: telemetry 43 B (JSON was about 268 B)");
    gp_check((0x11u == buf[0]) && (0xEFu == buf[1]) && (0xCDu == buf[2])
                 && (0xABu == buf[3]) && (2u == (buf[4] & 0x7u)),
             "frame: header (type 1, version 1), then run time and state");
    gp_check(26u == frames_heartbeat(buf, sizeof(buf), &hb),
             "frame: heartbeat 26 B (JSON was about 208 B)");
    gp_check(2u == frames_state(buf, sizeof(buf), VS_STOPPED, 3u), "frame: state 2 B");
    gp_check(3u == frames_barcode(buf, sizeof(buf), 'B', NAV_RIGHT), "frame: barcode 3 B");
    gp_check(5u == frames_hump(buf, sizeof(buf), 2u, 40.0f, 41.0f), "frame: hump 5 B");
    gp_check(11u == frames_obstacle(buf, sizeof(buf), &prof, AVOID_TURN_LEFT),
             "frame: obstacle 11 B");
    gp_check(2u == frames_impact(buf, sizeof(buf), 1u), "frame: impact 2 B");
    gp_check(4u == frames_reply(buf, sizeof(buf), 7u, COMMAND_SPEED, FRAMES_OK),
             "frame: reply 4 B");
    gp_check(24u == frames_gains(buf, sizeof(buf), &gains), "frame: gains 24 B");
    gp_check(24u == frames_timing(buf, sizeof(buf), &timing), "frame: timing 24 B");
    gp_check((2u == frames_status(buf, sizeof(buf), true)) && (0x1Bu == buf[0])
                 && (0x01u == buf[1]),
             "frame: status 2 B, same bytes as mqtt_lwip.c");
    gp_check(0u == frames_telemetry(buf, 10u, &tel), "frame: too small a buffer gives 0");
}

static void
test_command_frames (void)
{
    command_t cmd;
    command_t out;
    uint8_t   buf[MQTT_BRIDGE_CMD_MAX];
    uint32_t  len = 0u;

    memset(&cmd, 0, sizeof(cmd));
    cmd.kind   = COMMAND_PID;
    cmd.n_args = 3u;
    cmd.req_id = 70000u; /* low 16 bits travel */
    cmd.arg[0] = 0.08f;
    cmd.arg[1] = 0.6f;
    cmd.arg[2] = 1000.0f;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check((len <= MQTT_BRIDGE_CMD_MAX) && frames_parse_command(buf, len, &out)
                 && (COMMAND_PID == out.kind) && ((70000u & 0xFFFFu) == out.req_id)
                 && near(out.arg[0], 0.08f, TOL_MILLI) && near(out.arg[1], 0.6f, TOL_MILLI)
                 && near(out.arg[2], 1000.0f, TOL_MILLI),
             "command: pid= round trip, 3 decimals kept");
    cmd.kind   = COMMAND_FF;
    cmd.n_args = 2u;
    cmd.arg[0] = 0.1f;
    cmd.arg[1] = 18.0f;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check(frames_parse_command(buf, len, &out) && (COMMAND_FF == out.kind)
                 && near(out.arg[0], 0.1f, TOL_MILLI) && near(out.arg[1], 18.0f, TOL_MILLI),
             "command: ff= round trip");
    cmd.kind   = COMMAND_SPEED;
    cmd.n_args = 1u;
    cmd.arg[0] = 150.0f;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check((5u == len) && frames_parse_command(buf, len, &out)
                 && (COMMAND_SPEED == out.kind) && near(out.arg[0], 150.0f, TOL_UNIT),
             "command: speed= in 5 bytes (text was 9)");
    cmd.arg[0] = 400.0f;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check(!frames_parse_command(buf, len, &out) && (COMMAND_INVALID == out.kind),
             "command: out-of-range speed rejected like the text parser");
    cmd.kind   = COMMAND_RATE;
    cmd.arg[0] = 20.0f;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check(!frames_parse_command(buf, len, &out), "command: rate 20 ms rejected");
    cmd.kind   = COMMAND_STOP;
    cmd.n_args = 0u;
    len        = frames_command(buf, sizeof(buf), &cmd);
    gp_check((4u == len) && frames_parse_command(buf, len, &out) && (COMMAND_STOP == out.kind),
             "command: stop in 4 bytes");
    gp_check(!frames_parse_command(buf, 2u, &out), "command: truncated frame rejected");
    buf[0] = 0x2Cu; /* version 2 */
    gp_check(!frames_parse_command(buf, len, &out), "command: unknown version rejected");
    buf[0] = 0x11u; /* a telemetry frame */
    gp_check(!frames_parse_command(buf, len, &out), "command: other frame types rejected");
    buf[0] = 0x1Cu;
    buf[1] = 0x0Fu; /* kind 15 */
    gp_check(!frames_parse_command(buf, len, &out), "command: unknown kind rejected");
}

/*** end of file ***/
