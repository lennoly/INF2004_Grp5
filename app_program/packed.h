/** @file packed.h
 *
 * @brief Packed (minimum-bit) forms of the data shared between tasks and
 *        between an ISR and a task (Change Requirement 1).
 *
 * Tribunal 1 decisions applied here:
 * - Pack at the interfaces.  Each owner keeps its working values at full
 *   width inside its own task (PID integrators, the hump accumulator,
 *   odometry), packs one snapshot per cycle into the struct below, and
 *   readers unpack a private copy.  Packing therefore adds no rounding that
 *   accumulates.
 * - One documented format with shared encode / decode functions (risk R3):
 *   every field's range, unit and width is in docs/DATA_REPRESENTATION.md
 *   and in the comments below.
 * - Bit-fields are unsigned only.  Signed values are stored as n-bit two's
 *   complement (bitpack_to_field_s / bitpack_from_field_s) and every value
 *   is clamped into its field before it is stored, so a reading outside the
 *   range saturates instead of wrapping.
 * - Safe access: a packed struct is written by one task only, built in a
 *   local copy and stored with one interrupt-protected copy (single-core
 *   build, A13).  A word that an ISR or another task reads without a lock
 *   (packed_front_t, packed_ir_element_t) is written with a single store.
 *
 * Pure functions: no kernel calls, so tests/host checks every pack/unpack
 * pair at its range limits.
 */

#ifndef PACKED_H
#define PACKED_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"
#include "motion.h"
#include "imu.h"
#include "vehicle.h"
#include "avoidance.h"
#include "ultrasonic.h"
#include "barcode_decode.h"

/* Field widths (bits).  Ranges and units: docs/DATA_REPRESENTATION.md. */
#define PACKED_SPEED_BITS     (11u) /* mm/s, signed, +-1023              */
#define PACKED_PWM_BITS       (8u)  /* %, signed, -100..+100             */
#define PACKED_ANGLE_S_BITS   (12u) /* 0.1 deg, signed, -180.0..+179.9   */
#define PACKED_HEADING_BITS   (12u) /* 0.1 deg, unsigned, 0..359.9       */
#define PACKED_RATE_BITS      (13u) /* 0.1 deg/s, signed, +-409.5        */
#define PACKED_TURN_BITS      (14u) /* 0.1 deg/s, signed, +-819.1        */
#define PACKED_DIST_BITS      (24u) /* 0.1 mm, signed, +-838 m           */
#define PACKED_TILT_BITS      (11u) /* 0.1 deg, signed, +-102.3          */
#define PACKED_ACCEL_BITS     (13u) /* mm/s^2, signed, +-4095            */
#define PACKED_MG_BITS        (12u) /* mg, signed, +-2047                */
#define PACKED_HEIGHT_BITS    (12u) /* 0.1 mm, signed, +-204.7           */
#define PACKED_PEAK_BITS      (11u) /* 0.1 mm, unsigned, 0..204.7        */
#define PACKED_COUNT_BITS     (8u)  /* saturating counters, 0..255       */
#define PACKED_RANGE_BITS     (12u) /* mm, 0..4094; 4095 = no echo       */
#define PACKED_ERROR_BITS     (11u) /* permille, signed, -1000..+1000    */
#define PACKED_CRUISE_BITS    (9u)  /* mm/s, 0..511 (commands 50..300)   */
#define PACKED_RUN_BITS       (24u) /* ms, saturates after 4.6 h         */
#define PACKED_LATERAL_BITS   (13u) /* mm, signed, +-4095                */
#define PACKED_SERVO_BITS     (8u)  /* deg, 0..180                       */
#define PACKED_WIDTH_BITS     (8u)  /* 1 ms sampler ticks, 255 = over    */
#define PACKED_SYMBOL_BITS    (6u)  /* Code 39 index 0..42, 63 = none    */
#define PACKED_REASON_BITS    (5u)  /* vehicle_reason table index        */

#define PACKED_RANGE_NONE     (4095u) /* "no echo" marker                  */
#define PACKED_WIDTH_OVER     (255u)  /* bar or space longer than 254 ms   */
#define PACKED_SYMBOL_NONE    (63u)
#define PACKED_FRONT_NONE     (PACKED_RANGE_NONE << 2u) /* no reading yet */

/* Motion snapshot (Buddy 2): written by the motion task every 20 ms, read
   by the vehicle, imu and telemetry tasks.  32 bytes instead of 56. */
typedef struct
{
    uint32_t b_busy    : 1;
    uint32_t b_last_ok : 1;
    uint32_t speed_l   : 11; /* PACKED_SPEED_BITS */
    uint32_t speed_r   : 11;
    uint32_t pwm_l     : 8;  /* PACKED_PWM_BITS   */
    uint32_t pwm_r     : 8;
    uint32_t target_l  : 11;
    uint32_t target_r  : 11;
    uint32_t heading   : 12; /* PACKED_ANGLE_S_BITS, wrapped to +-180 */
    uint32_t turn_rate : 14; /* PACKED_TURN_BITS  */
    uint32_t odo       : 24; /* PACKED_DIST_BITS  */
    uint32_t dist_l    : 24;
    uint32_t dist_r    : 24;
    uint32_t ticks_l; /* raw ISR edge counter: full 32 bits by design; */
    uint32_t ticks_r; /* readers take deltas by unsigned subtraction   */
} packed_motion_t;

/* IMU snapshot (Buddy 4): written by the imu task every 20 ms.  The hump
   tracker's accumulators stay full width inside the task; only the
   published values are packed.  28 bytes instead of 84. */
typedef struct
{
    uint32_t b_ok       : 1; /* last read succeeded (freshness)       */
    uint32_t event      : 3; /* terrain_event_t, 8 values             */
    uint32_t hump_state : 2; /* terrain_hump_state_t, 3 values        */
    uint32_t humps      : 8; /* saturating                            */
    uint32_t impacts    : 8; /* saturating; vehicle detects increases */
    uint32_t i2c_errors : 8; /* saturating                            */
    uint32_t pitch      : 11; /* PACKED_TILT_BITS                     */
    uint32_t roll       : 11;
    uint32_t seq        : 8;  /* +1 per good sample: freshness (R1)   */
    uint32_t heading    : 12; /* PACKED_HEADING_BITS                  */
    uint32_t mag_rate   : 13; /* PACKED_RATE_BITS                     */
    uint32_t enc_rate   : 13;
    uint32_t accel      : 13; /* PACKED_ACCEL_BITS                    */
    uint32_t ax         : 12; /* PACKED_MG_BITS                       */
    uint32_t ay         : 12;
    uint32_t az         : 12;
    uint32_t height     : 12; /* PACKED_HEIGHT_BITS                   */
    uint32_t peak       : 11; /* PACKED_PEAK_BITS                     */
    uint32_t max_peak   : 11;
} packed_imu_t;

/* Vehicle status (Buddy 5): written by the vehicle task, read by the
   telemetry task.  16 bytes instead of 48. */
typedef struct
{
    uint32_t state       : 3; /* vehicle_state_t, 7 values       */
    uint32_t line_state  : 2; /* line_follow_state_t, 3 values   */
    uint32_t pending_nav : 3; /* barcode_decode_nav_t, 5 values  */
    uint32_t last_nav    : 3;
    uint32_t last_action : 3; /* avoidance_action_t, 5 values    */
    uint32_t reason      : 5; /* PACKED_REASON_BITS              */
    uint32_t state_seq   : 8; /* wraps; readers only test change */
    uint32_t barcodes    : 8; /* saturating                      */
    uint32_t obstacles   : 8; /* saturating                      */
    uint32_t cruise      : 9; /* PACKED_CRUISE_BITS              */
    uint32_t line_error  : 11; /* PACKED_ERROR_BITS              */
    uint32_t front       : 12; /* PACKED_RANGE_BITS              */
    uint32_t run_ms      : 24; /* PACKED_RUN_BITS                */
} packed_vehicle_t;

/* Last obstacle profile (Buddy 5): written after each scan, read by the
   telemetry task.  16 bytes instead of 36. */
typedef struct
{
    uint32_t b_found     : 1;
    uint32_t closest     : 12; /* PACKED_RANGE_BITS   */
    uint32_t angle       : 8;  /* PACKED_SERVO_BITS   */
    uint32_t left_edge   : 13; /* PACKED_LATERAL_BITS */
    uint32_t right_edge  : 13;
    uint32_t width       : 12; /* mm, 0..4095         */
    uint32_t clear_left  : 12;
    uint32_t centre      : 13; /* PACKED_LATERAL_BITS */
    uint32_t clear_right : 12;
} packed_profile_t;

/* Front sonar reading (Buddy 5): one 32-bit word written by the sonar
   monitor with a single store and read by the vehicle task without a lock
   (an aligned word access is atomic on the Cortex-M0+).
   Bits 0-1 class, 2-13 distance (mm, 4095 = none), 16-31 time (ms, mod
   65536) for the reading's age. */
typedef uint32_t packed_front_t;

/* Bar / space element from the 1 kHz IR sampler ISR to the barcode task
   (Buddy 3): bit 0 bar (1) or space (0), bits 1-8 width in sampler ticks
   (1 ms), 255 = over-range.  2 bytes instead of 8. */
typedef uint16_t packed_ir_element_t;

/* Barcode message to the vehicle task (Buddy 3): bits 0-2 command, bits
   3-8 Code 39 symbol index.  2 bytes instead of 20. */
typedef uint16_t packed_barcode_t;

void packed_motion_pack(motion_status_t const * p_in, packed_motion_t * p_out);
void packed_motion_unpack(packed_motion_t const * p_in,
                          motion_status_t * p_out);
void packed_imu_pack(imu_status_t const * p_in, uint32_t seq,
                     packed_imu_t * p_out);
void packed_imu_unpack(packed_imu_t const * p_in, imu_status_t * p_out);
uint32_t packed_imu_seq(packed_imu_t const * p_in);
void packed_vehicle_pack(vehicle_status_t const * p_in,
                         packed_vehicle_t * p_out);
void packed_vehicle_unpack(packed_vehicle_t const * p_in,
                           vehicle_status_t * p_out);
void packed_profile_pack(avoidance_profile_t const * p_in,
                         packed_profile_t * p_out);
void packed_profile_unpack(packed_profile_t const * p_in,
                           avoidance_profile_t * p_out);

packed_front_t     packed_front_make(ultrasonic_class_t cls, int32_t dist_mm,
                                     uint32_t now_ms);
int32_t            packed_front_mm(packed_front_t front);
ultrasonic_class_t packed_front_class(packed_front_t front);
uint32_t           packed_front_age_ms(packed_front_t front, uint32_t now_ms);

packed_ir_element_t packed_ir_element_make(bool b_bar, uint32_t width_us);
bool                packed_ir_element_bar(packed_ir_element_t element);
uint32_t            packed_ir_element_ticks(packed_ir_element_t element);

packed_barcode_t     packed_barcode_make(barcode_decode_nav_t cmd,
                                         char symbol);
barcode_decode_nav_t packed_barcode_cmd(packed_barcode_t message);
char                 packed_barcode_symbol(packed_barcode_t message);

uint32_t     packed_symbol_index(char symbol);
char         packed_symbol_char(uint32_t index);
uint32_t     packed_reason_code(char const * p_reason);
char const * packed_reason_text(uint32_t code);
uint32_t     packed_range(int32_t dist_mm);
int32_t      packed_range_mm(uint32_t field);

#endif /* PACKED_H */

/*** end of file ***/
