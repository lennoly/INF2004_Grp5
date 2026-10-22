/** @file frames.h
 *
 * @brief Buddy 1 - compact binary MQTT frames (Change Requirement 1).
 *
 * Every MQTT payload, in both directions, is one frame:
 *   byte 0     bits 0-3 frame type, bits 4-7 format version (1)
 *   byte 1..   fields packed with explicit masks and shifts, least
 *              significant bit first (bitpack.h); no bit-fields on the wire
 *
 * The field list of every frame, with ranges and units, is in
 * docs/DATA_REPRESENTATION.md, and tools/dashboard/frames.py decodes the
 * same layout.  Frames are at most FRAMES_MAX_BYTES long, so they fit one
 * MQTT TX ring slot.  The serial console prints the same frames as
 * '@' + hex so that the dashboard reads both links the same way.
 */

#ifndef FRAMES_H
#define FRAMES_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"
#include "motion.h"
#include "imu.h"
#include "vehicle.h"
#include "avoidance.h"
#include "command.h"
#include "fault.h"

#define FRAMES_VERSION   (1u)
#define FRAMES_MAX_BYTES (47u) /* largest frame: telemetry, 43 bytes */
#define FRAMES_IR_COUNT  (3u)
#define FRAMES_TIMING_SLOTS (8u) /* TIMING_COUNT sections */

typedef enum
{
    FRAME_TELEMETRY = 1,
    FRAME_HEARTBEAT,
    FRAME_STATE,
    FRAME_BARCODE,
    FRAME_HUMP,
    FRAME_OBSTACLE,
    FRAME_IMPACT,
    FRAME_REPLY,
    FRAME_GAINS,
    FRAME_TIMING,
    FRAME_STATUS,
    FRAME_COMMAND
} frames_type_t;

/* Reply error codes (4 bits). */
typedef enum
{
    FRAMES_OK = 0,
    FRAMES_ERR_INVALID,   /* not a valid command or argument  */
    FRAMES_ERR_FRAME,     /* not a version-1 command frame    */
    FRAMES_ERR_DRIVING,   /* tuning only while IDLE / STOPPED */
    FRAMES_ERR_UNAVAILABLE /* not in this build               */
} frames_error_t;

/* Everything one telemetry frame carries (gathered by the telemetry
   task from the subsystems' unpacked snapshots). */
typedef struct
{
    vehicle_status_t veh;
    motion_status_t  motion;
    imu_status_t     imu;
    float32_t        ir_norm[FRAMES_IR_COUNT]; /* 0 white .. 1 black */
    char             barcode;                  /* last decoded symbol */
    uint32_t         faults;                   /* fault_mask()        */
} frames_telemetry_t;

/* Heartbeat contents. */
typedef struct
{
    uint32_t up_s;
    uint32_t seq;
    uint32_t state;
    uint32_t link;
    uint32_t published;
    uint32_t dropped;
    uint32_t commands;
    uint32_t reconnects;
    uint32_t rejoins;
    uint32_t wifi_restarts;
    uint32_t wifi_code;
    uint32_t address; /* IPv4, network byte order */
    uint32_t attempts;
    uint32_t why_code;
    bool     b_imu_ok;
    uint32_t i2c_errors;
    uint32_t faults;
    uint32_t fault_counts[FAULT_COUNT];
} frames_heartbeat_t;

/* Current tuning values. */
typedef struct
{
    float32_t pid[3];
    float32_t ff[2];
    float32_t line[3];
    int32_t   speed_mm_s;
    uint32_t  rate_ms;
} frames_gains_t;

/* Timing window summary (per-section averages stay on the console). */
typedef struct
{
    uint32_t window_ms;
    uint32_t load_pct;
    uint32_t jitter_max_us;
    uint32_t periods;
    uint32_t max_us[FRAMES_TIMING_SLOTS];
} frames_timing_t;

uint32_t frames_telemetry(uint8_t * p_buf, uint32_t size,
                          frames_telemetry_t const * p_in);
uint32_t frames_heartbeat(uint8_t * p_buf, uint32_t size,
                          frames_heartbeat_t const * p_in);
uint32_t frames_state(uint8_t * p_buf, uint32_t size,
                      vehicle_state_t state, uint32_t reason_code);
uint32_t frames_barcode(uint8_t * p_buf, uint32_t size, char symbol,
                        barcode_decode_nav_t cmd);
uint32_t frames_hump(uint8_t * p_buf, uint32_t size, uint32_t humps,
                     float32_t peak_mm, float32_t max_mm);
uint32_t frames_obstacle(uint8_t * p_buf, uint32_t size,
                         avoidance_profile_t const * p_prof,
                         avoidance_action_t action);
uint32_t frames_impact(uint8_t * p_buf, uint32_t size, uint32_t impacts);
uint32_t frames_reply(uint8_t * p_buf, uint32_t size, uint32_t req_id,
                      command_id_t kind, frames_error_t error);
uint32_t frames_gains(uint8_t * p_buf, uint32_t size,
                      frames_gains_t const * p_in);
uint32_t frames_timing(uint8_t * p_buf, uint32_t size,
                       frames_timing_t const * p_in);
uint32_t frames_status(uint8_t * p_buf, uint32_t size, bool b_online);
uint32_t frames_command(uint8_t * p_buf, uint32_t size,
                        command_t const * p_cmd);
bool     frames_parse_command(uint8_t const * p_buf, uint32_t len,
                              command_t * p_cmd);

#endif /* FRAMES_H */

/*** end of file ***/
