/** @file vehicle.h
 *
 * @brief Vehicle controller: the mission state machine that coordinates
 *        the five subsystems (see the state diagram in the report).
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"
#include "barcode_decode.h"
#include "line_follow.h"
#include "avoidance.h"

typedef enum
{
    VS_IDLE = 0,
    VS_CALIBRATE,
    VS_LINE_FOLLOW,
    VS_NAV_TURN,
    VS_LINE_SEARCH,
    VS_OBSTACLE,
    VS_STOPPED
} vehicle_state_t;

typedef enum
{
    VCMD_START = 0,
    VCMD_STOP,
    VCMD_CALIBRATE,
    VCMD_SET_SPEED,
    VCMD_SET_LINE_GAINS
} vehicle_cmd_t;

typedef struct
{
    vehicle_state_t      state;
    uint32_t             state_seq; /* increments on every transition */
    line_follow_state_t  line_state;
    float32_t            line_error;
    barcode_decode_nav_t pending_nav;
    barcode_decode_nav_t last_nav;
    uint32_t             barcodes;
    avoidance_action_t   last_action;
    uint32_t             obstacles_passed;
    int32_t              front_mm;
    int32_t              cruise_mm_s;
    uint32_t             run_ms;
    char const *         p_reason; /* why the state last changed     */
} vehicle_status_t;

int32_t      vehicle_init(void);
bool         vehicle_command(vehicle_cmd_t cmd, int32_t arg);
bool         vehicle_set_line_gains(float32_t gain_p, float32_t gain_i,
                                    float32_t gain_d);
void         vehicle_get_status(vehicle_status_t * p_out);
char const * vehicle_state_name(vehicle_state_t state);

#endif /* VEHICLE_H */

/*** end of file ***/
