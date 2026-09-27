/** @file motion.h
 *
 * @brief Buddy 2 - motion-control library used by every other subsystem.
 *
 * A 50 Hz control task (woken by a micro T-Kernel cyclic handler) runs one
 * speed PID per wheel.  Commands are asynchronous; use motion_wait_done()
 * to block the caller until a distance or turn move completes.
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "car_types.h"

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_VELOCITY, /* continuous wheel speeds (line following) */
    MOTION_DISTANCE, /* straight move of a set distance          */
    MOTION_TURN,     /* pivot turn of a set angle                */
    MOTION_OPEN_LOOP /* fixed PWM duty, no PID (motor test only) */
} motion_mode_t;

typedef struct
{
    bool      b_busy;
    bool      b_last_ok;    /* false if the last move timed out */
    float32_t speed_l_mm_s; /* signed measured speeds           */
    float32_t speed_r_mm_s;
    float32_t target_l_mm_s;
    float32_t target_r_mm_s;
    float32_t pwm_l_pct;
    float32_t pwm_r_pct;
    uint32_t  ticks_l;
    uint32_t  ticks_r;
    float32_t dist_l_mm; /* signed odometry                  */
    float32_t dist_r_mm;
    float32_t odo_mm;        /* signed centre-line distance      */
    float32_t heading_deg;   /* encoder dead-reckoning heading   */
    float32_t turn_rate_dps; /* (vR - vL) / track                */
} motion_status_t;

typedef struct
{
    float32_t gain_p; /* speed PID, percent duty per mm/s */
    float32_t gain_i;
    float32_t gain_d;
    float32_t gain_ff;    /* feed-forward, percent duty per mm/s */
    float32_t offset_pct; /* static-friction offset, percent duty */
} motion_gains_t;

#define MOTION_WAIT_FAIL (-1) /* move ended early (stall or timeout) */
#define MOTION_WAIT_BUSY (0)  /* still moving                        */
#define MOTION_WAIT_OK   (1)

int32_t motion_init(void);

/* Required APIs from the brief (asynchronous). */
void motion_move_forward(float32_t dist_mm, float32_t speed_mm_s);
void motion_move_backward(float32_t dist_mm, float32_t speed_mm_s);
void motion_turn_left(float32_t angle_deg);
void motion_turn_right(float32_t angle_deg);
void motion_stop(void);

/* Continuous control for line following and searching. */
void motion_set_velocity(float32_t left_mm_s, float32_t right_mm_s);

/* Completion and telemetry. */
int32_t motion_wait(int32_t timeout_ms);
bool    motion_wait_done(int32_t timeout_ms);
void    motion_get_status(motion_status_t * p_out);

/* Live tuning (safe to call from any task; takes effect next cycle). */
void motion_set_speed_gains(float32_t gain_p, float32_t gain_i,
                            float32_t gain_d);
void motion_set_feedforward(float32_t gain_ff, float32_t offset_pct);
void motion_get_gains(motion_gains_t * p_out);

/* Motor characterisation: fixed duty per wheel (-100..100 %), no PID.
   Speeds and odometry are still measured.  Any other command ends it. */
void motion_set_open_loop(float32_t left_pct, float32_t right_pct);

#endif /* MOTION_H */

/*** end of file ***/
