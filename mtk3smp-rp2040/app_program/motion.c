/** @file motion.c
 *
 * @brief Buddy 2 - PID speed control, straight-line correction and
 *        encoder-based distance and turn moves.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "car_config.h"
#include "car_math.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "motion.h"

#define TASK_PRIORITY    (5)
#define TASK_STACK       (3072)
#define FLG_DONE         (1u << 0)
#define MS_PER_S         (1000.0f)
#define PCT_MAX          (100.0f)
#define RAMP_DOWN_MM     (80.0f) /* start slowing this far from the goal */
#define MIN_MOVE_MM_S    (60.0f) /* slowest reliable speed               */
#define MIN_TARGET_MM_S  (1.0f)  /* below this a wheel is stopped        */
#define TIMEOUT_FACTOR   (3.0f)  /* allowed time vs. ideal time          */
#define TIMEOUT_EXTRA_MS (1000.0f)
#define MAX_MOVE_MS      (600000.0f) /* no single move may take longer    */
#define FULL_TURN_DEG    (360.0f)
#define DIR_FORWARD      (1.0f)
#define DIR_BACKWARD     (-1.0f)

/* Cast: the period is a small number of milliseconds, which float32_t
   holds exactly. */
#define DT_S ((float32_t) CONTROL_PERIOD_MS / MS_PER_S)

/* Command shared between the API callers and the control task. */
typedef struct
{
    motion_mode_t mode;
    float32_t     v_l; /* velocity or open-loop targets (signed) */
    float32_t     v_r;
    float32_t     goal_mm; /* distance each wheel must travel        */
    float32_t     dir_l;   /* +1 or -1 per wheel                     */
    float32_t     dir_r;
    float32_t     speed;     /* cruise magnitude for moves             */
    bool          b_invalid; /* move rejected: report it as failed     */
    uint32_t      seq;       /* increments on every new command        */
} motion_cmd_t;

/* Shared between the API callers (any task), the cyclic handler and the
   control task: volatile (Rule 1.8.c); multi-field access under DI/EI. */
static volatile motion_cmd_t    g_cmd;
static volatile motion_status_t g_status;
static volatile motion_gains_t  g_gains      = {SPEED_KP, SPEED_KI, SPEED_KD,
                                                SPEED_KF, SPEED_OFFSET_PCT};
static volatile uint32_t        g_gains_seq  = 0u; /* bumped per change */
static volatile ID              gh_task      = 0;
static volatile ID              gh_done_flag = 0;

/* Used only by the task that initialises the drivers. */
static bool gb_started = false;

/* Control-task private state: initialised and used only by
   motion_task() and the functions it calls. */
static motion_status_t g_work;
static pid_ctrl_t      g_pid_l;
static pid_ctrl_t      g_pid_r;
static float32_t       g_offset_pct        = SPEED_OFFSET_PCT;
static uint32_t        g_applied_gains_seq = 0u;
static uint32_t        g_seen_seq          = 0u;
static float32_t       g_start_l           = 0.0f;
static float32_t       g_start_r           = 0.0f;
static uint32_t        g_elapsed_ms        = 0u;
static uint32_t        g_limit_ms          = 0u;
static int32_t         g_prev_dir_l        = 0; /* -1, 0 (idle) or +1 */
static int32_t         g_prev_dir_r        = 0;

static void post_command(motion_cmd_t const * p_cmd);
static void start_move(motion_mode_t mode, float32_t goal_mm, float32_t dir_l,
                       float32_t dir_r, float32_t speed);
static int32_t start_control(void);
static void    read_inputs(motion_cmd_t * p_cmd);
static void    apply_gains(pid_ctrl_t * p_pid, motion_gains_t const * p_gains);
static void    update_odometry(encoder_reading_t const * p_left,
                               encoder_reading_t const * p_right);
static void    accept_command(motion_cmd_t const * p_cmd);
static bool    move_targets(motion_cmd_t const * p_cmd, float32_t * p_target_l,
                            float32_t * p_target_r);
static bool    move_step(motion_cmd_t const * p_cmd, float32_t * p_target_l,
                         float32_t * p_target_r);
static float32_t wheel_control(pid_ctrl_t * p_pid, float32_t target,
                               float32_t measured, int32_t * p_prev_dir);
static void      drive_wheels(motion_cmd_t const * p_cmd, float32_t target_l,
                              float32_t target_r, float32_t speed_l,
                              float32_t speed_r);
static void      publish_status(void);
static void      control_step(void);
static void      control_cyclic_isr(void * p_exinf);
static void      motion_task(INT stacd, void * p_exinf);

/*!
 * @brief Start the motors, the control task, its cyclic handler and the
 *        "move done" event flag.  A second call does nothing.
 *
 * @return E_OK, E_LIMIT if a kernel object could not be created, or a
 *         tk_cre_cyc() error.
 */
int32_t
motion_init (void)
{
    int32_t ercd = E_OK;

    if (!gb_started)
    {
        ercd       = start_control();
        gb_started = (E_OK == ercd);
    }

    return (ercd);
}

/*!
 * @brief Drive straight forward for a distance.
 *
 * @param[in] dist_mm    Distance in mm.
 * @param[in] speed_mm_s Cruise speed in mm/s.
 */
void
motion_move_forward (float32_t dist_mm, float32_t speed_mm_s)
{
    start_move(MOTION_DISTANCE, dist_mm, DIR_FORWARD, DIR_FORWARD, speed_mm_s);
}

/*!
 * @brief Drive straight backward for a distance.
 *
 * @param[in] dist_mm    Distance in mm.
 * @param[in] speed_mm_s Cruise speed in mm/s.
 */
void
motion_move_backward (float32_t dist_mm, float32_t speed_mm_s)
{
    start_move(MOTION_DISTANCE, dist_mm, DIR_BACKWARD, DIR_BACKWARD,
               speed_mm_s);
}

/*!
 * @brief Pivot left on the spot: each wheel travels
 *        pi * track * angle / 360.
 *
 * @param[in] angle_deg Turn angle in degrees.
 */
void
motion_turn_left (float32_t angle_deg)
{
    float32_t arc = (CAR_MATH_PI * TRACK_WIDTH_MM * angle_deg) / FULL_TURN_DEG;

    start_move(MOTION_TURN, arc, DIR_BACKWARD, DIR_FORWARD, SPEED_TURN_MM_S);
}

/*!
 * @brief Pivot right on the spot.
 *
 * @param[in] angle_deg Turn angle in degrees.
 */
void
motion_turn_right (float32_t angle_deg)
{
    float32_t arc = (CAR_MATH_PI * TRACK_WIDTH_MM * angle_deg) / FULL_TURN_DEG;

    start_move(MOTION_TURN, arc, DIR_FORWARD, DIR_BACKWARD, SPEED_TURN_MM_S);
}

/*!
 * @brief Brake and end any move (waiting callers are released).
 */
void
motion_stop (void)
{
    motion_cmd_t cmd = {MOTION_IDLE, 0.0f, 0.0f,  0.0f, 0.0f,
                        0.0f,        0.0f, false, 0u};

    post_command(&cmd);
    (void) tk_set_flg(gh_done_flag, FLG_DONE);
}

/*!
 * @brief Continuous wheel speeds (line following and searching).
 *
 * @param[in] left_mm_s  Left wheel speed, signed.
 * @param[in] right_mm_s Right wheel speed, signed.
 */
void
motion_set_velocity (float32_t left_mm_s, float32_t right_mm_s)
{
    motion_cmd_t cmd = {MOTION_VELOCITY, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        false,           0u};

    /* Non-finite targets (Rule 5.4.b.v) stop the wheel instead. */
    cmd.v_l = car_math_is_finite(left_mm_s) ? left_mm_s : 0.0f;
    cmd.v_r = car_math_is_finite(right_mm_s) ? right_mm_s : 0.0f;
    post_command(&cmd);
}

/*!
 * @brief Fixed duty per wheel for motor characterisation (no PID).
 *
 * @param[in] left_pct  Left duty, -100..100 percent.
 * @param[in] right_pct Right duty, -100..100 percent.
 */
void
motion_set_open_loop (float32_t left_pct, float32_t right_pct)
{
    motion_cmd_t cmd = {
        MOTION_OPEN_LOOP, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0u};

    /* In this mode v_l and v_r carry percent duty. */
    cmd.v_l = car_math_clamp(left_pct, -PCT_MAX, PCT_MAX);
    cmd.v_r = car_math_clamp(right_pct, -PCT_MAX, PCT_MAX);
    post_command(&cmd);
}

/*!
 * @brief Wait for the current move to finish.
 *
 * @param[in] timeout_ms Wait time; TMO_FEVR waits for ever.
 *
 * @return MOTION_WAIT_OK, MOTION_WAIT_BUSY (still moving) or
 *         MOTION_WAIT_FAIL (ended by the stall timeout).
 */
int32_t
motion_wait (int32_t timeout_ms)
{
    UINT    pattern = 0u;
    ER      ercd    = E_OK;
    int32_t result  = MOTION_WAIT_BUSY;

    ercd = tk_wai_flg(gh_done_flag, FLG_DONE, TWF_ORW, &pattern, timeout_ms);

    if (E_OK == ercd)
    {
        result = g_status.b_last_ok ? MOTION_WAIT_OK : MOTION_WAIT_FAIL;
    }

    return (result);
}

/*!
 * @brief Block until the current move finishes.
 *
 * @param[in] timeout_ms Wait time; TMO_FEVR waits for ever.
 *
 * @return true if the move completed normally.
 */
bool
motion_wait_done (int32_t timeout_ms)
{
    return (MOTION_WAIT_OK == motion_wait(timeout_ms));
}

/*!
 * @brief Consistent copy of the latest control-cycle status.
 *
 * @param[out] p_out Status.
 */
void
motion_get_status (motion_status_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_status;
    EI(imask);
}

/*!
 * @brief Change the speed PID gains (both wheels) from any task.
 *
 * @param[in] gain_p Proportional gain.
 * @param[in] gain_i Integral gain.
 * @param[in] gain_d Derivative gain.
 */
void
motion_set_speed_gains (float32_t gain_p, float32_t gain_i, float32_t gain_d)
{
    UINT imask = 0u;

    DI(imask);
    g_gains.gain_p = gain_p;
    g_gains.gain_i = gain_i;
    g_gains.gain_d = gain_d;
    g_gains_seq++;
    EI(imask);
}

/*!
 * @brief Change the feed-forward gain and static-friction offset from any
 *        task.
 *
 * @param[in] gain_ff    Feed-forward gain, percent duty per mm/s.
 * @param[in] offset_pct Static-friction offset, 0..100 percent duty.
 */
void
motion_set_feedforward (float32_t gain_ff, float32_t offset_pct)
{
    UINT imask = 0u;

    DI(imask);
    g_gains.gain_ff    = gain_ff;
    g_gains.offset_pct = car_math_clamp(offset_pct, 0.0f, PCT_MAX);
    g_gains_seq++;
    EI(imask);
}

/*!
 * @brief Current speed-loop gains (both wheels share one set).
 *
 * @param[out] p_out Gains.
 */
void
motion_get_gains (motion_gains_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_gains;
    EI(imask);
}

/*!
 * @brief Publish a new command atomically and clear the done flag.
 *
 * @param[in] p_cmd Command (its seq field is ignored).
 */
static void
post_command (motion_cmd_t const * p_cmd)
{
    UINT imask = 0u;

    (void) tk_clr_flg(gh_done_flag, ~FLG_DONE);

    DI(imask);
    g_cmd.mode      = p_cmd->mode;
    g_cmd.v_l       = p_cmd->v_l;
    g_cmd.v_r       = p_cmd->v_r;
    g_cmd.goal_mm   = p_cmd->goal_mm;
    g_cmd.dir_l     = p_cmd->dir_l;
    g_cmd.dir_r     = p_cmd->dir_r;
    g_cmd.speed     = p_cmd->speed;
    g_cmd.b_invalid = p_cmd->b_invalid;
    g_cmd.seq++;
    EI(imask);
}

/*!
 * @brief Build and post a distance or turn move.
 *
 * A move with a non-finite goal or speed (Rule 5.4.b.v) is posted as an
 * empty move marked invalid: the car stays stopped, and the waiting
 * caller sees MOTION_WAIT_FAIL.
 *
 * @param[in] mode    MOTION_DISTANCE or MOTION_TURN.
 * @param[in] goal_mm Distance each wheel travels.
 * @param[in] dir_l   Left wheel direction, +1 or -1.
 * @param[in] dir_r   Right wheel direction, +1 or -1.
 * @param[in] speed   Cruise speed in mm/s.
 */
static void
start_move (motion_mode_t mode, float32_t goal_mm, float32_t dir_l,
            float32_t dir_r, float32_t speed)
{
    motion_cmd_t cmd = {MOTION_IDLE, 0.0f, 0.0f,  0.0f, 0.0f,
                        0.0f,        0.0f, false, 0u};

    cmd.mode  = mode;
    cmd.dir_l = dir_l;
    cmd.dir_r = dir_r;

    if ((car_math_is_finite(goal_mm)) && (car_math_is_finite(speed)))
    {
        cmd.goal_mm = car_math_abs(goal_mm);
        cmd.speed   = car_math_abs(speed);
    }
    else
    {
        cmd.b_invalid = true; /* goal 0: ends in the first cycle */
    }

    post_command(&cmd);
}

/*!
 * @brief Create the kernel objects of the control loop.
 *
 * @return E_OK, E_LIMIT or a tk_cre_cyc() error.
 */
static int32_t
start_control (void)
{
    T_CTSK  ctsk     = {0};
    T_CCYC  ccyc     = {0};
    T_CFLG  cflg     = {0};
    ID      h_cyclic = 0;
    int32_t ercd     = E_LIMIT;

    motor_init();

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = FLG_DONE;
    gh_done_flag = tk_cre_flg(&cflg);

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       motion_task(). */
    ctsk.task    = (FP) motion_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    gh_task      = tk_cre_tsk(&ctsk);

    if ((gh_done_flag >= E_OK) && (gh_task >= E_OK))
    {
        (void) tk_sta_tsk(gh_task, 0);
        ccyc.cycatr = TA_HLNG | TA_STA;

        /* Cast: a TA_HLNG cyclic handler is called as (void * exinf),
           matching control_cyclic_isr(). */
        ccyc.cychdr = (FP) control_cyclic_isr;
        ccyc.cyctim = CONTROL_PERIOD_MS;
        h_cyclic    = tk_cre_cyc(&ccyc);
        ercd        = (h_cyclic >= E_OK) ? E_OK : h_cyclic;
    }

    return (ercd);
}

/*!
 * @brief Snapshot the command and apply any new gains (control task).
 *
 * @param[out] p_cmd Current command.
 */
static void
read_inputs (motion_cmd_t * p_cmd)
{
    motion_gains_t gains;
    uint32_t       gains_seq = 0u;
    UINT           imask     = 0u;

    DI(imask);
    *p_cmd    = g_cmd;
    gains     = g_gains;
    gains_seq = g_gains_seq;
    EI(imask);

    /* A live tuning command arrived. */
    if (gains_seq != g_applied_gains_seq)
    {
        g_applied_gains_seq = gains_seq;
        apply_gains(&g_pid_l, &gains);
        apply_gains(&g_pid_r, &gains);
        g_offset_pct = gains.offset_pct;
    }
}

/*!
 * @brief Copy tuned gains into one PID controller (control task only).
 *
 * @param[out] p_pid   Controller.
 * @param[in]  p_gains Gains.
 */
static void
apply_gains (pid_ctrl_t * p_pid, motion_gains_t const * p_gains)
{
    p_pid->gain_p  = p_gains->gain_p;
    p_pid->gain_i  = p_gains->gain_i;
    p_pid->gain_d  = p_gains->gain_d;
    p_pid->gain_ff = p_gains->gain_ff;
}

/*!
 * @brief Integrate the odometry from the encoder tick deltas.
 *
 * The encoders give no direction, so the sign of each wheel's drive
 * decides it.
 *
 * @param[in] p_left  Left encoder reading.
 * @param[in] p_right Right encoder reading.
 */
static void
update_odometry (encoder_reading_t const * p_left,
                 encoder_reading_t const * p_right)
{
    float32_t mm_tick = encoder_mm_per_tick();
    float32_t sign_l  = (g_work.pwm_l_pct < 0.0f) ? -1.0f : 1.0f;
    float32_t sign_r  = (g_work.pwm_r_pct < 0.0f) ? -1.0f : 1.0f;
    float32_t d_l     = 0.0f;
    float32_t d_r     = 0.0f;

    /* Casts: a wheel moves a handful of ticks per 20 ms cycle, so the
       unsigned difference (wrap-around safe) is small and float32_t holds
       it exactly. */
    d_l = sign_l * mm_tick * (float32_t) (p_left->ticks - g_work.ticks_l);
    d_r = sign_r * mm_tick * (float32_t) (p_right->ticks - g_work.ticks_r);

    g_work.ticks_l = p_left->ticks;
    g_work.ticks_r = p_right->ticks;
    g_work.dist_l_mm += d_l;
    g_work.dist_r_mm += d_r;
    g_work.odo_mm += 0.5f * (d_l + d_r);
    g_work.heading_deg += ((d_r - d_l) / TRACK_WIDTH_MM) * CAR_MATH_DEG_PER_RAD;
    g_work.speed_l_mm_s = sign_l * p_left->speed_mm_s;
    g_work.speed_r_mm_s = sign_r * p_right->speed_mm_s;
    g_work.turn_rate_dps =
        ((g_work.speed_r_mm_s - g_work.speed_l_mm_s) / TRACK_WIDTH_MM)
        * CAR_MATH_DEG_PER_RAD;
}

/*!
 * @brief Start tracking a newly posted command.
 *
 * @param[in] p_cmd Current command.
 */
static void
accept_command (motion_cmd_t const * p_cmd)
{
    float32_t limit_ms = MAX_MOVE_MS;

    if (p_cmd->seq != g_seen_seq)
    {
        g_seen_seq   = p_cmd->seq;
        g_start_l    = g_work.dist_l_mm;
        g_start_r    = g_work.dist_r_mm;
        g_elapsed_ms = 0u;

        /* Stall timeout: TIMEOUT_FACTOR times the ideal move time. */
        limit_ms = ((TIMEOUT_FACTOR * MS_PER_S * p_cmd->goal_mm)
                    / ((p_cmd->speed > 1.0f) ? p_cmd->speed : 1.0f))
                   + TIMEOUT_EXTRA_MS;
        limit_ms = car_math_is_finite(limit_ms)
                       ? car_math_clamp(limit_ms, 0.0f, MAX_MOVE_MS)
                       : MAX_MOVE_MS;

        /* Cast: limit_ms is 0..MAX_MOVE_MS, which uint32_t holds. */
        g_limit_ms = (uint32_t) limit_ms;
        g_work.b_busy =
            (MOTION_DISTANCE == p_cmd->mode) || (MOTION_TURN == p_cmd->mode);
        g_work.b_last_ok = !p_cmd->b_invalid;
    }
}

/*!
 * @brief Wheel targets for a distance or turn move, with a ramp-down
 *        near the goal and straightness correction.
 *
 * @param[in]  p_cmd      Current move.
 * @param[out] p_target_l Left wheel target (signed mm/s).
 * @param[out] p_target_r Right wheel target (signed mm/s).
 *
 * @return true once the goal (less the braking margin) is reached.
 */
static bool
move_targets (motion_cmd_t const * p_cmd, float32_t * p_target_l,
              float32_t * p_target_r)
{
    float32_t dist_l    = car_math_abs(g_work.dist_l_mm - g_start_l);
    float32_t dist_r    = car_math_abs(g_work.dist_r_mm - g_start_r);
    float32_t remaining = p_cmd->goal_mm - (0.5f * (dist_l + dist_r));
    float32_t speed     = p_cmd->speed;
    float32_t corr      = 0.0f;
    bool      b_done    = (remaining <= MOTION_BRAKE_MARGIN_MM);

    *p_target_l = 0.0f;
    *p_target_r = 0.0f;

    if ((!b_done) && (remaining < RAMP_DOWN_MM))
    {
        speed = MIN_MOVE_MM_S
                + (((speed - MIN_MOVE_MM_S) * remaining) / RAMP_DOWN_MM);
        speed = (speed < MIN_MOVE_MM_S) ? MIN_MOVE_MM_S : speed;
    }

    /* The wheel that is ahead is slowed and the lagging one sped up. */
    if (!b_done)
    {
        corr = STRAIGHT_KP * (dist_l - dist_r);
        *p_target_l =
            p_cmd->dir_l * car_math_clamp(speed - corr, 0.0f, 2.0f * speed);
        *p_target_r =
            p_cmd->dir_r * car_math_clamp(speed + corr, 0.0f, 2.0f * speed);
    }

    return (b_done);
}

/*!
 * @brief One control cycle of a distance or turn move, with the stall
 *        timeout.
 *
 * @param[in]  p_cmd      Current move.
 * @param[out] p_target_l Left wheel target (signed mm/s).
 * @param[out] p_target_r Right wheel target (signed mm/s).
 *
 * @return true when the move has just finished (normally or not).
 */
static bool
move_step (motion_cmd_t const * p_cmd, float32_t * p_target_l,
           float32_t * p_target_r)
{
    bool b_done = move_targets(p_cmd, p_target_l, p_target_r);

    g_elapsed_ms += CONTROL_PERIOD_MS;

    /* Stalled or blocked. */
    if (g_elapsed_ms > g_limit_ms)
    {
        b_done           = true;
        g_work.b_last_ok = false;
    }

    if (b_done)
    {
        g_work.b_busy = false;
        *p_target_l   = 0.0f;
        *p_target_r   = 0.0f;
    }

    return (b_done);
}

/*!
 * @brief Closed-loop speed of one wheel to a signed PWM duty.
 *
 * The encoder measures |speed| only, so the commanded sign is applied to
 * the output.
 *
 * @param[in,out] p_pid      Wheel controller.
 * @param[in]     target     Signed target speed in mm/s.
 * @param[in]     measured   Measured speed magnitude in mm/s.
 * @param[in,out] p_prev_dir Direction of the previous cycle.
 *
 * @return Signed duty in percent.
 */
static float32_t
wheel_control (pid_ctrl_t * p_pid, float32_t target, float32_t measured,
               int32_t * p_prev_dir)
{
    int32_t   dir    = (target >= 0.0f) ? 1 : -1;
    float32_t sign   = (target >= 0.0f) ? 1.0f : -1.0f;
    float32_t output = 0.0f;

    if (car_math_abs(target) < MIN_TARGET_MM_S)
    {
        pid_reset(p_pid);
        *p_prev_dir = 0;
    }
    else
    {
        /* A direction change starts the controller clean. */
        if (dir != *p_prev_dir)
        {
            pid_reset(p_pid);
            *p_prev_dir = dir;
        }

        output = pid_update(p_pid, car_math_abs(target), measured, DT_S);
        output = sign * car_math_clamp(output + g_offset_pct, 0.0f, PCT_MAX);
    }

    return (output);
}

/*!
 * @brief Compute and apply both wheel duties.
 *
 * @param[in] p_cmd    Current command.
 * @param[in] target_l Left wheel target (signed mm/s).
 * @param[in] target_r Right wheel target (signed mm/s).
 * @param[in] speed_l  Left measured speed magnitude.
 * @param[in] speed_r  Right measured speed magnitude.
 */
static void
drive_wheels (motion_cmd_t const * p_cmd, float32_t target_l,
              float32_t target_r, float32_t speed_l, float32_t speed_r)
{
    if (MOTION_OPEN_LOOP == p_cmd->mode)
    {
        /* Motor characterisation: duty straight through, with the PIDs
           held clean so the next closed-loop command has no history. */
        pid_reset(&g_pid_l);
        pid_reset(&g_pid_r);
        g_prev_dir_l     = 0;
        g_prev_dir_r     = 0;
        g_work.pwm_l_pct = p_cmd->v_l;
        g_work.pwm_r_pct = p_cmd->v_r;
    }
    else
    {
        g_work.pwm_l_pct =
            wheel_control(&g_pid_l, target_l, speed_l, &g_prev_dir_l);
        g_work.pwm_r_pct =
            wheel_control(&g_pid_r, target_r, speed_r, &g_prev_dir_r);
    }

    motor_set(g_work.pwm_l_pct, g_work.pwm_r_pct);
}

/*!
 * @brief Publish the working status for the other tasks (short DI
 *        section).
 */
static void
publish_status (void)
{
    UINT imask = 0u;

    DI(imask);
    g_status = g_work;
    EI(imask);
}

/*!
 * @brief One 20 ms control cycle.
 */
static void
control_step (void)
{
    encoder_reading_t enc_l;
    encoder_reading_t enc_r;
    motion_cmd_t      cmd;
    float32_t         target_l   = 0.0f;
    float32_t         target_r   = 0.0f;
    bool              b_finished = false;

    read_inputs(&cmd);
    encoder_read(ENCODER_LEFT, &enc_l);
    encoder_read(ENCODER_RIGHT, &enc_r);
    update_odometry(&enc_l, &enc_r);
    accept_command(&cmd);

    if (MOTION_VELOCITY == cmd.mode)
    {
        target_l = cmd.v_l;
        target_r = cmd.v_r;
    }
    else if (g_work.b_busy)
    {
        b_finished = move_step(&cmd, &target_l, &target_r);
    }
    else
    {
        /* Idle: zero targets brake the motors. */
    }

    g_work.target_l_mm_s = target_l;
    g_work.target_r_mm_s = target_r;
    drive_wheels(&cmd, target_l, target_r, enc_l.speed_mm_s, enc_r.speed_mm_s);

    /* Publish b_last_ok before waking the waiting task. */
    publish_status();

    if (b_finished)
    {
        (void) tk_set_flg(gh_done_flag, FLG_DONE);
    }
}

/*!
 * @brief Cyclic handler, every CONTROL_PERIOD_MS (timer interrupt
 *        context): wake the control task.
 *
 * @param[in] p_exinf Extended information (unused).
 */
static void
control_cyclic_isr (void * p_exinf)
{
    (void) p_exinf;
    (void) tk_wup_tsk(gh_task);
}

/*!
 * @brief Control task: owns the PID controllers and runs one control
 *        cycle per wake-up.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
motion_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    /* The controllers belong to this task alone: they are created here,
       and live gain changes arrive through g_gains. */
    pid_init(&g_pid_l, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_KF, 0.0f, PCT_MAX);
    pid_init(&g_pid_r, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_KF, 0.0f, PCT_MAX);
    g_offset_pct        = SPEED_OFFSET_PCT;
    g_applied_gains_seq = 0u;

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        control_step();
    }
}

/*** end of file ***/
