/** @file vehicle.c
 *
 * @brief Vehicle controller (integration layer).
 *
 * IDLE --start--> LINE_FOLLOW --barcode+junction--> NAV_TURN --> LINE_FOLLOW
 *                   |  |  +--line lost--> LINE_SEARCH --found--> LINE_FOLLOW
 *                   |  +--2 front pings < OBST_DETECT_MM or impact--> OBSTACLE
 *                   |       OBSTACLE: scan, plan, bypass, reacquire line
 * any --stop / blocked / line not found--> STOPPED --start--> LINE_FOLLOW
 * IDLE/STOPPED --calibrate / START held 2 s--> CALIBRATE (360 degree spin)
 *
 * Manoeuvres are sequences of blocking motion moves executed in this task;
 * every wait, and every step of an obstacle scan, polls the STOP button and
 * queued commands, so any manoeuvre can be aborted.  STOP presses are also
 * latched by an interrupt, so a short press is never missed, and STOP never
 * moves the car.  The same diagram is drawn in docs/img/states.png (Rule
 * 2.2.f) and explained in the report (docs/REPORT.md).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "car_config.h"
#include "car_math.h"
#include "hal.h"
#include "hal_irq.h"
#include "motion.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "imu.h"
#include "servo.h"
#include "ultrasonic.h"
#include "obstacle.h"
#include "vehicle.h"

#define TASK_PRIORITY      (6)
#define TASK_STACK         (3072)
#define LOOP_MS            (20u)
#define MS_PER_S           (1000.0f)
#define MOVE_TIMEOUT_MS    (10000u)
#define MBF_MSGS           (4u)
#define LINE_GAINS         (3u)
#define REACQ_SPEED        (90.0f)
#define LINE_SEEN_LEVEL    (0.5f)
#define CENTRED_ERROR      (0.35f)
#define CENTRED_SUM        (0.6f)
#define QUARTER_TURN       (90.0f)
#define HALF_TURN          (180.0f)
#define FULL_TURN          (360.0f)
#define SERVO_FULL_DEG     (180)
#define REVERSE_MM         (120.0f)
#define PASS_STEP_MM       (50.0f)
#define SIDE_SEEN_EXTRA_MM (150.0f)
#define SEEK_EXTRA_MM      (200.0f)
#define FRONT_IGNORE_MM    (150.0f)
#define STRAIGHT_EXTRA_MM  (20.0f)
#define SPEED_MIN_CMD      (60)
#define SPEED_MAX_CMD      (300) /* NFR2: >= 10 IR samples per 3 mm bar */
#define STOP_SETTLE_MS     (200u)
#define CALIB_HOLD_MS      (2000u) /* START held => calibrate */
#define REACQ_SWEEPS       (5u)
#define REACQ_BASE_MS      (350u)
#define SIDE_LEFT          (1.0f)
#define SIDE_RIGHT         (-1.0f)

/* Cast: the loop period is a small number of milliseconds, which
   float32_t holds exactly. */
#define DT_S ((float32_t) LOOP_MS / MS_PER_S)

typedef struct
{
    vehicle_cmd_t cmd;
    int32_t       arg;
    float32_t     gains[LINE_GAINS]; /* VCMD_SET_LINE_GAINS: kp, ki, kd */
} vehicle_msg_t;

typedef enum
{
    SEARCH_RUNNING = 0,
    SEARCH_FOUND,
    SEARCH_ABORTED
} search_result_t;

/* Published status and the command queue, used by other tasks: volatile
   (Rule 1.8.c). */
static volatile vehicle_status_t g_st;
static volatile ID               gh_mbuf = 0;

/* Set by the STOP button interrupt, cleared by the vehicle task: volatile
   (Rule 1.8.c). */
static volatile bool gb_stop_latched = false;

/* Vehicle-task private state. */
static line_follow_t g_lf;
static float32_t     g_prev_odo           = 0.0f;
static float32_t     g_pending_odo        = 0.0f;
static float32_t     g_front_ignore_until = 0.0f;
static uint32_t      g_attempts           = 0u;
static uint32_t      g_seen_impacts       = 0u;
static uint32_t      g_run_start_ms       = 0u;
static uint32_t      g_start_down_ms      = 0u;
static bool          gb_abort             = false;
static bool          gb_start             = false;
static bool          gb_calib             = false;
static bool          gb_btn_start_prev    = false;
static bool          gb_btn_stop_prev     = false;
static bool          gb_start_armed       = false;
static bool          gb_after_impact      = false;

/* Names indexed by vehicle_state_t. */
static char const * const g_state_names[] = {
    "IDLE",        "CALIBRATE", "LINE_FOLLOW", "NAV_TURN",
    "LINE_SEARCH", "OBSTACLE",  "STOPPED"};

static uint32_t        now_ms(void);
static void            set_state(vehicle_state_t state, char const * p_reason);
static int32_t         clamp_speed(int32_t speed_mm_s);
static void            poll_start_button(bool b_start, bool b_idle);
static void            poll_buttons(void);
static void            poll_messages(void);
static void            poll_inputs(void);
static bool            stop_requested(void);
static bool            run_move(void);
static bool            turn_side(float32_t side, float32_t angle_deg);
static bool            forward(float32_t dist_mm, float32_t speed);
static bool            line_centred(ir_sensor_snapshot_t const * p_ir);
static search_result_t reacquire_poll(void);
static bool            reacquire_line(float32_t side);
static void            execute_nav(barcode_decode_nav_t cmd);
static bool            pass_obstacle(float32_t side, float32_t offset);
static bool            seek_line(float32_t max_mm);
static bool            bypass(avoidance_plan_t const * p_plan);
static void            act_on_scan(avoidance_profile_t const * p_prof);
static void            handle_obstacle(void);
static void            take_barcode(float32_t odo_mm);
static bool            nav_due(float32_t odo_mm);
static bool            obstacle_due(float32_t odo_mm);
static void            handle_line_follow(void);
static void            handle_line_search(void);
static void            start_run(void);
static void            calibrate(void);
static void            handle_requests(void);
static void            run_state(void);
static void            vehicle_task(INT stacd, void * p_exinf);
static void stop_button_isr(uint32_t pin, uint32_t events, uint32_t t_us);

/*!
 * @brief Configure the buttons, create the command queue and start the
 *        vehicle task.
 *
 * @return E_OK, E_LIMIT if a kernel object could not be created, or a
 *         tk_sta_tsk() or hal_irq_gpio_attach() error.
 */
int32_t
vehicle_init (void)
{
    T_CMBF cmbf     = {0};
    T_CTSK ctsk     = {0};
    ID     h_task   = 0;
    ER     ercd     = E_LIMIT;
    ER     btn_ercd = E_OK;

    hal_gpio_init_in(PIN_BTN_START, true);
    hal_gpio_init_in(PIN_BTN_STOP, true);

    /* Latch STOP presses: the task cannot poll the button while it waits
       for the sonar. */
    btn_ercd =
        hal_irq_gpio_attach(PIN_BTN_STOP, HAL_GPIO_EDGE_FALL, stop_button_isr);

    g_st.state       = VS_IDLE;
    g_st.cruise_mm_s = car_math_round(SPEED_CRUISE_MM_S);
    g_st.front_mm    = ULTRASONIC_NO_ECHO;
    g_st.p_reason    = hal_watchdog_caused_reset() ? "watchdog reset" : "boot";

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = MBF_MSGS * (sizeof(vehicle_msg_t) + sizeof(UW));
    cmbf.maxmsz = sizeof(vehicle_msg_t);
    gh_mbuf     = tk_cre_mbf(&cmbf);

    ctsk.tskatr = TA_HLNG | TA_RNG3;

    /* Cast: the kernel stores every task entry as the generic FP type and
       calls a TA_HLNG task as (INT stacd, void * exinf), matching
       vehicle_task(). */
    ctsk.task    = (FP) vehicle_task;
    ctsk.itskpri = TASK_PRIORITY;
    ctsk.stksz   = TASK_STACK;
    h_task       = tk_cre_tsk(&ctsk);

    if ((gh_mbuf >= E_OK) && (h_task >= E_OK))
    {
        ercd = tk_sta_tsk(h_task, 0);
    }

    /* Without the interrupt STOP is still polled; report the fault. */
    ercd = (E_OK == ercd) ? btn_ercd : ercd;

    return (ercd);
}

/*!
 * @brief Thread-safe request from any task (e.g. an MQTT or console
 *        command).
 *
 * @param[in] cmd Request.
 * @param[in] arg Argument (VCMD_SET_SPEED: speed in mm/s).
 *
 * @return false if the vehicle task is not running (test builds) or its
 *         queue is full.
 */
bool
vehicle_command (vehicle_cmd_t cmd, int32_t arg)
{
    vehicle_msg_t msg = {VCMD_START, 0, {0.0f, 0.0f, 0.0f}};

    msg.cmd = cmd;
    msg.arg = arg;

    /* Cast: the message is a few dozen bytes, far below INT's range. */
    return (E_OK == tk_snd_mbf(gh_mbuf, &msg, (INT) sizeof(msg), TMO_POL));
}

/*!
 * @brief Queue new line-following gains; the vehicle task applies them
 *        because it owns the follower.
 *
 * @param[in] gain_p Proportional gain.
 * @param[in] gain_i Integral gain.
 * @param[in] gain_d Derivative gain.
 *
 * @return false if the vehicle task is not running or its queue is full.
 */
bool
vehicle_set_line_gains (float32_t gain_p, float32_t gain_i, float32_t gain_d)
{
    vehicle_msg_t msg = {VCMD_SET_LINE_GAINS, 0, {0.0f, 0.0f, 0.0f}};

    msg.gains[0] = gain_p;
    msg.gains[1] = gain_i;
    msg.gains[2] = gain_d;

    /* Cast: the message is a few dozen bytes, far below INT's range. */
    return (E_OK == tk_snd_mbf(gh_mbuf, &msg, (INT) sizeof(msg), TMO_POL));
}

/*!
 * @brief Consistent copy of the vehicle status.
 *
 * @param[out] p_out Status.
 */
void
vehicle_get_status (vehicle_status_t * p_out)
{
    UINT imask = 0u;

    DI(imask);
    *p_out = g_st;
    EI(imask);
}

/*!
 * @brief Printable name of a vehicle state.
 *
 * @param[in] state Vehicle state.
 *
 * @return Upper-case name, or "?" for an out-of-range value.
 */
char const *
vehicle_state_name (vehicle_state_t state)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked below. */
    uint32_t     index  = (uint32_t) state;
    char const * p_name = "?";

    if (index < (sizeof(g_state_names) / sizeof(g_state_names[0])))
    {
        p_name = g_state_names[index];
    }

    return (p_name);
}

/*!
 * @brief The kernel's millisecond clock.
 *
 * @return Milliseconds since boot, modulo 2^32.
 */
static uint32_t
now_ms (void)
{
    SYSTIM tim = {0, 0u};

    (void) tk_get_tim(&tim);

    return (tim.lo);
}

/*!
 * @brief Change state (published atomically) and log it.
 *
 * @param[in] state    New state.
 * @param[in] p_reason Why, as a string literal.
 */
static void
set_state (vehicle_state_t state, char const * p_reason)
{
    UINT imask = 0u;

    DI(imask);
    g_st.state = state;
    g_st.state_seq++;
    g_st.p_reason = p_reason;
    EI(imask);

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    (void) tm_printf((UB const *) "[vehicle] -> %s (%s)\n",
                     vehicle_state_name(state), p_reason);
}

/*!
 * @brief Limit a requested cruise speed to the supported range.
 *
 * @param[in] speed_mm_s Requested speed.
 *
 * @return Speed limited to SPEED_MIN_CMD..SPEED_MAX_CMD.
 */
static int32_t
clamp_speed (int32_t speed_mm_s)
{
    int32_t result = speed_mm_s;

    if (speed_mm_s < SPEED_MIN_CMD)
    {
        result = SPEED_MIN_CMD;
    }
    else if (speed_mm_s > SPEED_MAX_CMD)
    {
        result = SPEED_MAX_CMD;
    }
    else
    {
        /* Already inside the range. */
    }

    return (result);
}

/*!
 * @brief START button, acted on when released so that the car never moves
 *        under the user's hand: a short press starts a run, a press held
 *        for at least CALIB_HOLD_MS requests a calibration.  Only presses
 *        that begin while the car is idle count.
 *
 * @param[in] b_start START is pressed now.
 * @param[in] b_idle  The car is idle or stopped.
 */
static void
poll_start_button (bool b_start, bool b_idle)
{
    uint32_t now = now_ms();

    if (b_start && (!gb_btn_start_prev))
    {
        /* Just pressed: time the hold. */
        g_start_down_ms = now;
        gb_start_armed  = b_idle;
    }
    else if ((!b_start) && gb_btn_start_prev && gb_start_armed)
    {
        gb_start_armed = false;

        if ((now - g_start_down_ms) >= CALIB_HOLD_MS)
        {
            gb_calib = true;
        }
        else
        {
            gb_start = true;
        }
    }
    else
    {
        /* Held, or released after a press that began during a run. */
    }
}

/*!
 * @brief Buttons (active low).  START: see poll_start_button().  STOP
 *        aborts a run or manoeuvre and is ignored while idle, so it never
 *        moves the car; a press latched by stop_button_isr() counts even if
 *        the button was released before this poll.
 */
static void
poll_buttons (void)
{
    bool b_start   = !hal_gpio_get(PIN_BTN_START);
    bool b_stop    = !hal_gpio_get(PIN_BTN_STOP);
    bool b_idle    = (VS_IDLE == g_st.state) || (VS_STOPPED == g_st.state);
    bool b_pressed = b_stop && (!gb_btn_stop_prev);
    UINT imask     = 0u;

    DI(imask);
    b_pressed       = b_pressed || gb_stop_latched;
    gb_stop_latched = false;
    EI(imask);

    poll_start_button(b_start, b_idle);
    gb_abort          = gb_abort || (b_pressed && (!b_idle));
    gb_btn_start_prev = b_start;
    gb_btn_stop_prev  = b_stop;
}

/*!
 * @brief STOP button edge callback (interrupt context): latch the press for
 *        poll_buttons().
 *
 * @param[in] pin    Button pin (unused).
 * @param[in] events HAL_GPIO_EDGE_* bits (unused: only the falling edge,
 *                   a press, is enabled).
 * @param[in] t_us   Time stamp of the interrupt (unused).
 */
static void
stop_button_isr (uint32_t pin, uint32_t events, uint32_t t_us)
{
    (void) pin;
    (void) events;
    (void) t_us;

    gb_stop_latched = true;
}

/*!
 * @brief Commands queued by other tasks.
 */
static void
poll_messages (void)
{
    vehicle_msg_t msg = {VCMD_START, 0, {0.0f, 0.0f, 0.0f}};

    /* Cast: the message is a few dozen bytes, far below INT's range. */
    while ((INT) sizeof(msg) == tk_rcv_mbf(gh_mbuf, &msg, TMO_POL))
    {
        switch (msg.cmd)
        {
            case VCMD_START:
                gb_start = true;
            break;

            case VCMD_STOP:
                gb_abort = true;
            break;

            case VCMD_CALIBRATE:
                gb_calib = true;
            break;

            case VCMD_SET_SPEED:
                g_st.cruise_mm_s = clamp_speed(msg.arg);
            break;

            case VCMD_SET_LINE_GAINS:
                line_follow_set_gains(&g_lf, msg.gains[0], msg.gains[1],
                                      msg.gains[2]);
            break;

            default:
                /* Unknown requests are ignored. */
            break;
        }
    }
}

/*!
 * @brief Buttons and queued commands; also feeds the watchdog, because every
 *        wait in this task calls this at least once a second (every LOOP_MS,
 *        or between the servo steps of a scan).
 */
static void
poll_inputs (void)
{
    hal_watchdog_feed();
    poll_buttons();
    poll_messages();
}

/*!
 * @brief Scan callback (see obstacle_scan()): service the buttons and
 *        queued commands between servo steps.
 *
 * @return true if a stop was requested, to end the scan early.
 */
static bool
stop_requested (void)
{
    poll_inputs();

    return (gb_abort);
}

/*!
 * @brief Wait for the issued motion move; abortable by STOP.
 *
 * @return true if the move completed normally.
 */
static bool
run_move (void)
{
    uint32_t waited    = 0u;
    int32_t  result    = MOTION_WAIT_BUSY;
    bool     b_ok      = false;
    bool     b_waiting = true;

    while (b_waiting)
    {
        result = motion_wait(LOOP_MS);

        if (MOTION_WAIT_BUSY != result)
        {
            b_ok      = (MOTION_WAIT_OK == result);
            b_waiting = false;
        }
        else
        {
            poll_inputs();
            waited += LOOP_MS;

            if (gb_abort || (waited > MOVE_TIMEOUT_MS))
            {
                motion_stop();
                b_waiting = false;
            }
        }
    }

    return (b_ok);
}

/*!
 * @brief Pivot towards a side and wait.
 *
 * @param[in] side      SIDE_LEFT (+1) or SIDE_RIGHT (-1).
 * @param[in] angle_deg Turn angle in degrees.
 *
 * @return true if the turn completed normally.
 */
static bool
turn_side (float32_t side, float32_t angle_deg)
{
    if (side > 0.0f)
    {
        motion_turn_left(angle_deg);
    }
    else
    {
        motion_turn_right(angle_deg);
    }

    return (run_move());
}

/*!
 * @brief Drive forward and wait.
 *
 * @param[in] dist_mm Distance in mm.
 * @param[in] speed   Speed in mm/s.
 *
 * @return true if the move completed normally.
 */
static bool
forward (float32_t dist_mm, float32_t speed)
{
    motion_move_forward(dist_mm, speed);

    return (run_move());
}

/*!
 * @brief Whether the line is centred under the two line sensors.
 *
 * @param[in] p_ir IR snapshot.
 *
 * @return true if both sensors see about the same, and enough, black.
 */
static bool
line_centred (ir_sensor_snapshot_t const * p_ir)
{
    float32_t left  = p_ir->norm[IR_LEFT];
    float32_t right = p_ir->norm[IR_RIGHT];

    return ((car_math_abs(left - right) < CENTRED_ERROR)
            && ((left + right) > CENTRED_SUM));
}

/*!
 * @brief One 20 ms step of a line search.
 *
 * @return SEARCH_ABORTED on STOP, SEARCH_FOUND once the line is centred,
 *         otherwise SEARCH_RUNNING.
 */
static search_result_t
reacquire_poll (void)
{
    ir_sensor_snapshot_t ir_snap;
    search_result_t      result = SEARCH_RUNNING;

    (void) tk_dly_tsk(LOOP_MS);
    ir_sensor_read(&ir_snap);
    poll_inputs();

    if (gb_abort)
    {
        result = SEARCH_ABORTED;
    }
    else if (line_centred(&ir_snap))
    {
        result = SEARCH_FOUND;
    }
    else
    {
        /* Keep sweeping. */
    }

    return (result);
}

/*!
 * @brief Rotate in growing alternating sweeps until the line is centred
 *        under the sensors.
 *
 * @param[in] side Direction of the first sweep (SIDE_LEFT or SIDE_RIGHT).
 *
 * @return true if the line was found.
 */
static bool
reacquire_line (float32_t side)
{
    uint32_t        sweep      = 0u;
    uint32_t        elapsed_ms = 0u;
    uint32_t        sweep_ms   = 0u;
    float32_t       dir        = side;
    search_result_t result     = SEARCH_RUNNING;

    for (sweep = 0u; (sweep < REACQ_SWEEPS) && (SEARCH_RUNNING == result);
         sweep++)
    {
        /* Even sweeps go towards side, odd sweeps back the other way. */
        dir      = (0u == (sweep % 2u)) ? side : -side;
        sweep_ms = REACQ_BASE_MS * (sweep + 1u);
        motion_set_velocity(-dir * REACQ_SPEED, dir * REACQ_SPEED);

        for (elapsed_ms = 0u;
             (elapsed_ms < sweep_ms) && (SEARCH_RUNNING == result);
             elapsed_ms += LOOP_MS)
        {
            result = reacquire_poll();
        }
    }

    motion_stop();

    if (SEARCH_FOUND == result)
    {
        line_follow_init(&g_lf);
    }

    return (SEARCH_FOUND == result);
}

/*!
 * @brief Execute a barcode navigation command (Buddy 3 with Buddy 2).
 *
 * @param[in] cmd Navigation command.
 */
static void
execute_nav (barcode_decode_nav_t cmd)
{
    bool b_ok = true;

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    (void) tm_printf((UB const *) "[vehicle] executing %s\n",
                     barcode_decode_command_name(cmd));

    switch (cmd)
    {
        case NAV_LEFT:
            b_ok = (forward(LINE_SENSOR_AHEAD_MM, SPEED_SLOW_MM_S))
                   && (turn_side(SIDE_LEFT, QUARTER_TURN))
                   && (reacquire_line(SIDE_LEFT));
        break;

        case NAV_RIGHT:
            b_ok = (forward(LINE_SENSOR_AHEAD_MM, SPEED_SLOW_MM_S))
                   && (turn_side(SIDE_RIGHT, QUARTER_TURN))
                   && (reacquire_line(SIDE_RIGHT));
        break;

        case NAV_UTURN:
            b_ok = (turn_side(SIDE_LEFT, HALF_TURN))
                   && (reacquire_line(SIDE_LEFT));
        break;

        case NAV_STRAIGHT:
            /* Same as the default: cross the junction. */
        default:
            b_ok = forward(LINE_SENSOR_AHEAD_MM + STRAIGHT_EXTRA_MM,
                           SPEED_SLOW_MM_S);
        break;
    }

    g_st.last_nav    = cmd;
    g_st.pending_nav = NAV_NONE;
    line_follow_init(&g_lf);

    if (b_ok)
    {
        set_state(VS_LINE_FOLLOW, "nav done");
    }
    else if (!gb_abort)
    {
        set_state(VS_LINE_SEARCH, "nav: line not found");
    }
    else
    {
        /* Stopped: handle_requests() sets the state on the next loop. */
    }
}

/*!
 * @brief Drive alongside the obstacle until the side-looking sensor sees
 *        past its far end, then clear the car's tail.
 *
 * @param[in] side   Bypass side, SIDE_LEFT or SIDE_RIGHT.
 * @param[in] offset Sideways offset of the car from the obstacle's edge.
 *
 * @return true if every move completed normally.
 */
static bool
pass_obstacle (float32_t side, float32_t offset)
{
    int32_t   look    = (side > 0.0f) ? OBST_SIDE_SERVO_DEG
                                      : (SERVO_FULL_DEG - OBST_SIDE_SERVO_DEG);
    float32_t travel  = 0.0f;
    int32_t   dist_mm = ULTRASONIC_NO_ECHO;
    bool      b_ok    = true;
    bool      b_near  = false;
    bool      b_seen  = false;
    bool      b_past  = false;

    while (b_ok && (!b_past) && (travel < OBST_PASS_MAX_MM))
    {
        dist_mm = obstacle_look_mm(look);

        /* Cast: distances are at most US_MAX_MM, exact in float32_t. */
        b_near = (ULTRASONIC_NO_ECHO != dist_mm)
                 && ((float32_t) dist_mm < (offset + SIDE_SEEN_EXTRA_MM));
        b_past = b_seen && (!b_near);
        b_seen = b_seen || b_near;

        if (!b_past)
        {
            b_ok = forward(PASS_STEP_MM, SPEED_SLOW_MM_S);
            travel += PASS_STEP_MM;
        }
    }

    servo_set_angle(SERVO_CENTRE_DEG);

    if (b_ok)
    {
        b_ok = forward(CAR_LENGTH_MM, SPEED_SLOW_MM_S);
    }

    return (b_ok);
}

/*!
 * @brief Drive back towards the line until the IR row sees it.
 *
 * @param[in] max_mm Give up after this distance.
 *
 * @return true if the line was seen.
 */
static bool
seek_line (float32_t max_mm)
{
    ir_sensor_snapshot_t ir_snap;
    motion_status_t      motion;
    float32_t            start  = 0.0f;
    float32_t            limit  = car_math_is_finite(max_mm) ? max_mm : 0.0f;
    search_result_t      result = SEARCH_RUNNING;

    motion_get_status(&motion);
    start = motion.odo_mm;
    motion_set_velocity(SPEED_SLOW_MM_S, SPEED_SLOW_MM_S);

    while (SEARCH_RUNNING == result)
    {
        (void) tk_dly_tsk(LOOP_MS);
        poll_inputs();
        ir_sensor_read(&ir_snap);
        motion_get_status(&motion);

        if ((ir_snap.norm[IR_LEFT] > LINE_SEEN_LEVEL)
            || (ir_snap.norm[IR_RIGHT] > LINE_SEEN_LEVEL))
        {
            result = SEARCH_FOUND;
        }
        else if (gb_abort || ((motion.odo_mm - start) > limit))
        {
            result = SEARCH_ABORTED;
        }
        else
        {
            /* Keep driving. */
        }
    }

    motion_stop();

    return (SEARCH_FOUND == result);
}

/*!
 * @brief Box-shaped bypass that keeps the deviation to the planned
 *        offset: sidestep, pass (side-looking sonar), return, rejoin the
 *        line.
 *
 * @param[in] p_plan Plan with AVOID_TURN_LEFT or AVOID_TURN_RIGHT.
 *
 * @return true if the car is back on the line.
 */
static bool
bypass (avoidance_plan_t const * p_plan)
{
    float32_t side =
        (AVOID_TURN_LEFT == p_plan->action) ? SIDE_LEFT : SIDE_RIGHT;
    bool b_ok = false;

    obstacle_monitor_enable(false);
    b_ok = (turn_side(side, QUARTER_TURN))
           && (forward(p_plan->offset_mm, SPEED_SLOW_MM_S))
           && (turn_side(-side, QUARTER_TURN))
           && (pass_obstacle(side, p_plan->offset_mm))
           && (turn_side(-side, QUARTER_TURN))
           && (seek_line(p_plan->offset_mm + SEEK_EXTRA_MM))
           && (forward(LINE_SENSOR_AHEAD_MM, SPEED_SLOW_MM_S))
           && (turn_side(side, QUARTER_TURN)) && (reacquire_line(side));
    servo_set_angle(SERVO_CENTRE_DEG);
    obstacle_monitor_enable(true);

    return (b_ok);
}

/*!
 * @brief Plan and carry out the avoidance action for a completed scan.
 *
 * @param[in] p_prof Obstacle profile from obstacle_scan().
 */
static void
act_on_scan (avoidance_profile_t const * p_prof)
{
    avoidance_plan_t plan;
    motion_status_t  motion;
    INT              offset_mm = 0;

    plan             = avoidance_plan(p_prof, g_attempts);
    g_st.last_action = plan.action;
    offset_mm        = car_math_round(plan.offset_mm);

    /* Cast: tm_printf() takes the kernel's UB string type; ASCII. */
    (void) tm_printf((UB const *) "[vehicle] plan %s offset=%d mm\n",
                     avoidance_action_name(plan.action), offset_mm);

    switch (plan.action)
    {
        case AVOID_CONTINUE:
            g_attempts = 0u;
            motion_get_status(&motion);
            g_front_ignore_until = motion.odo_mm + FRONT_IGNORE_MM;
            set_state(VS_LINE_FOLLOW, "path clear");
        break;

        case AVOID_REVERSE:
            /* Back off; the state stays OBSTACLE, so it re-scans. */
            g_attempts++;
            motion_move_backward(REVERSE_MM, SPEED_SLOW_MM_S);
            (void) run_move();
        break;

        case AVOID_TURN_LEFT:
            /* Same handling as AVOID_TURN_RIGHT. */
        case AVOID_TURN_RIGHT:
            g_attempts = 0u;

            if (bypass(&plan))
            {
                g_st.obstacles_passed++;
                set_state(VS_LINE_FOLLOW, "obstacle bypassed");
            }
            else if (!gb_abort)
            {
                set_state(VS_LINE_SEARCH, "bypass: line not found");
            }
            else
            {
                /* Stopped: handle_requests() sets the state next loop. */
            }
        break;

        case AVOID_STOP:
            /* Same handling as an unexpected action. */
        default:
            g_attempts = 0u;
            set_state(VS_STOPPED, "path blocked");
        break;
    }
}

/*!
 * @brief Obstacle state (Buddy 5 with Buddy 2): stop, back off after an
 *        impact, scan, then plan and act.  A STOP during the reverse or the
 *        scan skips the rest; handle_requests() then stops the car.
 */
static void
handle_obstacle (void)
{
    avoidance_profile_t prof;

    motion_stop();
    (void) tk_dly_tsk(STOP_SETTLE_MS);

    /* After an impact the obstacle can be closer than US_MIN_MM, where the
       sonar gets no echo and the scan would report a clear path. */
    if (gb_after_impact)
    {
        gb_after_impact = false;
        motion_move_backward(REVERSE_MM, SPEED_SLOW_MM_S);
        (void) run_move();
    }

    if (!gb_abort)
    {
        (void) obstacle_scan(&prof, stop_requested);
    }

    if (!gb_abort)
    {
        act_on_scan(&prof);
    }
}

/*!
 * @brief Take a decoded barcode, if any, as the pending navigation
 *        command.
 *
 * @param[in] odo_mm Current odometer reading.
 */
static void
take_barcode (float32_t odo_mm)
{
    barcode_event_t event;

    if ((barcode_get_event(&event, TMO_POL)) && (NAV_NONE != event.cmd))
    {
        g_st.pending_nav = event.cmd;
        g_st.barcodes++;
        g_pending_odo = odo_mm;
    }
}

/*!
 * @brief Whether the pending navigation command should run now: at the
 *        next junction (or at once, if so configured), immediately for a
 *        U-turn, or after NAV_PENDING_MAX_MM without a junction.
 *
 * @param[in] odo_mm Current odometer reading.
 *
 * @return true to execute the pending command.
 */
static bool
nav_due (float32_t odo_mm)
{
    barcode_decode_nav_t pending = g_st.pending_nav;

    return ((NAV_NONE != pending)
            && ((0 == NAV_EXECUTE_AT_JUNCTION) || (NAV_UTURN == pending)
                || (LINE_JUNCTION == g_st.line_state)
                || ((odo_mm - g_pending_odo) > NAV_PENDING_MAX_MM)));
}

/*!
 * @brief Whether an obstacle is ahead (front sonar, confirmed by
 *        OBST_CONFIRM_READINGS readings) or the car was hit (IMU impact).
 *        Both triggers are ignored while the IMU reports the car on a hump:
 *        the sonar sees the floor when the nose dips, and the landing jolt
 *        can look like an impact.  Updates the published front distance
 *        and records whether an impact was the reason.
 *
 * @param[in] odo_mm Current odometer reading.
 *
 * @return true to enter the obstacle state.
 */
static bool
obstacle_due (float32_t odo_mm)
{
    imu_status_t imu;
    bool         b_hump  = false;
    bool         b_ahead = false;
    bool         b_hit   = false;

    g_st.front_mm = obstacle_front_mm();
    imu_get_status(&imu);
    b_hump = (HUMP_FLAT != imu.hump.state);
    b_ahead =
        (!b_hump) && (obstacle_ahead()) && (odo_mm > g_front_ignore_until);
    b_hit = (!b_hump) && (imu.impacts > g_seen_impacts);

    /* Always follow the IMU's impact counter: an impact on a hump is
       dropped, not kept for later, and a smaller count is the reset at the
       start of a run (imu_reset_run()), not an impact. */
    g_seen_impacts  = imu.impacts;
    gb_after_impact = b_hit;

    return (b_ahead || b_hit);
}

/*!
 * @brief Line-following state (Buddies 2, 3, 4 and 5): classify the line,
 *        watch for barcodes, line loss and obstacles, then steer.
 */
static void
handle_line_follow (void)
{
    ir_sensor_snapshot_t ir_snap;
    motion_status_t      motion;
    float32_t            steer = 0.0f;
    float32_t            speed = 0.0f;

    ir_sensor_read(&ir_snap);
    motion_get_status(&motion);
    g_st.line_state = line_follow_classify(&g_lf, ir_snap.norm[IR_LEFT],
                                           ir_snap.norm[IR_RIGHT],
                                           motion.odo_mm - g_prev_odo, LOOP_MS);
    g_prev_odo      = motion.odo_mm;
    take_barcode(motion.odo_mm);

    if (nav_due(motion.odo_mm))
    {
        motion_stop();
        set_state(VS_NAV_TURN, barcode_decode_command_name(g_st.pending_nav));
    }
    else if (LINE_LOST == g_st.line_state)
    {
        set_state(VS_LINE_SEARCH, "line lost");
    }
    else if (obstacle_due(motion.odo_mm))
    {
        motion_stop();
        set_state(VS_OBSTACLE, "obstacle ahead / impact");
    }
    else
    {
        steer = line_follow_steer(&g_lf, ir_snap.norm[IR_LEFT],
                                  ir_snap.norm[IR_RIGHT], DT_S);

        /* Cast: the cruise speed is 60..300 mm/s, exact in float32_t. */
        speed = line_follow_speed((float32_t) g_st.cruise_mm_s, g_lf.error);
        g_st.line_error = g_lf.error;
        motion_set_velocity(speed - steer, speed + steer);
    }
}

/*!
 * @brief Line-search state: sweep towards where the line was last seen.
 */
static void
handle_line_search (void)
{
    float32_t side = (g_lf.last_seen_error >= 0.0f) ? SIDE_LEFT : SIDE_RIGHT;

    if (reacquire_line(side))
    {
        set_state(VS_LINE_FOLLOW, "line reacquired");
    }
    else if (!gb_abort)
    {
        set_state(VS_STOPPED, "line not found / end");
    }
    else
    {
        /* Stopped: handle_requests() sets the state on the next loop. */
    }
}

/*!
 * @brief Begin a mission run.  Barcodes decoded while the car stood still
 *        (pushed by hand, or seen during the calibration spin) are
 *        discarded: they are not commands for this run.
 */
static void
start_run (void)
{
    imu_status_t    imu;
    barcode_event_t stale;
    uint32_t        discarded = 0u;

    while (barcode_get_event(&stale, TMO_POL))
    {
        discarded++;
    }

    if (discarded > 0u)
    {
        /* Casts: tm_printf() takes the kernel's UB string type (ASCII); at
           most a few queued events, so the count fits INT. */
        (void) tm_printf((UB const *) "[vehicle] %d stale barcode(s) "
                                      "discarded\n",
                         (INT) discarded);
    }

    imu_reset_run();
    imu_get_status(&imu);
    g_seen_impacts = imu.impacts;
    line_follow_init(&g_lf);
    g_st.pending_nav      = NAV_NONE;
    g_st.barcodes         = 0u;
    g_st.obstacles_passed = 0u;
    g_attempts            = 0u;
    g_run_start_ms        = now_ms();
    set_state(VS_LINE_FOLLOW, "start");
}

/*!
 * @brief Spin once on the spot to calibrate the IR sensors and the
 *        magnetometer.
 */
static void
calibrate (void)
{
    bool b_ok = false;

    set_state(VS_CALIBRATE, "spin 360");
    ir_sensor_calib_begin();
    imu_mag_calib_begin();
    motion_turn_left(FULL_TURN);
    (void) run_move();
    b_ok = ir_sensor_calib_end();
    imu_mag_calib_end();
    set_state(VS_IDLE, b_ok ? "calibrated" : "IR contrast too low");
}

/*!
 * @brief Act on stop, start and calibrate requests.
 */
static void
handle_requests (void)
{
    if (gb_abort)
    {
        gb_abort = false;
        gb_start = false;
        motion_stop();
        servo_set_angle(SERVO_CENTRE_DEG);
        set_state(VS_STOPPED, "stop command");
    }

    if ((VS_IDLE == g_st.state) || (VS_STOPPED == g_st.state))
    {
        if (gb_calib)
        {
            gb_calib = false;
            calibrate();
        }
        else if (gb_start)
        {
            gb_start = false;
            start_run();
        }
        else
        {
            motion_stop();
        }
    }
    else
    {
        /* Requests are ignored during a run. */
        gb_start    = false;
        gb_calib    = false;
        g_st.run_ms = now_ms() - g_run_start_ms;
    }
}

/*!
 * @brief One step of the current state.
 */
static void
run_state (void)
{
    switch (g_st.state)
    {
        case VS_LINE_FOLLOW:
            handle_line_follow();
        break;

        case VS_NAV_TURN:
            execute_nav(g_st.pending_nav);
        break;

        case VS_LINE_SEARCH:
            handle_line_search();
        break;

        case VS_OBSTACLE:
            handle_obstacle();
        break;

        default:
            /* IDLE, CALIBRATE and STOPPED need no periodic work. */
        break;
    }
}

/*!
 * @brief Vehicle task: the mission state machine, every LOOP_MS.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
vehicle_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    /* The line follower is private to this task, so it is set up here. */
    line_follow_init(&g_lf);

    /* From now on a stalled vehicle loop (a hung or starved task) resets
       the chip, which stops the motors. */
    hal_watchdog_start(WATCHDOG_TIMEOUT_MS);

    for (;;)
    {
        poll_inputs();
        handle_requests();
        run_state();
        (void) tk_dly_tsk(LOOP_MS);
    }
}

/*** end of file ***/
