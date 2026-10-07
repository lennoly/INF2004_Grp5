/* Host simulation of the vehicle task: see vehicle_sim.h.
 *
 * vehicle.c is included so that one call of vehicle_sim_run() can execute
 * the same loop body as vehicle_task() (poll_inputs, handle_requests,
 * run_state, then a LOOP_MS delay).  obstacle.c and servo.c are compiled
 * as they are; the obstacle monitor task is run one iteration at a time.
 * Simulated time only advances in kernel delays, motion waits and pings,
 * which is where buttons, commands and sonar readings take effect. */
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "vehicle.c"
#include "vehicle_sim.h"

#define SIM_START_MS    (1000u)
#define SIM_MOVE_MS     (400)  /* every simulated move takes this long */
#define SIM_PING_MS     (61u)  /* HC-SR04 cycle incl. the 60 ms gap     */
#define SIM_BOX_MM      (200)  /* box ahead, seen between these angles  */
#define SIM_BOX_FROM    (75)
#define SIM_BOX_TO      (105)
#define SIM_ON_LINE     (0.6f) /* normalised IR level on / off the line */
#define SIM_OFF_LINE    (0.0f)
#define SIM_STOP_DELAY  (100u) /* vehicle_sim_stop_on_next_turn()       */
#define SIM_STOP_HOLD   (30u)
#define SIM_MAX_PRESSES (16u)
#define SIM_MAX_QUEUE   (8u)
#define SIM_MAX_MSG     (64u)
#define SIM_MAX_TASKS   (4u)
#define SIM_LOG_SIZE    (16384u)

typedef struct
{
    uint32_t pin;
    uint32_t down_ms;
    uint32_t up_ms;
} sim_press_t;

static uint32_t             gs_now = SIM_START_MS;
static sim_press_t          gs_press[SIM_MAX_PRESSES];
static uint32_t             gs_presses;
static hal_irq_gpio_cb_t    gs_stop_isr;
static bool                 gs_b_cmd_pending;
static uint32_t             gs_cmd_at;
static bool                 gs_b_stop_on_turn;
static char                 gs_log[SIM_LOG_SIZE];
static uint32_t             gs_pings;
static bool                 gs_b_box;
static float32_t            gs_ir = SIM_ON_LINE;
static terrain_hump_state_t gs_hump = HUMP_FLAT;
static uint32_t             gs_impacts;
static int32_t              gs_move_left_ms;
static float32_t            gs_odo;
static float32_t            gs_vel;
static uint32_t             gs_servo_us;
static int32_t              gs_front = ULTRASONIC_NO_ECHO;
static FP                   gs_tasks[SIM_MAX_TASKS];
static uint32_t             gs_tasks_made;
static jmp_buf              gs_jump;
static bool                 gs_b_in_monitor;
static uint8_t              gs_queue[SIM_MAX_QUEUE][SIM_MAX_MSG];
static INT                  gs_queue_size[SIM_MAX_QUEUE];
static uint32_t             gs_queue_head;
static uint32_t             gs_queue_count;
static barcode_decode_nav_t gs_barcodes[SIM_MAX_QUEUE];
static uint32_t             gs_barcode_count;
static uint32_t             gs_wd_timeout;
static uint32_t             gs_wd_last;
static uint32_t             gs_wd_gap;

static void sim_log(char const * p_format, ...)
{
    size_t  used = strlen(gs_log);
    va_list args;

    va_start(args, p_format);
    (void) vsnprintf(gs_log + used, sizeof(gs_log) - used, p_format, args);
    va_end(args);
}

static void sim_advance(uint32_t ms)
{
    uint32_t t0 = gs_now;
    uint32_t t1 = gs_now + ms;
    uint32_t idx;

    for (idx = 0u; idx < gs_presses; idx++)
    {
        if ((PIN_BTN_STOP == gs_press[idx].pin) && (gs_press[idx].down_ms > t0)
            && (gs_press[idx].down_ms <= t1) && (NULL != gs_stop_isr))
        {
            gs_stop_isr(PIN_BTN_STOP, HAL_GPIO_EDGE_FALL,
                        gs_press[idx].down_ms * 1000u);
        }
    }

    if (gs_b_cmd_pending && (gs_cmd_at > t0) && (gs_cmd_at <= t1))
    {
        gs_b_cmd_pending = false;
        sim_log("MQTTSTOP;");
        (void) vehicle_command(VCMD_STOP, 0);
    }

    gs_odo += gs_vel * ((float32_t) ms / 1000.0f);
    gs_now = t1;
}

static int32_t sim_distance(int32_t angle_deg)
{
    return ((gs_b_box && (angle_deg >= SIM_BOX_FROM) && (angle_deg <= SIM_BOX_TO))
                ? SIM_BOX_MM
                : ULTRASONIC_NO_ECHO);
}

static void sim_move(char const * p_what, float32_t amount)
{
    sim_log("%s%.0f;", p_what, (double) amount);
    gs_move_left_ms = SIM_MOVE_MS;

    if (gs_b_stop_on_turn && ((0 == strcmp(p_what, "L")) || (0 == strcmp(p_what, "R"))))
    {
        gs_b_stop_on_turn = false;
        vehicle_sim_press(PIN_BTN_STOP, SIM_STOP_DELAY, SIM_STOP_HOLD);
    }
}

/* ---- fake kernel ---- */
ID tk_cre_tsk(T_CTSK const * pk_ctsk)
{
    ID id = E_LIMIT;

    if (gs_tasks_made < SIM_MAX_TASKS)
    {
        gs_tasks[gs_tasks_made] = pk_ctsk->task;
        gs_tasks_made++;
        id = (ID) gs_tasks_made;
    }
    return (id);
}

ER tk_sta_tsk(ID tskid, INT stacd)
{
    (void) tskid;
    (void) stacd;
    return (E_OK);
}

ER tk_dly_tsk(RELTIM dlytim)
{
    if (gs_b_in_monitor)
    {
        longjmp(gs_jump, 1); /* one monitor iteration done */
    }
    sim_advance(dlytim);
    return (E_OK);
}

ID tk_cre_mbf(T_CMBF const * pk_cmbf)
{
    (void) pk_cmbf;
    return (1);
}

ER tk_snd_mbf(ID mbfid, void const * msg, INT msgsz, TMO tmout)
{
    ER       ercd = E_TMOUT;
    uint32_t tail = (gs_queue_head + gs_queue_count) % SIM_MAX_QUEUE;

    (void) mbfid;
    (void) tmout;
    if ((gs_queue_count < SIM_MAX_QUEUE) && (msgsz > 0) && ((uint32_t) msgsz <= SIM_MAX_MSG))
    {
        (void) memcpy(gs_queue[tail], msg, (size_t) msgsz);
        gs_queue_size[tail] = msgsz;
        gs_queue_count++;
        ercd = E_OK;
    }
    return (ercd);
}

INT tk_rcv_mbf(ID mbfid, void * msg, TMO tmout)
{
    INT size = E_TMOUT;

    (void) mbfid;
    (void) tmout;
    if (gs_queue_count > 0u)
    {
        size = gs_queue_size[gs_queue_head];
        (void) memcpy(msg, gs_queue[gs_queue_head], (size_t) size);
        gs_queue_head = (gs_queue_head + 1u) % SIM_MAX_QUEUE;
        gs_queue_count--;
    }
    return (size);
}

ER tk_get_tim(SYSTIM * pk_tim)
{
    pk_tim->hi = 0;
    pk_tim->lo = gs_now;
    return (E_OK);
}

INT tm_printf(UB const * format, ...)
{
    char    text[256];
    va_list args;

    va_start(args, format);
    (void) vsnprintf(text, sizeof(text), (char const *) format, args);
    va_end(args);
    text[strcspn(text, "\n")] = '\0';
    if ((NULL != strstr(text, "[scan]")) || (NULL != strstr(text, "[vehicle]")))
    {
        sim_log("{%s};", text);
    }
    return (0);
}

/* ---- fake HAL and subsystems ---- */
void hal_gpio_init_in(uint32_t pin, bool b_pull_up)
{
    (void) pin;
    (void) b_pull_up;
}

bool hal_gpio_get(uint32_t pin)
{
    bool     b_level = true; /* buttons are active low */
    uint32_t idx;

    for (idx = 0u; idx < gs_presses; idx++)
    {
        if ((gs_press[idx].pin == pin) && (gs_press[idx].down_ms <= gs_now)
            && (gs_now < gs_press[idx].up_ms))
        {
            b_level = false;
        }
    }
    return (b_level);
}

void hal_pwm_setup(uint32_t pin, uint32_t clkdiv, uint32_t wrap)
{
    (void) pin;
    (void) clkdiv;
    (void) wrap;
}

void hal_pwm_set_level(uint32_t pin, uint32_t level)
{
    (void) pin;
    gs_servo_us = level;
}

void hal_watchdog_start(uint32_t timeout_ms)
{
    gs_wd_timeout = timeout_ms;
    gs_wd_last    = gs_now;
}

void hal_watchdog_feed(void)
{
    uint32_t gap = gs_now - gs_wd_last;

    gs_wd_gap  = (gap > gs_wd_gap) ? gap : gs_wd_gap;
    gs_wd_last = gs_now;
}

bool hal_watchdog_caused_reset(void)
{
    return (false);
}

int32_t hal_irq_gpio_attach(uint32_t pin, uint32_t edges, hal_irq_gpio_cb_t callback)
{
    if ((PIN_BTN_STOP == pin) && (HAL_GPIO_EDGE_FALL == edges))
    {
        gs_stop_isr = callback;
    }
    return (E_OK);
}

void motion_move_forward(float32_t dist_mm, float32_t speed_mm_s)
{
    (void) speed_mm_s;
    sim_move("fwd", dist_mm);
}

void motion_move_backward(float32_t dist_mm, float32_t speed_mm_s)
{
    (void) speed_mm_s;
    sim_move("back", dist_mm);
}

void motion_turn_left(float32_t angle_deg)
{
    sim_move("L", angle_deg);
}

void motion_turn_right(float32_t angle_deg)
{
    sim_move("R", angle_deg);
}

void motion_stop(void)
{
    gs_move_left_ms = 0;
    gs_vel          = 0.0f;
}

void motion_set_velocity(float32_t left_mm_s, float32_t right_mm_s)
{
    gs_vel = 0.5f * (left_mm_s + right_mm_s);
}

int32_t motion_wait(int32_t timeout_ms)
{
    sim_advance((uint32_t) timeout_ms);
    gs_move_left_ms -= timeout_ms;
    return ((gs_move_left_ms <= 0) ? MOTION_WAIT_OK : MOTION_WAIT_BUSY);
}

void motion_get_status(motion_status_t * p_out)
{
    (void) memset(p_out, 0, sizeof(*p_out));
    p_out->odo_mm = gs_odo;
}

void ir_sensor_read(ir_sensor_snapshot_t * p_out)
{
    (void) memset(p_out, 0, sizeof(*p_out));
    p_out->norm[IR_LEFT]  = gs_ir;
    p_out->norm[IR_RIGHT] = gs_ir;
}

void ir_sensor_calib_begin(void)
{
    sim_log("ircal;");
}

bool ir_sensor_calib_end(void)
{
    return (true);
}

bool barcode_get_event(barcode_event_t * p_event, int32_t timeout_ms)
{
    bool b_got = false;

    (void) timeout_ms;
    if (gs_barcode_count > 0u)
    {
        (void) memset(p_event, 0, sizeof(*p_event));
        p_event->cmd = gs_barcodes[0];
        (void) memmove(&gs_barcodes[0], &gs_barcodes[1],
                       sizeof(gs_barcodes[0]) * (size_t) (gs_barcode_count - 1u));
        gs_barcode_count--;
        b_got = true;
    }
    return (b_got);
}

void imu_get_status(imu_status_t * p_out)
{
    (void) memset(p_out, 0, sizeof(*p_out));
    p_out->impacts    = gs_impacts;
    p_out->hump.state = gs_hump;
}

void imu_reset_run(void)
{
    gs_impacts = 0u;
}

void imu_mag_calib_begin(void)
{
}

void imu_mag_calib_end(void)
{
}

int32_t ultrasonic_init(void)
{
    return (E_OK);
}

int32_t ultrasonic_measure_mm(void)
{
    return (gs_front);
}

int32_t ultrasonic_median_mm(uint32_t samples)
{
    gs_pings++;
    sim_log("u%d;", (int) servo_get_angle());
    sim_advance(SIM_PING_MS * (samples + 1u)); /* worst case incl. the wait */
    return (sim_distance(servo_get_angle()));
}

/* ---- simulation API ---- */
void vehicle_sim_init(void)
{
    (void) obstacle_init(); /* first task created: the front monitor */
    (void) vehicle_init();

    /* The preamble of vehicle_task(). */
    line_follow_init(&g_lf);
    hal_watchdog_start(WATCHDOG_TIMEOUT_MS);
}

void vehicle_sim_step(void)
{
    /* The loop body of vehicle_task(). */
    poll_inputs();
    handle_requests();
    run_state();
    (void) tk_dly_tsk(LOOP_MS);
}

void vehicle_sim_run(uint32_t ms)
{
    uint32_t end = gs_now + ms;

    do
    {
        vehicle_sim_step();
    } while (gs_now < end);
}

void vehicle_sim_press(uint32_t pin, uint32_t after_ms, uint32_t hold_ms)
{
    uint32_t idx;
    uint32_t kept = 0u;

    /* Released presses no longer matter: drop them to make room. */
    for (idx = 0u; idx < gs_presses; idx++)
    {
        if (gs_press[idx].up_ms > gs_now)
        {
            gs_press[kept] = gs_press[idx];
            kept++;
        }
    }
    gs_presses = kept;

    if (gs_presses < SIM_MAX_PRESSES)
    {
        gs_press[gs_presses].pin     = pin;
        gs_press[gs_presses].down_ms = gs_now + after_ms;
        gs_press[gs_presses].up_ms   = gs_now + after_ms + hold_ms;
        gs_presses++;
    }
}

void vehicle_sim_stop_command_at(uint32_t after_ms)
{
    gs_b_cmd_pending = true;
    gs_cmd_at        = gs_now + after_ms;
}

void vehicle_sim_stop_on_next_turn(void)
{
    gs_b_stop_on_turn = true;
}

void vehicle_sim_front(int32_t mm)
{
    gs_front        = mm;
    gs_b_in_monitor = true;
    if (0 == setjmp(gs_jump))
    {
        gs_tasks[0](0, NULL); /* returns through tk_dly_tsk()'s longjmp */
    }
    gs_b_in_monitor = false;
}

void vehicle_sim_box(bool b_present)
{
    gs_b_box = b_present;
}

void vehicle_sim_line(bool b_on_line)
{
    gs_ir = b_on_line ? SIM_ON_LINE : SIM_OFF_LINE;
}

void vehicle_sim_hump(terrain_hump_state_t state)
{
    gs_hump = state;
}

void vehicle_sim_impact(void)
{
    gs_impacts++;
}

void vehicle_sim_barcode(barcode_decode_nav_t cmd)
{
    if (gs_barcode_count < SIM_MAX_QUEUE)
    {
        gs_barcodes[gs_barcode_count] = cmd;
        gs_barcode_count++;
    }
}

void vehicle_sim_clear_log(void)
{
    gs_log[0] = '\0';
}

uint32_t vehicle_sim_count(char const * p_text)
{
    uint32_t     count = 0u;
    char const * p_at  = strstr(gs_log, p_text);

    while (NULL != p_at)
    {
        count++;
        p_at = strstr(p_at + 1, p_text);
    }
    return (count);
}

bool vehicle_sim_log_starts(char const * p_text)
{
    return (0 == strncmp(gs_log, p_text, strlen(p_text)));
}

uint32_t vehicle_sim_pings(void)
{
    return (gs_pings);
}

uint32_t vehicle_sim_servo_us(void)
{
    return (gs_servo_us);
}

uint32_t vehicle_sim_queued_barcodes(void)
{
    return (gs_barcode_count);
}

uint32_t vehicle_sim_watchdog_gap_ms(void)
{
    uint32_t tail = gs_now - gs_wd_last;

    return ((tail > gs_wd_gap) ? tail : gs_wd_gap);
}

uint32_t vehicle_sim_watchdog_timeout_ms(void)
{
    return (gs_wd_timeout);
}
