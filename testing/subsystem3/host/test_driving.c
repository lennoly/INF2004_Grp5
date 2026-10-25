/* Exercise the actual vehicle controller without robot hardware.
 * No IMU, sonar or servo stubs: a reachable dependency fails to link. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/vehicle.c"

static bool stop_latched;
static vehicle_msg_t command_queue[4];
static unsigned queue_count;
static float32_t pending_distance;
static unsigned moves_issued;
static int32_t next_move_result = MOTION_WAIT_OK;
static unsigned fail_on_move;
static bool stop_at_completion;
static bool start_pressed;
static bool stop_pressed;
static bool barcode_enabled;
static bool event_ready;
static barcode_event_t next_event;
static ir_sensor_snapshot_t sensor;
static motion_status_t wheels;
static float32_t velocity_left;
static float32_t velocity_right;

bool hal_gpio_get(uint32_t pin)
{
    return !(pin == PIN_BTN_START ? start_pressed : stop_pressed);
}
ER tk_get_tim(SYSTIM *p_time) { memset(p_time, 0, sizeof(*p_time)); return E_OK; }
INT tm_printf(UB const *format, ...) { (void) format; return 0; }
void motion_stop(void) { velocity_left = 0; velocity_right = 0; }
void motion_emergency_stop(void) { stop_latched = true; motion_stop(); }
void motion_clear_stop(void) { stop_latched = false; }
bool motion_stop_requested(void) { return stop_latched; }
INT tk_rcv_mbf(ID id, void *out, INT timeout)
{
    (void)id; (void)timeout;
    if (!queue_count) { return -1; }
    memcpy(out, command_queue, sizeof(vehicle_msg_t));
    queue_count--;
    memmove(command_queue, command_queue + 1, queue_count * sizeof(vehicle_msg_t));
    return sizeof(vehicle_msg_t);
}
ER tk_snd_mbf(ID id, void const *src, INT size, INT timeout)
{
    (void)id; (void)timeout;
    if (queue_count == 4) { return -1; }
    memcpy(&command_queue[queue_count++], src, (size_t)size);
    return E_OK;
}
ER tk_dly_tsk(uint32_t ms) { (void)ms; return E_OK; }
void motion_move_forward(float32_t d, float32_t speed)
{
    pending_distance = d; moves_issued++; motion_set_velocity(speed, speed);
}
void motion_turn_left(float32_t angle)
{
    (void)angle; pending_distance = 0; moves_issued++; motion_set_velocity(-120,120);
}
void motion_turn_right(float32_t angle)
{
    (void)angle; pending_distance = 0; moves_issued++; motion_set_velocity(120,-120);
}
int32_t motion_wait(int32_t timeout)
{
    (void)timeout;
    if (stop_at_completion) {
        stop_at_completion = false;
        assert(vehicle_command(VCMD_STOP, 0));
    }
    if (fail_on_move && moves_issued == fail_on_move) { return MOTION_WAIT_FAIL; }
    if (next_move_result == MOTION_WAIT_OK) { wheels.odo_mm += pending_distance; }
    pending_distance = 0;
    motion_stop(); return next_move_result;
}
void motion_set_velocity(float32_t left, float32_t right)
{
    if (!stop_latched) { velocity_left = left; velocity_right = right; }
}
void motion_get_status(motion_status_t *p_out) { *p_out = wheels; }
void ir_sensor_read(ir_sensor_snapshot_t *p_out) { *p_out = sensor; }
void barcode_set_enabled(bool enabled)
{
    barcode_enabled = enabled; event_ready = false;
}
bool barcode_get_event(barcode_event_t *p_out, int32_t timeout)
{
    (void) timeout;
    if (!event_ready) { return false; }
    *p_out = next_event; event_ready = false; return true;
}

int main(void)
{
    g_st.state = VS_IDLE;
    g_st.cruise_mm_s = 180;
    sensor.norm[IR_LEFT] = 0.5f;
    sensor.norm[IR_RIGHT] = 0.5f;
    start_pressed = true;
    poll_buttons(); handle_requests();
    assert(g_st.state == VS_IDLE && !barcode_enabled);
    assert(!barcode_decode_get_mapping(NULL));
    assert(barcode_decode_set_mapping("WXYZ"));
    start_pressed = false;
    poll_buttons(); handle_requests();
    start_pressed = true;
    poll_buttons(); handle_requests();
    assert(g_st.state == VS_LINE_FOLLOW && barcode_enabled);
    handle_line_follow();
    assert(velocity_left > 0 && velocity_right > 0);

    /* Steering follows the left sensor; no obstacle polling is linked. */
    sensor.norm[IR_LEFT] = 0.8f;
    sensor.norm[IR_RIGHT] = 0.2f;
    handle_line_follow();
    assert(velocity_right > velocity_left);

    /* A left barcode waits for a junction. */
    next_event.cmd = barcode_decode_to_command('Y');
    strcpy(next_event.text, "Y"); event_ready = true;
    handle_line_follow();
    assert(g_st.pending_nav == NAV_LEFT && g_st.state == VS_LINE_FOLLOW);
    assert(g_st.pending_code == 'Y');

    /* An unassigned scan must not overwrite the letter awaiting a junction. */
    next_event.cmd = barcode_decode_to_command('A');
    strcpy(next_event.text, "A"); event_ready = true;
    handle_line_follow();
    assert(g_st.pending_code == 'Y' && g_st.pending_nav == NAV_LEFT);
    sensor.norm[IR_LEFT] = 1.0f;
    sensor.norm[IR_RIGHT] = 1.0f;
    wheels.odo_mm = 11.0f;
    handle_line_follow();
    assert(g_st.state == VS_NAV_TURN);
    assert(velocity_left == 0 && velocity_right == 0);

    /* STOP overrides START, disables barcode capture and never calibrates. */
    stop_pressed = true;
    poll_buttons(); handle_requests();
    assert(g_st.state == VS_STOPPED && !barcode_enabled && !gb_calib);
    poll_buttons(); handle_requests();
    assert(g_st.state == VS_STOPPED);
    start_pressed = false; stop_pressed = false;
    poll_buttons(); handle_requests();
    start_pressed = true;
    poll_buttons(); handle_requests();
    assert(g_st.state == VS_LINE_FOLLOW && g_st.pending_nav == NAV_NONE);
    assert(g_st.pending_code == '\0');

    /* A cross without a barcode stays in line-follow mode. */
    wheels.odo_mm += 11.0f;
    handle_line_follow();
    assert(g_st.state == VS_LINE_FOLLOW);
    /* A configured U-turn also waits for a junction, even after 500 mm. */
    sensor.norm[IR_LEFT] = 0.5f; sensor.norm[IR_RIGHT] = 0.5f;
    next_event.cmd = barcode_decode_to_command('X'); event_ready = true;
    handle_line_follow();
    assert(g_st.pending_nav == NAV_UTURN && g_st.state == VS_LINE_FOLLOW);
    wheels.odo_mm += 500.0f;
    handle_line_follow();
    assert(g_st.state == VS_LINE_FOLLOW);
    sensor.norm[IR_LEFT] = 1.0f; sensor.norm[IR_RIGHT] = 1.0f;
    wheels.odo_mm += 11.0f;
    handle_line_follow();
    assert(g_st.state == VS_NAV_TURN);
    start_run();
    assert(g_st.state == VS_LINE_FOLLOW && g_st.pending_nav == NAV_NONE);

    /* White for 160 ms starts the existing line-search routine. */
    sensor.norm[IR_LEFT] = 0; sensor.norm[IR_RIGHT] = 0;
    for (unsigned i = 0; i < 8; i++) { handle_line_follow(); }
    assert(g_st.state == VS_LINE_SEARCH);
    /* STOP bypasses a full queue, wins over queued START, and stays stopped. */
    sensor.norm[0] = .5f; sensor.norm[1] = .5f;
    start_pressed = false;
    g_st.state = VS_LINE_FOLLOW;
    for (unsigned i = 0; i < 4; i++) { assert(vehicle_command(VCMD_START, 0)); }
    assert(queue_count == 4 && !vehicle_command(VCMD_START, 0));
    gb_calib = true; /* STOP must discard every pending movement request. */
    assert(vehicle_command(VCMD_STOP, 0));
    assert(stop_latched);
    poll_inputs(); handle_requests(); run_state();
    assert(g_st.state == VS_STOPPED && !queue_count && !gb_calib);
    for (unsigned i = 0; i < 5; i++) {
        poll_inputs(); handle_requests(); run_state();
        assert(g_st.state == VS_STOPPED && !velocity_left && !velocity_right);
    }
    assert(vehicle_command(VCMD_START, 0));
    poll_inputs(); handle_requests();
    assert(g_st.state == VS_LINE_FOLLOW && !stop_latched);

    /* START ignores travel that happened before this run. */
    wheels.odo_mm = 500; g_prev_odo = 100;
    start_run(); sensor.norm[0] = 1; sensor.norm[1] = 1;
    handle_line_follow();
    assert(g_st.line_state == LINE_ON && g_lf.junction_mm == 0);

    /* A successful manoeuvre rebases odometry before accepting another code. */
    sensor.norm[0] = .5f; sensor.norm[1] = .5f;
    g_st.state = VS_NAV_TURN; next_move_result = MOTION_WAIT_OK;
    execute_nav(NAV_STRAIGHT);
    assert(g_prev_odo == wheels.odo_mm);
    sensor.norm[0] = 1; sensor.norm[1] = 1;
    next_event.cmd = NAV_LEFT; next_event.text[0] = 'Y'; event_ready = true;
    handle_line_follow();
    assert(g_st.state == VS_LINE_FOLLOW && g_st.line_state == LINE_ON);
    wheels.odo_mm += 11; handle_line_follow();
    assert(g_st.state == VS_NAV_TURN);

    /* A failed first leg must stop instead of starting a turn or search. */
    moves_issued = 0; next_move_result = MOTION_WAIT_FAIL;
    execute_nav(NAV_LEFT);
    assert(moves_issued == 1 && g_st.state == VS_STOPPED && stop_latched);
    run_state();
    assert(moves_issued == 1 && !velocity_left && !velocity_right);
    /* Failures in either forward or turning leg cannot begin recovery. */
    for (unsigned leg = 1; leg <= 2; leg++) {
        start_run(); moves_issued = 0; fail_on_move = leg;
        next_move_result = MOTION_WAIT_OK;
        execute_nav(NAV_LEFT);
        assert(moves_issued == leg && g_st.state == VS_STOPPED && stop_latched);
    }
    fail_on_move = 0;
    start_run(); moves_issued = 0; stop_at_completion = true;
    execute_nav(NAV_LEFT);
    assert(moves_issued == 1 && g_st.state == VS_STOPPED && stop_latched);
    puts("Buddy 3 driving: start, stop, steering, barcode junction and line loss PASS");
    return 0;
}
