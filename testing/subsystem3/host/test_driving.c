/* Exercise the actual vehicle controller without robot hardware.
 * No IMU, sonar or servo stubs: a reachable dependency fails to link. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/vehicle.c"

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
void motion_set_velocity(float32_t left, float32_t right)
{
    velocity_left = left; velocity_right = right;
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
    puts("Buddy 3 driving: start, stop, steering, barcode junction and line loss PASS");
    return 0;
}
