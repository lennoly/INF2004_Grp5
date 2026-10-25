
#define main original_tests_main
#include "../../testing/subsystem3/host/test_driving.c"
#undef main
INT tk_rcv_mbf(ID id,void *out,INT t) { (void)id;(void)out;(void)t;return -1; }
ER tk_dly_tsk(uint32_t ms) { (void)ms; return E_OK; }
void motion_move_forward(float32_t d,float32_t s) { (void)d; motion_set_velocity(s,s); }
void motion_turn_left(float32_t a) { (void)a;motion_set_velocity(-120,120); }
void motion_turn_right(float32_t a) { (void)a;motion_set_velocity(120,-120); }
int32_t motion_wait(int32_t timeout) { (void)timeout;motion_stop();return MOTION_WAIT_FAIL; }
int main(void) {
 original_tests_main();
 sensor.norm[0]=1; sensor.norm[1]=1; wheels.odo_mm=500; g_prev_odo=100;
 start_run();
 event_ready=false;
 handle_line_follow();
 printf("restart with no new travel: line_state=%d junction_mm=%.1f (expected <10)\n",g_st.line_state,g_lf.junction_mm);
 /* Mimic returning from a blocking manoeuvre with a barcode queued during it. */
 g_st.state=VS_LINE_FOLLOW;line_follow_init(&g_lf);g_prev_odo=100;wheels.odo_mm=500;
 next_event.cmd=NAV_LEFT;next_event.text[0]='Y';event_ready=true;
 handle_line_follow();
 printf("post-manoeuvre queued barcode, no new travel: state=%s\n",vehicle_state_name(g_st.state));
 sensor.norm[0]=.5f;sensor.norm[1]=.5f; g_st.state=VS_LINE_FOLLOW;
 motion_stop(); /* exact telemetry fallback when STOP queue submission fails */
 handle_line_follow();
 printf("STOP fallback next vehicle step: state=%s velocities=%.0f,%.0f (expected stopped)\n",vehicle_state_name(g_st.state),velocity_left,velocity_right);
 g_st.state=VS_NAV_TURN; g_st.pending_nav=NAV_LEFT;g_st.pending_code='Y';
 execute_nav(NAV_LEFT);
 printf("failed navigation move: state=%s (motor failure is treated as line loss)\n",vehicle_state_name(g_st.state));
 return 0;
}
