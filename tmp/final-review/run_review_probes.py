
from pathlib import Path
import subprocess
root=Path.cwd(); out=root/'tmp/final-review'; app=root/'app_program'; host=root/'testing/subsystem3/host'
flags=['gcc','-std=c99','-Wall','-Wextra','-Werror','-O2','-flto','-fwhole-program','-I'+str(host),'-I'+str(app),*['-I'+str(app/f'subsystem{i}') for i in range(1,6)]]
noise=r"""
#define main original_tests_main
#include "../../testing/subsystem3/host/test_subsystem3.c"
#undef main
int main(void) {
 test_calibration(); barcode_init(); barcode_decode_set_mapping("CDAB");
 barcode_event_t ev; uint32_t widths[29]; make_frame(widths,0,10u);
 for(unsigned reverse=0;reverse<2;reverse++) {
  barcode_set_enabled(true);
  fake_sample(WHITE,WHITE,WHITE,300u,true);
  fake_sample(WHITE,WHITE,BLACK,5u,true);
  fake_sample(WHITE,WHITE,WHITE,5u,true);
  for(unsigned i=0;i<29;i++) fake_sample(WHITE,WHITE,i%2==0?BLACK:WHITE,widths[reverse?28-i:i],true);
  fake_sample(WHITE,WHITE,WHITE,15u,true);
  bool noisy=barcode_get_event(&ev,TMO_POL);
  stream_frame(0u,10u,reverse!=0);
  bool clean=barcode_get_event(&ev,TMO_POL);
  printf("leading noise direction=%u: detected=%d; clean control detected=%d\n",reverse,noisy,clean);
 }
 return 0;
}
"""
vehicle=r"""
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
"""
for name,source,extra in [
 ('barcode_noise_probe',noise,[host/'fakes.c',app/'car_math.c',app/'subsystem2/pid.c',app/'app_tests.c',*sorted((app/'subsystem3').glob('*.c'))]),
 ('vehicle_probe',vehicle,[app/'car_math.c',app/'subsystem2/pid.c',app/'subsystem3/line_follow.c',app/'subsystem3/barcode_decode.c'])]:
 p=out/(name+'.c');p.write_text(source);exe=out/name
 cmd=flags+['-DAPP_MODE=APP_MODE_TEST_BUDDY3',str(p),*map(str,extra),'-lm','-o',str(exe)]
 subprocess.run(cmd,check=True)
 result=subprocess.run([str(exe)],check=True,text=True,capture_output=True)
 print(result.stdout,end='');(out/(name+'.log')).write_text(result.stdout)

# Model a higher-priority vehicle reset at a legal consumer pre-emption point.
race = r"""
#define main original_tests_main
#include "../../testing/subsystem3/host/test_subsystem3.c"
#undef main
#include "../../app_program/subsystem3/ir_sensor.c"
bool inject_reset = false;
int main(void) {
 test_calibration();barcode_init();barcode_set_enabled(true);
 fake_sample(WHITE,WHITE,WHITE,300u,true);
 fake_sample(WHITE,WHITE,BLACK,10u,false);
 fake_sample(WHITE,WHITE,WHITE,10u,false);
 ir_sensor_element_t elem;
 inject_reset=true;
 bool got=ir_sensor_barcode_pop(&elem);
 printf("reset during ring pop: returned=%d head=%u tail=%u (disabled empty ring requires head==tail)\n",got,g_ring_head,g_ring_tail);
 return 0;
}
"""
fakes=(host/'fakes.c').read_text().replace('void hal_memory_barrier(void) {}', 'extern bool inject_reset; void hal_memory_barrier(void) { if (inject_reset) { inject_reset=false; barcode_set_enabled(false); } }')
fp=out/'fakes_reset.c';fp.write_text(fakes)
p=out/'barcode_reset_probe.c';p.write_text(race);exe=out/'barcode_reset_probe'
extra=[fp,app/'car_math.c',app/'subsystem2/pid.c',app/'app_tests.c',*[p for p in sorted((app/'subsystem3').glob('*.c')) if p.name!='ir_sensor.c']]
subprocess.run(flags+['-DAPP_MODE=APP_MODE_TEST_BUDDY3',str(p),*map(str,extra),'-lm','-o',str(exe)],check=True)
r=subprocess.run([str(exe)],check=True,capture_output=True,text=True);print(r.stdout,end='');(out/'barcode_reset_probe.log').write_text(r.stdout)
