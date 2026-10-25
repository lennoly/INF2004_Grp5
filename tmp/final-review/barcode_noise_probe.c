
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
