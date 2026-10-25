
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
