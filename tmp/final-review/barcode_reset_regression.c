
#define main original_tests_main
#include "../../testing/subsystem3/host/test_subsystem3.c"
#undef main
#include "../../app_program/subsystem3/ir_sensor.c"
#undef QUIET_US
#include "../../app_program/subsystem3/barcode.c"
#include <assert.h>
unsigned inject_at;
unsigned barriers_seen;
bool inject_send;
int main(void) {
    test_calibration(); barcode_init();
    for (unsigned point = 1; point <= 2; point++) {
        barcode_set_enabled(true);
        fake_sample(WHITE, WHITE, WHITE, 300u, true);
        fake_sample(WHITE, WHITE, BLACK, 10u, false);
        fake_sample(WHITE, WHITE, WHITE, 10u, false);
        uint32_t head = g_ring_head, tail = g_ring_tail;
        ir_sensor_element_t element;
        barriers_seen = 0; inject_at = point;
        assert(ir_sensor_barcode_pop(&element));
        assert(g_ring_head == head && g_ring_tail == ((tail + 1u) & 63u));
        assert(inject_at == 0);
        barcode_poll();
        assert(g_ring_head == g_ring_tail);
        stream_frame(0u, 10u, false);
        barcode_event_t event;
        assert(barcode_get_event(&event, 0) && event.text[0] == 'A');
        assert(!barcode_get_event(&event, 0));
    }
    /* Reset immediately before the old decoder enqueues its result. */
    inject_send = true;
    stream_frame(0u, 10u, false);
    barcode_event_t event;
    assert(!inject_send && !barcode_get_event(&event, 0));
    stream_frame(1u, 10u, true);
    assert(barcode_get_event(&event, 0) && event.text[0] == 'B');
    puts("Barcode: reset during both pop barriers and stale enqueue rejection PASS");
    return 0;
}
