"""Regression tests for reset pre-emption and the actual motion STOP latch.

Run with Python 3 and GCC in WSL/Linux. Assertions fail on unsafe behavior.
Generated C files and executables stay in tmp/final-review.
"""
from pathlib import Path
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "tmp/final-review"
APP = ROOT / "app_program"
HOST = ROOT / "testing/subsystem3/host"
OUT.mkdir(parents=True, exist_ok=True)
FLAGS = [os.environ.get("CC", "gcc"), "-std=c99", "-Wall", "-Wextra",
         "-Werror", "-O2", "-flto", "-fwhole-program", f"-I{HOST}", f"-I{APP}",
         *[f"-I{APP / f'subsystem{i}'}" for i in range(1, 6)]]


def run(name, source, extra):
    binary = OUT / name
    subprocess.run([*FLAGS, "-DAPP_MODE=APP_MODE_TEST_BUDDY3", str(source),
                    *map(str, extra), "-lm", "-o", str(binary)], check=True)
    result = subprocess.run([str(binary)], capture_output=True, text=True, check=True)
    print(result.stdout, end="", flush=True)
    (OUT / (name + ".log")).write_text(result.stdout)


race = r"""
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
"""
source = OUT / "barcode_reset_regression.c"
source.write_text(race)
fakes = (HOST / "fakes.c").read_text().replace(
    "void hal_memory_barrier(void) {}",
    "extern unsigned inject_at, barriers_seen; extern bool inject_send; "
    "void hal_memory_barrier(void) { "
    "if (inject_at && ++barriers_seen == inject_at) { inject_at = 0; "
    "barcode_set_enabled(false); barcode_set_enabled(true); } }")
fakes = fakes.replace(
    "    memcpy(g_messages[g_head], p_src, (size_t) size);",
    "    if (inject_send) { inject_send = false; barcode_set_enabled(false); "
    "barcode_set_enabled(true); }\n"
    "    memcpy(g_messages[g_head], p_src, (size_t) size);")
fake_source = OUT / "fakes_reset_regression.c"
fake_source.write_text(fakes)
run("barcode_reset_regression", source,
    [fake_source, APP / "car_math.c", APP / "subsystem2/pid.c", APP / "app_tests.c",
     *[p for p in sorted((APP / "subsystem3").glob("*.c"))
       if p.name not in {"ir_sensor.c", "barcode.c"}]])
run("motion_stop_regression", HOST / "test_motion_stop.c",
    [APP / "car_math.c", APP / "subsystem2/pid.c"])
