# Subsystem 2 — test log

**Subsystem:** Motors, encoders, wheel PID, moves  ·  **Student ID:** 2503109  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | PID step and anti-windup | FR4 | host | `make -C tests/host` | 0 failed |
| S2-U02 | motor response and speed loop | A4 | board | `APP_MODE=TEST_MOTOR`: open-loop PWM sweep, then step responses (dashboard serial, Fast 20 Hz) | speed follows the step without sustained oscillation |
| S2-U03 | straight distance | NFR7, A3 | manual | `APP_MODE=TEST_MOTION`; 5 × 500 mm, measure with a tape | error within the NFR7 limit |
| S2-U04 | 90° turns each way | NFR7, A3 | manual | `APP_MODE=TEST_MOTION`; 5 turns each way, protractor | error within the NFR7 limit |
| S2-U06 | velocity command expires (FR14) | FR14, A14 | board | build TEST_MOTOR with the `motion_keep_alive()` line in `log_step()` commented out; press START | in the step CSV the wheels drive for at most 120 ms (3 rows at 40 ms), then target speed is ignored and PWM drops to 0; with the line restored the steps run the full 3 s |
| S2-U05 | encoder glitch filter below the shortest half-period (CSC05) | A3 | board | boot any build with the encoder; read the `[encoder]` line; measure edges per wheel turn from telemetry `enc` (turn a wheel once) and set ENCODER_SLOTS, max speed from the TEST_MOTOR sweep into SPEED_WHEEL_MAX_MM_S | no `WARNING` line |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S2-U01 | 740df22 + timer tick fix and obstacle changes (committed 28 Oct), build 17:28 | Levia | PASS | `build_all/host_tests.log` (not kept; rerun `make -C tests/host`) | `make -C tests/host CC=gcc`: 304 passed, 0 failed (PID step and anti-windup cases included) |
