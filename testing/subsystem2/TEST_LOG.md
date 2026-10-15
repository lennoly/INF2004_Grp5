# Subsystem 2 — test log

**Subsystem:** Motors, encoders, wheel PID, moves  ·  **Student ID:**  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | PID step and anti-windup | FR4 | host | `make -C tests/host` | 0 failed |
| S2-U02 | motor response and speed loop | A4 | board | `APP_MODE=TEST_MOTOR`: open-loop PWM sweep, then step responses (dashboard serial, Fast 20 Hz) | speed follows the step without sustained oscillation |
| S2-U03 | straight distance | NFR7, A3 | manual | `APP_MODE=TEST_MOTION`; 5 × 500 mm, measure with a tape | error within the NFR7 limit |
| S2-U04 | 90° turns each way | NFR7, A3 | manual | `APP_MODE=TEST_MOTION`; 5 turns each way, protractor | error within the NFR7 limit |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
