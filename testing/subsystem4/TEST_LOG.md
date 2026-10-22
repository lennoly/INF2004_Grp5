# Subsystem 4 — test log

**Subsystem:** IMU, terrain, humps  ·  **Student ID:**  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | hump integrator on a simulated profile | FR5–FR6 | host | `make -C tests/host` | 0 failed |
| S4-U02 | no false humps on flat ground | A7 | board | `APP_MODE=TEST_IMU`; drive 5 m on a flat floor | 0 humps |
| S4-U03 | hump height | FR6, NFR6, A7, A12 | manual | `APP_MODE=TEST_IMU`; drive over a measured hump 5 times | height within ±5 mm of the ruler value |
| S4-U04 | motion events | brief: motion classification | board | `APP_MODE=TEST_IMU`; leave still 5 s, push forward, turn by hand, push over a hump, stop | `event` column shows STATIONARY, ACCELERATING / CRUISING, TURNING, CLIMBING, DESCENDING in that order, with no flicker between them |
| S4-U05 | impact (collision) detection | brief: collision detection | board | `APP_MODE=TEST_IMU`; 30 s at rest, then tap the bumper firmly 10 times, ≥ 1 s apart | `impacts` 0 at rest, then 10 |
| S4-U06 | IMU bus fault does not hang the car | FAULT_PATHS IMU row, R4 | board | `APP_MODE=TEST_IMU`; unplug the IMU Grove cable for 5 s, then plug it back | `ok` drops to 0 within 3 rows and the CSV keeps printing every 100 ms (no hang); after reconnecting `ok` returns to 1 and pitch follows the car again when the front is lifted (not stuck at 0) |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
