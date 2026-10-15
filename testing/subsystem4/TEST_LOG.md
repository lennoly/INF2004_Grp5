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

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
