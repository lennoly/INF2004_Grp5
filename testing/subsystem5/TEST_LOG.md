# Subsystem 5 — test log

**Subsystem:** Ultrasonic, servo, obstacle profiling and bypass  ·  **Student ID:**  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | obstacle profile and bypass plan | FR7–FR8 | host | `make -C tests/host` | 0 failed |
| S5-U02 | profile of known boxes | FR7, A9 | board | `APP_MODE=TEST_ULTRASONIC`; boxes at 150 and 250 mm, straight and offset | edges match tape measurements; servo 0° points right |
| S5-U03 | bypass and line re-acquisition | FR8–FR9, A10 | manual | full bypass on the course, each allowed obstacle position, 5 times | clean rejoin every time |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
