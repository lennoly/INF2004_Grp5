# Subsystem 3 — test log

**Subsystem:** IR sensors, line following, barcode  ·  **Student ID:**  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | line classifier and Code 39 decoder (forwards and reversed) | FR1–FR3 | host | `make -C tests/host` | 0 failed |
| S3-U02 | IR calibration contrast | A5 | board | `APP_MODE=TEST_IR`; STOP while idle = calibration spin | white/black contrast > 50 % of ADC range |
| S3-U03 | barcode reading | FR2, A6 | board | `APP_MODE=TEST_BARCODE`; 20 passes each at 100 and 180 mm/s, both directions | ≥ 90 % correct, 0 wrong commands |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
