# Subsystem 3 — test log

**Subsystem:** IR sensors, line following, barcode  ·  **Student ID:** 2501660  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | line classifier and Code 39 decoder (forwards and reversed) | FR1–FR3 | host | `make -C tests/host` | 0 failed |
| S3-U02 | IR calibration contrast | A5 | board | `APP_MODE=TEST_IR`; STOP while idle = calibration spin | white/black contrast > 50 % of ADC range |
| S3-U03 | barcode reading | FR2, A6 | board | `APP_MODE=TEST_BARCODE` over the 14 cm barcodes; 5 passes each at 100 and 180 mm/s, mixed letters and both directions | ≥ 4 of 5 read at each speed, 0 wrong letters |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | S3-U02 | 2dd218e + uncommitted changes | Max | PASS | TEST_IR serial output | Calibration OK. Raw white / black (ADC counts of 4095): left about 200 / 2700 (61 % of range), right about 220 / 3600 (83 %), barcode about 170 / 3000–3600 (up to 83 %). All above the 50 % contrast needed. The barcode sensor now follows the bars (the constant raw ~40 of 22 Oct is gone) |
| 2026-10-24 | S3-U03 | 2dd218e + uncommitted changes | Max | PARTIAL | TEST_BARCODE serial output | 100 mm/s only: 11 passes, 9 read correctly (A ×3, B, C, D, F, Y, Z), 2 not read, 0 wrong letters. Both misses show the barcode sensor jumping to ~4000 raw (sheet lifting or the car rocking), so the frame broke and was dropped. Decoder now splits bars and spaces separately; levels 20 % / 10 %. Directions not recorded. 180 mm/s still to run |
