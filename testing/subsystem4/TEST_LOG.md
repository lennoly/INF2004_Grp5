# Subsystem 4 — test log

**Subsystem:** IMU, terrain, humps  ·  **Student ID:** 2503438  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | hump integrator on a simulated profile | FR5–FR6 | host | `make -C tests/host` | 0 failed |
| S4-U02 | no false humps on flat ground | A7 | board | `APP_MODE=TEST_IMU`; press START, push the car forward about 1 m by hand on a flat floor, lift it back; 5 times (never pull it backwards: with the motors off the encoders cannot tell direction) | 0 humps |
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
| 2026-10-23 | S4-U02 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | PASS | TEST_IMU serial CSV | Pushed by hand on a flat floor, lifted back between pushes (odo about 1.6 m in total): `n` 0 and `max` 0 throughout; one 1 mm CLIMBING/DESCENDING blip, far below the 5 mm minimum |
| 2026-10-23 | S4-U03 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | FAIL | TEST_IMU serial CSV | Pushed by hand over 1 cm then 2 cm humps. On the track the barcode sensor leaves room for only one wheel, so the car rolls (10–12°) instead of pitching; the pitch estimator saw nose-down and counted nothing (`n` 0). Fix: one-wheel humps read from the roll, h = lever × tan(roll) |
| 2026-10-23 | S4-U03 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | FAIL | TEST_IMU serial CSV | Roll estimate with the 138 mm track as lever: 1 cm read 14 mm, 2 cm read 29–32 mm (both humps high by the same factor), and a hand jolt counted as a 23 mm hump; pulling the car back counted as an extra crossing. Fix: lever calibrated to 100 mm (1 cm rolled 5.6°, 2 cm 11.5°), lean must last before a hump counts, height held while the wheels are still |
| 2026-10-23 | S4-U03 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | PASS (2 cm) | TEST_IMU serial CSV | 2 cm one-wheel hump, pushed forward by hand, 5 crossings: `n` 5, peaks 17, 18, 16, 15, 19 mm (all within 20 ± 5). Lean check 40 mm at the time; a 2 cm crossing leans for 60–80 mm so it also passes at 20 mm |
| 2026-10-23 | S4-U03 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | FAIL (1 cm) | TEST_IMU serial CSV | 1 cm one-wheel hump: peaks 10–12 mm (correct) but `n` 0: the roll stays past 3° for only about 25 mm, under the 40 mm lean check. Earlier 1 cm runs were also spoiled by the left encoder dropping counts (fixed on the car). Fix: `HUMP_ROLL_MIN_MM` 40 → 20 (a hand jolt leans about 15 mm) |
| 2026-10-23 | S4-U03 | 14b73a8 + uncommitted Buddy 4 changes | Lennon | PASS (1 cm) | TEST_IMU serial CSV | 1 cm one-wheel hump, pushed forward by hand at about 10 cm/s, 5 crossings: `n` 5, peaks 11, 11, 10, 12, 11 mm (all within 10 ± 5). A lift between crossings briefly read 17 mm and was correctly not counted. Both encoders counting together |
