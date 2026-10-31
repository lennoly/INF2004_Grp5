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
| 2026-10-28 | S2-U02 | 8e2ece2, TEST_MOTOR usbcdc | Levia | PASS | `logs/2026-10-28_U02_motor/step_results.txt` (CSV from the console; dashboard files alongside) | No sustained oscillation in any step. 0 to 160 mm/s: peak 258/261 at 120 ms (61 % overshoot), settled by ~400 ms at 171-180 mm/s (about +10 % steady offset, PWM sitting at 19-20 %). 160 to 50: 49-51 at 120 ms, then 52-55 (+5 %). Stop: 0 by 200 ms. Open loop: dead band between 10 and 20 % duty; left/right differ up to 16 % at low duty; 40 % row (207/222) below the trend and 100 % row (410/538) far below 90 % (866/850): likely read while slowing or encoder edges missed at top speed, check in S2-U05 |
| 2026-10-28 | S2-U03 | 8e2ece2 + uncommitted line-search fix, TEST_MOTION usbcdc (diameter 65.0, track 138) | Levia | FAIL | `logs/2026-10-28_U03_U04_before/console.txt` | 5 runs: encoders 496-497 mm, tape about 450 mm (-10 %). Wheels measure 63-65 mm, so not the tyre size: hand tests (`logs/2026-10-28_encoder_hand_turn/`, `logs/2026-10-28_encoder_hand_push/`) show the right encoder ~9 % more edges per mm than 2 x 559 per turn (left one miscounts at hand speed, check its disc and gap). Calibrated as the data sheet says: WHEEL_DIAMETER_MM 65 x 450 / 497 = 58.9 (effective) |
| 2026-10-28 | S2-U04 | 8e2ece2 + uncommitted line-search fix, TEST_MOTION usbcdc (diameter 65.0, track 138) | Levia | FAIL | `logs/2026-10-28_U03_U04_before/console.txt` | 5 runs each way: encoders 86.7-87.8 deg, protractor 85-87 deg. Recalibrated after the diameter fix (next rows) |
| 2026-10-28 | S2-U04 | 8e2ece2 + uncommitted line-search fix and diameter 58.9 (track 138), TEST_MOTION usbcdc | Levia | FAIL | `logs/2026-10-28_U04_diameter_58.9/` (console.log) | 2 runs: distance now 500 mm on the tape; turns over-rotate, about 93 deg for 90 and about 190 deg for the U-turn (encoders 87 / 177.5) => effective track ~129 mm. Set TRACK_WIDTH_MM 132 (covers the turn ending ~3 deg early) |
| 2026-10-28 | S2-U03 | 8e2ece2 + uncommitted line-search fix, diameter 58.9, track 132, TEST_MOTION usbcdc | Levia | PASS | `logs/2026-10-28_U03_U04_after/` (console.log) | 5 runs: encoders 496-497 mm forward, 495-496 backward; tape 500 mm. NFR7 limit is not written in the repo: check against the project spec |
| 2026-10-28 | S2-U04 | 8e2ece2 + uncommitted line-search fix, diameter 58.9, track 132, TEST_MOTION usbcdc | Levia | PASS | `logs/2026-10-28_U03_U04_after/` (console.log) | 5 runs: left and right 90 deg turns measure about 91 deg (+1), U-turn about 183 deg (+3); encoders 86.6-87.5 / 176.8-177.0 deg (-273 / 273 printed = 87 deg, the test does not wrap the change). NFR7 limit is not written in the repo: check against the project spec |
| 2026-10-29 | S2-U05 | 1839712 + uncommitted `help` reprints the encoder line, MISSION usbcdc | Levia | PASS | `logs/2026-10-29_U05_encoder_filter/` (console.log, commands.log) | `[encoder] 559 slots: half-period 183 us at max wheel speed, glitch filter 180 us`, no WARNING. Slots 559 from the 1118 edges per turn (2 per slot), max speed 900 mm/s, diameter 58.9. Margin only 3 us (2 %): uneven slots could still drop edges at full speed (the low 100 % row in S2-U02) |
| 2026-10-30 | S2-U06 | e50d700 + uncommitted changes, one-off TEST_MOTOR usbcdc with `motion_keep_alive()` commented out in `log_step()` (source restored after the build) | Kent | PASS | `logs/2026-10-30_U06_velocity_expires/` (console.log, commands.log) | 160 mm/s step: PWM 39/40 at 40 ms, 37/38 at 80 ms, 0 from 120 ms; wheels at 0 by 360 ms. 50 mm/s step: PWM 14, 8, then 0 from 120 ms; wheels at 0 by 320 ms. Both steps stayed at PWM 0 to the end of the 3 s while the target was still logged. Full 3 s with the line restored: S2-U02 |
