# Subsystem 5 — test log

**Subsystem:** Ultrasonic, servo, obstacle profiling and bypass  ·  **Student ID:** 2501955  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | obstacle profile and bypass plan | FR7–FR8 | host | `make -C tests/host` | 0 failed |
| S5-U02 | profile of known boxes | FR7, A9 | board | `APP_MODE=TEST_ULTRASONIC`; boxes at 150 and 250 mm, straight and offset | edges match tape measurements; servo 0° points right |
| S5-U03 | bypass and line re-acquisition | FR8–FR9, A10 | manual | full bypass on the course, each allowed obstacle position, 5 times | clean rejoin every time |
| S5-U04 | HC-SR04 no-target behaviour at 3.3 V (CSC05: test first) | A9, FR7 | board | `APP_MODE=TEST_ULTRASONIC`, sensor aimed at open space > 4 m; read the `a9,...` lines | every ping OUT_OF_RANGE; FAULT means no pulse: switch to 5 V + 1 k / 2 k divider before relying on "clear" |
| S5-U05 | vehicle task and scanner in simulation: START/STOP rules, two-echo trigger, scan, box bypass, reverse after impact, hump guard, STOP during scan / line search / U-turn / bypass, stale barcodes, watchdog | FR7, FR8, FR12, NFR3, NFR9 | host | `make -C tests/host` | 0 failed |
| S5-U06 | servo direction | A9 | board | `APP_MODE=TEST_ULTRASONIC`, box 250 mm ahead and 60 mm to the right | `closest echo on the RIGHT`; otherwise set `SERVO_INVERT` to 1 |
| S5-U07 | profile accuracy against tape | FR7, A9 | board | `TEST_ULTRASONIC`, 3 scans per position (centred 150 / 250 mm, 60 mm right, 60 mm left); `python3 tools/scan_plot.py <log> --box X,Y,W` | edges and width within the beam-widening margin; fill the table in REPORT.md 4.5 |
| S5-U08 | STOP during a scan | FR12, NFR3, A14 | board | `MISSION`, obstacle ahead; tap STOP while the sonar sweeps (logic analyser on GP21 and the servo signal if available) | scan ends within about one ping (0.1 s), servo re-centred, `STOPPED (stop command)` |
| S5-U09 | reverse after an impact | FR8 | board | `MISSION`, push the car gently into a box | car reverses 120 mm, then scans |
| S5-U10 | no obstacle stop on the hump | A12 | board | `MISSION` over the course hump; watch `front` in telemetry | no OBSTACLE while on the hump; front range above 300 mm on the approach |
| S5-U11 | watchdog stays quiet | NFR9 | board | `MISSION`: a scan, a bypass and a calibration spin | no `watchdog reset` reason after any of them |
| S5-U12 | front sensor failure stops the car | FR7, A9, NFR9 | board | `MISSION`, line following: unplug ECHO (then TRIG) on the sonar | `STOPPED (sonar fault)` within about 0.2 s; reconnect and START again |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-16 | S5-U04 | main of 16 Oct (log added in 7a9a144) | team run, log committed by Kent | Partial: VALID 4, OUT_OF_RANGE 6, FAULT 0 | `logs/2026-10-16_a9_check.txt.jpg` | No FAULT, so at 3.3 V the module still gives a pulse with no target and a clear path reads OUT_OF_RANGE (no divider needed). The 4 VALID pings (293–301 mm) mean something was about 30 cm away: repeat aimed at open space. The same run's scan found a box at 196 mm, 137 mm wide (edges −188 / −52 mm) and planned TURN_RIGHT with an 88 mm offset. |
| 2026-10-18 | S5-U04 | f7c5053 | Raees | FAIL: VALID 20 at 2–9 mm (pulses 12–57 µs), OUT_OF_RANGE 0, FAULT 0 | `logs/2026-10-18_bench_TEST_ULTRASONIC.log` | The sensor faced clear space, so readings below the HC-SR04's 20 mm minimum mean the trigger pulse reaches ECHO (a loose sensor wire or no sensor power). The servo did sweep 30–150° (first visual confirmation), but the sensor is not fixed to the servo arm and its wires pull it off. Fix: reseat the 4 wires, fix the sensor to the arm, leave slack in the wires. Firmware after this run: pulses under 20 mm are FAULT, servo moves 5° per 20 ms. |
| 2026-10-22 | S5-U04 | ef0c661 (main merged into Raees) | Raees | Partial: VALID 20 at 205 mm (1198–1200 µs), then VALID 20 at 202 mm; OUT_OF_RANGE 0, FAULT 0 | `logs/2026-10-22_bench_TEST_ULTRASONIC.log` | Sensor aimed at a wall the tester measured at 170 mm; the sensor read 35 mm more. Where the tape was measured from (sensor face or car front) is to be confirmed, then repeat at about 300 and 500 mm to tell a fixed offset from a percentage error. Readings are stable and the 18 Oct 2–9 mm junk is gone, so the wiring is fixed. Open-space (no-target) part of A9 not run yet. |
| 2026-10-22 | S5-U06 (servo sweep range) | ef0c661 | Raees | FAIL: the sensor turns only about ±25–30° from centre | `logs/2026-10-22_bench_TEST_ULTRASONIC.log` (second run) | Scan commanded 10–170°. Every reading from 10° to 60° was 205 mm and from 120° to 170° was 221 mm, so the sensor stopped turning between 60° and 65° and between 115° and 120°; to be confirmed by eye. The mount still meets the main board. Remount so the sensor reaches 15° and 165° (side looks) before more scans; turning direction (SERVO_INVERT) still to check. |
