# Integration — test log

Tests that two or more subsystems work together: at least one per interface in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | barcode command executed at the next junction | IF-01, IF-03 | manual | MISSION: drive over each barcode before a junction | car turns the decoded way |
| INT-02 | MQTT STOP brakes the car while driving | IF-02, IF-05, NFR3 | manual | MISSION + WiFi: send `stop` from the dashboard | motors braked within 100 ms of arrival |
| INT-03 | WiFi does not disturb the control loop | IF-05, A2 | board | TEST_MOTION with 5 Hz telemetry streaming, WiFi on and off; send `timing` after each | `jit_max_us` ≤ 2000 (else lower the cyw43 priority) |
| INT-04 | sonar shared between monitor and scans | IF-04 | manual | MISSION: obstacle ahead; watch scan output | no overlapping pings, scan completes |
| INT-05 | telemetry carries every subsystem's status | IF-06, IF-07 | manual | MISSION + dashboard: drive over a hump, a barcode and an obstacle | plots and events show all of them |
| INT-06 | WCET and total CPU load (CSC05) | A2, A14, NFR1 | board | MISSION + WiFi telemetry at 5 Hz, one full lap; then send `timing` | record max/avg of motion, vehicle, barcode, imu, hump, ir_isr, net; `load_pct`; `jit_max_us` ≤ 2000 |
| INT-07 | packed encoder data under load (Change Requirement 1) | IF-03, IF-07 | board | encoders at top speed while motion and imu read the packed snapshots, 5 min; compare with a reference count | no torn or lost counts |
| INT-08 | barcode reaches the dashboard as a binary frame (CR1) | IF-01, IF-05, IF-06 | manual | MISSION + WiFi: drive over each test barcode; watch the dashboard and the mosquitto log | right symbol and command on the dashboard; telemetry 43 B, barcode event 3 B |
| INT-09 | RAM and message size before / after (CR1, Week 13) | all | host + build | `arm-none-eabi-size` and the object symbol sizes of MISSION at d5f5199 and after; frame lengths in tests/host | numbers as in docs/DATA_REPRESENTATION.md section 5 and 6 |
| INT-10 | every fault bit stops the car (CR1, A14) | FR14, NFR3 | manual | line following: stall a wheel, unplug an encoder, the IMU, the sonar ECHO; switch the hotspot off | STOPPED with the docs/FAULT_PATHS.md reason within its latency; Wi-Fi loss only reported |
| INT-11 | timing cost of packing (CR1) | A2, NFR1 | board | INT-06 on d5f5199 and after the change, same lap | no task over its period; `jit_max_us` ≤ 2000 in both |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
