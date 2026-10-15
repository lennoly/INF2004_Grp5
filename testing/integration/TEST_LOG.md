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

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
