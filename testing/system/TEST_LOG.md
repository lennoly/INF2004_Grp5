# System — test log

Tests of the complete car against the project requirements (docs/REPORT.md section 1), on the final hardware. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | follow the line track autonomously | Req 1 | manual | MISSION, full course | completes without leaving the line |
| SYS-02 | decode barcodes and execute LEFT / RIGHT / STRAIGHT / U-TURN | Req 2–3 | manual | MISSION, every barcode on the course | correct manoeuvre each time |
| SYS-03 | detect humps and report the highest peak | Req 4–5 | manual | MISSION over the humps; read `max_peak_mm` | within ±5 mm of the measured highest hump |
| SYS-04 | detect, profile and bypass obstacles, then reacquire the line | Req 6–9 | manual | MISSION with obstacles in the allowed positions | no collision, line reacquired |
| SYS-05 | report telemetry over WiFi for the whole run | Req 10 | link | MISSION + WiFi with the dashboard (`--mqtt`) | status online, telemetry and events throughout |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
