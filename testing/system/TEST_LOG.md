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
| 2026-10-27 | SYS-01 (bends) | 46ec444 + uncommitted tuning (cruise 50, `line=` 45, `pid=` 0.104, `ff=` 0.12) and line-loss hold in `line_follow.c`, MISSION wifi build 17:56 | team (logged by Kent) | PARTIAL: bends held, a full course run not recorded | `logs/2026-10-27_MISSION_wifi_course/` | After calibrating: 115 s (17:59:17-18:01:12) and 23 s (18:01:17-18:01:40) of line following through the bends with no `line lost` event, and 40 s after a bypass with none. Before the fixes (17:45 run) almost every loss was on an anticlockwise bend: the line slid out past the left sensor and the car drove straight for the 150 ms loss check; it now steers fully to the side the line was last seen. Raising `line=` above 45 made the car wobble on straights, so KP stays 45 |
| 2026-10-27 | SYS-04 | 46ec444 + uncommitted tuning (cruise 50, `line=` 45, `pid=` 0.104, `ff=` 0.12) and line-loss hold in `line_follow.c`, MISSION wifi build 17:56 | team (logged by Kent) | PARTIAL: 3 of 3 completed bypasses, no collision, line reacquired | `logs/2026-10-27_MISSION_wifi_course/` | Details under S5-U03 (2026-10-27). 2 more runs were stopped by hand (one during the bypass, after a 1733 mg impact; one after the scan). Other obstacle positions still to run |
