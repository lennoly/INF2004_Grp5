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
| 2026-10-27 | SYS-02 | 46ec444 + uncommitted tuning in progress (gains set live from the dashboard), MISSION wifi build | team (logged by Kent) | PARTIAL: every letter read executed its mapped command; STRAIGHT not seen | `logs/2026-10-27_1627_MISSION_wifi/`, `logs/2026-10-27_1640_MISSION_wifi/`, `logs/2026-10-27_1659_MISSION_wifi/`, `logs/2026-10-27_1720_MISSION_wifi/` | A LEFT (16:29:20), B LEFT (16:30:07), D UTURN (16:41:35), B RIGHT (16:43:09), D RIGHT (16:44:55, after `map=` at 16:44:41), A LEFT (16:51:54); the first five followed by `NAV_TURN` and, except B RIGHT (an obstacle 4 s later), `nav done`; after A LEFT at 16:51:54 the car was stopped by hand 1.6 s later, before the junction. Letters with no mapping read as NONE with no turn: D (16:52:23), Z (17:11:46, 17:34:30), Y (17:33:57). No wrong letter. Whether each turn took the right branch is not in the log |
| 2026-10-27 | SYS-03 | 46ec444 + uncommitted tuning in progress (gains set live from the dashboard), MISSION wifi build | team (logged by Kent) | PARTIAL: humps detected and reported; the real hump height was not measured | `logs/2026-10-27_{1627,1640,1659,1720,1745}_MISSION_wifi/`, `logs/2026-10-27_MISSION_wifi_course/` | Highest `max_mm` per run: 11.9, 18.4, 8.9, 13.4, 19.0 and 18.1 mm; single peaks 8.0-19.0 mm. To finish: measure the highest hump with a ruler and compare (±5 mm) |
| 2026-10-27 | SYS-04 | 46ec444 + uncommitted tuning in progress (gains set live from the dashboard), MISSION wifi build | team (logged by Kent) | PARTIAL: 2 more bypasses rejoined; most runs were stopped by hand after the scan | `logs/2026-10-27_{1627,1640,1659,1745}_MISSION_wifi/` | 9 obstacles scanned. Completed: 17:06:15 d=221 `TURN_RIGHT`, back on the line by 17:06:38 after 5 short losses; 17:51:15 d=221 `TURN_LEFT`, `obstacle bypassed` at 17:51:20, then a line loss and STOP by hand. 16:31:03 d=218 `TURN_LEFT` ended `STOPPED (path blocked)`. 16:43:19 d=196 `CONTINUE` (box judged off the path). 17:51:53 d=48 `TURN_RIGHT`, then a rescan at d=151 chose `REVERSE`. Stopped by hand just after the scan: 16:47:03, 16:48:29 (no edge seen, blind `TURN_LEFT`), 16:50:00 and 17:50:44. No collision recorded |
| 2026-10-27 | SYS-05 | 46ec444 + uncommitted tuning in progress (gains set live from the dashboard), MISSION wifi build | team (logged by Kent) | PASS | `logs/2026-10-27_{1640,1659,1720,1745}_MISSION_wifi/` (heartbeat.csv, telemetry.csv) | 4 sessions of 17.5, 19.4, 22.9 and 9.4 min: `link` CONNECTED in every heartbeat, `recon` 0, heartbeat `seq` about 1 a second. Telemetry 13.3 Hz at `rate=50`, else 4.5-4.7 Hz. Board `drop` counter rose by 709, 463, 187 and 32 (1-7 % of `pub`), with gaps up to 8.6 s. Events reached the dashboard throughout. `i2cerr` rose to 51, 83 and 108 in three sessions while `imu` stayed 1 (for Buddy 4) |
| 2026-10-30 | SYS-02 | e50d700, MISSION usbcdc | team (logged by Kent) | PARTIAL: STRAIGHT now seen and executed; full course not run | `logs/2026-10-30_1657_MISSION_usbcdc/` (console.log, commands.log, events.csv) | C read as STRAIGHT twice (16:58:09, 16:58:25), no wrong letter; `NAV_TURN (STRAIGHT)` followed each after about 1.2 s. The second finished `nav done` at 16:58:27.9; the first was stopped by hand 0.2 s in. 3 `[barcode] short frame` rejects on earlier passes (16:57:14, 16:57:16, 16:57:32), no command from them. Went straight through the junction, watched by Kent (not in the log) |
| 2026-10-30 | SYS-02 | e50d700 + uncommitted STRAIGHT at cruise, BARCODE levels 0.15/0.08, decoder searches the whole frame, MISSION usbcdc | team (logged by Kent) | PARTIAL: C read 5 of 5 and each followed by `NAV_TURN (STRAIGHT)`; other letters and the full course not rerun | `logs/2026-10-30_1733_MISSION_usbcdc/` (console.log, commands.log, events.csv) | Before the decoder change, same levels: 5 of 9 (17:14-17:16, 3 misses rejected -4 with a valid C and `*` in the frame, 1 short frame). After: 5 of 5, no wrong letter. At the junction each read turned into `NAV_TURN` on the junction line |
