# Tests still to do (4 Nov 2026)

Full definitions and results are in each `TEST_LOG.md`. Dropped tests
(S5-U09, S3-U03) are left out.

## To run

| # | Test | What to do | Build | Pass when |
|---|---|---|---|---|
| 1 | **SYS-04** obstacle bypass + rejoin | MISSION on the course, obstacles in the allowed positions (left, centre, right). Box 20 cm or less | MISSION (wifi) | no collision, line picked up again every time |
| 2 | **S5-U03** bypass and re-acquisition | Each obstacle position 5 times. Include a **centred box of 20 cm or less** (the blind-pass rerun; the 31 Oct FAIL was a box wider than 20 cm) | TEST_OBSTACLE | clean rejoin every time |
| 3 | **INT-06** WCET and CPU load | One full lap with WiFi telemetry at 5 Hz, then send `timing` | MISSION (wifi) | `jit_max_us` ≤ 2000; record max/avg per task and `load_pct` |
| 4 | **INT-11** packing timing cost | Same as INT-06, on d5f5199 (before the packing) and on the current build | MISSION (wifi), two builds | no task over its period, `jit_max_us` ≤ 2000 in both |
| 5 | **S5-U07** profile accuracy (FAIL so far) | `TEST_ULTRASONIC`, 8 cm and 15 cm boxes centred at 200 mm, 3 scans each, `python3 tools/scan_plot.py <log> --box X,Y,W` | TEST_ULTRASONIC | width and centre error recorded, width within the beam margin. A recorded FAIL with the numbers is acceptable for the report |

## Have the log, not written in yet

| Test | Log | Result |
|---|---|---|
| **SYS-01** follow the line | `logs/20261104_181307` (full lap, no interruptions); also 172935 | PASS |
| **S5-U12** sonar failure stops the car | `logs/20261104_172213` (sonar fault while driving stopped the car) | PASS, with a note that it was a real loose connection, not a pulled wire |
| **S5-U07** | `logs/20261104_164356` (100 mm box read 114-155) | FAIL |

## Done

- **Integration:** INT-01 to INT-05, INT-07 to INT-10
- **S1:** U01 to U09
- **S2:** U01 to U06
- **S3:** U01, U02
- **S4:** U01 to U06
- **S5:** U01, U02, U04, U05, U06, U08, U10, U11
- **System:** SYS-02 (LEFT, RIGHT, STRAIGHT, U-TURN: 184558), SYS-03, SYS-05

## Suggested order

1. **SYS-04 and S5-U03 together.** Same runs on the new shorter bypass; log both from the same session.
2. **INT-06.** On a full clean lap, send `timing` at the end. SYS-01 can be logged from the same lap.
3. **INT-11.** Needs the old d5f5199 build flashed once.
4. **S5-U07.** Only if there is time.
