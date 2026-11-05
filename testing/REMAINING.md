# Tests still to do (4 Nov 2026)

Full definitions and results are in each `TEST_LOG.md`. Dropped tests
(S5-U09, S3-U03) are left out.

## To run

| # | Test | What to do | Build | Pass when |
|---|---|---|---|---|
| 1 | **SYS-04** obstacle bypass + rejoin | MISSION on the course, obstacles in the allowed positions (left, centre, right). Box 20 cm or less | MISSION (wifi) | no collision, line picked up again every time |
| 2 | **S5-U07** profile accuracy (FAIL so far) | `TEST_ULTRASONIC`, 8 cm and 15 cm boxes centred at 200 mm, 3 scans each, `python3 tools/scan_plot.py <log> --box X,Y,W` | TEST_ULTRASONIC | width and centre error recorded, width within the beam margin. A recorded FAIL with the numbers is acceptable for the report |

## Have the log, not written in yet

| Test | Log | Result |
|---|---|---|
| **S5-U12** sonar failure stops the car | `logs/20261104_172213` (sonar fault while driving stopped the car) | PASS at the old limit of 2, with a note that it was a real loose connection; rerun at the new limit of 4 (see below) |
| **S5-U07** | `logs/20261104_164356` (100 mm box read 114-155) | FAIL |

## Done

- **Integration:** INT-01 to INT-11 (INT-06 and INT-11 from the 185643 full lap; INT-11 on the current build only)
- **S1:** U01 to U09
- **S2:** U01 to U06
- **S3:** U01, U02
- **S4:** U01 to U06
- **S5:** U01, U02, U03 (191119: 6 of 6, left- and right-aligned boxes), U04, U05, U06, U08, U10, U11
- **System:** SYS-01 (181307, 185643), SYS-02 (LEFT, RIGHT, STRAIGHT, U-TURN: 184558), SYS-03, SYS-05

## What to record

Write these down while you test (phone notes are fine); I fill in the rows.

**SYS-04 (MISSION, course run with boxes)**
- Log folder name (`logs/2026...`) and whether it was WiFi or USB
- For each box: where it sat (left / centre / right of the line, rough cm
  off the line) and its width
- What the car did: side it passed, any touch or hit, line picked up again
  (yes / no), stopped by hand (yes / no, why)
- Anything odd: sonar no-echo stop, missed barcode, hump

**S5-U12 rerun (fault limit is now 4 pings, was 2)**
- MISSION, line following, then pull the sonar ECHO wire (then TRIG)
- Log folder name; roughly how long after pulling it the car stopped
  (expected about 0.3 s now)
- Whether it showed `fault: sonar no echo` and restarted after reconnecting

**S5-U07 (optional)**
- TEST_ULTRASONIC log folder; box width and how far it sat (tape), 3 scans
  per box

## Suggested order

1. **SYS-04.** MISSION on the course with boxes left, centre and right (default side is now left).
2. **S5-U12 rerun.** Quick: pull the ECHO wire while line following.
3. **S5-U07.** Only if there is time.
