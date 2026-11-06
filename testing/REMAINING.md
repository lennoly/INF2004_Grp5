# Tests still to do (6 Nov 2026)

Full definitions and results are in each `TEST_LOG.md`. Dropped tests
(S5-U09, S3-U03) are left out.

## To run

| # | Test | What to do | Build | Pass when |
|---|---|---|---|---|
| 1 | **SYS-04** obstacle bypass + rejoin | MISSION on the course, obstacles in the allowed positions (left, centre, right). Box 20 cm or less | MISSION (wifi) | no collision, line picked up again every time |
| 2 | **S5-U12** rerun, sonar failure stops the car | MISSION, line following, pull the sonar ECHO wire (then TRIG) | MISSION | `fault: sonar no echo` within about 0.3 s (limit now 4 pings); starts again after reconnecting |
| 3 | **SYS-01** rerun, full lap on the final build | MISSION, calibrate, one full lap at speed 75, line gain 35 (default since 6 Nov, was 45) | MISSION | full lap, no line loss or hand stop; also counts as the SYS-02/03 check on this build |

## Have the log, not written in yet

| Test | Log | Result |
|---|---|---|
| **S5-U12** sonar failure stops the car | `logs/20261104_172213` (sonar fault while driving stopped the car) | PASS at the old limit of 2, with a note that it was a real loose connection; rerun at the new limit of 4 (see below) |

## Done

- **Integration:** INT-01 to INT-11 (INT-06 and INT-11 from the 185643 full lap; INT-11 on the current build only)
- **S1:** U01 to U09
- **S2:** U01 to U06
- **S3:** U01, U02
- **S4:** U01 to U06
- **S5:** U01, U02, U03 (191119: 6 of 6, left- and right-aligned boxes), U04, U05, U06, U07 (6 Nov: 100 mm FAIL on width, 185 mm PARTIAL; centre accurate), U08, U10, U11
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

1. **SYS-04.** MISSION on the course with boxes left, centre and right. The car passes away from the side the box reads on (scan 40 deg each way).
2. **S5-U12 rerun.** Quick: pull the ECHO wire while line following.
