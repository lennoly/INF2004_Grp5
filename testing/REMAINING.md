# Tests still to do (10 Nov 2026)

Full definitions and results are in each `TEST_LOG.md`. Dropped tests
(S5-U09, S3-U03) are left out.

## To run

Nothing. S5-U12 is marked done for now; Kent will rerun it at school (unplug at the sonar end) to try to see `fault: sonar no echo` on the current build.

## Done

- **Integration:** INT-01 to INT-11 (INT-06 and INT-11 from the 185643 full lap; INT-11 on the current build only)
- **S1:** U01 to U09
- **S2:** U01 to U06
- **S3:** U01, U02
- **S4:** U01 to U06
- **S5:** U01, U02, U03 (191119: 6 of 6, left- and right-aligned boxes), U04, U05, U06, U07 (6 Nov: 100 mm FAIL on width, 185 mm PARTIAL; centre accurate), U08, U10, U11, U12 (marked done 10 Nov: the car always stops when a sonar wire is pulled; the sonar fault itself shown on 30 Oct and 4 Nov (172213); the 10 Nov reruns show `fault: IMU`, as unplugging at the board disturbs the IMU link)
- **System:** SYS-01 (181307, 185643; 9 Nov final build, video), SYS-02 (LEFT, RIGHT, STRAIGHT, U-TURN: 184558; 10 Nov final build, videos), SYS-03 (10 Nov: 10 mm hump read 9.8 / 12.2 mm, video), SYS-04 (9 Nov: box left, centre and right, videos), SYS-05

## What to record

Write these down while you test (phone notes are fine); I fill in the rows.

**S5-U07 (optional)**
- TEST_ULTRASONIC log folder; box width and how far it sat (tape), 3 scans
  per box

## Left

- Upload the 9 silent videos (`Downloads\muted\`) as unlisted YouTube; links go in `videos/README.md` and the result rows.
- Optional: S5-U12 at school, unplugging at the sonar end.
