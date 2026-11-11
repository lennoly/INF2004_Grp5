# Tests still to do (11 Nov 2026)

Full definitions and results are in each `TEST_LOG.md`. Dropped tests
(S5-U09, S3-U03) are left out.

## To run

On the 11 Nov build (lift stop, side vote, sonar noise filter, impacts off), USB console logged each time:

- **S4-U07** lifted / overturned: steps in `subsystem4/TEST_LOG.md`. Note for each lift what you did and whether the wheels stopped.
- **S5-U03** again, 6 bypasses: box left, centred and right, twice each. Note the true side and the side the car took.
- **SYS-01** one full lap: keep the console log; any `[sonar] FAULT ping: ...` line says why the sonar dropped out.

## Done

- **Integration:** INT-01 to INT-11 (INT-06 and INT-11 from the 185643 full lap; INT-11 on the current build only)
- **S1:** U01 to U09
- **S2:** U01 to U06
- **S3:** U01, U02
- **S4:** U01 to U06 (U05, impacts, is no longer a requirement)
- **S5:** U01, U02, U03 (191119: 6 of 6, left- and right-aligned boxes), U04, U05, U06, U07 (6 Nov: 100 mm FAIL on width, 185 mm PARTIAL; centre accurate), U08, U10, U11, U12 (PASS on the 30 Oct run: ECHO and TRIG unplugged before START gave `fault: sonar no echo` within 0.2 s)
- **System:** SYS-01 (181307, 185643; 9 Nov final build, video), SYS-02 (LEFT, RIGHT, STRAIGHT, U-TURN: 184558; 10 Nov final build, videos), SYS-03 (10 Nov: 10 mm hump read 9.8 / 12.2 mm, video), SYS-04 (9 Nov: box left, centre and right, videos), SYS-05

## What to record

Write these down while you test (phone notes are fine); I fill in the rows.

**S5-U07 (optional)**
- TEST_ULTRASONIC log folder; box width and how far it sat (tape), 3 scans
  per box

## Left

- Upload the 9 silent videos (`Downloads\muted\`) as unlisted YouTube; links go in `videos/README.md` and the result rows.
- Optional: S5-U12 with a sonar wire pulled while driving (only shown for a wire already out at START).
