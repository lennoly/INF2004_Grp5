# Testing

All test records for the project live in this folder, laid out like the
course's INF2004_ProjectTemplate. A test only counts as done when it is
**written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/ … subsystem5/   one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, CSV, photos, scope captures
├── integration/           tests across two or more subsystems (shared)
└── system/                the whole car against the project requirements (shared)
```

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished car against a requirement | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## How our tests run

| Kind | How | Output |
|---|---|---|
| **host** (automatic, no hardware) | `make -C tests/host` | `N passed, 0 failed` |
| **board** (test program on the car, in `app_program/subsystemN/subsystemN_test.c`) | build with `APP_MODE=TEST_xxx` (README "Build and flash"), flash, press **GP20 START** | CSV / text on the USB console |
| **link** (WiFi) | `python tools/dashboard/wifi_test.py --mqtt <broker IP>` (on the `buddy1-wifi` branch until it is merged) | PASS / FAIL + `logs/<date_time>/wifi_*` |
| **manual** | a person measures (tape, protractor, ruler, scope) | numbers written in the result row |

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Commit your code, then run the test** on that commit.
3. **Save the output** in that folder's `logs/`, named
   `YYYY-MM-DD_<what>.txt` (or `.csv`). Dashboard runs save to
   `logs/<date_time>/` in the repo root, which git ignores: copy the files
   you need as evidence into `testing/subsystemN/logs/`.
4. **Record the result.** Add a row to the *Results* table: date, test ID,
   commit (`git rev-parse --short HEAD`), tester, PASS/FAIL, log file.
   Commit the log and the table together.

## Videos

A test you have to see (the car turning, a hump being counted) is best shown
as a 15–60 s **unlisted YouTube** video. List it in `videos/README.md` and
put the link in the result row.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result means nothing without the code it ran on.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging into `main`, re-run your subsystem's tests. Before a demo
  or hand-in, run every test and record the full set.
