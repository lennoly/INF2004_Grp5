# Subsystem 3 — test log

**Subsystem:** IR sensors, line following, barcode  ·  **Student ID:**  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

Standalone bench instructions: [README.md](README.md). Both updated board modes
keep the motors off. Use a newly built image, not the older prebuilt firmware.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | line classifier and Code 39 decoder (forwards and reversed) | FR1–FR3 | host | `make -C tests/host` | 0 failed |
| S3-U02 | IR calibration contrast | A5 | board | `APP_MODE=TEST_IR`; START then manually sweep all three sensors for 5 s; STOP aborts | each captured span ≥ 2048 counts; all-or-nothing application |
| S3-U03 | barcode reading | Week 6 FR3, A6 | board | `APP_MODE=TEST_BARCODE`; manually move prints; 20 passes per symbol at measured 100, 180 and 300 mm/s, both directions and bends | ≥ 90 % correct, 0 wrong commands |
| S3-U04 | standalone software regressions | FR1–FR3, A5, A6 | host | `python testing/subsystem3/run_host_tests.py`; both test modes and both polarities | all checks pass with warnings treated as errors |
| S3-U05 | line decision output, no motors | FR1–FR2 | board | TEST_IR; move sensors left/right/off line and across a junction | correct steering sign; LOST after 150 ms; both_black candidate on junction |
| S3-U06 | manual STOP and repeat | bench lifecycle | board | STOP during calibration and streaming, then START again | ends without movement; fresh calibration/session |
| S3-U07 | packed stream and queue integrity | Change Requirement 1, partial | host | standalone runner: overflow, full queue, reverse frames, wraparound | damaged frame rejected; queue losses counted; decoded content preserved |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |

## Development verification (not a committed hardware result)

2026-10-20: the standalone runner passed 610 checks in each of four
configurations (TEST_IR / TEST_BARCODE, black-high / black-low). The changed
firmware files and shared launcher passed the repository's BARR checker with
zero findings excluding its documented deviations. The existing full host
regression suite also passed 123 checks, including 1000/1000 noisy barcode
frames. No board test or ARM firmware build was performed.

[Host verification output and source hashes](logs/2026-10-20_host_working_tree.txt)
identify the base commit and uncommitted source state. This is provisional
development evidence; commit the changes and rerun before adding formal result
rows above.
