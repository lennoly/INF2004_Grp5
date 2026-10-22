# PicoCar handover: what is left against the Project Write-up

Checked on 22 Oct 2026 against `main` at `9213b98` (Change Requirement 1),
plus the unmerged branches `origin/Raees` and `origin/max`.
Brief: `Project Write-up.pdf` (10 mission requirements, expected behaviour,
per-buddy tasks, assessment criteria).

## 1. Where we stand in one paragraph

The **code** for all 10 mission requirements exists on `main`, builds with
0 warnings (MISSION and every TEST mode), and passes **187/187 host tests**.
Change Requirement 1 (minimum-bit data, binary MQTT, fault mask) is merged.
What is missing is almost entirely **evidence from the real car**: every
Results table in `testing/` is empty, every `___` in `docs/REPORT.md` is
blank, and every `[CALIBRATE]` value in `car_config.h` is still a default.
Two branches with real fixes (`Raees`, `max`) are **not merged and conflict
with `main`**. A few verified logic gaps are listed in section 4.

**Host tests prove the logic. They do not prove the car works.** Nothing in
the brief is "done" until it has a hardware result row.

## Your checklist (find your buddy number)

Each item points to the section with the details. "Integration" items need
whoever owns `vehicle.c`, together with the buddy named.

**Buddy 1: WiFi, MQTT, telemetry**
- [ ] Run S1-U02..U09 on the current `main` (binary frames, not JSON) and record the results (§2, §6)
- [ ] INT-08: barcode reaches the dashboard as a binary frame; INT-02: MQTT `stop` brakes within 100 ms (§2)
- [x] Highest hump peak over WiFi: **no code needed**. The run maximum is already in every telemetry frame (`max`, 0.1 mm) and every hump event frame, and the dashboard shows "MAX". It only needs proving in SYS-03 (§6)
- [ ] Measure the message sizes on the wire for the CR1 Week 13 table (§7)

**Buddy 2: motors, encoders, motion**
- [ ] Calibrate wheel diameter, track width, slots, top speed, speed PID; fill in the REPORT PID and accuracy tables (§5)
- [ ] Run S2-U02..U06; INT-07 (packed encoder data under load) (§2, §6)
- [x] 100 ms velocity-expiry brake (FR14): done, uncommitted (§4.1). Run the new S2-U06 on the car
- [x] Clear the BARR findings in `subsystem2_test.c` and `hal.c` from the TEST_WHEELS commit: done, firmware BARR now 0 (§4.8)

**Buddy 3: IR, line following, barcode**
- [ ] Merge `origin/max` into `main` (resolve the CR1 conflicts) and move the PDFs out of git (§3)
- [ ] Barcode retry after a failed window, if `max` doesn't already fix it (§4.4)
- [ ] With integration: barcode command after 400 mm (§4.2) and old commands surviving into a new run (§4.3)
- [ ] Calibrate IR on the real track; run S3-U02, U03 and the new lighting/speed robustness test (§5, §6)

**Buddy 4: IMU, terrain, humps**
- [ ] Decide on the hump estimator (6° `ds·sin` vs front-contact 10 mm). This is a team decision, not code; REPORT §4.4 already matches the code (§4.5)
- [x] Add the timeout to the I2C FIFO wait in `hal_i2c_read()`: done, uncommitted (§4.6)
- [x] IMU recovers after a re-plug or a boot failure: done, uncommitted (§4.9)
- [ ] Run S4-U02..U06 (U04 events, U05 impact, U06 unplug are newly defined); save the logs (§6, §7)
- [ ] Pitch sign, magnetometer spin calibration, hump heights vs ruler (§5)

**Buddy 5: ultrasonic, servo, avoidance**
- [ ] Merge `origin/Raees` into `main` first (STOP during scans, watchdog, sonar stale) (§3, §4.1)
- [ ] Calibrate the servo; run S5-U02..U04; measure the lateral deviation during a bypass (§5, §6)
- [ ] INT-10 fault injection with the other buddies (FAULT_PATHS.md)

**Everyone / integration owner**
- [ ] SMP guard (§4.7); the 138 BARR findings in `tests/host/test_data.c` and `fault_stub.c` (§4.8)
- [ ] INT-01, INT-06, INT-11, then SYS-01..05 on the full course, with videos (§8)
- [ ] Fill in TEAM.md IDs, the REPORT author line and blanks, and regenerate `wiring.png` (§7)

## 2. Mission requirements vs status

| # | Brief requirement | Code (on `main`) | On the car | Test that proves it |
|---|---|---|---|---|
| 1 | Follow a line | `line_follow.c`, `vehicle.c` | not tested; `line=` gains are defaults | S3-U02, SYS-01 |
| 2 | Detect and decode barcodes | `ir_sensor.c`, `barcode*.c` (Code 39) | not tested | S3-U03, SYS-02 |
| 3 | Execute LEFT/RIGHT/STRAIGHT/U-TURN with encoder motion | `vehicle.c` + `motion.c` | not tested; see 4.2 and 4.3 | S2-U04, INT-01, SYS-02 |
| 4 | Detect humps | `imu.c`, `terrain.c` (LSM303D) | IMU reads correctly **at rest only** (before CR1) | S4-U02, SYS-03 |
| 5 | Report highest hump peak | `terrain.c` `max_peak_mm` → telemetry | not tested | S4-U03, SYS-03 |
| 6 | Detect obstacles on/near line | `obstacle.c` front monitor | one A9 screenshot only | S5-U04, SYS-04 |
| 7 | Profile shape/position (ultrasonic) | coarse 30–150° + fine scan, `avoidance_profile()` | not tested | S5-U02 |
| 8 | Bypass without collision | `avoidance.c` + `vehicle.c` | not tested | S5-U03, SYS-04 |
| 9 | Reacquire the line | `vehicle.c` line search | not tested | S5-U03, SYS-04 |
| 10 | Telemetry over WiFi | `subsystem1/` binary frames + `tools/dashboard` | no WiFi result recorded; **binary frames not run on the car yet** | S1-U03..U06, INT-08, SYS-05 |

Behaviour the brief also asks for, with no test written yet:
- **Line following robust to lighting, track imperfections, speed changes**
  (Expected Behaviour) → no test. Add S3 tests: run under two lighting
  conditions and at two speeds.
- **Avoidance "minimises deviation from the route"** → designed (smaller
  sideways offset, REPORT §4.5) but not measured. Add the measured lateral
  deviation to S5-U03.
- **Buddy 4 motion events** (stationary, accelerating, turning, climbing,
  descending, sudden impact) and **collision detection** → now defined as
  S4-U04 and S4-U05 (section 6), not yet run.
- **Buddy 1 connection recovery and heartbeat** → tests exist (S1-U04, U05),
  never run.

## 3. Branches that must be merged first

| Branch | What it has | State |
|---|---|---|
| `origin/Raees` (Buddy 5) | STOP latch on a GP21 edge interrupt; scans check STOP every 20 ms; sonar stale stop (300 ms); ECHO-stuck-high = FAULT; 250 ms watchdog; vehicle and obstacle host tests | 16 commits ahead. **Conflicts** with `main` in `README.md`, `obstacle.c`, `vehicle.c`, `REPORT.md`, `tests/host`, every `firmware/*.uf2` |
| `origin/max` (Buddy 3) | subsystem 3 rework, standalone host tests in `testing/subsystem3/`, `SUBSYSTEM3_REVIEW.md` | 2 commits ahead. **Conflicts** in `barcode.c`, `barcode.h`, `ir_sensor.c`, `subsystem3_test.c` (CR1 changed the same files). Also commits about 95k lines of PDFs into `max/` (datasheets, tribunal docs): move those out of git |

Merge order suggestion: Raees first (safety), then max. Re-run
`make -C tests/host`, build all modes, then rebuild `firmware/`. Do not
hand-pick the `.uf2` conflicts: rebuild them after the merge.

`docs/FAULT_PATHS.md` already marks the Raees items as "Raees branch, not merged".

## 4. Verified gaps in the code on `main`

Each was checked in the source on `9213b98`. Items marked **Fixed
(uncommitted)** are changed in the working tree and not yet on `main`. They
build with 0 warnings in all 9 modes and pass 187/187 host tests. None has
run on the car.

### 4.1 Safety (A14 / FR14). Owner: Buddy 5 with Buddy 2, integration
- **STOP is not seen during an obstacle scan** (up to 12 s), the
  **watchdog is missing**, and **sonar stale** is missing: all three are
  fixed on `Raees` and only need merging (section 3).
- **Velocity request older than 100 ms → brake** (FR14): **implemented
  (uncommitted)**. `motion.c` brakes a velocity command that is not
  refreshed for `MOTION_VELOCITY_TIMEOUT_MS` (100 ms). The vehicle task
  refreshes it with `motion_keep_alive()` from `poll_inputs()`, every 20 ms,
  in line following, line search and line seek. A keep-alive cannot restart
  an expired command: only a new `motion_set_velocity()` can. The TEST_MOTOR
  step test refreshes it too. Not run on the car yet: S2-U06.

### 4.2 Barcode command after 400 mm without a junction. Owner: integration + Buddy 3
`vehicle.c` `nav_due()`: when no junction is seen within
`NAV_PENDING_MAX_MM` (400 mm), the pending turn is **executed anyway**
(`car_config.h`: "no junction => act anyway"). The Week 6 design (FR3) says
**discard and report** it. The team must pick one, make code and REPORT
agree, and test at 399, 400 and 401 mm.

### 4.3 Old barcode commands survive into a new run. Owner: integration + Buddy 3
`start_run()` clears `pending_nav` but not the barcode queue or the partial
capture. A barcode seen while idle or calibrating can fire on the next
START (e.g. an instant U-turn). Reproduced on the host in
`SUBSYSTEM3_REVIEW.md` (on `max`). Fix: the barcode task flushes its
queue on a reset request from the vehicle task.

### 4.4 Barcode window not retried after a failed decode. Owner: Buddy 3
A small noise pair before a real barcode loses the whole barcode until 250
ms of white (`SUBSYSTEM3_REVIEW.md` finding 1, reproduced on the host).
Check whether `max` fixes it; if not, retry after shifting by one bar/space
pair.

### 4.5 Hump height estimator differs from the design. Owner: Buddy 4
`terrain.c` uses `h = Σ ds·sin(pitch)` with a **6°** trigger. The Week 6
design and the tribunal notes describe a **front-contact estimator with a
10 mm trigger**. Pick one and update REPORT §4.4. Then the brief's "accurate
peak measurement" needs real humps: S4-U03, ±5 mm (NFR6). If it fails,
report NFR6 as unmet; don't hide it.

### 4.6 I2C can hang the IMU task. Owner: Buddy 4. **Fixed (uncommitted)**
`hal_i2c_read()` waited for command-FIFO space with **no timeout** (only
the receive wait had one). A jammed bus would have spun the IMU task at
priority 8 and starved obstacle (9) and telemetry (10). The wait now uses
the same `HAL_I2C_TIMEOUT_US` (2 ms) and reports a failed read. Not
reproduced on the car; covered by S4-U06.

### 4.7 No guard against an SMP build. Owner: integration
All shared snapshots rely on DI/EI, which is atomic only because the build
is single-core (`CNF_SMP=0`; SMP only if built with `SMP=1`). Nothing in
`app_program/` checks it. Add `#if CNF_SMP` `#error` in one shared file.

### 4.8 BARR-C checker is not at 0
`python3 tools/barr_check.py app_program/*.[ch] app_program/subsystem*/*.[ch] tests/host/*.[ch]`
gave **TOTAL 149** on `9213b98`:
- 11 in firmware, from the TEST_WHEELS commit. **Fixed (uncommitted)**:
  firmware is now TOTAL 0.
- 138 in `tests/host/test_data.c` and `fault_stub.c`, from the CR1 commit:
  **still open** (owner: whoever wrote CR1 / integration).

### 4.9 IMU never recovers after a re-plug or a failed boot. Owner: Buddy 4. **Fixed (uncommitted)**
Found while defining S4-U06. The LSM303D powers up in power-down mode, so
after a re-plug its reads were acknowledged but stayed at zero, and `ok`
went back to 1 with dead data. A sensor that failed at boot was never
configured at all. Now, while the IMU fault is set, the imu task rewrites
the five configuration registers every step and reads again from the next
step. Not run on the car yet: S4-U06.

The README says "expect TOTAL 0" before committing.

## 5. Calibration and measurements still to do on the car

Everything below is a `[CALIBRATE]` value or a `___` in `docs/REPORT.md`.

**No new calibration code is needed.** The test modes and live commands
already measure everything. Put the result in `car_config.h` and reflash:

| Value | How | Formula |
|---|---|---|
| `WHEEL_DIAMETER_MM` | TEST_MOTION "forward 500", measure with a tape | new = old × tape mm / printed mm |
| `TRACK_WIDTH_MM` | TEST_MOTION "left 90", measure with a protractor | new = old × 90 / measured degrees |
| `ENCODER_SLOTS` | turn one wheel exactly once by hand, read `enc` in the dashboard (S2-U05) | ticks for one turn |
| `SPEED_WHEEL_MAX_MM_S` | TEST_MOTOR open-loop sweep, speed at 100 % | read directly |
| speed PID / feed-forward | TEST_MOTOR, then `ff=` and `pid=` live, START repeats the steps | copy the final `gains` |
| `IR_DEFAULT_WHITE/BLACK`, `line=` | TEST_IR (S3-U02); `line=` live in MISSION | read directly |
| `IMU_FORWARD_SIGN` | TEST_IMU, lift the front: pitch must go positive | flip to `-1.0f` if negative |
| magnetometer offsets | MISSION, START held 2 s (or `calibrate`) = 360° spin | automatic, RAM only: repeat after every boot |
| `SERVO_MIN_US` | TEST_ULTRASONIC (Buddy 5) | no live command: edit and reflash until 0° points right |

Do the wheel diameter before the track width, because the turn uses the diameter too.

| Owner | Measure | Where it goes |
|---|---|---|
| Buddy 2 | Wheel diameter, track width, encoder slots per turn, top wheel speed (TEST_MOTOR sweep), speed PID + feed-forward | `car_config.h`; REPORT PID table (around line 272), motion accuracy table (forward/back 500 mm, left/right 90°, U-turn) |
| Buddy 3 | IR white/black per sensor on the real track; line PID `line=` | `car_config.h` `IR_DEFAULT_*`; REPORT line table; barcode success A/B/C/D ___/10 at 100 and 180 mm/s |
| Buddy 4 | Pitch sign (`IMU_FORWARD_SIGN`); magnetometer spin calibration; hump heights vs ruler | REPORT hump table (around line 368) |
| Buddy 5 | Servo pulse at 0°/180° (`SERVO_MIN_US`); sonar on known boxes | REPORT profile table; bypass ___/10, re-acquisition ___/10 |
| Buddy 1 | WiFi soak (10 min), broker and AP outage recovery, message sizes on the wire | S1 results; INT-08 |
| All | CPU load and jitter, before/after CR1 | INT-06, INT-11 (`timing` command) |

## 6. Do the existing tests help?

**Yes for logic, no for the car.**

| Kind | What exists | What it proves | What it can't prove |
|---|---|---|---|
| Host (`make -C tests/host`) | 187 checks: PID, line classifier, Code 39 both directions, hump integrator, obstacle profile/plan, MQTT client, packed data + binary frames | The algorithms and data packing are correct on a PC with fake inputs | Real sensor noise, timing on the M0+, ISR interleaving, wiring, speed |
| Board modes (`APP_MODE=TEST_*`, 9 modes) | One test program per buddy, CSV on the USB console | Good tools for every S*-U board test | Nothing until someone runs them and saves the log |
| Test plans (`testing/*/TEST_LOG.md`) | S1–S5 unit, INT-01..INT-11, SYS-01..SYS-05 | Good coverage of the 10 requirements | **0 results recorded in any table**; only one log file in all of `testing/` |

Now defined in the test logs (uncommitted): **S4-U04** motion events,
**S4-U05** impact, **S4-U06** IMU unplug (no hang, recovers), and **S2-U06**
velocity expiry.

Tests still to add, because the brief asks for these and nothing checks them:
- **S3 robustness:** line following under two lighting levels and two
  speeds (brief: "lighting, imperfections, speed").
- **S5-U03 extension:** measure the lateral deviation during a bypass
  (brief item 8 of obstacle behaviour).
- **SYS-03 evidence:** the run maximum is already sent in every telemetry
  frame and hump event (brief Req 5, "measure and report"). A video or
  screenshot of the dashboard's "MAX" after the run proves it.

## 7. Paperwork left

- `TEAM.md`: student IDs and GitHub usernames are empty.
- `docs/REPORT.md`: author line blank, 16 `___` gaps. line 538 still says
  "No hardware measurements yet".
- `docs/img/wiring.png`: last generated before the LSM303D doc change. The
  `.dot` says LSM303D but the PNG probably still says GY-511. Regenerate it
  with `dot -Tpng docs/img/wiring.dot -o docs/img/wiring.png` (needs
  `brew install graphviz`).
- `videos/README.md`: the index is empty. Each SYS test should get a 15–60 s
  unlisted video.
- Week 13 for CR1: the RAM and message-size tables are filled from the host
  build (`docs/DATA_REPRESENTATION.md`). **Timing before/after (INT-11) and
  correctness on the car (INT-07, INT-08, INT-10) are still missing.**
- Buddy 4: the at-rest TEST_IMU logs from 16 Oct (scan finding 0x1D/0x49,
  pitch and roll ±0.2°, impacts 0 after the fix) were never saved to
  `testing/subsystem4/logs/`. Re-run on the current `main` (CR1 changed
  `imu.c`) and save them.

## 8. Suggested order to a working demo

1. **Merge `Raees` then `max`** into `main`. Fix the conflicts together,
   rebuild, and check 0 warnings and that host tests pass.
2. **Bring-up on the car, one buddy at a time:** motor → motion → IR →
   barcode → IMU → ultrasonic → telemetry (README "Bring-up order"). Fill in
   `[CALIBRATE]` values and record S*-U results with logs.
3. **Fix 4.2 and 4.3** (they cause wrong turns in the demo), then 4.5, 4.6
   and 4.7.
4. **Integration tests:** INT-01 (barcode → turn), INT-02/INT-10 (STOP and
   faults), INT-06/INT-11 (timing).
5. **System runs:** SYS-01..05 on the full course, with videos. Fill in
   REPORT tables from the logs.
6. Clear BARR to 0, then fill in TEAM.md, regenerate wiring.png and update
   REPORT §8.

## 9. Quick commands

```
sh setup.sh                                   # after every pull that touches the template patch
make -C tests/host                            # host tests: expect 187 passed, 0 failed
gmake -C template/build_make -j8 APP_MODE=TEST_IMU CONSOLE=usb_cdc   # make on WSL
python3 tools/barr_check.py app_program/*.[ch] app_program/subsystem*/*.[ch] tests/host/*.[ch]
screen /dev/cu.usbmodemMTK3PICO1 115200       # macOS console; Windows: PuTTY on the COM port
python tools/dashboard/picocar_dashboard.py --serial auto
```
Record every test as described in `testing/README.md`: commit hash, PASS or FAIL, and
the saved log file.
