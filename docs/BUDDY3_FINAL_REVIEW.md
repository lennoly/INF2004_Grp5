# Buddy 3 source and firmware review

Reviewed on 25 October 2026 against source at commit
`022387b488c03d40f3593e12190194d4241a3d72`, with the user's document deletions.
The submission folder and deleted documents are excluded from this assessment.
The original review was followed by the fixes described below. The deleted
submission documents were not used to implement the fixes.

## Current status after fixes

All five reproduced findings have been fixed in the working tree and covered
by assertion-based regression tests. The UART UF2 was rebuilt from the final
source with `MAKEFLAGS=-B`, without compiler warnings or errors, and its format,
vectors and source-cache identity were verified. Physical car performance and
on-board timing remain unverified.

| Finding | Implemented correction | Regression evidence |
|---|---|---|
| STOP lost with a full queue | Queue-independent STOP request plus a motion-owner latch; later commands remain idle until accepted restart | Full queue containing START requests, persistent braking, explicit restart, actual motion controller |
| Reset interrupts barcode consumption | API only requests a session; consumer owns ring/frame reset; 32-bit session tags reject stale queued results | Reset at each consumer memory barrier and immediately before old-result enqueue |
| Motor failure starts recovery | Failed/aborted navigation moves remain STOPPED; STOP also discards pending calibration | Forward and turn failures, STOP arriving at move completion |
| Old odometry confirms a junction | Rebase at START and after blocking maneuvers or successful search | No junction without fresh travel; new travel confirms normally |
| Noise loses a valid barcode | Slide failed candidates by a bar/space pair; disarm only after success | All A-Z, both directions, leading noise, no duplicate event |

Current UF2: 140288 bytes, 274 blocks. SHA-256:
`815ce7986ee7eddc7b16c457f27f4978fb756a9354b8cd5ba7b4578454be0d93`.
ELF text/data/bss: 69997 / 72 / 15024 bytes.

The expanded suite passes 3128 checks in each of four configurations, 13
junction decisions, driving regressions, barcode interleaving regressions and
actual motion STOP tests. The rebuilt general suite passes 136 tests. The BARR
checker reports zero non-deviation findings in the touched production files.

The queue payload now includes four session bytes: 10 bytes of payload and
12 aligned bytes, compared with the prior 6/8. The four-message buffer increases
from 48 to 64 bytes including headers. This is intentional stale-session
protection; the ISR ring remains 192 bytes. See
`testing/subsystem3/DATA_REPRESENTATION.md`.

Reproduce with `python3 testing/subsystem3/run_host_tests.py` and
`MAKEFLAGS=-B sh tools/build_buddy3.sh` in WSL/Linux. Current evidence is saved in
`testing/subsystem3/logs/2026-10-25_buddy3_fixes.txt`. No source commit or hardware
pass is claimed for these uncommitted changes.

## Original findings before fixes

The following records describe the original commit and original probes. They
are preserved as the reason for the changes, not as the current status.

### 1. High priority: STOP can be lost when the vehicle command queue is full

Locations: `app_program/subsystem1/telemetry.c:316` and
`app_program/vehicle.c:919`.

If submitting a STOP request fails, telemetry calls `motion_stop()` directly.
That stops the motion command but neither latches STOP nor changes the vehicle
state. A vehicle still in LINE_FOLLOW issues another velocity command on its
next iteration. The probe executes the production fallback action and the
production line-follow step: the state remains LINE_FOLLOW and both requested
wheel speeds return to 180 mm/s. Queue saturation itself is inferred from the
checked error branch; this probe isolates the behavior after that branch.

Give STOP a queue-independent latch that the motion and vehicle owners honor.
Reject further movement until an explicit accepted START clears it. Add a test
that fills the real command queue, sends STOP and verifies continued braking
across subsequent control cycles.

### 2. High priority: START/STOP can reset the barcode ring during consumption

Locations: `app_program/subsystem3/barcode.c:102`,
`app_program/subsystem3/barcode.c:135`,
`app_program/subsystem3/ir_sensor.c:277`, and
`app_program/vehicle.c:959`.

The vehicle task has higher priority than the barcode task. It can interrupt
an in-flight `ir_sensor_barcode_pop()` and call `barcode_set_enabled()`, which
changes the ring tail and decoder state. Disabling interrupts inside the reset
does not prevent the interrupted consumer from resuming afterward. The
consumer checks enabled only at entry to its poll, not throughout the operation.

A deterministic pre-emption probe calls the real reset at a consumer memory
barrier. The reset empties the ring, but the resumed pop advances its tail:
head becomes 6 and tail becomes 7. An empty ring should have equal indices.
This is a modeled legal interleaving, not an observed on-board failure rate.

Have the barcode owner perform resets through a request/acknowledgment protocol,
or otherwise synchronize the complete reset with consumer execution. Protecting
only individual index writes is insufficient.

### 3. High priority: a failed motor move starts line-search recovery

Location: `app_program/vehicle.c:573` through `execute_nav()`.

Forward/turn completion failure and unsuccessful line reacquisition are combined
into one Boolean. Both lead to LINE_SEARCH. The probe injects MOTION_WAIT_FAIL
for the initial forward move and obtains LINE_SEARCH. That state issues rotating
wheel commands and can return to ordinary driving if the sensors see a line.
A timed-out motor move has not established that further motion is appropriate.

Distinguish motion failure, STOP/cancellation and unsuccessful line reacquisition.
Brake and enter STOPPED on a failed move; reserve search recovery for a completed
movement whose line reacquisition failed. Test failures in each navigation leg.

### 4. Medium priority: stale odometry can falsely confirm a junction

Locations: `app_program/vehicle.c:892`, `app_program/vehicle.c:948`, and
the return paths from navigation/search into LINE_FOLLOW.

`g_prev_odo` is updated only in the line-follow step. START resets the classifier
but does not rebase that odometry value. Blocking maneuvers also move the car
without updating it. The first following sample can therefore credit old travel
as distance traveled while both sensors were black.

With a previous reading of 100 mm, a current reading of 500 mm and no additional
travel, the restart probe immediately reports JUNCTION with 10 mm confirmation.
The post-maneuver probe also supplies a barcode queued during the maneuver and
obtains NAV_TURN without new confirmation travel. The barcode in this second
case is injected; this is a controller test, not a complete sensor-stream test.

Rebase odometry when starting a run and when returning from blocking movement
or search. Require fresh samples and travel for junction confirmation.

### 5. Medium priority: a short leading noise pair loses a valid barcode

Location: `app_program/subsystem3/barcode.c:277`.

A failed 29-element candidate clears the entire window and disarms collection
until the next white quiet interval. It does not retry using the remaining
elements of the same pass.

The actual sampler, framer and decoder were exercised with 300 ms of white,
5 ms black, 5 ms white, then a valid *A* barcode. No event was produced in either
direction. Clean controls decoded in both directions. These are missed reads,
not evidence of wrong decoded commands.

On an invalid candidate, advance by a bar/space pair and retry a bounded window.
Disarm after successful decoding to preserve one event per pass. Add leading
noise, overflow and quiet-reset regression cases.

## Original pre-fix firmware verification

`firmware/picocar_TEST_BUDDY3_uart.uf2` was rebuilt with `MAKEFLAGS=-B` through
`tools/build_buddy3.sh`, forcing the compiler and host converter to run again.
The build exited successfully with no compiler warnings or errors. Every cached
application C/header file matches the current workspace; no extra cached
application C/header files were found. The delivered UF2 matches the build output.

| Check | Result |
|---|---|
| Target | TEST_BUDDY3, single-core RP2040, UART console, Wi-Fi disabled |
| UF2 size | 139264 bytes, 272 blocks |
| UF2 structure | Valid magic, block numbering, payload sizes, RP2040 family and sequential flash addresses |
| Initial stack pointer | 0x20040000 |
| Reset vector | 0x100084a1, valid Thumb address in flash |
| Build identity | Expected Buddy 3 and runtime mapping strings present |
| ELF text / data / bss | 69321 / 72 / 15008 bytes |
| SHA-256 | `818f087acee65ad72340729b65452d76b97249555b62adaf38da052c85ed4be8` |

The checksum is unchanged from the previous build. Document deletions did not
change the firmware source. A correct build reproduces the source's defects;
it does not remove them.

## Original review tests and evidence

- Existing Buddy 3 suite: 3022 checks in each of four configurations, zero
  failures; mission/telemetry compilation, 13 junction decisions and existing
  driving integration checks passed.
- Rebuilt general host suite: 136 passed, zero failed.
- Additional probes reproduced all five cases above. Run
  the historical diagnostic probes with GCC in WSL/Linux. Their original
  output is retained in the review evidence. The current
  `testing/subsystem3/review_probes.py` has been converted to assertion-based
  regression tests and fails on unsafe behavior.
- The repository BARR checker reported three non-deviation findings across
  Buddy 3, vehicle and app_main: app_main line length and missing cast comment,
  plus an unparenthesized logical operand in vehicle. These are separate from
  the runtime findings and from the warning-free compiler build.
- Saved evidence: `testing/subsystem3/logs/2026-10-25_final_review.txt`.
  Full rebuild output and generated probes are in `tmp/final-review/`.

The UART image still needs its mapping configured after reboot and uses the
configured IR defaults. It has no MQTT transport and therefore cannot receive
the instructor's mapping over MQTT. A network demonstration needs the MQTT
profile and local network settings. No source-only review can establish sensor
polarity, calibration, motor direction, encoder scale, braking accuracy, barcode
placement after U-turns or loaded timing on the physical car. Those remain
hardware checks, and no flashing or on-car testing was performed here.
