# Subsystem 3 review

Reviewed on 21 October 2026 against checkout `c6793d4`, including the additional documents in `max/`.

Subsystem 3 is not yet ready to be called fully correct or integration-ready. Its existing software tests pass, but an additional test reproduces missed barcode recovery, the representation changes are incomplete, and the shared vehicle controller violates parts of the barcode contract. No production source was changed during this review.

The detailed code review covers all ten files in `app_program/subsystem3/`, their host tests, and the shared launcher, configuration, HAL, PID and vehicle interfaces they depend on. Repository documentation, course requirements, the Week 6 design, lecturer feedback and the supplied Week 8 tribunal document were used as context. Other subsystems are not being certified by this review. Prebuilt UF2 files are not proof that current source builds or works on hardware.

## Findings in your subsystem

### 1. Recover from a failed barcode window instead of ignoring the remaining barcode

Priority: P2. Location: `app_program/subsystem3/barcode.c:242`, especially lines 277–278 and 318–320.

After the first 29 elements, `finish_frame()` clears the capture and disarms it even when decoding fails. All subsequent elements are ignored until 250 ms of white. A small spurious black/white pair before a valid barcode therefore shifts the window and loses the entire barcode.

Reproduced using the actual sampler, framing and decoder with the existing host fakes: 300 ms white, 5 ms black, 5 ms white, then a clean `*A*` with 10 ms narrow elements and 30 ms wide elements. Result: no event. A clean `*A*` after the next quiet interval produces LEFT. The Week 6 design, section 6.4, explicitly calls for advancing a failed window by one bar/space pair and retrying. That recovery is absent.

Recommended fix: retain the last 27 elements after a failed candidate and try again after the next bar/space pair. Disarm after successful decoding to preserve the no-duplicate behaviour. Add leading-noise tests in both reading directions, plus overflow and quiet-reset tests.

### 2. Finish the minimum-bit representation work and reconcile the tribunal proposal

Priority: P2, requirement compliance. Locations: `ir_sensor.h:25`, `ir_sensor.h:37`, `line_follow.h:35`, `line_follow.h:36`, `ir_sensor.c:23`, `barcode.c:34`.

The three-byte edge ring and six-byte event payload are useful reductions, but they do not fulfil the supplied change requirement throughout subsystem 3. Examples: raw and calibration counts carry 12-bit values in 16-bit fields; the three-state line enum has no explicit two-bit representation; `lost_ms` is capped at 150 but stored in 32 bits. Floating-point interface precision/ranges still need an agreed representation. Full-width arithmetic can remain internal where needed; compact persistent/shared representations need an explicit design.

The supplied `max/CSC05_ImmTri1.docx`, Version B, also proposes 1 ms widths, an eight-bit width with 255 reserved for over-range, and a one-bit colour in a 16-bit ring entry. Current code instead uses an 18-bit microsecond width in three bytes and saturates durations at 250000 us. It does not expose the proposed over-range marker. Decide whether the proposal remains authoritative before changing the timing format; jitter and quantisation need validation.

`testing/subsystem3/DATA_REPRESENTATION.md` correctly labels the implementation partial. Keep that qualification. The expected 128-byte ring in the tribunal proposal is not the current 192-byte ring. Week 13 still needs measured linked RAM and MCU timing comparisons.

## Shared integration findings affecting your subsystem

These belong to the shared vehicle integration owner, with your input on the barcode and line interfaces. They do not require you to take over another buddy's subsystem.

### 3. Old barcode commands can survive calibration and a new START

Priority: P1. Locations: `app_program/vehicle.c:953`, `app_program/vehicle.c:974`, `app_program/subsystem3/barcode.c:30`.

The barcode task starts enabled. Mission calibration does not disable it, and `start_run()` clears only `pending_nav`, not the queued decoded events or partial capture. Commands detected while idle, during calibration, or before STOP can be consumed on the next run. In particular, an old D can cause an immediate U-turn.

An additional host probe began calibration, streamed a valid D, cancelled calibration, and then received NAV_UTURN from the queue. This reproduces the queue behaviour; source inspection establishes that the mission path does not clear it at START. The standalone barcode bench already disables/resets capture around calibration, so its existing passing tests do not cover the mission defect.

Recommended fix: define mission capture lifecycle explicitly, discard events from prior sessions, and pause/reset capture around calibration. Do not blindly call the existing reset from any priority: its frame state is normally owned by the barcode task, and the vehicle task has higher priority. Use an owner-handled reset/acknowledgement or another properly synchronised reset protocol.

### 4. Expired navigation commands execute instead of being discarded

Priority: P1. Location: `app_program/vehicle.c:839`, especially line 846.

`nav_due()` returns true when a pending command has travelled more than 400 mm without a junction. This executes a turn where no junction was detected. Week 6 FR3 and the accepted design change require discarding and reporting an unused junction command after 400 mm. The current root report describes the old behaviour and conflicts with that design.

Recommended fix: expire LEFT/RIGHT/STRAIGHT separately from the execute condition, clear the pending command, and report expiration. Keep the intended immediate U-turn behaviour. Verify at 399, 400 and 401 mm and with a detected junction.

### 5. Cruise speed limiting does not enforce the barcode sensor speed limit

Priority: P2. Location: `app_program/vehicle.c:923`; associated constants in `car_config.h`.

The controller commands `speed - steer` and `speed + steer` without accounting for the barcode sensor's forward/lateral offsets. NFR2 explicitly limits speed at the sensor, including turns, to 300 mm/s. A cruise-command cap alone cannot guarantee this because the sensor also moves through the turn's angular velocity.

Recommended fix with the motion/integration owner: measure the offsets and enforce the documented sensor-speed bound, preserving wheel-speed ratios where practical. Test tight steering and navigation turns as well as straight travel. Do not present the simulated 100–300 mm/s decoder tests as evidence that this physical bound is enforced.

## Verification performed

| Check | Result |
|---|---|
| Standalone runner, TEST_IR and TEST_BARCODE, both polarities | 610 checks per configuration, zero failures; 2440 total |
| Existing full host suite compiled from its Makefile source list and flags | 123 passed, zero failed; includes 1000/1000 noisy synthetic barcode frames |
| Repository BARR checker on subsystem 3 production files | Zero findings excluding documented deviations, using forward-slash paths |
| Additional leading-noise barcode probe | Missed valid A reproduced; clean control decoded |
| Additional calibration queue probe | D remained queued after calibration cancellation |
| Current ARM firmware build | Not performed; template/SDK absent and ARM GCC not found on PATH |
| Actual car tests | Not performed; user confirmed no hardware testing yet |

The BARR tool misinterprets Windows backslash paths as module names; rerunning with forward slashes removes those false findings. It is a heuristic checker, not full BARR certification.

The existing tests support steering sign, line-loss and junction thresholds, normalisation, both sensor polarities, valid forward/reverse decoding, malformed input rejection, packed transport, queue/ring overflow and deterministic pause/resume behaviour. They do not simulate arbitrary task/interrupt interleavings, electrical noise, real speed, MCU execution time or physical line tracking.

## Hardware work still needed

1. Identify the installed IR modules and measure black/white ADC values on all three sensors at the intended height. Confirm polarity and the required 2048-count contrast under lighting changes and with motors running.
2. Build fresh TEST_IR and TEST_BARCODE firmware. The subsystem test README says the checked-in UF2 files predate the standalone changes.
3. Confirm barcode-side coverage in both travel directions, especially after U-turns. The user confirmed this is unresolved. Reversing the decoder's widths cannot compensate for a sensor physically missing the barcode strip.
4. Measure recognition at 100, 180 and 300 mm/s, both reading directions and bends, with zero wrong commands and the stated success-rate target. Include a spurious leading edge and stopping over a bar.
5. Measure sampler/task timing under load and perform moving-car line/junction checks with the motion owner. The motor-off bench deliberately cannot validate closed-loop tracking or measured junction travel.

Update `docs/REPORT.md` before submission: it still says calibration rejects spans below 200 counts (current threshold is 2048), frames finish after a 250 ms quiet interval (current code tries at element 29), and barcode queue messages are never lost (full queues increment `dropped`).

Recommended order: fix barcode recovery and agree the data representation in your subsystem; hand the three shared integration findings to the vehicle/motion owners; then build and perform hardware validation before claiming subsystem 3 complete.
