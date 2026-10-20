# Subsystem 3 standalone testing

These tests cover the three IR sensors, calibration, line classification and
steering, Code 39 decoding, and navigation command output. They do not need
WiFi, an IMU, ultrasonic sensing, encoders, or a working motor controller.

**Both board test modes keep the motor outputs low. Move the car or barcode
by hand.** Line steering is printed, not applied to the wheels. Autonomous
driving and execution of turns belong to later integration with the motion
and vehicle modules.

## 1. Run the PC tests now

From the repository root in PowerShell:

```powershell
python testing/subsystem3/run_host_tests.py
```

Requirements: Python 3 and GCC on PATH (or set `CC` to the compiler executable).
No Python packages, board, SDK, make, or WiFi are needed. The runner builds in a
temporary directory and removes the executables afterward.

The runner compiles the actual subsystem C files, the board test routines and
their shared launcher, with C99 and warnings treated as errors. It runs four
configurations: `TEST_IR` and `TEST_BARCODE`, each with black-high and black-low
IR polarity. The only reused motion component is the existing pure-C PID
arithmetic library; no motor task or driver is linked.

Coverage includes:

- The supplied course barcode photo, A/B/C/D and a report-only Z.
- Forward/reverse decoding, simulated speeds of 100–300 mm/s, distorted widths.
- Exact frame length, missing/oversized gaps, zero widths, invalid symbols,
  invalid output buffers, and failure clearing the output string.
- Actual 1 kHz ADC sampler and 10 ms barcode consumer, decoding within 15 ms of
  the last bar in the simulation without waiting 250 ms.
- All-three-channel calibration, failed-capture rollback, cancellation,
  polarity, and normalisation.
- 150 ms line-loss confirmation, 10 mm junction confirmation, recovery,
  steering direction, invalid inputs and counter saturation.
- ISR ring overflow, full message queue, pause/resume, no duplicate commands,
  timer wraparound, repeat initialisation, and immediate STOP in each bench
  routine.

These are deterministic software tests. Their elapsed time is simulated;
they do not measure MCU execution time, electrical behaviour or on-car accuracy.
The host kernel adapter does not simulate arbitrary interrupt interleavings.

## 2. Connect the three sensors

| Sensor | Analogue output | Supply |
|---|---|---|
| Left line sensor | GP26 / ADC0 | 3V3 and GND |
| Right line sensor | GP27 / ADC1 | 3V3 and GND |
| Barcode sensor | GP28 / ADC2 | 3V3 and GND |

Use the actual module's AOUT pin and check its pin order. The design uses jumper
wires to the GPIO breakout, not a straight-through Grove cable. GP26 appears on
two Grove connectors; the GP28 VBAT-sense jumper must remain open.

Place the two line sensor spots on the two edges of the 18 mm line. Position
the third over the barcode strip; the specification shows a 20 mm gap from
the line edge to the barcode. Record actual mounting height and offsets.

`IR_BLACK_IS_HIGH` in `app_program/car_config.h` defaults to 1. If raw counts
fall over black, use 0. Calibration captures extrema; it cannot determine
which surface you intended as black without this setting.

## 3. Build and flash one bench mode

Use the project's existing WSL setup described in the root README. After
`sh setup.sh` has installed the template and SDK, build in that Linux checkout:

```sh
make -C template/build_make -j8 APP_MODE=TEST_IR CONSOLE=usb_cdc
```

Or build the barcode test:

```sh
make -C template/build_make -j8 APP_MODE=TEST_BARCODE CONSOLE=usb_cdc
```

Flash the newly built `.uf2` from `template/build_make`, using BOOTSEL.
The existing files in `firmware/` predate these changes and do not contain
the new standalone tests.

Open the board's USB serial port using a serial terminal (115200 baud if
requested). These modes print directly to the terminal; they do not start
telemetry or accept the mission's serial tuning commands.

The firmware build was not run during this implementation: this checkout
has no fetched template/SDK and the inspected WSL installation has no
`arm-none-eabi-gcc`. Host compilation is not a substitute for the ARM build.

## 4. Test IR and line decisions

1. Flash `TEST_IR`. Press and release GP20 START.
2. During the five-second calibration, move **every sensor** across black and
   white at the intended mounting height.
3. Every channel must span at least 2048 ADC counts. Failure keeps the old
   calibration and ends the test. The diagnostic prints captured `min/max`
   and active `white/black` values; these can differ after a rejected capture.
4. On success, read the CSV output for 60 seconds:
   `ms,rawL,rawR,rawB,nL,nR,nB,error,state,both_black,steer_mm_s`.
5. Press GP21 STOP to end early. Press and release START to repeat.

Normalised values and error are in permille: 0 is white and 1000 is black.
State 0 = ON, 1 = LOST, 2 = JUNCTION. Positive steering means turn left;
negative means turn right. Both sensors white for at least 150 ms should
produce LOST and zero steering.

The bench has **no measured distance**. `both_black=1` identifies a junction
candidate; it does not falsely claim 10 mm travel or confirm a junction.
The PC tests exercise the 10 mm rule with known distances. A measured,
moving-car junction test remains part of later integration.

## 5. Test barcodes

1. Flash `TEST_BARCODE`, press/release START and perform the same calibration.
2. Leave the barcode sensor on white for at least 250 ms.
3. Move a printed `*A*`, `*B*`, `*C*`, or `*D*` through its field of view.
4. Expect `barcode,<timestamp_ms>,<symbol>,<command>`.
5. Repeat in the other direction; leave white for at least 250 ms between passes.
6. STOP ends the test; otherwise it ends after 60 seconds and prints counters.

| Symbol | Reported command |
|---|---|
| A | LEFT |
| B | RIGHT |
| C | STRAIGHT |
| D | UTURN |
| Other valid single Code 39 symbol | NONE |

The course capture task intentionally accepts **one data symbol per frame**
(29 elements). The pure decoder also supports longer frames, but they are not
the course navigation format. There is no motor movement on a decoded command.

`frames` counts complete candidates, `decoded` counts valid decodes, `errors`
includes incomplete/malformed candidates, `overruns` counts lost ISR elements,
and `dropped` counts decoded events that could not fit the queue. All printed
counters are differences for this test session.

An empty queue or zero errors does not prove that every physical pass was
detected. Count the passes yourself. For A6, record 20 passes per symbol and
direction at measured 100, 180 and 300 mm/s, including bends: at least 90%
correct and **zero wrong commands**. Hand movement without a speed measurement
is a functional check, not evidence of meeting the speed requirement.

## 6. Record evidence

Keep the existing test IDs in `TEST_LOG.md`. Save terminal output, photos and
measurements in `logs/`. Commit the implementation before formal hardware
testing, then record the commit ID with each result. Do not mark hardware
tests passed based on host output.

See `DATA_REPRESENTATION.md` for the local packing changes and their limits.
