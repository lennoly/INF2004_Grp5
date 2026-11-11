# Handover

The car as it stands on `main` (11 Nov 2026): what it has to do, how the code
does it, and what has been tested. Build and setup: [README.md](README.md).
Who owns what: [TEAM.md](TEAM.md). Every test run: `testing/*/TEST_LOG.md`.

## 1. What the robot must do

On a line track, autonomously:

1. Follow the line.
2. Read Code 39 barcodes beside the line and turn LEFT / RIGHT / STRAIGHT / U-TURN at the next junction.
3. Detect humps and report the highest hump's height.
4. Detect an obstacle on or near the line, profile it with the servo-mounted sonar, go around it without touching it, and rejoin the line.
5. Send telemetry over WiFi (MQTT) and accept commands (start, stop, tuning) from the laptop dashboard.
6. Stop safely on any fault (sensor lost, wheel stalled, task hung).

## 2. How it runs

One firmware image, µT-Kernel tasks by priority:

| Task | Rate | Does |
|---|---|---|
| IR sampler (timer ISR) | 1 kHz | reads 3 IR channels, times barcode bars |
| `motion` | 50 Hz | wheel speed PIDs, distance/turn moves, odometry |
| `vehicle` | 50 Hz | mission state machine, line following (the only task that drives the car) |
| `barcode` | 100 Hz | frames and decodes Code 39 |
| `imu` | 50 Hz | pitch/roll, humps, impacts |
| `sonar` | ~14 Hz | front distance monitor, scans |
| `telemetry` | 20 ms loop | commands in (USB/UART/MQTT), telemetry 5 Hz, heartbeat 1 Hz |

Vehicle states: `IDLE → CALIBRATE → LINE_FOLLOW ⇄ NAV_TURN / LINE_SEARCH / OBSTACLE → STOPPED`.
Every state change is sent as an event with its reason (e.g. `STOPPED (fault: encoder)`).

**Buttons:** GP20 START (short press: run; hold 2 s while idle: calibration
spin, 330°). GP21 STOP. The same from the dashboard.

## 3. Code map

| Where | What |
|---|---|
| `app_program/car_config.h` | every pin and tunable. Values marked `[CALIBRATE]` were set on this car |
| `app_program/vehicle.c` | mission state machine, junction turns, obstacle bypass, line search |
| `subsystem1/` (Kent) | `mqtt_lwip`, `mqtt_bridge`, `telemetry`, `command`, `console_in`, `frames` (binary messages) |
| `subsystem2/` (Levia) | `motor`, `encoder`, `pid`, `motion` |
| `subsystem3/` (Max) | `ir_sensor` (calibration), `line_follow`, `barcode`, `barcode_decode` |
| `subsystem4/` (Lennon) | `imu` (LSM303D on I2C0), `terrain` (hump height) |
| `subsystem5/` (Raees) | `ultrasonic`, `servo`, `obstacle` (scan task), `avoidance` (profile and plan) |
| `app_program/hal*.c` | Pico SDK and kernel glue: PWM, ADC, GPIO interrupts, I2C, watchdog |
| `tests/host/` | 325 PC tests, including a simulation of `vehicle.c` |
| `tools/dashboard/` | dashboard, car simulator, `wifi_test.py` |

Pins: motors GP8–11, encoders GP2/3, IR GP26/27/28, IMU GP4/5, sonar TRIG/ECHO
GP6/7, servo GP12. Full table: [docs/WIRING.md](docs/WIRING.md).

## 4. Current settings (what the tests passed on)

| Area | Setting |
|---|---|
| Speed | 75 mm/s for cruise, turns and bypass (`speed=` 50–300 live) |
| Wheel PID | kp 0.104, ki 0.60, kd 0, feed-forward 0.12, offset 2.3 % |
| Geometry | wheel 58.9 mm, track 132 mm, IR row 140 mm ahead of the axle |
| Line follow | gain 18 (also for the first 150 mm after a start or turn), ki = kd = 0 |
| Barcode | black above 0.08, white below 0.05 (normalised), frame ends after 500 ms of white |
| Barcode map | A LEFT, B RIGHT, C STRAIGHT, D U-TURN by default; `map=<1..4>,<idx>` changes it live (A=10 … Z=35) |
| Hump | starts at 6° pitch or 3° roll; height from pitch/roll × distance travelled |
| Obstacle | stop at 2 readings under 200 mm; echoes at 280 mm or more are floor; scan 50°–130° in 5° steps |
| Bypass | 45° out, straight past (box assumed ≤ 150 mm deep), 45° back to the line; blind or centred boxes pass on the left |
| Safety | watchdog 250 ms while moving; sonar: 4 bad pings or 300 ms without a reading stops the car |

Live tuning only works while IDLE or STOPPED, and resets on reboot: copy good
values into `car_config.h` and rebuild.

## 5. Tested features

All on the real car unless marked host. Details and logs in `testing/`.

| Feature | Test | Result |
|---|---|---|
| Full lap, line following | SYS-01 | PASS: full laps without leaving the line (4 and 9 Nov, video) |
| Barcode turns at junctions | SYS-02 | PASS: LEFT, RIGHT, STRAIGHT, U-TURN 4 of 4, read on the first pass (10 Nov, videos) |
| Hump height | SYS-03 | PASS: 10 mm hump read 9.8 / 12.2 mm (10 Nov, video) |
| Obstacle bypass and rejoin | SYS-04 | PASS: box right of, centred on and left of the line (9 Nov, videos) |
| Telemetry over WiFi | SYS-05, S1-U03 | PASS: 10 min steady at 4.75 Hz; runs of up to 23 min connected |
| Broker / hotspot outage recovery | S1-U04, U05 | PASS: reconnects by itself |
| Live commands, tuning refused while driving | S1-U07, U08 | PASS |
| STOP from the dashboard | INT-02 | PASS: motors told to stop 9–12 ms after the command arrives |
| Wheel speed PID | S2-U02 | PASS: no oscillation, settles in ≈ 400 ms |
| Distance and turn accuracy | S2-U03, U04 | PASS: 500 mm exact, 90° turns +1°, U-turn +3° |
| IR calibration | S3-U02 | PASS |
| Barcode reads | S3-U03 | PASS by team decision: 16/20 at 100 mm/s, 25/27 at 180 mm/s |
| Hump, IMU events, impacts, IMU unplug | S4-U02 to U06 | PASS |
| Obstacle side choice | S5-U03 | PASS: 6 of 6 correct side, line rejoined 6 of 6 |
| STOP during a scan, humps not seen as obstacles, watchdog | S5-U08, U10, U11 | PASS |
| Fault stops (encoder, motor stall, IMU, WiFi loss) | INT-10 | PASS |
| Timing over a full lap | INT-06, INT-11 | PASS: jitter 1779 µs (limit 2000), CPU 12 % |
| Unit tests and vehicle simulation | host | PASS: 325 of 325 |

## 6. Known limits

- **Barcode:** the sensor sits high, so narrow bars barely register. Reliable at 75 mm/s; faster runs miss frames.
- **Obstacle width:** the sonar beam makes a small box read 100–120 mm too wide (S5-U07 FAIL on width). The centre is accurate, so the side choice is right and the extra width is margin.
- **Sonar wire pulled:** the car always stops, but pulling a sonar wire at the board also upsets the IMU's I2C, so the reason shown is `fault: IMU` (S5-U12, marked done). A rerun unplugging at the sensor end is optional.
- **Obstacle depth** is not measured: a box deeper than 150 mm would be clipped on the way back to the line.
- Tuned values live in RAM only.

## 7. Left to do

- Upload the 9 silent videos (unlisted YouTube); put the links in `videos/README.md` and the matching result rows.
- Optional: S5-U12 rerun at school.
- Dropped, do not run: S5-U09, S3-U03 reruns.

## 8. Before a demo

1. Build `MISSION` (WiFi build if showing the dashboard), flash, power on with the car still.
2. Hold START 2 s over the line: calibration spin.
3. Set the barcode letters with `map=` if the course uses other letters.
4. Short press START. Watch the dashboard: state, barcode, hump peak, obstacle events.

Report with the design and full evidence: [docs/PicoCar_Project_Report.docx](docs/PicoCar_Project_Report.docx).
