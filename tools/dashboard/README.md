# PicoCar dashboard (Buddy 1)

Live telemetry, live tuning and CSV logging for the PicoCar, in your web
browser. A small server on the laptop (Python standard library) talks to the
car and serves one page at http://127.0.0.1:8050, which opens by itself.

## Install (once)
```
pip install -r requirements.txt
```

## Connect
| Car firmware | Command |
|---|---|
| USB-console build (`*_usbcdc.uf2`), cable to the car | `python picocar_dashboard.py --serial auto` |
| WiFi build (`*_wifi.uf2`) | `python picocar_dashboard.py --mqtt <broker-ip> --car car1` |

* `--serial auto` finds the Pico by its USB vendor id; or give the port
  (`--serial COM5`). `--list-ports` shows them.
* Close PuTTY or any other serial monitor first (one program per port).
* MQTT: run mosquitto on the laptop (`mosquitto -c template/config/mosquitto.conf -v`),
  with the laptop's IP in `template/config/mqtt_config.h`.
* `--http-port 8051` if 8050 is taken; `--no-browser` to open the page yourself.
  Ctrl+C in the terminal stops it (the page then says "dashboard stopped?").

## The page
* **Header:** link up/down, car online/offline, vehicle state, messages per
  second, active faults.
* **Charts (last 30 s, plain canvas):** wheel speed vs target (left/right),
  line error with the left/right IR sensors.
* **Values:** motion (speed, target, PWM, encoder ticks, odometer, heading),
  line and barcode (IR bars, last barcode, pending turn), IMU and humps
  (yaw/pitch/roll, terrain event, speed and task, bumps, shock count),
  barcode (letter read, what the car does with it, pending turn, the demo
  letters), obstacles (front distance, last action, last scan's distance,
  size, centre and clearances; type the real box width to see the width
  error, or "unknown" when an edge was not seen), link heartbeat (WiFi state, sent/dropped, re-joins, chip resets).
* **Terrain profile:** bump height and pitch over the last 2 m driven.
* **SHOCK pill:** red in the header for 5 s after each impact, with its
  strength in mg.
* **Distances** (front and last scan) show the sonar reading and, in
  brackets, the distance from the wheel axle (+70 mm, `OBST_SENSOR_AHEAD_MM`).
* **Barcode turns** show whether they ran at the junction or after 400 mm
  without one ("LEFT, no junction" in the events).
* The page fits one screen (about 1366 x 650 or larger); a smaller window
  falls back to one long page.
* **Events and car console:** newest at the top.
* **Controls:** Start / Stop / Calibration spin, Fast 20 Hz / Normal 5 Hz,
  Gains, the tuning boxes, the demo barcode letters and a free command box.

The page only gets at most 5 updates a second and redraws a chart only when
new data arrived, so it stays smooth at 20 Hz telemetry.

## Live tuning (no reflashing)
The tuning boxes fill in from the car. Edit and press **Apply**:

| Box / command | Meaning | Default (`car_config.h`) |
|---|---|---|
| `pid=kp,ki,kd` | wheel-speed PID | `SPEED_KP/KI/KD` 0.08, 0.6, 0 |
| `ff=kf,offset` | feed-forward %/(mm/s), friction offset % | `SPEED_KF` 0.104, `SPEED_OFFSET_PCT` 2.3 |
| `line=kp,ki,kd` | line-following PID | `LINE_KP/KI/KD` 160, 0, 12 |
| `speed=mm/s` | cruise speed (50..300) | `SPEED_CRUISE_MM_S` 50 |
| `rate=ms` | telemetry period, 0 = off | 200 (MQTT), 1000 (USB) |

Other commands: `start`, `stop`, `calibrate`, `gains`, `help`, `timing`
(task times, control jitter and CPU load), `nav=1..4` (turn as if a barcode
was read: 1 left, 2 right, 3 straight, 4 U-turn), `map=1..4,index` (the
barcode letter for that turn as its Code 39 index, A=10 .. Z=35; the
**Demo barcode letters** boxes send these for you, and the car goes back to
A B C D when reset).
Tuning (`pid=`, `ff=`, `line=`, `speed=`, `map=`) is accepted only while the car is
IDLE or STOPPED; while it drives, the reply says so.
The car answers every command with an event (`cmd ... ok` or the reason it
failed) and then reports the gains now in use.

**Gains live in RAM:** they reset on reboot. When you are happy, copy the
final values (shown in the tuning boxes, and in `commands.log`) into `car_config.h`.

Suggested workflow for speed PID (with `APP_MODE=TEST_MOTOR`):
1. Press START on the car: an open-loop sweep, then step responses (CSV in
   `console.log`).
2. Press **Fast 20 Hz** to watch the step response in the speed chart.
3. Change `pid=` / `ff=`, press START again (only the steps repeat), compare.

Line PID: build `MISSION`, press **Start** and watch the line-error chart;
press **Stop**, change `line=`, press **Start** again and compare.

## Logs (for the report)
Each run creates `logs/<date_time>/`:

| File | Content |
|---|---|
| `telemetry.csv` | every telemetry message in engineering units (deg, mm, mm/s, 0..1) |
| `events.csv` | all events as JSON (barcodes, humps, obstacles, command acks, gains) |
| `heartbeat.csv` | link health once per second (MQTT) |
| `commands.log` | every command you sent, timestamped |
| `console.log` | raw serial console (includes the TEST_xxx CSV output) |

## WiFi link test (bare Pico W is enough)
Flash `APP_MODE=TEST_TELEMETRY` with the WiFi flags (main README), start
mosquitto, then:
```
python wifi_test.py --mqtt <broker-ip> [--car car1] [--minutes 10]
```

| Check | Target |
|---|---|
| telemetry rate while connected | ≥ 4.5 Hz (5 Hz) |
| heartbeat | 1 Hz, none missed while connected |
| command round trip (`#<id> gains`) | median < 100 ms, no reply lost while connected |
| messages dropped by the board | 0 while connected |
| outages (stop mosquitto ~60 s, then the hotspot ~30 s) | every one recovered; recovery time listed |

Results: `logs/<date_time>/wifi_summary.txt`, `wifi_pings.csv`,
`wifi_outages.csv`. `python wifi_test.py --self-test` checks the script
itself with a simulated board; it also runs against `sim_car.py`.

## Encoder calibration (`ENCODER_SLOTS`)
Flash a USB-console build (TEST_MOTOR; don't press START), close the
dashboard and PuTTY, then:
```
python encoder_cal.py --turns 10
```
Press Enter, turn the wheel exactly 10 full turns forward by hand, press
Enter. It prints `ENCODER_SLOTS` (ticks per turn / 2: the firmware counts
both edges) and whether `ENCODER_GLITCH_US` is short enough at
`--max-speed`. The dashboard's `ENC` line shows the same raw counts.

## Try it without the car
```
mosquitto -v                                   # terminal 1
python sim_car.py --mqtt localhost --autostart # terminal 2
python picocar_dashboard.py --mqtt localhost   # terminal 3
```
`sim_car.py` publishes the same messages as the firmware (barcode every
15 s, 35 mm hump every 20 s, obstacle every 30 s) and obeys the same
commands; its wheels use the firmware's PID law, so tuning changes are
visible.
