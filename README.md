# PicoCar — Autonomous Robotic Car (µT-Kernel 3.0, Pico W + Robo Pico)

Built on the course template `sirfonzie/mtk3smp-rp2040`. This repository
holds only our side: `setup.sh` fetches the template (and the Pico SDK) and
applies our six template edits (`docs/template_changes.patch`). Full design,
MQTT documentation, tuning/evaluation procedures and wiring:
`docs/REPORT.md` (also `PicoCar_Project_Report.docx`). Wiring diagram:
`docs/img/wiring.png`.

## Layout
| Path | Content |
|---|---|
| `app_program/car_config.h` | **all pins and tunables** (edit [CALIBRATE] values) |
| `app_program/motor, encoder, pid, motion` | Buddy 2 motion control |
| `app_program/ir_sensor, line_follow, barcode*` | Buddy 3 line + barcode |
| `app_program/imu, terrain` | Buddy 4 IMU / humps |
| `app_program/servo, ultrasonic, obstacle, avoidance` | Buddy 5 scanning / avoidance |
| `app_program/mqtt_*, telemetry` | Buddy 1 WiFi / MQTT / telemetry |
| `app_program/vehicle` | integration state machine |
| `app_program/hal*` | Pico C SDK HAL + µT-Kernel interrupt binding |
| `app_program/app_main, app_tests` | entry point + per-buddy test programs |
| `app_program/demo_tasks.*, usb_console_compat.h` | from the template, unchanged (used by `app_main.c`) |
| `app_program/command, console_in` | text commands over USB/UART/MQTT (live tuning) |
| `tools/dashboard` | **laptop dashboard** (plots, tuning, CSV logs) + car simulator |
| `tests/host` | PC unit tests (`make -C tests/host`) — 89/89 pass |
| `firmware/` | prebuilt `.uf2` (USB console) for every mode |
| `setup.sh` | fetches `template/` (course template + our patch, with `app_program/` linked in) and `sdk/pico-sdk` |
| `docs/template_changes.patch` | the 6 edits made to template files (applied by `setup.sh`) |
| `tools/barr_*`, `app_program/.clang-format` | BARR-C:2018 formatter and checker (see below) |
| `docs/ABBREVIATIONS.md`, `docs/templates/` | BARR-C abbreviations table and module templates |

## Quick start
1. `sh setup.sh` once (needs git; macOS, Linux or WSL). It fetches the
   template at the version our patch was made against, and Pico SDK
   **2.2.0**; set `PICO_SDK_PATH` first to use an SDK you already have.
2. `gmake -C template/build_make -j8 APP_MODE=TEST_MOTOR CONSOLE=usb_cdc`
   (modes: MISSION, TEST_MOTOR, TEST_MOTION, TEST_IR, TEST_BARCODE,
   TEST_IMU, TEST_ULTRASONIC, TEST_TELEMETRY). On Linux `make` works too;
   macOS's own `make` (3.81) does not, so install GNU make (`brew install make`).
3. Hold BOOTSEL, plug USB, copy `template/build_make/*.uf2` (or one from
   `firmware/`). Open the USB serial port.
4. Press **GP20 (START)** to run a test / the mission. **GP21 (STOP)**
   stops and never moves the car. In the mission build START acts on
   release: a short press starts a run; held for 2 s while idle, it runs
   the 360° calibration spin (IR + magnetometer), as `calibrate` does.

## Dashboard and live tuning (no reflashing)
```
pip install -r tools/dashboard/requirements.txt
python tools/dashboard/picocar_dashboard.py --serial auto      # USB cable, any build
```
Edit the tuning boxes and press Apply, or type commands on any serial
terminal: `pid=kp,ki,kd`, `ff=kf,offset`, `line=kp,ki,kd`, `speed=150`,
`rate=50` (20 Hz telemetry), `gains`, `start`, `stop`, `help`.
Every session is logged to `logs/<date_time>/*.csv` for the report.
Gains reset on reboot: copy the final ones into `car_config.h`.
Details: `tools/dashboard/README.md`.

## WiFi + MQTT
```
cd template/config
cp wifi_credentials.example.h wifi_credentials.h   # SSID/pass, SET=1
cp mqtt_config.example.h mqtt_config.h             # broker IP, SET=1
cd ../..
gmake -C template/build_make -j8 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
python tools/dashboard/picocar_dashboard.py --mqtt <broker>
# or: mosquitto_pub -h <broker> -t picocar/car1/cmd -m start
```
(The prebuilt firmware has no WiFi because credentials are yours to add.
They stay in `template/config/`, which git ignores.)

## Bring-up order
motor → motion → IR (calibrate) → barcode → IMU → ultrasonic → telemetry → mission.
Fix motor direction with `MOTOR_x_INVERT`, tune the speed loop live in
TEST_MOTOR (`ff=` then `pid=`), calibrate `WHEEL_DIAMETER_MM` and
`TRACK_WIDTH_MM` with TEST_MOTION, then tune `line=` in MISSION.

## Coding standard (BARR-C:2018)
All application and test code follows BARR-C:2018; report section 8 lists
how it is checked and the few deviations the kernel and SDK force.
Before committing:
```
sh tools/barr_format.sh                                   # layout (clang-format 18 + barr_layout.py)
python3 tools/barr_check.py app_program/*.[ch] tests/host/*.[ch]   # expect TOTAL 0
make -C tests/host                                        # unit tests, -Werror
```
New modules start from `docs/templates/module.c` and `module.h`.
