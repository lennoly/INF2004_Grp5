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

## First-time setup
You need git, GNU make, a C compiler (to build the `elf2uf2` tool) and the
Arm GCC toolchain (`arm-none-eabi-gcc`).

### macOS
```
xcode-select --install                       # git + C compiler
brew install make                            # GNU make: use `gmake`, not macOS's make 3.81
brew install --cask gcc-arm-embedded         # arm-none-eabi-gcc
git clone <repo URL> && cd INF2004_Grp5
sh setup.sh
```

### Windows (use WSL)
`setup.sh` creates a symbolic link, which plain Windows and Git Bash do not
handle, so build inside WSL (Ubuntu):
1. In PowerShell (as administrator): `wsl --install`, restart, then open
   **Ubuntu** from the Start menu and create a user.
2. In Ubuntu:
   ```
   sudo apt update
   sudo apt install git make build-essential gcc-arm-none-eabi python3
   cd ~                                     # clone into the Linux home, NOT /mnt/c
   git clone <repo URL> && cd INF2004_Grp5
   sh setup.sh
   ```
3. In WSL use `make` wherever this README says `gmake`.
4. To flash, open the build folder in Windows Explorer with
   `explorer.exe template/build_make` and drag the `.uf2` onto the Pico drive.
   Run the dashboard and mosquitto with Windows' own Python and mosquitto
   (WSL cannot see the USB serial port by default); the repo is reachable from
   Windows at `\\wsl$\Ubuntu\home\<user>\INF2004_Grp5`.

### What `setup.sh` does
Run it once after cloning (it skips anything already present). It downloads
the course template (at the version our patch was made against) into
`template/` and Pico SDK **2.2.0** into `sdk/`. It applies our template edits
and links `template/app_program` to our `app_program/`. Both folders are
git-ignored, so edit code in `app_program/` only. Set `PICO_SDK_PATH` first to
use an SDK you already have.

## Build and flash
1. `gmake -C template/build_make -j8 APP_MODE=TEST_MOTOR CONSOLE=usb_cdc`
   (modes: MISSION, TEST_MOTOR, TEST_MOTION, TEST_IR, TEST_BARCODE,
   TEST_IMU, TEST_ULTRASONIC, TEST_TELEMETRY).
2. Hold BOOTSEL, plug USB, copy `template/build_make/*.uf2` (or one from
   `firmware/`). Open the USB serial port.
3. Press **GP20 (START)** to run a test / the mission. **GP21 (STOP)**
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
The WiFi settings are compiled into the firmware and live only on your own
machine (`template/` is git-ignored), so never commit them. The prebuilt
firmware in `firmware/` has no WiFi for the same reason.

**1. Create your two config files** (after `setup.sh`):
```
cd template/config
cp wifi_credentials.example.h wifi_credentials.h
cp mqtt_config.example.h mqtt_config.h
cd ../..
```
- `wifi_credentials.h`: set `UTK_WIFI_SSID` and `UTK_WIFI_PASSWORD`, then
  `UTK_WIFI_CREDENTIALS_SET` to `1`. The Pico W needs **2.4 GHz WPA2**
  WiFi (a phone hotspot works); school logins such as eduroam do not.
- `mqtt_config.h`: set `MQTT_BROKER_IP` to the IP of the laptop running
  mosquitto, on that same WiFi, then `MQTT_CONFIG_SET` to `(1)`.
  Find the IP with `ipconfig getifaddr en0` (macOS) or `ipconfig` in
  PowerShell (Windows: the "IPv4 Address" of the WiFi adapter).
  Leave the port (1883) and the rest as they are.

Until both `SET` values are 1 the build stops with an error naming the file.

**2. Start the broker** on that laptop. Create `mosquitto.conf`:
```
listener 1883
allow_anonymous true
```
then run `mosquitto -c mosquitto.conf -v`
(install: `brew install mosquitto` on macOS; the installer from
mosquitto.org on Windows, and allow it through Windows Firewall).

**3. Build, flash and watch:**
```
gmake -C template/build_make -j8 APP_MODE=TEST_TELEMETRY CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
# flash template/build_make/mtk3pico_smp0_usb_cdc_wifi_dns.uf2
python tools/dashboard/picocar_dashboard.py --mqtt <broker IP>
# or: mosquitto_pub -h <broker IP> -t picocar/car1/cmd -m start
```
Use `APP_MODE=MISSION` for the full mission with WiFi. If the car does not
connect, the USB serial console shows the boot `[init]` lines and the link
state.

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
