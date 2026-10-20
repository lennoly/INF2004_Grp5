# PicoCar — Autonomous Robotic Car (µT-Kernel 3.0, Pico W + Robo Pico)

Built on the course template `sirfonzie/mtk3smp-rp2040`. This repository
holds only our side: `setup.sh` fetches the template (and the Pico SDK) and
applies our six template edits (`docs/template_changes.patch`). Full design,
MQTT documentation, tuning/evaluation procedures and wiring:
`docs/REPORT.md` (also `PicoCar_Project_Report.docx`). Wiring diagram:
`docs/img/wiring.png`.

## Layout
Follows the course's
[INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)
convention: one folder per student under `app_program/`, `TEAM.md`,
`testing/` and `videos/`.

| Path | Content |
|---|---|
| `TEAM.md` | **who owns what**: subsystems, pins, peripherals, task priorities, interfaces, kernel objects |
| `app_program/car_config.h` | **all pins and tunables** (edit [CALIBRATE] values) |
| `app_program/subsystem1/` | Buddy 1: WiFi / MQTT (`mqtt_*`), telemetry, commands (`command`, `console_in`) |
| `app_program/subsystem2/` | Buddy 2: `motor`, `encoder`, `pid`, `motion` |
| `app_program/subsystem3/` | Buddy 3: `ir_sensor`, `line_follow`, `barcode*` |
| `app_program/subsystem4/` | Buddy 4: `imu`, `terrain` |
| `app_program/subsystem5/` | Buddy 5: `servo`, `ultrasonic`, `obstacle`, `avoidance` |
| `app_program/vehicle` | integration state machine (shared) |
| `app_program/hal*` | Pico C SDK HAL + µT-Kernel interrupt binding (shared) |
| `app_program/timing` | task times, control jitter and CPU load: send `timing` (shared) |
| `app_program/app_main, app_tests` | entry point + test runner (shared); each buddy's test program is `subsystemN/subsystemN_test.c` |
| `app_program/demo_tasks.*, usb_console_compat.h` | from the template, unchanged (used by `app_main.c`) |
| `testing/` | test cases and results per subsystem, integration and system (`testing/README.md`) |
| `videos/` | index of unlisted YouTube videos of tests and demos |
| `tools/dashboard` | **laptop dashboard** (plots, tuning, CSV logs) + car simulator |
| `tests/host` | PC unit tests (`make -C tests/host`) — 187/187 pass |
| `firmware/` | prebuilt `.uf2` (USB console) for every mode |
| `setup.sh` | fetches `template/` (course template + our patch, with `app_program/` linked in) and `sdk/pico-sdk` |
| `docs/template_changes.patch` | the 6 edits made to template files (applied by `setup.sh`) |
| `tools/barr_*`, `app_program/.clang-format` | BARR-C:2018 formatter and checker (see below) |
| `docs/ABBREVIATIONS.md`, `docs/templates/` | BARR-C abbreviations table and module templates |
| `docs/BUDDY2_DATA_SHEET.md` | bench sheet for the PID tuning and motion accuracy measurements |

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
`template/` and Pico SDK **2.2.0** into `sdk/`. It applies our template edits,
removes the template's ports for other boards (only the Pico W / RP2040
port is kept) and links `template/app_program` to our `app_program/`. Both folders are
git-ignored, so edit code in `app_program/` only. Set `PICO_SDK_PATH` first to
use an SDK you already have.

**Already have a `template/` from before the subsystem folders?** Its build
rule only looks in `app_program/` itself, so the build stops with
`motion.h: No such file or directory`. Refresh that one file once:
```
git -C template checkout -- build_make/mtkernel_3/app_program/subdir.mk
git -C template apply --include=build_make/mtkernel_3/app_program/subdir.mk ../docs/template_changes.patch
```

## Build and flash
1. `gmake -C template/build_make -j8 APP_MODE=TEST_MOTOR CONSOLE=usb_cdc`
   (modes: MISSION, TEST_MOTOR, TEST_MOTION, TEST_IR, TEST_BARCODE,
   TEST_IMU, TEST_ULTRASONIC, TEST_TELEMETRY).
2. Hold BOOTSEL, plug USB, copy `template/build_make/*.uf2` (or one from
   `firmware/`). Open the USB serial port.
3. Press **GP20 (START)** to run a test / the mission. **GP21 (STOP)**
   stops; STOP while idle = 360° calibration spin (IR + magnetometer).

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
  Leave the port (1883, the same as `listener` in `mosquitto.conf`) and the
  rest as they are.

Until both `SET` values are 1 the build stops with an error naming the file.

**2. Start the broker** on that laptop. Create `mosquitto.conf`:
```
listener 1883
allow_anonymous true
```
then run `mosquitto -c mosquitto.conf -v`
(install: `brew install mosquitto` on macOS; the installer from
mosquitto.org on Windows). On Windows, stop the installer's background
service first (`net stop mosquitto`) and let port 1883 in, in an
administrator PowerShell:
```
New-NetFirewallRule -DisplayName "MQTT 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
```

**3. Build, flash and watch:**
```
gmake -C template/build_make -j8 APP_MODE=TEST_TELEMETRY CONSOLE=uart WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
# flash template/build_make/mtk3pico_smp0_uart_wifi_dns.uf2
python tools/dashboard/picocar_dashboard.py --mqtt <broker IP>
# commands on MQTT are binary frames (docs/DATA_REPRESENTATION.md); "gains" is:
# printf '\x1c\x03\x00\x00' | mosquitto_pub -h <broker IP> -t picocar/car1/cmd -s
```
`TEST_TELEMETRY` runs only WiFi, MQTT and telemetry (no robot hardware), so
**a bare Pico W is enough**. Sensor values in the telemetry read 0 in this
build.

**WiFi builds use `CONSOLE=uart` (no USB console).** With the USB console,
the board joined the WiFi only while a computer was on the USB port: on a
power bank or phone charger it never connected. Without it, it connects on
any supply. So watch a WiFi build over MQTT (dashboard, `mosquitto_sub`, or
the mosquitto window), not over the USB cable; the console is on GP0/GP1
(UART0, 115200 baud) if you need it.
Use `APP_MODE=MISSION` for the full mission with WiFi on the car.

| Heartbeat field | Meaning |
|---|---|
| `link` | `WAIT_NET` (no WiFi yet), `CONNECTING`, `CONNECTED`, `BACKOFF` (refused, retrying) |
| `wifi` | WiFi state in words: `joined`, `joining`, `hotspot not found ...`, `wrong WiFi password`, `joined, waiting for an IP address` |
| `ip` | the board's own address |
| `tries`, `recon`, `rejoin` | broker attempts, reconnects after a loss, WiFi re-joins |
| `wreset` | WiFi chip restarts (no link for 10 s while the car is not driving) |
| `pub`, `drop` | messages sent, and dropped while offline |
| `why` | how the last broker attempt ended, in words (below) |
| `faults`, `fcount` | active fault bits (motor stall, encoder, IMU, no echo, Wi-Fi loss) and how often each was raised |

The heartbeat is a 26-byte binary frame (Change Requirement 1);
`tools/dashboard/frames.py` decodes it into these names.

If `link` never reaches `CONNECTED`, `why` says where to look:

| `why` | Fix |
|---|---|
| `no answer from broker (firewall? address?)` | firewall rule above; `MQTT_BROKER_IP` must be the laptop's current IP |
| `closed by broker or network (TCP reset)` | nothing accepted the connection: antivirus/VPN firewall, or wrong port |
| `refused: not authorised ...` | mosquitto started without `allow_anonymous true` (use `-c`) |
| `link` `WAIT_NET` | not on the WiFi yet: `wifi` says why. The board re-joins every 5 s and restarts the WiFi chip every 10 s until it is back |

An iPhone hotspot hides itself from new devices after a while: keep
Settings → Personal Hotspot open (with "Maximise Compatibility" on) while
the board powers up.

The mosquitto window shows `New client connected from <board ip>` when the
board gets through.

**4. Test the link** (with the board from step 3):
```
python tools/dashboard/wifi_test.py --mqtt <broker IP> --minutes 10
```
It times command round trips (`#<id> gains`; the reply repeats the id),
counts telemetry and heartbeats, and lists every outage with its recovery
time, then prints PASS / FAIL and saves `logs/<date_time>/wifi_*`. During the
run, stop mosquitto for about 60 s (the board reconnects: `recon` goes up),
then switch the hotspot off for about 30 s (the board re-joins the WiFi
3 s after losing it, then every 5 s, and restarts the WiFi chip if it is
still off after 10 s: `rejoin` and `wreset` go up).

The USB cable carries only power and the console, never MQTT: everything
`wifi_test.py` measures went over WiFi. To show it, power the board from a
phone charger instead of the laptop and run the test again.

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
python3 tools/barr_check.py app_program/*.[ch] app_program/*/*.[ch] tests/host/*.[ch]   # expect TOTAL 0
make -C tests/host                                        # unit tests, -Werror
```
New modules start from `docs/templates/module.c` and `module.h`.
