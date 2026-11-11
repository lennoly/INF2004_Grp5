# PicoCar (INF2004 Group 5)

An autonomous line-following car on a Raspberry Pi Pico W and a Cytron Robo
Pico. It runs µT-Kernel 3.0, decodes Code 39 barcodes into turns, measures
humps with an IMU, scans and bypasses obstacles with a servo-mounted sonar,
and reports telemetry over WiFi/MQTT.

- Report: [`docs/PicoCar_Project_Report.docx`](docs/PicoCar_Project_Report.docx) (source text: `docs/REPORT.md`)
- Who owns what: [`TEAM.md`](TEAM.md)
- Tests and results: [`testing/`](testing/)
- Videos: [`videos/README.md`](videos/README.md)

## Layout

| Path | What |
|---|---|
| `app_program/` | the firmware; `car_config.h` holds every pin and tunable |
| `app_program/subsystem1..5/` | one folder per buddy (WiFi/MQTT, motion, IR/barcode, IMU, sonar/obstacle) |
| `tests/host/` | PC unit tests and a simulation of the vehicle task |
| `tools/dashboard/` | laptop dashboard, car simulator, WiFi link test |
| `docs/` | report, wiring, data formats, template patch |
| `setup.sh` | fetches the course template and Pico SDK, applies our patch |

## Setup (once)

You need git, GNU make, a C compiler and `arm-none-eabi-gcc`.

- **macOS:** `xcode-select --install`, `brew install make`, `brew install --cask gcc-arm-embedded`. Use `gmake`.
- **Windows:** use WSL (Ubuntu): `sudo apt install git make build-essential gcc-arm-none-eabi python3`. Clone into the Linux home, not `/mnt/c`. Use `make` wherever this says `gmake`.

```bash
git clone <repo URL> && cd INF2004_Grp5
sh setup.sh
```

`setup.sh` downloads the course template (`sirfonzie/mtk3smp-rp2040`) into
`template/` and Pico SDK 2.2.0 into `sdk/`, applies
`docs/template_changes.patch` and links `template/app_program` to our
`app_program/`. Both folders are git-ignored, so edit code in `app_program/` only.

## Build

Firmware is built locally and never committed (`*.uf2` is git-ignored).

```bash
gmake -C template/build_make -j8 APP_MODE=MISSION CONSOLE=usb_cdc
```

`APP_MODE` is one of `MISSION` (the full car) or a single-subsystem test:
`TEST_MOTOR`, `TEST_MOTION`, `TEST_WHEELS`, `TEST_IR`, `TEST_BARCODE`,
`TEST_IMU`, `TEST_ULTRASONIC`, `TEST_OBSTACLE`, `TEST_TELEMETRY`.
The output is `template/build_make/*.uf2`.

### WiFi build

1. Create the two config files (they hold your WiFi password; never commit them or a WiFi `.uf2`):
   ```bash
   cd template/config
   cp wifi_credentials.example.h wifi_credentials.h
   cp mqtt_config.example.h mqtt_config.h
   ```
   In `wifi_credentials.h` set the SSID and password (2.4 GHz WPA2; a phone
   hotspot works) and `UTK_WIFI_CREDENTIALS_SET` to 1. In `mqtt_config.h` set
   `MQTT_BROKER_IP` to the laptop running mosquitto and `MQTT_CONFIG_SET` to 1.
2. Build with the UART console (with the USB console the board only joins
   WiFi while a computer is plugged in):
   ```bash
   gmake -C template/build_make -j8 APP_MODE=MISSION CONSOLE=uart WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
   ```
3. Run the broker on the laptop with a `mosquitto.conf` of `listener 1883`
   and `allow_anonymous true`: `mosquitto -c mosquitto.conf -v`. On Windows,
   allow TCP port 1883 inbound in the firewall.

## Flash and run

Hold BOOTSEL, plug in USB and copy the `.uf2` to the RPI-RP2 drive.
**GP20 (START)**: a short press starts a run; held 2 s while idle it runs the
calibration spin. **GP21 (STOP)** stops the car.

## Dashboard

```bash
pip install -r tools/dashboard/requirements.txt
python tools/dashboard/picocar_dashboard.py --serial auto     # USB build
python tools/dashboard/picocar_dashboard.py --mqtt <broker IP> # WiFi build
```

Live tuning (accepted while IDLE or STOPPED): `pid=kp,ki,kd`, `ff=kf,offset`,
`line=kp,ki,kd`, `speed=`, `map=<1..4>,<letter idx>`, `rate=`, `gains`, `timing`, `help`.
Gains reset on reboot; copy final values into `car_config.h`. Every session
is logged to `logs/<date_time>/`. More: [`tools/dashboard/README.md`](tools/dashboard/README.md).

## Tests and coding standard (BARR-C:2018)

```bash
make -C tests/host                        # 313 unit and simulation tests
sh tools/barr_format.sh --check           # layout
python3 tools/barr_check.py app_program/*.[ch] app_program/*/*.[ch] tests/host/*.[ch]   # expect TOTAL 0
```
