# Buddy 3 moving-car test

**Rebuilt on 25 October 2026:** **picocar_TEST_BUDDY3_uart.uf2** includes runtime
barcode mapping and stores the pending decoded letter until its junction.
It also includes the five review fixes: latched queue-independent STOP,
consumer-owned barcode reset with stale-session rejection, stopping on failed
moves, fresh odometry baselines, and recovery after leading barcode noise.
Flash this file for combined black-line following and
barcode navigation. Only the Buddy 3 driving UF2 is kept here. TEST_IR and
TEST_BARCODE are stationary bench modes that can be rebuilt from source.

1. Hold BOOTSEL while connecting USB, then copy the UF2 to RPI-RP2.
2. Power the car and its motors normally. Connect the UART console on GP0/GP1
   at 115200 baud and send `map=<forward>,<uturn>,<left>,<right>` using the
   instructor-selected four distinct letters. Send `mapping` to verify. Resend
   after reboot. The PC can be disconnected after configuration.
3. Position the left/right IR sensors over the edges of the black line on a
   light surface, pointing along the track. The current design assumes an
   approximately 18 mm line with the two sensor spots about 18 mm apart.
4. Press GP20 to start. Press GP21 to stop; press GP20 again to restart.
   GP21 while idle also means STOP in this firmware, not a calibration spin.

The car uses Buddy 2 motor/encoder control and Buddy 3 IR/barcode logic.
The HC-SR04, servo, IMU and WiFi are inactive. There is no obstacle avoidance.
GP12 is held low rather than generating servo pulses.

Code 39 markers support all A-Z letters in either scan direction. The four
movement assignments are configured at runtime, with no fixed letter defaults.
Every movement, including U-turns, waits for a junction. There is no 400 mm
fallback. Leave white gaps between barcode passes. With no assigned barcode,
the car keeps following the line; a cross generally continues straight. A lost
line triggers a search, then a stop if it cannot be found.

This build uses the configured IR defaults (white 400, black 3000 ADC counts,
black-high polarity), cruise speed 180 mm/s and existing motor/encoder tuning.
It does not perform automatic calibration. Correct sensor height, polarity,
motor direction and encoder wiring are required. Use the stationary IR test
to measure readings if the car does not track correctly, then update
`app_program/car_config.h` and rebuild. Physical track performance has not
been verified on this car.

The UART console is required to configure the non-WiFi test build (GP0/GP1, 115200 baud); there is no USB COM port
in this build. To rebuild in WSL with git, make, gcc and arm-none-eabi-gcc:

```sh
sh tools/build_buddy3.sh
sh testing/subsystem3/run_driving_tests.sh
python3 testing/subsystem3/run_host_tests.py
```

The build script caches the pinned course template and Pico SDK in
`~/.cache/picocar-buddy3` and copies the resulting UF2 into this folder.

## MQTT version

The reference `max/briefing/barcode/barcode questions answer.png` requires
movement assignments to arrive via MQTT. The rebuilt UART file is suitable
for local testing but does not meet that network requirement by itself.
Wi-Fi SSID/password and broker IP have been left blank as requested.
The template rejects blank Wi-Fi settings, so no MQTT UF2 is supplied yet.

Fill in the ignored local files `config/wifi_credentials.h` and
`config/mqtt_config.h`, enable their `SET` values, then build:

```sh
sh tools/build_buddy3.sh "$HOME/.cache/picocar-buddy3" mqtt
```

This produces `firmware/picocar_TEST_BUDDY3_mqtt.uf2`. Send
`map=<forward>,<uturn>,<left>,<right>` to `picocar/car1/cmd` (or your configured
car ID), then `mapping` to verify. The MQTT build also streams barcode events,
telemetry and heartbeat. It runs the same IR/motor controller as the UART
build; sonar, servo and IMU remain inactive.
