# Buddy 3 moving-car test

Flash **picocar_TEST_BUDDY3_uart.uf2** for combined black-line following and
barcode navigation. The older TEST_IR and TEST_BARCODE files are stationary
bench tests, not driving firmware.

1. Hold BOOTSEL while connecting USB, then copy the UF2 to RPI-RP2.
2. Power the car and its motors normally. USB and a PC are not required.
3. Position the left/right IR sensors over the edges of the black line on a
   light surface, pointing along the track. The current design assumes an
   approximately 18 mm line with the two sensor spots about 18 mm apart.
4. Press GP20 to start. Press GP21 to stop; press GP20 again to restart.
   GP21 while idle also means STOP in this firmware, not a calibration spin.

The car uses Buddy 2 motor/encoder control and Buddy 3 IR/barcode logic.
The HC-SR04, servo, IMU and WiFi are inactive. There is no obstacle avoidance.
GP12 is held low rather than generating servo pulses.

Code 39 markers: `*A*` = left, `*B*` = right, `*C*` = straight,
`*D*` = U-turn. A/B/C wait for a detected junction, with the existing 400 mm
fallback if no junction is detected. D acts immediately. Leave white gaps
between barcode passes. With no barcode the car keeps following the line;
at a cross it generally continues straight. A lost line triggers a search,
then a stop if it cannot be found.

This build uses the configured IR defaults (white 400, black 3000 ADC counts,
black-high polarity), cruise speed 180 mm/s and existing motor/encoder tuning.
It does not perform automatic calibration. Correct sensor height, polarity,
motor direction and encoder wiring are required. Use the stationary IR test
to measure readings if the car does not track correctly, then update
`app_program/car_config.h` and rebuild. Physical track performance has not
been verified on this car.

The UART console is optional (GP0/GP1, 115200 baud); there is no USB COM port
in this build. To rebuild in WSL with git, make, gcc and arm-none-eabi-gcc:

```sh
sh tools/build_buddy3.sh
sh testing/subsystem3/run_driving_tests.sh
python3 testing/subsystem3/run_host_tests.py
```

The build script caches the pinned course template and Pico SDK in
`~/.cache/picocar-buddy3` and copies the resulting UF2 into this folder.
