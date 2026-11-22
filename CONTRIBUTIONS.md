# Contributions

Who did what on the PicoCar, taken from the git history. Subsystem
ownership is in [`TEAM.md`](TEAM.md).

**Testing:** Kent ran and recorded the board, integration and system
tests, including the ones under the other subsystems. Every test log in
`testing/*/logs/` and every result row was committed by Kent, apart from
Raees's sonar bench runs of 18 and 22 Oct (three rows in the S5 test log,
tester Raees).

## Members

| # | Member | Student ID | GitHub | Subsystem |
|---|---|---|---|---|
| 1 | Kent | 2500343 | kentlowj | WiFi, MQTT, telemetry, commands |
| 2 | Levia | 2503109 | whatiamfeelingrn | Motors, encoders, wheel PID, moves |
| 3 | Max | 2501660 | symhsaw | IR sensors, line following, barcode |
| 4 | Lennon | 2503438 | lennoly | IMU, terrain, humps |
| 5 | Raees | 2501955 | iraeeeees | Ultrasonic, servo, obstacle profiling and bypass |

## By member

### Kent (Buddy 1, integration)

- **Subsystem 1:** MQTT client on lwIP (`mqtt_lwip`, `mqtt_bridge`), telemetry and heartbeat, command parser (`command`, `console_in`), binary frames (`frames`, Change Requirement 1), WiFi re-join and broker-outage recovery.
- **Integration:** vehicle state machine (`vehicle.c`) with junction turns, line search, obstacle bypass and the lift / overturn stop; HAL and kernel glue; shared fault mask; timing and CPU-load measurement.
- **Cross-subsystem fixes and tuning:** wheel and track calibration, line-follow gains, barcode decoding of flipped and broken frames, sonar scan range and side choice, IMU one-wheel humps, the 1 µs timer tick fix.
- **Tools:** laptop dashboard and car simulator (`tools/dashboard/`), `wifi_test.py`, `setup.sh` and the template patch, BARR-C checker and formatter.
- **Tests:** ran and recorded the board tests of all five subsystems (S1 to S5), the integration tests (INT-01 to INT-11) and the system tests on the track (SYS-01 to SYS-05), with the logs and videos as evidence.
- **Docs:** README, TEAM.md, report (`docs/REPORT.md` and the Word report), wiring, data representation and fault-path documents.

### Levia (Buddy 2)

- **Subsystem 2:** motor driver with the PWM reset fix and the right-motor direction fix in the wiring; `TEST_WHEELS` pin check.

### Max (Buddy 3)

- **Subsystem 3:** first IR and barcode implementation (`ir_sensor`, `line_follow`, `barcode`, `barcode_decode`); no built-in barcode letters; dropped turns with no junction; junctions after a late barcode.
- **Final review (15 Nov):** turns only at the cross-junction, IMU Z-axis auto-detection, watchdog-reset detection, BARR-C clean-up of the firmware to 0 findings, report, diagrams and Word report brought in line with the code.

### Lennon (Buddy 4)

- **Subsystem 4:** IMU driver ported to the LSM303D with an I2C scan in `TEST_IMU`; velocity expiry, IMU fault recovery and I2C timeouts; repository set-up (initial commit, `.gitignore`).
- **Docs:** corrected the IMU part (LSM303D, not GY-511) and its settings (I2C address, register values, 0.061 mg/LSB, ±4 gauss magnetometer) in the wiring table, the report's IMU section, the abbreviations and the wiring diagram (16 Oct).

### Raees (Buddy 5)

- **Subsystem 5:** RP2040 watchdog fed by the vehicle task; obstacle-handling and STOP gaps from the Week 6 and Tribunal 1 lists; sonar FAULT class for echoes under 20 mm; gentle servo steps; obstacle triggers ignored on humps; SMP=1 build guard.
- **Tests:** host tests for the vehicle task and obstacle scanner (first 35 checks); sonar bench runs of 18 and 22 Oct (three S5 rows).
- **Tools:** `tools/scan_plot.py` for scan evidence.

## Shared

- Design reviews and tribunal documents were written by the whole team outside this repository.
