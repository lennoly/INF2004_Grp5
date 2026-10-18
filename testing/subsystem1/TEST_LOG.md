# Subsystem 1 — test log

**Subsystem:** WiFi, MQTT, telemetry, commands  ·  **Student ID:**  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | MQTT client logic: connect windows 2/4/8 s, reconnect, rings drop and count, command parser | FR9–FR11, NFR8 | host | `make -C tests/host` | 0 failed |
| S1-U02 | telemetry, events and 1 Hz heartbeat over USB (no WiFi) | FR9 | board | `APP_MODE=TEST_TELEMETRY`; dashboard `--serial auto` | JSON telemetry + heartbeat every 1 s |
| S1-U03 | WiFi link steady for 10 min | FR9, A1 | link | WiFi `TEST_TELEMETRY` build on a bare Pico W; `wifi_test.py --minutes 10` | telemetry ≥ 4.5 Hz, no heartbeat missed, median round trip < 100 ms, drop 0 |
| S1-U04 | broker outage recovered | FR11 | link | during S1-U03 stop mosquitto ~60 s, restart | `recon` +1, outage listed with its recovery time |
| S1-U05 | WiFi outage recovered (re-join) | FR11 | link | during S1-U03 switch the hotspot off ~30 s | `rejoin` rises, board recovers without reset |
| S1-U06 | link is WiFi, not the cable | FR9 | link | power the board from a phone charger; `wifi_test.py --minutes 2` | PASS with no cable to the laptop |
| S1-U07 | live command from the dashboard | FR10 | manual | dashboard `--mqtt`; send `speed=200` | car acknowledges; gains report shows 200 |
| S1-U08 | tuning refused while driving (CSC05) | FR10 | manual | MISSION: press START, send `pid=0.1,0.6,0`; then STOP and send it again | first reply `"ok":0` "only while IDLE or STOPPED"; after STOP `"ok":1` |
| S1-U09 | Wi-Fi re-join does not stall the car (CSC05) | FR11, A2 | link | MISSION + WiFi, driving: hotspot off ~30 s, then send `timing` | `rejoin` max_us small against the 20 ms period; `jit_max_us` still within NFR1 |

*Method:* **host** = `make -C tests/host`; **board** = `APP_MODE=TEST_xxx`
build on the car; **link** = `wifi_test.py`; **manual** = a person measures.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |
