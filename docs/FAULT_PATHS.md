# Fault Paths

One table for every fault (CSC05 Week 6 feedback): how it is detected, how
long that takes, what the car does, how it recovers and the reason string
it reports. It is the checklist for the A14 fault-injection tests.

Faults that stop a run are bits of the shared fault mask (`fault.c`,
Change Requirement 1). Each bit has one owner task that sets and clears it.
The vehicle task reads the whole mask with one call in every wait loop
(`check_faults()`, every 20 ms) and stops the run if a stop fault is set.
Each owner also counts how often its fault was raised (8-bit, saturating)
and records when it last ran, so a fault shorter than 20 ms and a stalled
owner can still be seen (heartbeat `faults` and `fcount`).

State of the code: `main` after the Change Requirement 1 commit. Rows marked
*designed* are in the Week 6 design but not yet in the code; rows marked
*Raees branch* are implemented on `origin/Raees` and not merged yet.

| Fault | Detection (owner) | Detection latency | Response | Recovery | Reason (state event) |
|---|---|---|---|---|---|
| STOP button GP21 | polled by the vehicle task (falling edge) | ≤ 20 ms; **not seen during an obstacle scan** (up to 12 s) | brake, servo centred, STOPPED | short START | `stop command` |
| `stop` from console or MQTT | telemetry task, then the vehicle queue | ≤ 40 ms after it reaches the car (+ network for MQTT) | as above | as above | `stop command` |
| STOP latch on GP21 edge interrupt, scans check STOP | *Raees branch* | one ping (~70 ms) during scans | as above | accepted START only | `stop command` |
| Motor stall | motion task: move exceeds 3 × ideal time + 1 s (`FAULT_MOTOR_STALL`) | move-dependent, plus ≤ 20 ms | move fails, STOPPED | bit clears when the next move is accepted; START | `fault: motor stall` |
| Move never finishes | vehicle `run_move()`: 10 s (`MOVE_TIMEOUT_MS`) | 10 s | `motion_stop()`, the manoeuvre fails | next manoeuvre | depends on the manoeuvre |
| Encoder fault | motion task: a wheel driven at ≥ friction offset + 10 % with no edge for 500 ms (`FAULT_ENCODER`) | 500 ms + ≤ 20 ms | STOPPED | bit clears as soon as edges return; START | `fault: encoder` |
| IMU fault | imu task: 3 failed reads in a row (`FAULT_IMU`); `b_ok` false; freshness `seq` stops | 60 ms + ≤ 20 ms | STOPPED | bit clears on the next good read; START | `fault: IMU` |
| Sonar no echo | sonar monitor: 2 FAULT pings in a row (`FAULT_NO_ECHO`) | ~140 ms + ≤ 20 ms | STOPPED | bit clears on the next VALID or OUT_OF_RANGE ping; START | `fault: sonar no echo` |
| Sonar stale (no front reading for 300 ms) | *Raees branch* (the front word now carries the reading's age: `packed_front_age_ms()`) | 300 ms | STOPPED | START | `sonar stale` |
| Wi-Fi loss | cyw43 task: link down after it was up (`FAULT_WIFI_LOSS`) | one poll (2 ms) | report only: STOP button and console still work | re-join 3 s after the loss, then every 5 s; chip restart after 10 s (not while driving); bit clears when the link is back | – (heartbeat `faults`) |
| Line lost | line classifier: both sensors white for 150 ms | 150 ms | LINE_SEARCH: 5 alternating sweeps of growing length | line found → LINE_FOLLOW | `line lost`, then `line not found / end` |
| Path blocked | obstacle planner: no bypass after 2 attempts | after the scans | STOPPED | START | `path blocked` |
| Velocity request older than 100 ms | *designed* (FR14) | 100 ms | brake | new request | – |
| Watchdog (vehicle stalled) | *Raees branch* (250 ms) | 250 ms | RP2040 reset; the car starts braked | reboot, then START | – |

## Fault injection tests (A14)

For each row, while line following with telemetry streaming:

| Fault | How to inject | Pass |
|---|---|---|
| STOP | button, console `stop`, MQTT `stop`; 20 times each, also during a scan and a turn | STOPPED within 100 ms of the edge or arrival (NFR3) |
| Motor stall | hold one wheel during a distance move | STOPPED `fault: motor stall` |
| Encoder fault | unplug one encoder | STOPPED `fault: encoder` within 520 ms |
| IMU fault | unplug the IMU Grove cable | STOPPED `fault: IMU` within 80 ms; reconnecting clears the bit |
| Sonar no echo | unplug ECHO | STOPPED `fault: sonar no echo` within 160 ms |
| Wi-Fi loss | switch the hotspot off | car keeps driving; heartbeat shows `wifi_loss`, then recovers |
