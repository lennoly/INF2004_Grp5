# Barcode demo setup

The decoder recognises all uppercase Code 39 letters A-Z in either scan
direction. Movement assignments are supplied by the PC at runtime. There
are no default letters assigned to movements in the firmware.

## Send the instructor's assignments

Build and flash the updated `MISSION` firmware with `WIFI_MQTT=1`, using your
WiFi/broker settings as described in the root README. Existing prebuilt UF2
files do not contain these changes. Connect the dashboard to the broker.

With the car IDLE or STOPPED, use the dashboard command box or publish a
plain text MQTT message to `picocar/<car-id>/cmd`:

```text
map=<forward>,<uturn>,<left>,<right>
```

Replace the placeholders with four distinct letters from A-Z. Example only:

```text
map=W,X,Y,Z
```

This means W = forward, X = U-turn, Y = left, Z = right. Letters can be
lowercase in the message; the car normalises them to uppercase. Spaces
around `=` and `,` are accepted. All four assignments are replaced together.
Duplicate, incomplete or invalid assignments are rejected without changing
the previous mapping. Updates while driving are rejected.

For a broker on the PC and car ID `car1`, an example terminal command is:

```sh
mosquitto_pub -h <broker-ip> -t picocar/car1/cmd -m "map=W,X,Y,Z"
```

Subscribe to `picocar/car1/event` to see the mapping report and the command
acknowledgement (`"ok":1` on success). Send `mapping` on the command topic to
request the current configuration:

```json
{"type":"mapping","configured":1,"forward":"W","uturn":"X","left":"Y","right":"Z"}
```

Before configuration, `mapping` reports `"configured":0`. Mission start,
including the physical START button, requires a configured mapping.
Assignments live in RAM; resend them after every reboot. The same commands
are available on the USB/UART console for bench testing.

## Robot behaviour

1. Scan one complete `*<letter>*` Code 39 barcode in either direction.
2. Decode and record the letter, and keep its assigned movement pending.
3. Continue following the line until a junction is detected.
4. Execute forward, U-turn, left or right at that junction.
5. Clear the pending movement and continue to the next barcode.

U-turns also wait for the junction. No distance timeout triggers an early
movement. A valid unassigned letter is still decoded/reported with movement
`NONE`; it does not create a new navigation command. If another assigned
barcode arrives before the junction, the latest assigned barcode wins.
New mission starts discard queued scans from before start.

## Verification completed on the PC

Run `python testing/subsystem3/run_host_tests.py` for the actual C decoder,
simulated sensor stream, both IR polarities and both bench modes. Tests cover
all A-Z letters forwards and backwards, noisy widths, mapping replacement
and validation, plus the actual vehicle junction decision. The runner also
checks the mission vehicle and telemetry source with warnings as errors.
The general `tests/host` suite covers command parsing and the MQTT transport.

Physical IR calibration, printed barcode sizes, cross-junction detection and
motor movement accuracy still need robot testing. No robot results are
claimed by the host tests.
