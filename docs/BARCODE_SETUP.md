# Barcode demo setup

The decoder reads every Code 39 letter A-Z in either scan direction. No
letter is built into the firmware: the PC sends the instructor's four
letters before the run, and the car refuses START (reason
`barcode map not set`) until all four are set and are different.

## Send the instructor's letters

With the car IDLE or STOPPED (updates while driving are refused):

- **Dashboard:** type the four letters into the *Demo barcode letters*
  boxes (Left, Right, Straight, U-turn) and press **Set**.
- **Command box, MQTT or USB console:** send one `map=` per movement,
  `map=<movement>,<Code 39 index>`:

| Movement | `<movement>` |
|---|---|
| Left | 1 |
| Right | 2 |
| Straight | 3 |
| U-turn | 4 |

The index is the letter's position in `0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ`
(A = 10, B = 11, ... Z = 35). Example only, W left, X right, Y straight,
Z U-turn:

```text
map=1,32
map=2,33
map=3,34
map=4,35
```

For a broker on the PC and car ID `car1`:

```sh
mosquitto_pub -h <broker-ip> -t picocar/car1/cmd -m "map=1,32"
```

After each `map=` the car prints `# map L=W R=X S=Y U=Z` (`-` for a
movement with no letter yet); the dashboard shows it under *Letters L R S U*.
The letters live in RAM: send them again after every reboot.

## Robot behaviour

1. Scan one `*<letter>*` barcode in either direction.
2. Decode the letter and keep its movement pending.
3. Keep following the line until the barcode sensor passes the junction
   line (a 12-30 mm black stretch). A junction line passed in the 250 ms
   it takes to finish decoding the barcode also counts.
4. Make the movement at that junction (U-turns too).
5. If no junction comes within 400 mm, the movement is dropped and reported
   (`LEFT, no junction` etc.), and the car keeps following the line.

A letter with no movement is decoded and reported as `NONE`. Barcodes
decoded while the car stood still are discarded at START.

## Verification on the PC

`make -C tests/host` covers the decoder (A-Z, both directions, noise, broken
frames), the letter map (none at boot, START refused, duplicates) and the
vehicle's junction decisions in the simulation. IR levels, printed barcode
sizes, junction-line detection and turn accuracy still need the car.
