#!/usr/bin/env python3
"""
Measure ENCODER_SLOTS (car_config.h) by turning the wheels by hand.

  python encoder_cal.py [--port auto|COM4] [--turns 10]

1. Flash a USB-console build that starts the encoders: TEST_MOTOR is
   best.  Do not press START.  Close the dashboard and PuTTY (one program
   per serial port).
2. Run this script.  It asks the car for telemetry every 100 ms.
3. Put a tape mark on the tyre, press Enter, turn the wheel slowly forward
   exactly --turns full turns, press Enter.  Turn one wheel or both.

It prints the ticks per wheel turn and the ENCODER_SLOTS value: the
firmware counts both edges of each pulse, so slots = ticks per turn / 2.
Telemetry carries only the low 16 bits of each count, so the script adds
up the differences between frames.  The encoder has one channel, so
turning backwards adds ticks too: turn forward only.
"""

import argparse
import math
import sys
import threading
import time

import frames

RASPBERRY_PI_USB_VID = 0x2E8A
BAUD = 115200
RATE_MS = 100
ENC_WRAP = 1 << 16
SERIAL_CHUNK = 16          # the car's console reads small chunks
SERIAL_CHUNK_GAP_S = 0.03
FIRST_FRAME_S = 5.0


class Counter:
    """Unwraps the 16-bit tick counts carried by each telemetry frame."""

    def __init__(self):
        self.lock = threading.Lock()
        self.total = [0, 0]
        self.last = None
        self.frames = 0

    def add(self, enc):
        with self.lock:
            if self.last is not None:
                for i in (0, 1):
                    self.total[i] += (enc[i] - self.last[i]) % ENC_WRAP
            self.last = list(enc)
            self.frames += 1

    def snapshot(self):
        with self.lock:
            return list(self.total), self.frames


def find_port():
    """First USB serial port with the Raspberry Pi vendor id."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == RASPBERRY_PI_USB_VID:
            return p.device
    return None


def send(ser, text):
    data = (text.strip() + "\n").encode("ascii")
    for i in range(0, len(data), SERIAL_CHUNK):
        ser.write(data[i:i + SERIAL_CHUNK])
        ser.flush()
        time.sleep(SERIAL_CHUNK_GAP_S)


def feed_line(line, counter):
    """One console line: telemetry frames update the counter."""
    if not line.startswith("@"):
        return
    try:
        kind, obj = frames.decode_console_line(line)
    except ValueError:
        return
    if kind == "telemetry":
        counter.add(obj["enc"])


def reader(ser, counter, stop):
    buf = b""
    while not stop.is_set():
        try:
            buf += ser.read(256)
        except Exception:
            return
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            feed_line(raw.decode("ascii", errors="replace").strip(), counter)


def live(counter, start, stop):
    while not stop.is_set():
        total, _ = counter.snapshot()
        print("\r  ticks so far: left %6d   right %6d   "
              % (total[0] - start[0], total[1] - start[1]), end="", flush=True)
        time.sleep(0.2)


def report(name, ticks, args):
    if ticks == 0:
        print("  %-5s wheel: not turned" % name)
        return
    per_turn = ticks / args.turns
    slots = per_turn / 2.0
    print("  %-5s wheel: %d ticks / %g turns = %.1f ticks per turn"
          "  ->  ENCODER_SLOTS = %d" % (name, ticks, args.turns, per_turn,
                                        round(slots)))
    # The boot check in encoder.c: the glitch filter must stay below the
    # shortest half-period at the fastest wheel speed.
    half_us = (math.pi * args.wheel_mm / slots) * 1e6 / (2 * args.max_speed)
    note = ("OK" if args.glitch_us < half_us else
            "TOO LONG: lower ENCODER_GLITCH_US below %d" % half_us)
    print("        half-period at %g mm/s = %d us, glitch filter %d us: %s"
          % (args.max_speed, half_us, args.glitch_us, note))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", default="auto",
                    help="serial port, or 'auto' to find the Pico")
    ap.add_argument("--turns", type=float, default=10.0,
                    help="full wheel turns you will make (default 10)")
    ap.add_argument("--wheel-mm", type=float, default=65.0,
                    help="WHEEL_DIAMETER_MM, for the glitch check")
    ap.add_argument("--max-speed", type=float, default=600.0,
                    help="SPEED_WHEEL_MAX_MM_S, for the glitch check")
    ap.add_argument("--glitch-us", type=float, default=300.0,
                    help="ENCODER_GLITCH_US, for the glitch check")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("needs pyserial:  pip install pyserial")
    port = find_port() if args.port == "auto" else args.port
    if port is None:
        sys.exit("no Pico found: plug it in (USB-console build) or use --port")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as err:
        sys.exit("cannot open %s (%s): close the dashboard / PuTTY" % (port, err))

    counter = Counter()
    stop = threading.Event()
    threading.Thread(target=reader, args=(ser, counter, stop),
                     daemon=True).start()
    send(ser, "rate=%d" % RATE_MS)

    deadline = time.time() + FIRST_FRAME_S
    while counter.snapshot()[1] < 2 and time.time() < deadline:
        time.sleep(0.1)
    if counter.snapshot()[1] < 2:
        stop.set()
        sys.exit("no telemetry from %s: flash a USB-console build "
                 "(TEST_MOTOR) and try again" % port)

    print("Connected to %s, telemetry every %d ms." % (port, RATE_MS))
    input("Mark the tyre, then press Enter to start counting... ")
    start, _ = counter.snapshot()
    show = threading.Event()
    threading.Thread(target=live, args=(counter, start, show),
                     daemon=True).start()
    input("Turn the wheel exactly %g full turns forward, then press Enter.\n"
          % args.turns)
    show.set()
    time.sleep(2 * RATE_MS / 1000.0)        # let the last frame arrive
    end, _ = counter.snapshot()
    print("\n")

    report("left", end[0] - start[0], args)
    report("right", end[1] - start[1], args)
    print("\nPut the value in app_program/car_config.h (ENCODER_SLOTS) and "
          "rebuild.")

    send(ser, "rate=0")
    stop.set()
    ser.close()


if __name__ == "__main__":
    main()
