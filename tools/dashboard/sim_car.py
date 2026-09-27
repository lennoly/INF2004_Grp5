#!/usr/bin/env python3
"""
PicoCar simulator - publishes the same MQTT messages as the real firmware so
the dashboard, broker and tuning workflow can be tested without the car.

    python sim_car.py --mqtt localhost [--car car1]

It drives a simulated course: line following with wobble, a barcode every
15 s, a 35 mm hump every 20 s and an obstacle every 30 s.  The wheel model
uses the same PID law as motion.c, so pid= / ff= commands from the dashboard
visibly change the speed response.  Accepts the firmware's commands:
start, stop, calibrate, gains, help, speed=, pid=, ff=, line=, rate=.
"""

import argparse
import json
import math
import random
import sys
import time

DT = 0.02                                   # 50 Hz simulation step
MOTOR_GAIN = (5.0, 4.7)                     # mm/s per %duty, L / R
MOTOR_DEADBAND = 15.0                       # %duty needed to move
MOTOR_TAU = 0.15                            # s
HUMP_LEN = 250.0                            # mm
HUMP_HEIGHT = 35.0                          # mm


class Pid:
    """Same law as app_program/pid.c (derivative on measurement)."""

    def __init__(self):
        self.integ = 0.0
        self.prev = None

    def update(self, g, sp, meas):
        err = sp - meas
        der = 0.0 if self.prev is None else (meas - self.prev) / DT
        self.prev = meas
        out = g["kf"] * sp + g["kp"] * err + self.integ - g["kd"] * der
        sat = min(max(out, 0.0), 100.0)
        if not ((out > 100 and err > 0) or (out < 0 and err < 0)):
            self.integ = min(max(self.integ + g["ki"] * err * DT, 0), 100)
        return sat


class Car:
    def __init__(self, publish):
        self.pub = publish
        self.g = {"kp": 0.08, "ki": 0.6, "kd": 0.0, "kf": 0.10, "off": 18.0}
        self.line = [160.0, 0.0, 12.0]
        self.cruise = 180
        self.rate_ms = 200
        self.state, self.state_t = "IDLE", time.time()
        self.v = [0.0, 0.0]
        self.pwm = [0.0, 0.0]
        self.pid = [Pid(), Pid()]
        self.odo = 0.0
        self.enc = [0, 0]
        self.enc_f = [0.0, 0.0]
        self.run_t0 = time.time()
        self.hump = {"h": 0, "pk": 0, "max": 0, "n": 0}
        self.bc, self.nav, self.nbc, self.nobs = "", "NONE", 0, 0
        self.act, self.front = "CONTINUE", -1
        self.pitch = 0.0
        self.seq = 0
        self.tgt = [0.0, 0.0]
        self.le = 0.0

    # ---- helpers ---------------------------------------------------------
    def event(self, obj):
        self.pub("event", json.dumps(obj, separators=(",", ":")))

    def set_state(self, st, why):
        self.state, self.state_t = st, time.time()
        self.event({"type": "state", "st": st, "why": why})

    def gains_event(self):
        f = lambda x: round(x, 3)                           # noqa: E731
        self.event({"type": "gains",
                    "pid": [f(self.g["kp"]), f(self.g["ki"]), f(self.g["kd"])],
                    "ff": [f(self.g["kf"]), f(self.g["off"])],
                    "line": [f(x) for x in self.line],
                    "speed": self.cruise, "rate": self.rate_ms})

    # ---- commands (same grammar and limits as command.c) -----------------
    def command(self, text):
        text = text.strip()
        name, _, rest = text.partition("=")
        name = name.strip().lower()
        spec = {"start": 0, "stop": 0, "calibrate": 0, "gains": 0, "help": 0,
                "speed": 1, "pid": 3, "ff": 2, "line": 3, "rate": 1}
        err = None
        try:
            vals = [float(x) for x in rest.split(",")] if rest.strip() else []
        except ValueError:
            vals, err = [], "bad number"
        if err is None and name not in spec:
            err = "unknown command (try help)"
        elif err is None and len(vals) != spec[name]:
            err = "wrong number of values"
        elif name == "rate" and vals[0] != 0 and not 50 <= vals[0] <= 5000:
            err = "rate: 0 (off) or 50..5000 ms"
        elif name in ("pid", "line") and any(v < 0 or v > 1000 for v in vals):
            err = "gain out of range 0..1000"
        if err:
            self.event({"type": "cmd", "cmd": text, "ok": 0, "err": err})
            return
        if name == "start":
            self.run_t0 = time.time()
            self.set_state("LINE_FOLLOW", "start")
        elif name == "stop":
            self.set_state("STOPPED", "stop command")
        elif name == "calibrate":
            self.set_state("CALIBRATE", "spin 360")
        elif name == "speed":
            self.cruise = int(min(max(vals[0], 60), 400))
        elif name == "pid":
            self.g["kp"], self.g["ki"], self.g["kd"] = vals
        elif name == "ff":
            self.g["kf"], self.g["off"] = vals
        elif name == "line":
            self.line = vals
        elif name == "rate":
            self.rate_ms = int(vals[0])
        self.event({"type": "cmd", "cmd": text, "ok": 1})
        if name == "help":
            self.event({"type": "help", "cmds": "start|stop|calibrate|gains|"
                        "help|speed=|pid=|ff=|line=|rate="})
        if name in ("speed", "pid", "ff", "line", "rate", "gains"):
            self.gains_event()

    # ---- physics ---------------------------------------------------------
    def step(self, now):
        t_run = now - self.run_t0
        moving = self.state == "LINE_FOLLOW"
        if self.state == "CALIBRATE" and now - self.state_t > 3:
            self.set_state("IDLE", "calibrated")
        if self.state == "OBSTACLE" and now - self.state_t > 4:
            self.nobs += 1
            self.act = "CONTINUE"
            self.set_state("LINE_FOLLOW", "obstacle bypassed")

        # line wobble: smaller and faster with more line Kp
        amp = min(0.8, 0.25 * 160.0 / max(self.line[0], 1.0))
        le = amp * math.sin(t_run * (1.0 + self.line[0] / 80.0)) \
            + random.gauss(0, 0.03) if moving else 0.0
        le = max(-1.0, min(1.0, le))
        self.le = le
        steer = self.line[0] * le * 0.5 if moving else 0.0
        tgt = [self.cruise * (1 - 0.5 * min(abs(le), 1)) - steer,
               self.cruise * (1 - 0.5 * min(abs(le), 1)) + steer] \
            if moving else [0.0, 0.0]
        self.tgt = tgt
        for i in (0, 1):
            if tgt[i] > 1:
                self.pwm[i] = min(self.pid[i].update(self.g, tgt[i],
                                                     self.v[i])
                                  + self.g["off"], 100)
            else:
                self.pid[i] = Pid()
                self.pwm[i] = 0.0
            v_ss = MOTOR_GAIN[i] * max(self.pwm[i] - MOTOR_DEADBAND, 0)
            self.v[i] += (v_ss - self.v[i]) * DT / MOTOR_TAU
        ds = 0.5 * (self.v[0] + self.v[1]) * DT
        self.odo += ds
        self.enc_f = [self.enc_f[i] + self.v[i] * DT / 5.1 for i in (0, 1)]
        self.enc = [int(e) for e in self.enc_f]      # 5.1 mm per tick

        # hump every 20 s of running: raised-cosine profile
        x = (t_run % 20.0) * self.cruise - 1000.0
        if moving and 0 < x < HUMP_LEN:
            slope = HUMP_HEIGHT * math.pi / HUMP_LEN * \
                math.sin(2 * math.pi * x / HUMP_LEN)
            self.pitch = math.degrees(math.atan(slope))
            self.hump["h"] = HUMP_HEIGHT * 0.5 * \
                (1 - math.cos(2 * math.pi * x / HUMP_LEN))
            self.hump["pk"] = max(self.hump["pk"], self.hump["h"])
        elif self.hump["h"] > 0:
            self.hump["n"] += 1
            peak = round(self.hump["pk"] * random.uniform(0.95, 1.03))
            self.hump["max"] = max(self.hump["max"], peak)
            self.event({"type": "hump", "n": self.hump["n"], "peak_mm": peak,
                        "max_mm": self.hump["max"]})
            self.hump["h"], self.hump["pk"], self.pitch = 0, peak, 0.0

        # barcode every 15 s, obstacle every 30 s
        if moving and int(t_run) % 15 == 7 and self.bc_due(t_run):
            self.bc = "ABCD"[self.nbc % 4]
            self.nav = ["LEFT", "RIGHT", "STRAIGHT", "UTURN"][self.nbc % 4]
            self.nbc += 1
            self.event({"type": "barcode", "code": self.bc, "cmd": self.nav})
        phase = t_run % 30.0
        self.front = int(max(200, 1400 - 140 * (phase - 20))) \
            if moving and phase > 20 else -1
        if moving and 0 <= self.front < 300:
            self.act = random.choice(["TURN_LEFT", "TURN_RIGHT"])
            self.event({"type": "obstacle", "found": 1, "dist": self.front,
                        "ang": 90, "width": 110, "centre": 15, "cl": 1000,
                        "cr": 420, "act": self.act})
            self.set_state("OBSTACLE", "obstacle ahead / impact")

    _last_bc = -1

    def bc_due(self, t_run):
        k = int(t_run) // 15
        if k != self._last_bc:
            self._last_bc = k
            return True
        return False

    def telemetry(self, now):
        o = {"t": int((now - self.run_t0) * 1000), "st": self.state,
             "spd": [int(v) for v in self.v],
             "tgt": [int(v) for v in self.tgt],
             "pwm": [int(p) for p in self.pwm], "enc": self.enc,
             "odo": int(self.odo), "hdg": 0,
             "ir": [int(500 + 450 * self.le), int(500 - 450 * self.le), 20],
             "le": int(self.le * 1000), "ls": "ON", "bc": self.bc,
             "nav": self.nav,
             "nbc": self.nbc, "pitch": int(self.pitch * 10), "roll": 0,
             "yaw": 1800, "rate": 0,
             "hump": {"h": int(self.hump["h"]), "pk": int(self.hump["pk"]),
                      "max": int(self.hump["max"]), "n": self.hump["n"]},
             "ev": "CLIMBING" if self.pitch > 3 else (
                 "DESCENDING" if self.pitch < -3 else "CRUISING"),
             "front": self.front, "act": self.act, "nobs": self.nobs}
        self.pub("telemetry", json.dumps(o, separators=(",", ":")))



def main():
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("pip install paho-mqtt")
    ap = argparse.ArgumentParser(description="PicoCar MQTT simulator")
    ap.add_argument("--mqtt", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--car", default="car1")
    ap.add_argument("--autostart", action="store_true",
                    help="start driving immediately")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()
    root = "picocar/%s/" % args.car

    if hasattr(mqtt, "CallbackAPIVersion"):
        cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                          client_id="picocar-sim")
    else:
        cli = mqtt.Client(client_id="picocar-sim")
    cli.will_set(root + "status", "offline", retain=True)
    car = Car(lambda topic, text: cli.publish(root + topic, text))

    def on_connect(c, *_):
        c.subscribe(root + "cmd")
        c.publish(root + "status", "online", retain=True)
        car.gains_event()

    cli.on_connect = on_connect
    cli.on_message = lambda c, u, m: car.command(m.payload.decode())
    cli.connect(args.mqtt, args.port, 30)
    cli.loop_start()
    if args.autostart:
        car.command("start")
    print("simulating %s on %s:%d (Ctrl+C to stop)" % (root, args.mqtt,
                                                       args.port))
    t_start = last_tel = last_hb = time.time()
    try:
        while args.duration is None or time.time() - t_start < args.duration:
            now = time.time()
            car.step(now)
            if car.rate_ms and (now - last_tel) * 1000 >= car.rate_ms:
                last_tel = now
                car.telemetry(now)
            if now - last_hb >= 1.0:
                last_hb = now
                car.seq += 1
                cli.publish(root + "heartbeat", json.dumps(
                    {"up": int(now - t_start), "seq": car.seq,
                     "st": car.state, "link": "CONNECTED", "pub": 0,
                     "drop": 0, "rx": 0, "recon": 0, "imu": 1,
                     "i2cerr": 0}, separators=(",", ":")))
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    cli.publish(root + "status", "offline", retain=True)
    time.sleep(0.2)
    cli.loop_stop()


if __name__ == "__main__":
    main()
