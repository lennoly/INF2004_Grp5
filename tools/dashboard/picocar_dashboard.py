#!/usr/bin/env python3
"""
PicoCar dashboard - live telemetry, tuning and CSV logging (Buddy 1).

Connects to the car in one of two ways:
  * MQTT   : python picocar_dashboard.py --mqtt 192.168.1.100 [--car car1]
             (firmware built with WIFI_MQTT=1, broker e.g. mosquitto)
  * Serial : python picocar_dashboard.py --serial auto      (or COM5, /dev/ttyACM0)
             (any build; the car prints JSON on its USB console)

What it does
  * live plots  : wheel speed vs target, line error + IR, pitch + hump height,
                  front ultrasonic distance (last 30 s)
  * status panel: state, barcode, hump peak, obstacles, link health, gains
  * tuning panel: edit speed PID / feed-forward / line PID and press Apply -
                  sends pid= / ff= / line= to the car, no reflashing needed
  * commands    : Start / Stop / Calibrate buttons, a free command box, and
                  Fast (20 Hz) / Normal telemetry rate buttons
  * logging     : every run is saved in logs/<date_time>/ as CSV files
                  (telemetry.csv, events.csv, heartbeat.csv, commands.log,
                  console.log) ready for the report tables; "Save PNG" stores
                  the current plots next to them.

Headless mode (--headless) logs and prints a status line every second, and
reads commands typed in the terminal.  --snapshot out.png --duration N renders
the dashboard to an image after N seconds (no display needed).

Requires: matplotlib, paho-mqtt (MQTT mode), pyserial (serial mode).
"""

import argparse
import csv
import importlib.util
import json
import os
import queue
import sys
import threading
import time
from collections import deque
from datetime import datetime

WINDOW_S = 30.0            # seconds of history shown in the plots
GUI_REFRESH_MS = 250       # redraw period
SERIAL_CHUNK = 16          # bytes per write: the car's UART FIFO is 32 deep
SERIAL_CHUNK_GAP_S = 0.03  # and is polled every 20 ms
RASPBERRY_PI_USB_VID = 0x2E8A
FAST_RATE_MS = 50          # "Fast" button: 20 Hz telemetry for tuning


# ---------------------------------------------------------------------------
# Message model
# ---------------------------------------------------------------------------
def classify(obj):
    """Tell telemetry, heartbeat and event JSON apart (serial has no topics)."""
    if not isinstance(obj, dict):
        return "unknown"
    if "type" in obj:
        return "event"
    if "up" in obj and "seq" in obj:
        return "heartbeat"
    if "st" in obj and "spd" in obj:
        return "telemetry"
    return "unknown"


def pair(obj, key, index, default=None):
    """Safe access to list fields such as spd[0]."""
    try:
        return obj[key][index]
    except (KeyError, IndexError, TypeError):
        return default


def scaled(value, factor):
    """Integer telemetry unit -> engineering unit (x0.1 deg, x0.001 ...)."""
    if value is None:
        return None
    digits = 1 if factor >= 0.1 else 3
    return round(value * factor, digits)


# CSV columns: (name with units, function(telemetry dict) -> value).
# Firmware sends integers: deci-degrees and per-mille are converted here.
TELEMETRY_COLUMNS = [
    ("t_ms", lambda o: o.get("t")),
    ("state", lambda o: o.get("st")),
    ("spd_l_mm_s", lambda o: pair(o, "spd", 0)),
    ("spd_r_mm_s", lambda o: pair(o, "spd", 1)),
    ("tgt_l_mm_s", lambda o: pair(o, "tgt", 0)),
    ("tgt_r_mm_s", lambda o: pair(o, "tgt", 1)),
    ("pwm_l_pct", lambda o: pair(o, "pwm", 0)),
    ("pwm_r_pct", lambda o: pair(o, "pwm", 1)),
    ("enc_l_ticks", lambda o: pair(o, "enc", 0)),
    ("enc_r_ticks", lambda o: pair(o, "enc", 1)),
    ("odo_mm", lambda o: o.get("odo")),
    ("heading_deg", lambda o: scaled(o.get("hdg"), 0.1)),
    ("ir_l", lambda o: scaled(pair(o, "ir", 0), 0.001)),
    ("ir_r", lambda o: scaled(pair(o, "ir", 1), 0.001)),
    ("ir_barcode", lambda o: scaled(pair(o, "ir", 2), 0.001)),
    ("line_error", lambda o: scaled(o.get("le"), 0.001)),
    ("line_state", lambda o: o.get("ls")),
    ("barcode", lambda o: o.get("bc")),
    ("nav", lambda o: o.get("nav")),
    ("n_barcodes", lambda o: o.get("nbc")),
    ("pitch_deg", lambda o: scaled(o.get("pitch"), 0.1)),
    ("roll_deg", lambda o: scaled(o.get("roll"), 0.1)),
    ("yaw_deg", lambda o: scaled(o.get("yaw"), 0.1)),
    ("turn_rate_dps", lambda o: scaled(o.get("rate"), 0.1)),
    ("hump_h_mm", lambda o: (o.get("hump") or {}).get("h")),
    ("hump_peak_mm", lambda o: (o.get("hump") or {}).get("pk")),
    ("hump_max_mm", lambda o: (o.get("hump") or {}).get("max")),
    ("n_humps", lambda o: (o.get("hump") or {}).get("n")),
    ("motion_event", lambda o: o.get("ev")),
    ("front_mm", lambda o: o.get("front")),
    ("avoid_action", lambda o: o.get("act")),
    ("n_obstacles", lambda o: o.get("nobs")),
]

HEARTBEAT_COLUMNS = ["up", "seq", "st", "link", "pub", "drop", "rx", "recon",
                     "imu", "i2cerr"]


# ---------------------------------------------------------------------------
# Links (MQTT / serial).  Both push (kind, payload) tuples into an inbox queue
# ---------------------------------------------------------------------------
class MqttLink:
    """Subscribes to picocar/<car>/# and publishes commands to .../cmd."""

    def __init__(self, host, port, car, inbox):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            sys.exit("MQTT mode needs paho-mqtt:  pip install paho-mqtt")
        self.inbox = inbox
        self.root = "picocar/%s/" % car
        self.connected = False
        name = "picocar-dashboard-%d" % os.getpid()
        if hasattr(mqtt, "CallbackAPIVersion"):          # paho-mqtt 2.x
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                      client_id=name)
        else:                                            # paho-mqtt 1.x
            self.client = mqtt.Client(client_id=name)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 8)
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()
        self.where = "mqtt://%s:%d  topic %s#" % (host, port, self.root)

    def _on_connect(self, client, userdata, flags, rc, *args):
        failed = rc.is_failure if hasattr(rc, "is_failure") else rc != 0
        if failed:
            self.inbox.put(("link", "broker refused connection: %s" % rc))
            return
        self.connected = True
        client.subscribe(self.root + "#")    # retained "status" arrives now
        self.inbox.put(("link", "broker connected"))

    def _on_disconnect(self, client, userdata, *args):
        self.connected = False
        self.inbox.put(("link", "broker disconnected - retrying"))

    def _on_message(self, client, userdata, msg):
        topic = msg.topic[len(self.root):]
        text = msg.payload.decode("utf-8", errors="replace").strip()
        if topic == "status":                # "online" / "offline" (LWT)
            self.inbox.put(("status", text))
            if text == "online":
                self.send("gains")           # ask the car for its gains
            return
        if topic == "cmd":
            return                           # our own commands echoed back
        try:
            obj = json.loads(text)
        except ValueError:
            self.inbox.put(("raw", "[%s] %s" % (topic, text)))
            return
        kind = {"telemetry": "telemetry", "heartbeat": "heartbeat",
                "event": "event"}.get(topic, classify(obj))
        self.inbox.put((kind, obj))

    def send(self, command):
        self.client.publish(self.root + "cmd", command.strip())

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


class SerialLink:
    """Reads the car's USB/UART console; JSON lines are data, the rest is
    kept as console text.  Reopens the port if the car is reset."""

    def __init__(self, port, baud, inbox):
        if importlib.util.find_spec("serial") is None:
            sys.exit("Serial mode needs pyserial:  pip install pyserial")
        self.inbox = inbox
        self.baud = baud
        self.port_arg = port
        self.port = None
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()
        self.running = True
        self.where = "serial %s @ %d" % (port, baud)
        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def find_port():
        """First USB serial port with the Raspberry Pi vendor id."""
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid == RASPBERRY_PI_USB_VID:
                return p.device
        return None

    def _open(self):
        import serial
        port = self.find_port() if self.port_arg == "auto" else self.port_arg
        if port is None:
            return False
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.2)
        except (serial.SerialException, OSError):
            return False
        self.port = port
        self.where = "serial %s @ %d" % (port, self.baud)
        self.connected = True
        self.inbox.put(("link", "opened " + port))
        self.send("gains")
        return True

    def _reader(self):
        buf = b""
        while self.running:
            if not self.connected and not self._open():
                time.sleep(1.0)
                continue
            try:
                chunk = self.ser.read(512)
            except Exception:                  # unplugged / car reset
                self.connected = False
                self.inbox.put(("link", "serial lost - reconnecting"))
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1.0)
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self._dispatch(line)

    def _dispatch(self, line):
        self.inbox.put(("console", line))
        if line.startswith("{"):
            try:
                obj = json.loads(line)
            except ValueError:
                return
            self.inbox.put((classify(obj), obj))

    def send(self, command):
        """Paced write so the car's 32-byte UART FIFO never overflows."""
        data = (command.strip() + "\n").encode("ascii", errors="ignore")
        with self.lock:
            if not self.connected:
                return
            try:
                for i in range(0, len(data), SERIAL_CHUNK):
                    self.ser.write(data[i:i + SERIAL_CHUNK])
                    self.ser.flush()
                    time.sleep(SERIAL_CHUNK_GAP_S)
            except Exception:
                self.connected = False

    def close(self):
        self.running = False
        if self.ser is not None:
            self.ser.close()


# ---------------------------------------------------------------------------
# CSV logger
# ---------------------------------------------------------------------------
class Logger:
    """One folder per session; files are line-buffered so a crash loses
    nothing."""

    def __init__(self, base_dir):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(base_dir, stamp)
        os.makedirs(self.dir, exist_ok=True)

        def open_csv(name, header):
            f = open(os.path.join(self.dir, name), "w", newline="",
                     buffering=1, encoding="utf-8")
            w = csv.writer(f)
            w.writerow(header)
            return f, w

        self.f_tel, self.w_tel = open_csv(
            "telemetry.csv", ["pc_time"] + [c[0] for c in TELEMETRY_COLUMNS])
        self.f_hb, self.w_hb = open_csv(
            "heartbeat.csv", ["pc_time"] + HEARTBEAT_COLUMNS)
        self.f_ev, self.w_ev = open_csv(
            "events.csv", ["pc_time", "type", "json"])
        self.f_cmd = open(os.path.join(self.dir, "commands.log"), "w",
                          buffering=1, encoding="utf-8")
        self.f_con = open(os.path.join(self.dir, "console.log"), "w",
                          buffering=1, encoding="utf-8")
        self.rows = 0

    @staticmethod
    def now():
        return datetime.now().isoformat(timespec="milliseconds")

    def telemetry(self, obj):
        self.w_tel.writerow([self.now()] + [f(obj) for _, f in
                                             TELEMETRY_COLUMNS])
        self.rows += 1

    def heartbeat(self, obj):
        self.w_hb.writerow([self.now()] + [obj.get(k) for k in
                                            HEARTBEAT_COLUMNS])

    def event(self, obj):
        self.w_ev.writerow([self.now(), obj.get("type"),
                            json.dumps(obj, separators=(",", ":"))])

    def command(self, text):
        self.f_cmd.write("%s  > %s\n" % (self.now(), text))

    def console(self, line):
        self.f_con.write("%s  %s\n" % (self.now(), line))

    def close(self):
        for f in (self.f_tel, self.f_hb, self.f_ev, self.f_cmd, self.f_con):
            f.close()


# ---------------------------------------------------------------------------
# Dashboard state (shared by GUI and headless mode)
# ---------------------------------------------------------------------------
class State:
    SERIES = ["spd_l", "spd_r", "tgt_l", "tgt_r", "le", "ir_l", "ir_r",
              "pitch", "hump_h", "front"]

    def __init__(self, logger):
        self.log = logger
        self.t0 = time.time()
        self.tel = {}
        self.hb = {}
        self.gains = None
        self.car_status = "unknown"
        self.link_msg = "starting"
        self.events = deque(maxlen=14)
        self.n_events = 0          # total ever added (headless printing)
        self.console = deque(maxlen=6)
        self.n_tel = 0
        self.last_tel_time = None
        self.t = deque()
        self.data = {k: deque() for k in self.SERIES}

    def handle(self, kind, payload):
        if kind == "telemetry":
            self._telemetry(payload)
        elif kind == "heartbeat":
            self.hb = payload
            self.log.heartbeat(payload)
        elif kind == "event":
            self.log.event(payload)
            if payload.get("type") == "gains":
                self.gains = payload
            self._add_event(describe_event(payload))
        elif kind == "status":
            self.car_status = payload
            self._add_event("car is %s" % payload)
        elif kind == "link":
            self.link_msg = payload
        elif kind in ("console", "raw"):
            self.log.console(payload)
            if not payload.startswith("{"):
                self.console.append(payload[:90])

    def _add_event(self, text):
        line = "%s  %s" % (datetime.now().strftime("%H:%M:%S"), text)
        self.events.append(line if len(line) <= 78 else line[:75] + "...")
        self.n_events += 1

    def _telemetry(self, o):
        self.tel = o
        self.n_tel += 1
        self.last_tel_time = time.time()
        self.log.telemetry(o)
        now = time.time() - self.t0
        front = o.get("front")
        values = {
            "spd_l": pair(o, "spd", 0), "spd_r": pair(o, "spd", 1),
            "tgt_l": pair(o, "tgt", 0), "tgt_r": pair(o, "tgt", 1),
            "le": scaled(o.get("le"), 0.001),
            "ir_l": scaled(pair(o, "ir", 0), 0.001),
            "ir_r": scaled(pair(o, "ir", 1), 0.001),
            "pitch": scaled(o.get("pitch"), 0.1),
            "hump_h": (o.get("hump") or {}).get("h"),
            "front": front if (front is not None and front >= 0) else None,
        }
        self.t.append(now)
        for k in self.SERIES:
            v = values[k]
            self.data[k].append(float("nan") if v is None else v)
        while self.t and now - self.t[0] > WINDOW_S:
            self.t.popleft()
            for k in self.SERIES:
                self.data[k].popleft()

    def status_lines(self, link_where, link_up):
        o, hb, g = self.tel, self.hb, self.gains
        hump = o.get("hump") or {}
        age = ("%.1f s ago" % (time.time() - self.last_tel_time)
               if self.last_tel_time else "none yet")
        lines = [
            "LINK   %s  [%s]" % (link_where, "up" if link_up else "DOWN"),
            "       %s | car %s" % (self.link_msg, self.car_status),
            "DATA   %d telemetry msgs, last %s" % (self.n_tel, age),
            "",
            "STATE  %s" % o.get("st", "-"),
            "RUN    %.1f s   odo %s mm   hdg %s deg" % (
                (o.get("t") or 0) / 1000.0, o.get("odo", "-"),
                fmt(scaled(o.get("hdg"), 0.1))),
            "LINE   %s  err %s" % (o.get("ls", "-"),
                                   fmt(scaled(o.get("le"), 0.001), 3)),
            "BARCODE last '%s' -> %s   count %s" % (
                o.get("bc", ""), o.get("nav", "-"), o.get("nbc", 0)),
            "HUMP   now %s mm  last peak %s mm  MAX %s mm  (n=%s)" % (
                hump.get("h", "-"), hump.get("pk", "-"),
                hump.get("max", "-"), hump.get("n", 0)),
            "OBST   front %s mm  last %s  passed %s" % (
                o.get("front", "-"), o.get("act", "-"), o.get("nobs", 0)),
            "MOTION %s   pitch %s deg" % (
                o.get("ev", "-"), fmt(scaled(o.get("pitch"), 0.1))),
            "",
            "HEART  up %s s  seq %s  link %s" % (
                hb.get("up", "-"), hb.get("seq", "-"), hb.get("link", "-")),
            "       published %s  dropped %s  reconnects %s" % (
                hb.get("pub", "-"), hb.get("drop", "-"),
                hb.get("recon", "-")),
        ]
        if g:
            lines += [
                "GAINS  pid=%s  ff=%s" % (join_num(g.get("pid")),
                                          join_num(g.get("ff"))),
                "       line=%s  speed=%s  rate=%s ms" % (
                    join_num(g.get("line")), g.get("speed"),
                    g.get("rate")),
            ]
        else:
            lines.append("GAINS  (not received yet - press Gains)")
        return lines


def fmt(value, digits=1):
    return "-" if value is None else ("%." + str(digits) + "f") % value


def describe_event(e):
    """One readable line per event for the event list / terminal."""
    t = e.get("type")
    if t == "state":
        return "state -> %s (%s)" % (e.get("st"), e.get("why"))
    if t == "barcode":
        return "BARCODE '%s' -> %s" % (e.get("code"), e.get("cmd"))
    if t == "hump":
        return "HUMP #%s peak %s mm (max %s mm)" % (
            e.get("n"), e.get("peak_mm"), e.get("max_mm"))
    if t == "obstacle":
        return "OBSTACLE d=%s w=%s L=%s R=%s -> %s" % (
            e.get("dist"), e.get("width"), e.get("cl"), e.get("cr"),
            e.get("act"))
    if t == "impact":
        return "IMPACT #%s" % e.get("n")
    if t == "cmd":
        ok = "ok" if e.get("ok") else "FAILED: %s" % e.get("err", "")
        return "cmd '%s' %s" % (e.get("cmd"), ok)
    if t == "gains":
        return "gains pid=%s ff=%s line=%s" % (
            join_num(e.get("pid")), join_num(e.get("ff")),
            join_num(e.get("line")))
    return json.dumps(e)


def join_num(values):
    """[0.08, 0.6, 0.0] -> '0.08,0.6,0'  (compact, same as the command)."""
    return ",".join("%g" % v for v in (values or []))


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class DashboardGUI:
    def __init__(self, state, link, snapshot=None):
        import matplotlib
        if snapshot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, TextBox
        self.plt = plt
        self.state = state
        self.link = link
        self.snapshot = snapshot

        self.fig = plt.figure(figsize=(15, 9))
        try:
            self.fig.canvas.manager.set_window_title("PicoCar dashboard")
        except AttributeError:
            pass                                  # no window (snapshot)
        # plots in the top 2/3, controls in the bottom third
        gs = self.fig.add_gridspec(2, 3, width_ratios=[1, 1, 1.2],
                                   hspace=0.45, wspace=0.3, left=0.05,
                                   right=0.99, top=0.92, bottom=0.36)
        self.ax_spd = self.fig.add_subplot(gs[0, 0])
        self.ax_line = self.fig.add_subplot(gs[0, 1])
        self.ax_hump = self.fig.add_subplot(gs[1, 0])
        self.ax_front = self.fig.add_subplot(gs[1, 1])
        self.ax_text = self.fig.add_subplot(gs[:, 2])
        self.ax_text.axis("off")
        self.fig.suptitle("PicoCar live dashboard", fontsize=13,
                          fontweight="bold", x=0.05, y=0.985, ha="left")

        self.lines = {}
        a = self.ax_spd
        a.set_title("Wheel speed vs target (mm/s)", fontsize=10)
        self.lines["spd_l"], = a.plot([], [], color="#2563eb", label="left")
        self.lines["spd_r"], = a.plot([], [], color="#dc2626", label="right")
        self.lines["tgt_l"], = a.plot([], [], color="#2563eb", ls="--",
                                      lw=1, label="target L")
        self.lines["tgt_r"], = a.plot([], [], color="#dc2626", ls="--",
                                      lw=1, label="target R")
        a.legend(loc="lower left", fontsize=7, ncol=4, framealpha=0.8)

        a = self.ax_line
        a.set_title("Line: error (L-R) and IR (0 white .. 1 black)",
                    fontsize=10)
        a.set_ylim(-1.05, 1.05)
        self.lines["le"], = a.plot([], [], color="#111827", label="error")
        self.lines["ir_l"], = a.plot([], [], color="#16a34a", lw=1,
                                     label="IR left")
        self.lines["ir_r"], = a.plot([], [], color="#9333ea", lw=1,
                                     label="IR right")
        a.axhline(0, color="#9ca3af", lw=0.8)
        a.legend(loc="lower left", fontsize=7, ncol=3, framealpha=0.8)

        a = self.ax_hump
        a.set_title("Terrain: pitch (deg) and hump height (mm)", fontsize=10)
        self.lines["pitch"], = a.plot([], [], color="#ea580c",
                                      label="pitch deg")
        a.set_ylabel("pitch (deg)", fontsize=8)
        self.ax_hump2 = a.twinx()
        self.lines["hump_h"], = self.ax_hump2.plot([], [], color="#0891b2",
                                                   label="hump mm")
        self.ax_hump2.set_ylabel("height (mm)", fontsize=8)
        a.legend(handles=[self.lines["pitch"], self.lines["hump_h"]],
                 loc="upper left", fontsize=7)

        a = self.ax_front
        a.set_title("Front ultrasonic distance (mm)", fontsize=10)
        self.lines["front"], = a.plot([], [], color="#4b5563")
        a.axhline(300, color="#dc2626", lw=0.8, ls=":")
        a.text(0.99, 0.02, "300 mm = obstacle threshold", fontsize=7,
               color="#dc2626", ha="right", transform=a.transAxes)

        for ax in (self.ax_spd, self.ax_line, self.ax_hump, self.ax_front):
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.set_xlabel("time (s)", fontsize=8)

        self.status_text = self.ax_text.text(
            0.0, 1.0, "", va="top", ha="left", family="monospace",
            fontsize=7.8, transform=self.ax_text.transAxes)

        # ---- controls row ------------------------------------------------
        self.widgets = []

        def button(x, y, w, label, cb):
            ax = self.fig.add_axes([x, y, w, 0.04])
            b = Button(ax, label)
            b.on_clicked(lambda _e: cb())
            self.widgets.append(b)
            return b

        def box(x, y, w, label, initial=""):
            ax = self.fig.add_axes([x, y, w, 0.035])
            t = TextBox(ax, label, initial=initial, label_pad=0.05)
            t.label.set_fontsize(8)
            self.widgets.append(t)
            return t

        y1, y2, y3 = 0.235, 0.165, 0.10
        button(0.05, y1, 0.07, "Start", lambda: self.send("start"))
        button(0.125, y1, 0.07, "Stop", lambda: self.send("stop"))
        button(0.20, y1, 0.08, "Calibrate", lambda: self.send("calibrate"))
        button(0.285, y1, 0.07, "Gains", lambda: self.send("gains"))
        button(0.36, y1, 0.09, "Fast 20 Hz",
               lambda: self.send("rate=%d" % FAST_RATE_MS))
        button(0.455, y1, 0.09, "Normal rate", lambda: self.send("rate=200"))
        button(0.55, y1, 0.08, "Save PNG", self.save_png)

        self.pid = [box(0.09 + i * 0.075, y2, 0.05, lbl)
                    for i, lbl in enumerate(["speed Kp", "Ki", "Kd"])]
        button(0.315, y2, 0.075, "Apply PID", self.apply_pid)
        self.ff = [box(0.45 + i * 0.085, y2, 0.05, lbl)
                   for i, lbl in enumerate(["Kf", "offset %"])]
        button(0.63, y2, 0.07, "Apply FF", self.apply_ff)

        self.line = [box(0.09 + i * 0.075, y3, 0.05, lbl)
                     for i, lbl in enumerate(["line Kp", "Ki", "Kd"])]
        button(0.315, y3, 0.075, "Apply line", self.apply_line)
        self.speed = box(0.45, y3, 0.05, "cruise")
        button(0.505, y3, 0.07, "Set speed",
               lambda: self.send("speed=%s" % self.speed.text.strip()))

        self.cmd = box(0.09, 0.035, 0.45, "command")
        self.fig.text(0.05, 0.285, "Controls", fontsize=10,
                      fontweight="bold")
        self.cmd.on_submit(self.submit_command)
        self.fig.text(0.55, 0.045, "Enter to send, e.g. pid=0.1,0.8,0  "
                      "line=150,0,10  rate=100  help", fontsize=8,
                      color="#6b7280")
        self.feedback = self.fig.text(0.64, 0.245, "", fontsize=8,
                                      color="#6b7280")
        self.gains_seen = None

    # ---- actions ----------------------------------------------------------
    def send(self, command):
        command = command.strip()
        if not command:
            return
        self.link.send(command)
        self.state.log.command(command)
        self.feedback.set_text("sent: " + command)

    def submit_command(self, text):
        if text.strip():
            self.send(text)
            self.cmd.set_val("")

    def _values(self, boxes):
        return ",".join(b.text.strip() for b in boxes)

    def apply_pid(self):
        self.send("pid=" + self._values(self.pid))

    def apply_ff(self):
        self.send("ff=" + self._values(self.ff))

    def apply_line(self):
        self.send("line=" + self._values(self.line))

    def save_png(self):
        path = os.path.join(self.state.log.dir, "dashboard_%s.png" %
                            datetime.now().strftime("%H%M%S"))
        self.fig.savefig(path, dpi=110)
        self.feedback.set_text("saved " + path)

    # ---- refresh ----------------------------------------------------------
    def fill_gains(self):
        """Copy the car's gains into the tuning boxes when they change."""
        g = self.state.gains
        if not g or g is self.gains_seen:
            return
        self.gains_seen = g
        for boxes, key in ((self.pid, "pid"), (self.ff, "ff"),
                           (self.line, "line")):
            for b, v in zip(boxes, g.get(key, [])):
                b.set_val(str(v))
        if g.get("speed") is not None:
            self.speed.set_val(str(g.get("speed")))

    def refresh(self):
        s = self.state
        t = list(s.t)
        for k, ln in self.lines.items():
            ln.set_data(t, list(s.data[k]))
        for ax in (self.ax_spd, self.ax_line, self.ax_hump, self.ax_hump2,
                   self.ax_front):
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=(ax is not self.ax_line))
            if t:
                ax.set_xlim(max(0.0, t[-1] - WINDOW_S), max(WINDOW_S, t[-1]))
        self.ax_front.set_ylim(bottom=0)       # keep 0 and 300 mm in view
        self.ax_front.set_ylim(top=max(self.ax_front.get_ylim()[1], 400))
        text = s.status_lines(self.link.where, self.link.connected)
        text += ["", "EVENTS"] + list(s.events)
        if s.console:
            text += ["", "CONSOLE"] + list(s.console)
        self.status_text.set_text("\n".join(text))
        self.fill_gains()

    def run(self, pump, duration=None):
        from matplotlib.animation import FuncAnimation

        def tick(_frame):
            pump()
            self.refresh()
            return []

        if self.snapshot:
            end = time.time() + (duration or 5)
            while time.time() < end:
                pump()
                time.sleep(0.05)
            self.refresh()
            self.fig.savefig(self.snapshot, dpi=100)
            print("snapshot saved to", self.snapshot)
            return
        self._anim = FuncAnimation(self.fig, tick, interval=GUI_REFRESH_MS,
                                   cache_frame_data=False)
        if duration:
            timer = self.fig.canvas.new_timer(interval=int(duration * 1000))
            timer.add_callback(self.plt.close, self.fig)
            timer.start()
        self.plt.show()


# ---------------------------------------------------------------------------
# Headless mode
# ---------------------------------------------------------------------------
def run_headless(state, link, pump, duration):
    """Log, print a status line every second, send typed commands."""
    def stdin_reader():
        for line in sys.stdin:
            if line.strip():
                link.send(line)
                state.log.command(line.strip())

    threading.Thread(target=stdin_reader, daemon=True).start()
    print("headless: logging to %s  (type commands, Ctrl+C to quit)" %
          state.log.dir)
    end = time.time() + duration if duration else None
    shown = 0
    last = 0.0
    while end is None or time.time() < end:
        pump()
        new = state.n_events - shown
        if new > 0:
            for e in list(state.events)[-min(new, len(state.events)):]:
                print("  event:", e)
            shown = state.n_events
        if time.time() - last >= 1.0:
            last = time.time()
            o = state.tel
            print("[%s] %s msgs=%d st=%s spd=%s le=%s hump_max=%s front=%s" % (
                "up" if link.connected else "DOWN", state.link_msg,
                state.n_tel, o.get("st"), o.get("spd"), o.get("le"),
                (o.get("hump") or {}).get("max"), o.get("front")))
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mqtt", metavar="BROKER", help="broker host/IP")
    src.add_argument("--serial", metavar="PORT",
                     help="serial port, or 'auto' to find the Pico")
    src.add_argument("--list-ports", action="store_true",
                     help="list serial ports and exit")
    ap.add_argument("--port", type=int, default=1883, help="MQTT port")
    ap.add_argument("--car", default="car1", help="MQTT_CAR_ID of the car")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--snapshot", metavar="PNG",
                    help="render the dashboard to PNG after --duration s")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after N seconds")
    args = ap.parse_args()

    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            tag = "  <- Pico" if p.vid == RASPBERRY_PI_USB_VID else ""
            print(p.device, p.description, tag)
        return
    if not args.mqtt and not args.serial:
        ap.error("choose --mqtt BROKER or --serial PORT|auto")

    inbox = queue.Queue()
    link = (MqttLink(args.mqtt, args.port, args.car, inbox) if args.mqtt
            else SerialLink(args.serial, args.baud, inbox))
    logger = Logger(args.log_dir)
    state = State(logger)
    print("logging to", logger.dir)

    def pump():
        while True:
            try:
                kind, payload = inbox.get_nowait()
            except queue.Empty:
                return
            state.handle(kind, payload)

    try:
        if args.headless:
            run_headless(state, link, pump, args.duration)
        else:
            DashboardGUI(state, link, args.snapshot).run(pump, args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        pump()
        link.close()
        logger.close()
        print("saved %d telemetry rows in %s" % (logger.rows, logger.dir))


if __name__ == "__main__":
    main()
