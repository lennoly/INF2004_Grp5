#!/usr/bin/env python3
"""
PicoCar dashboard - live telemetry, tuning and CSV logging in the browser
(Buddy 1).

  * Serial : python picocar_dashboard.py --serial auto      (or COM5)
             (any build with the USB console; frames are printed as '@' hex)
  * MQTT   : python picocar_dashboard.py --mqtt 172.20.10.3 [--car car1]
             (WiFi builds; mosquitto on the laptop)

It starts a small web server on this PC (standard library only) and opens
http://127.0.0.1:8050 in the browser: live values, two light charts (wheel
speed vs target, line error + IR), events, the car's console, buttons,
tuning boxes and a command box.  Every session is logged to
logs/<date_time>/ (telemetry.csv, events.csv, heartbeat.csv,
commands.log, console.log) for the report.

Change Requirement 1: the car sends compact binary frames; frames.py
decodes them.  Commands typed on the page go to the car as text over
serial, or as binary command frames over MQTT.

Requires: paho-mqtt (MQTT mode), pyserial (serial mode).
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
import webbrowser
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import frames

HISTORY_S = 30.0           # chart history kept for a page (re)load
PUSH_PERIOD_S = 0.2        # page updates: at most 5 per second
SERIAL_CHUNK = 16          # bytes per write: the car's UART FIFO is 32 deep
SERIAL_CHUNK_GAP_S = 0.03  # and is polled every 20 ms
RASPBERRY_PI_USB_VID = 0x2E8A
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


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
    ("faults", lambda o: "|".join(o.get("faults", []))),
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
        if topic == "cmd":
            return                           # our own commands echoed back
        try:
            kind, obj = frames.decode(msg.payload)
        except ValueError as err:
            self.inbox.put(("raw", "[%s] %s (%s)" % (topic, msg.payload.hex(),
                                                     err)))
            return
        if kind == "status":                 # online / offline (LWT)
            self.inbox.put(("status", obj))
            if obj == "online":
                self.send("gains")           # ask the car for its gains
            return
        self.inbox.put((kind, obj))

    def send(self, command):
        """Typed command -> binary command frame (frames.py)."""
        try:
            payload = frames.command_from_text(command)
        except ValueError as err:
            self.inbox.put(("raw", "command not sent: %s" % err))
            return
        self.client.publish(self.root + "cmd", payload)

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
        if line.startswith("@"):             # a binary frame in hex
            try:
                kind, obj = frames.decode_console_line(line)
            except ValueError:
                return
            self.inbox.put((kind, obj))
        elif line.startswith("{"):           # older JSON firmware
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
# Dashboard state (filled by the link thread, read by the web server)
# ---------------------------------------------------------------------------
class State:
    """Latest values plus numbered event, console and chart-sample lists,
    so each page asks only for what it has not seen yet."""

    def __init__(self, logger):
        self.log = logger
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.tel = {}
        self.hb = {}
        self.gains = None
        self.car_status = "unknown"
        self.link_msg = "starting"
        self.n_tel = 0
        self.last_tel_time = None
        self.events = deque(maxlen=200)      # (number, text)
        self.n_events = 0
        self.console = deque(maxlen=200)     # (number, text)
        self.n_console = 0
        self.samples = deque()               # (number, t, values)
        self.n_samples = 0

    def handle(self, kind, payload):
        with self.lock:
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
                if not payload.startswith(("{", "@")):   # frames: decoded
                    self.n_console += 1
                    self.console.append((self.n_console, payload[:160]))

    def _add_event(self, text):
        self.n_events += 1
        self.events.append(
            (self.n_events, "%s  %s" % (datetime.now().strftime("%H:%M:%S"),
                                        text)))

    def _telemetry(self, o):
        self.tel = o
        self.n_tel += 1
        self.last_tel_time = time.time()
        self.log.telemetry(o)
        now = round(time.time() - self.t0, 3)
        values = [pair(o, "spd", 0), pair(o, "spd", 1),
                  pair(o, "tgt", 0), pair(o, "tgt", 1),
                  scaled(o.get("le"), 0.001),
                  scaled(pair(o, "ir", 0), 0.001),
                  scaled(pair(o, "ir", 1), 0.001)]
        self.n_samples += 1
        self.samples.append((self.n_samples, now, values))
        while self.samples and now - self.samples[0][1] > HISTORY_S:
            self.samples.popleft()

    def update_since(self, seen, link):
        """Everything a page has not seen yet; seen = [events, console,
        samples] counters, updated in place."""
        with self.lock:
            age = (round(time.time() - self.last_tel_time, 1)
                   if self.last_tel_time else None)
            msg = {
                "link": {"where": link.where, "up": link.connected,
                         "msg": self.link_msg, "car": self.car_status},
                "tel": self.tel, "hb": self.hb, "gains": self.gains,
                "n_tel": self.n_tel, "age_s": age,
                "events": [e for n, e in self.events if n > seen[0]],
                "console": [c for n, c in self.console if n > seen[1]],
                "samples": [[t] + v for n, t, v in self.samples
                            if n > seen[2]],
            }
            seen[0], seen[1], seen[2] = (self.n_events, self.n_console,
                                         self.n_samples)
        return msg


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
# Web server: the page, a Server-Sent Events stream and a command endpoint
# ---------------------------------------------------------------------------
def make_handler(state, link):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt_, *args):          # keep the terminal quiet
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(WEB_DIR, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/stream":
                self._stream()
            else:
                self._send(404, b"not found", "text/plain")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seen = [0, 0, 0]
            try:
                while True:
                    data = json.dumps(state.update_since(seen, link),
                                      separators=(",", ":"), default=str)
                    self.wfile.write(("data: %s\n\n" % data).encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(PUSH_PERIOD_S)
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, OSError):
                return                                # page closed

        def do_POST(self):
            if self.path != "/cmd":
                self._send(404, b"not found", "text/plain")
                return
            length = int(self.headers.get("Content-Length") or 0)
            text = self.rfile.read(min(length, 200)).decode(
                "utf-8", errors="ignore").strip()
            if text:
                link.send(text)
                state.log.command(text)
            self._send(200, b"ok", "text/plain")

    return Handler


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
    ap.add_argument("--http-port", type=int, default=8050,
                    help="web page port (default 8050)")
    ap.add_argument("--no-browser", action="store_true",
                    help="don't open the page automatically")
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

    def pump():
        while True:
            kind, payload = inbox.get()
            if kind is None:
                return
            state.handle(kind, payload)

    threading.Thread(target=pump, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port),
                                 make_handler(state, link))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d" % args.http_port
    print("PicoCar dashboard on %s  (%s)" % (url, link.where))
    print("logging to %s   Ctrl+C to stop" % logger.dir)
    if not args.no_browser:
        webbrowser.open(url)

    try:
        end = time.time() + args.duration if args.duration else None
        while end is None or time.time() < end:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        link.close()
        inbox.put((None, None))
        time.sleep(0.1)
        logger.close()
        print("saved %d telemetry rows in %s" % (logger.rows, logger.dir))


if __name__ == "__main__":
    main()
