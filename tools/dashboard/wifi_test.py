#!/usr/bin/env python3
"""
PicoCar WiFi / MQTT link test (Buddy 1): measures the link against the
design targets and writes the evidence for the report.

  python wifi_test.py --mqtt <broker-ip> [--car car1] [--minutes 10]
  python wifi_test.py --self-test        # checks this script; no car needed

Board: build APP_MODE=TEST_TELEMETRY with the WiFi flags (see the README).
That build runs only WiFi, MQTT and telemetry, so a bare Pico W is enough
(no Robo Pico, no sensors).  Keep the USB cable in: the console prints the
heartbeat every second, so you can see the link state even while MQTT is
down.

While the test runs it
  * sends "#<id> gains" every --ping-s seconds and times the reply
    (command round trip, design assumption A1);
  * counts telemetry (5 Hz, FR9) and heartbeats (1 Hz, FR9);
  * reads the heartbeat: dropped messages, broker reconnects and WiFi
    re-joins;
  * treats more than GAP_S of silence as an outage and times the recovery.

Recovery tests (FR11, NFR8), done by hand during the run:
  1. broker loss: stop mosquitto for about 60 s, then start it again;
  2. access-point loss: switch the hotspot off for about 30 s, then on.
Each outage is listed with how long the car took to come back.

At the end it prints PASS / FAIL per target and writes
logs/<date_time>/wifi_pings.csv, wifi_outages.csv and wifi_summary.txt.
"""

import argparse
import csv
import json
import os
import random
import statistics
import sys
import threading
import time
from datetime import datetime

GAP_S = 2.0              # car silent this long = an outage
PING_TIMEOUT_S = 2.0     # no reply within this = a lost command
AFTER_OUTAGE_S = 1.0     # pings and drops this soon after an outage don't count
REPORT_EVERY_S = 5.0

TARGET_TEL_HZ = 4.5      # FR9: 5 Hz telemetry (10 % margin)
TARGET_RTT_MS = 100.0    # A1: < 100 ms round trip


class WifiAnalyzer:
    """Consumes the car's messages (with arrival times) and the pings sent,
    and judges them against the design targets.  No network code here, so
    it can be checked with --self-test."""

    def __init__(self):
        self.first = None
        self.last_msg = None
        self.tel = 0
        self.hb = 0
        self.prev_hb = None
        self.hb_missed = 0
        self.reboots = 0
        self.drops_connected = 0
        self.first_hb = None
        self.outages = []          # dicts: start, end (None while ongoing)
        self.outage_since_hb = False
        self.status = []           # (t, "online" / "offline")
        self.pings = {}            # id -> dict(sent, reply, res)

    # ---- inputs ----------------------------------------------------------
    def message(self, t, kind, obj):
        if self.first is None:
            self.first = t
        if self.outages and self.outages[-1]["end"] is None:
            self.outages[-1]["end"] = t
        self.last_msg = t
        if kind == "telemetry":
            self.tel += 1
        elif kind == "heartbeat":
            self._heartbeat(obj)
        elif kind == "event" and obj.get("type") == "cmd":
            ping = self.pings.get(obj.get("id"))
            if ping is not None and ping["reply"] is None:
                ping["reply"] = t
                ping["res"] = "ok" if obj.get("ok") else obj.get("err", "?")

    def car_status(self, t, text):
        self.status.append((t, text))

    def sent(self, t, req_id):
        self.pings[req_id] = {"sent": t, "reply": None, "res": None}

    def tick(self, t):
        """Call regularly: notices when the car has gone quiet."""
        ongoing = self.outages and self.outages[-1]["end"] is None
        if self.last_msg is not None and not ongoing and \
                t - self.last_msg > GAP_S:
            self.outages.append({"start": self.last_msg, "end": None})
            self.outage_since_hb = True

    def _heartbeat(self, obj):
        self.hb += 1
        if self.first_hb is None:
            self.first_hb = dict(obj)
        prev = self.prev_hb
        if prev is not None:
            if obj.get("up", 0) < prev.get("up", 0):
                self.reboots += 1               # counters start again
                self.first_hb = dict(obj)
            elif not self.outage_since_hb:
                self.hb_missed += max(0, obj.get("seq", 0)
                                      - prev.get("seq", 0) - 1)
                self.drops_connected += max(0, obj.get("drop", 0)
                                            - prev.get("drop", 0))
        self.outage_since_hb = False
        self.prev_hb = dict(obj)

    # ---- results ---------------------------------------------------------
    def _in_outage(self, t, end_t):
        for o in self.outages:
            end = o["end"] if o["end"] is not None else end_t
            if o["start"] <= t <= end + AFTER_OUTAGE_S:
                return True
        return False

    def results(self, now):
        open_outage = bool(self.outages) and self.outages[-1]["end"] is None
        # An outage still going on runs until now, so the span must too, or
        # the connected time goes negative and the rates explode.
        end_t = now if open_outage else self.last_msg
        span = (end_t - self.first) if self.first is not None else 0.0
        down = sum(((o["end"] if o["end"] is not None else now) - o["start"])
                   for o in self.outages)
        online_s = max(span - down, 1e-6)
        rtts, lost, skipped = [], 0, 0
        for p in self.pings.values():
            if p["reply"] is not None:
                rtts.append((p["reply"] - p["sent"]) * 1000.0)
            elif now - p["sent"] < PING_TIMEOUT_S:
                skipped += 1                    # still waiting
            elif self._in_outage(p["sent"], now):
                skipped += 1                    # sent into an outage
            else:
                lost += 1
        hb0, hb1 = self.first_hb or {}, self.prev_hb or {}
        return {
            "duration_s": span,
            "online_s": online_s if self.first is not None else 0.0,
            "tel_hz": self.tel / online_s if self.first is not None else 0.0,
            "hb_hz": self.hb / online_s if self.first is not None else 0.0,
            "hb_missed": self.hb_missed,
            "rtt_n": len(rtts),
            "rtt_med": statistics.median(rtts) if rtts else None,
            "rtt_p95": sorted(rtts)[int(0.95 * (len(rtts) - 1))]
            if rtts else None,
            "rtt_max": max(rtts) if rtts else None,
            "lost": lost,
            "skipped": skipped,
            "drops_connected": self.drops_connected,
            "recon": hb1.get("recon", 0) - hb0.get("recon", 0),
            "rejoin": hb1.get("rejoin", 0) - hb0.get("rejoin", 0),
            "reboots": self.reboots,
            "outages": [dict(o) for o in self.outages],
            "open_outage": open_outage,
            "seen": self.first is not None,
        }

    def verdicts(self, r):
        """(target, PASS/FAIL/INFO, detail) lines."""
        out = []

        def judge(name, ok, detail):
            out.append((name, "PASS" if ok else "FAIL", detail))

        if r["online_s"] <= 0.0:
            return [("messages from the car", "FAIL", "nothing received")]
        judge("telemetry rate (FR9, 5 Hz)", r["tel_hz"] >= TARGET_TEL_HZ,
              "%.2f Hz while connected" % r["tel_hz"])
        judge("heartbeat (FR9, 1 Hz)", r["hb_missed"] == 0 and r["hb_hz"] > 0.9,
              "%.2f Hz, %d missed while connected" % (r["hb_hz"],
                                                    r["hb_missed"]))
        if r["rtt_n"]:
            judge("command round trip (A1, < 100 ms)",
                  r["rtt_med"] < TARGET_RTT_MS and r["lost"] == 0,
                  "median %.0f ms, 95%% %.0f ms, max %.0f ms; %d replies, "
                  "%d lost" % (r["rtt_med"], r["rtt_p95"], r["rtt_max"],
                               r["rtt_n"], r["lost"]))
        else:
            judge("command round trip (A1, < 100 ms)", False,
                  "no replies (%d lost)" % r["lost"])
        judge("no messages dropped while connected",
              r["drops_connected"] == 0,
              "%d dropped" % r["drops_connected"])
        judge("every outage recovered (FR11)", not r["open_outage"],
              "%d outage(s); broker reconnects %d, WiFi re-joins %d"
              % (len(r["outages"]), r["recon"], r["rejoin"]))
        out.append(("car reboots during the test", "INFO", str(r["reboots"])))
        return out


def classify(topic, obj):
    if topic in ("telemetry", "heartbeat", "event"):
        return topic
    return "event" if isinstance(obj, dict) and "type" in obj else "unknown"


# ---------------------------------------------------------------------------
# Live test over MQTT
# ---------------------------------------------------------------------------
def run_live(args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("needs paho-mqtt:  pip install -r requirements.txt")

    root = "picocar/%s/" % args.car
    an = WifiAnalyzer()
    lock = threading.Lock()             # paho calls back from its own thread
    state = {"connected": False}
    name = "picocar-wifi-test-%d" % os.getpid()
    if hasattr(mqtt, "CallbackAPIVersion"):
        cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
    else:
        cli = mqtt.Client(client_id=name)

    def on_connect(c, userdata, flags, rc, *rest):
        failed = rc.is_failure if hasattr(rc, "is_failure") else rc != 0
        state["connected"] = not failed
        if not failed:
            c.subscribe(root + "#")
        print("[test] broker %s" % ("refused: %s" % rc if failed
                                    else "connected"))

    def on_disconnect(c, userdata, *rest):
        state["connected"] = False
        print("[test] broker connection lost - retrying")

    def on_message(c, userdata, msg):
        t = time.monotonic()
        topic = msg.topic[len(root):]
        text = msg.payload.decode("utf-8", errors="replace").strip()
        if topic == "status":
            with lock:
                an.car_status(t, text)
            print("[car] status %s" % text)
            return
        if topic == "cmd":
            return
        try:
            obj = json.loads(text)
        except ValueError:
            return
        with lock:
            an.message(t, classify(topic, obj), obj)

    cli.on_connect = on_connect
    cli.on_disconnect = on_disconnect
    cli.on_message = on_message
    cli.reconnect_delay_set(1, 2)       # come back fast after a broker outage
    cli.connect_async(args.mqtt, args.port, keepalive=30)
    cli.loop_start()

    print(__doc__.split("While the test")[0].strip().splitlines()[0])
    print("[test] %s:%d topic %s#  for %.1f min - Ctrl+C to stop early"
          % (args.mqtt, args.port, root, args.minutes))
    print("[test] try during the run: stop mosquitto ~60 s, then the "
          "hotspot ~30 s")
    start = time.monotonic()
    next_ping = start + 1.0
    next_report = start + REPORT_EVERY_S
    req_id = random.randint(100000, 900000)
    try:
        while time.monotonic() - start < args.minutes * 60.0:
            now = time.monotonic()
            with lock:
                an.tick(now)
            if now >= next_ping:
                next_ping += args.ping_s
                if state["connected"]:
                    req_id += 1
                    with lock:
                        an.sent(now, req_id)
                    cli.publish(root + "cmd", "#%d gains" % req_id)
            if now >= next_report:
                next_report += REPORT_EVERY_S
                with lock:
                    print_progress(an, now - start, now)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[test] stopped early")
    time.sleep(PING_TIMEOUT_S if state["connected"] else 0.0)
    cli.loop_stop()
    cli.disconnect()
    now = time.monotonic()
    an.tick(now)
    return finish(an, now, start)


def print_progress(an, elapsed, now):
    r = an.results(now)
    rtt = "-" if r["rtt_med"] is None else "%.0f ms" % r["rtt_med"]
    if not r["seen"]:
        down = "WAITING (nothing from the car yet)"
    else:
        down = "DOWN" if r["open_outage"] else "up"
    print("[%4.0f s] link %s  tel %.1f Hz  rtt %s  lost %d  drop %d  "
          "recon %d  rejoin %d  outages %d"
          % (elapsed, down, r["tel_hz"], rtt, r["lost"],
             r["drops_connected"], r["recon"], r["rejoin"],
             len(r["outages"])))


def finish(an, now, start, log_dir=None):
    r = an.results(now)
    lines = ["PicoCar WiFi test  %s" % datetime.now().strftime("%Y-%m-%d %H:%M"),
             "duration %.0f s, connected %.0f s" % (r["duration_s"],
                                                    r["online_s"]), ""]
    for o in r["outages"]:
        end = o["end"]
        lines.append("outage at %6.1f s: %s" % (
            o["start"] - start,
            "back after %.1f s" % (end - o["start"]) if end is not None
            else "NOT recovered"))
    lines.append("")
    verdicts = an.verdicts(r)
    for name, verdict, detail in verdicts:
        lines.append("%-4s  %-38s %s" % (verdict, name, detail))
    text = "\n".join(lines)
    print("\n" + text)
    if log_dir is None:
        log_dir = os.path.join("logs", datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    with open(os.path.join(log_dir, "wifi_summary.txt"), "w") as f:
        f.write(text + "\n")
    with open(os.path.join(log_dir, "wifi_pings.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "sent_s", "rtt_ms", "result"])
        for pid, p in sorted(an.pings.items()):
            rtt = "" if p["reply"] is None else \
                "%.1f" % ((p["reply"] - p["sent"]) * 1000.0)
            w.writerow([pid, "%.2f" % (p["sent"] - start), rtt,
                        p["res"] or "no reply"])
    with open(os.path.join(log_dir, "wifi_outages.csv"), "w",
              newline="") as f:
        w = csv.writer(f)
        w.writerow(["start_s", "recovered_after_s"])
        for o in r["outages"]:
            w.writerow(["%.1f" % (o["start"] - start),
                        "" if o["end"] is None
                        else "%.1f" % (o["end"] - o["start"])])
    print("\n[test] written to %s" % log_dir)
    return 0 if all(v != "FAIL" for _, v, _ in verdicts) else 1


# ---------------------------------------------------------------------------
# Self-test: a simulated car stream (no broker, no car)
# ---------------------------------------------------------------------------
def simulate(an, seconds, outage, rtt_s, end_in_outage=False):
    """Feed `an` a car that publishes 5 Hz telemetry and a 1 Hz heartbeat,
    answers pings after rtt_s and goes silent during `outage` (start, end);
    the broker reconnect is counted afterwards, as the firmware does."""
    seq, drop, recon = 0, 0, 0
    t, step = 0.0, 0.2
    pending = []
    req_id = 1000
    while t < seconds:
        down = outage[0] <= t < outage[1] or (end_in_outage and
                                              t >= outage[0])
        if not down and t >= outage[1] and recon == 0 and t > outage[0]:
            recon = 1
        if abs(t - round(t)) < 1e-6:                     # every second
            seq += 1
            if down:
                drop += 6                                # the car drops them
            else:
                an.message(t, "heartbeat",
                           {"up": int(t), "seq": seq, "drop": drop,
                            "recon": recon, "rejoin": 0})
            if int(t) % 2 == 0:
                req_id += 1
                an.sent(t, req_id)
                if not down:
                    pending.append((t + rtt_s, req_id))
        if not down:
            an.message(t, "telemetry", {"t": int(t * 1000)})
        for due, rid in [p for p in pending if p[0] <= t]:
            an.message(due, "event", {"type": "cmd", "id": rid,
                                      "ok": 1})
            pending.remove((due, rid))
        an.tick(t)
        t = round(t + step, 6)
    return t


def self_test():
    ok = True

    good = WifiAnalyzer()
    end = simulate(good, 120.0, (40.0, 60.0), 0.03)
    r = good.results(end + 3.0)
    v = dict((n, s) for n, s, _ in good.verdicts(r))
    checks = [
        ("one outage found", len(r["outages"]) == 1),
        ("outage length ~20 s", r["outages"] and
         19.0 <= r["outages"][0]["end"] - r["outages"][0]["start"] <= 21.0),
        ("telemetry ~5 Hz", 4.8 <= r["tel_hz"] <= 5.2),
        ("rtt median 30 ms", r["rtt_med"] is not None
         and abs(r["rtt_med"] - 30.0) < 1.0),
        ("pings into the outage not lost", r["lost"] == 0),
        ("heartbeat gap after the outage ignored", r["hb_missed"] == 0),
        ("drops during the outage ignored", r["drops_connected"] == 0),
        ("reconnect counted", r["recon"] == 1),
        ("all PASS", all(s != "FAIL" for s in v.values())),
    ]

    bad = WifiAnalyzer()
    end = simulate(bad, 60.0, (50.0, 1e9), 0.25, end_in_outage=True)
    r = bad.results(end)
    v = dict((n, s) for n, s, _ in bad.verdicts(r))
    checks += [
        ("unrecovered outage fails", v["every outage recovered (FR11)"]
         == "FAIL"),
        ("250 ms round trip fails",
         v["command round trip (A1, < 100 ms)"] == "FAIL"),
    ]

    short = WifiAnalyzer()                  # 10 s up, then 60 s silent
    end = simulate(short, 70.0, (10.0, 1e9), 0.03, end_in_outage=True)
    r = short.results(end)
    checks.append(("rate stays ~5 Hz in a long open outage",
                   4.5 <= r["tel_hz"] <= 5.5))

    empty = WifiAnalyzer()
    checks.append(("no car = FAIL",
                   empty.verdicts(empty.results(10.0))[0][1] == "FAIL"))
    checks.append(("no car is not shown as up",
                   not empty.results(10.0)["seen"]))

    for name, passed in checks:
        print("%s  %s" % ("ok  " if passed else "FAIL", name))
        ok = ok and passed
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description="PicoCar WiFi / MQTT link test")
    p.add_argument("--mqtt", help="broker address (the laptop running "
                                  "mosquitto)")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--car", default="car1", help="MQTT_CAR_ID of the car")
    p.add_argument("--minutes", type=float, default=10.0,
                   help="test length (design A1: 10 min)")
    p.add_argument("--ping-s", type=float, default=2.0,
                   help="seconds between command round-trip probes")
    p.add_argument("--self-test", action="store_true",
                   help="check this script with a simulated car")
    args = p.parse_args()
    if args.self_test:
        return self_test()
    if not args.mqtt:
        p.error("give --mqtt <broker-ip> (or --self-test)")
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
