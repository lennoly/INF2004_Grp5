#!/usr/bin/env python3
"""
Binary MQTT frames of the PicoCar firmware (Change Requirement 1).

Mirror of app_program/subsystem1/frames.c: the same field order and widths,
least significant bit first.  decode() turns a frame into the same dict the
dashboard used for the old JSON messages, so the rest of the tools did not
change.  The field tables are documented in docs/DATA_REPRESENTATION.md.

    kind, obj = decode(payload_bytes)      # "telemetry" / "heartbeat" /
                                           # "event" / "status" / "command"
    frame = command_from_text("#7 pid=0.08,0.6,0")   # ValueError if invalid
    kind, obj = decode_console_line("@1101...")       # serial: '@' + hex
"""

VERSION = 1

TELEMETRY, HEARTBEAT, STATE, BARCODE, HUMP, OBSTACLE, IMPACT, REPLY, \
    GAINS, TIMING, STATUS, COMMAND = range(1, 13)

STATES = ["IDLE", "CALIBRATE", "LINE_FOLLOW", "NAV_TURN", "LINE_SEARCH",
          "OBSTACLE", "STOPPED"]
LINES = ["ON", "LOST", "JUNC"]
NAVS = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"]
EVENTS = ["STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING",
          "TURNING", "CLIMBING", "DESCENDING", "IMPACT"]
ACTIONS = ["CONTINUE", "STOP", "TURN_LEFT", "TURN_RIGHT", "REVERSE"]
LINKS = ["DISABLED", "WAIT_NET", "CONNECTING", "CONNECTED", "BACKOFF"]
WIFI_TEXTS = ["-", "joined", "joined, waiting for an IP address", "joining",
              "hotspot not found (2.4 GHz? name?)", "wrong WiFi password",
              "join failed", "not joined"]
WHY_TEXTS = ["-", "connected", "closed by broker or network (TCP reset)",
             "refused: protocol version", "refused: client id",
             "refused: broker unavailable",
             "refused: bad username or password",
             "refused: not authorised (allow_anonymous true?)",
             "broker did not reply to CONNECT",
             "no answer from broker (firewall? address?)",
             "could not start (bad broker address?)"]
REASONS = ["other", "boot", "start", "stop command", "spin 360", "calibrated",
           "IR contrast too low", "line lost", "line reacquired",
           "line not found / end", "nav done", "nav: line not found",
           "obstacle ahead / impact", "path clear", "obstacle bypassed",
           "bypass: line not found", "path blocked", "LEFT", "RIGHT",
           "STRAIGHT", "UTURN", "fault: motor stall", "fault: encoder",
           "fault: IMU", "fault: sonar no echo"]
FAULTS = ["motor_stall", "encoder", "imu", "no_echo", "wifi_loss"]
COMMANDS = ["start", "stop", "calibrate", "gains", "help", "speed", "pid",
            "ff", "line", "rate", "timing"]
ARG_COUNT = {"start": 0, "stop": 0, "calibrate": 0, "gains": 0, "help": 0,
             "speed": 1, "pid": 3, "ff": 2, "line": 3, "rate": 1,
             "timing": 0}
ERRORS = ["", "invalid command or value", "not a version-1 command frame",
          "only while IDLE or STOPPED", "not available in this build"]
TIMING_NAMES = ["motion", "vehicle", "barcode", "imu", "hump", "ir_isr",
                "net", "rejoin"]
CODE39 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"
RANGE_NONE = 4095
SYMBOL_NONE = 63


# ---------------------------------------------------------------------------
# Bit stream (same as app_program/bitpack.c)
# ---------------------------------------------------------------------------
class Reader:
    def __init__(self, data):
        self.value = int.from_bytes(bytes(data), "little")
        self.bits = 8 * len(data)
        self.pos = 0

    def u(self, bits):
        if self.pos + bits > self.bits:
            raise ValueError("frame too short")
        out = (self.value >> self.pos) & ((1 << bits) - 1)
        self.pos += bits
        return out

    def s(self, bits):
        raw = self.u(bits)
        return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw


class Writer:
    def __init__(self, kind):
        self.value = 0
        self.pos = 0
        self.u(kind, 4)
        self.u(VERSION, 4)

    def u(self, value, bits):
        value = max(0, min(int(round(value)), (1 << bits) - 1))
        self.value |= value << self.pos
        self.pos += bits

    def s(self, value, bits):
        high = (1 << (bits - 1)) - 1
        value = max(-high - 1, min(int(round(value)), high))
        self.u(value & ((1 << bits) - 1), bits)

    def bytes(self):
        return self.value.to_bytes((self.pos + 7) // 8, "little")


def _name(table, index):
    return table[index] if 0 <= index < len(table) else "?"


def _index(table, name):
    return table.index(name) if name in table else 0


def _range(raw):
    return -1 if raw == RANGE_NONE else raw


def _symbol(index):
    return CODE39[index] if index < len(CODE39) else ""


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------
def decode(payload):
    """Frame bytes -> (kind, dict)."""
    data = bytes(payload)
    if not data:
        raise ValueError("empty frame")
    rd = Reader(data)
    ftype, version = rd.u(4), rd.u(4)
    if version != VERSION:
        raise ValueError("frame version %d" % version)
    handler = _DECODERS.get(ftype)
    if handler is None:
        raise ValueError("frame type %d" % ftype)
    return handler(rd)


def decode_console_line(line):
    """'@' + hex from the serial console -> (kind, dict), or None."""
    line = line.strip()
    if not line.startswith("@"):
        return None
    return decode(bytes.fromhex(line[1:]))


def _telemetry(rd):
    o = {"t": rd.u(24), "st": _name(STATES, rd.u(3)),
         "ls": _name(LINES, rd.u(2)), "nav": _name(NAVS, rd.u(3)),
         "pend": _name(NAVS, rd.u(3))}
    o["spd"] = [rd.s(11), rd.s(11)]
    o["tgt"] = [rd.s(11), rd.s(11)]
    o["pwm"] = [rd.s(8), rd.s(8)]
    o["enc"] = [rd.u(16), rd.u(16)]          # low 16 bits: use differences
    o["odo"] = rd.s(20)
    o["hdg"] = rd.s(12)                      # tenths of a degree
    o["ir"] = [rd.u(10), rd.u(10), rd.u(10)]  # permille
    o["le"] = rd.s(11)                       # permille
    o["nbc"] = rd.u(8)
    o["bc"] = _symbol(rd.u(6))
    o["pitch"], o["roll"] = rd.s(11), rd.s(11)
    o["yaw"], o["rate"] = rd.u(12), rd.s(13)
    h = rd.s(12) / 10.0
    pk = rd.u(11) / 10.0
    mx = rd.u(11) / 10.0
    o["hump"] = {"h": h, "pk": pk, "max": mx, "n": rd.u(8)}
    o["ev"] = _name(EVENTS, rd.u(3))
    o["front"] = _range(rd.u(12))
    o["act"] = _name(ACTIONS, rd.u(3))
    o["nobs"] = rd.u(8)
    mask = rd.u(5)
    o["faults"] = [f for i, f in enumerate(FAULTS) if mask & (1 << i)]
    return "telemetry", o


def _heartbeat(rd):
    o = {"up": rd.u(20), "seq": rd.u(16), "st": _name(STATES, rd.u(3)),
         "link": _name(LINKS, rd.u(3)), "pub": rd.u(16), "drop": rd.u(16),
         "rx": rd.u(16), "recon": rd.u(8), "rejoin": rd.u(8),
         "wreset": rd.u(8), "wifi": _name(WIFI_TEXTS, rd.u(3))}
    ip = rd.u(32)
    o["ip"] = ".".join(str((ip >> (8 * i)) & 0xFF) for i in range(4))
    o["tries"] = rd.u(8)
    o["why"] = _name(WHY_TEXTS, rd.u(4))
    o["imu"] = rd.u(1)
    o["i2cerr"] = rd.u(8)
    mask = rd.u(5)
    o["faults"] = [f for i, f in enumerate(FAULTS) if mask & (1 << i)]
    o["fcount"] = [rd.u(4) for _ in FAULTS]
    return "heartbeat", o


def _state(rd):
    return "event", {"type": "state", "st": _name(STATES, rd.u(3)),
                     "why": _name(REASONS, rd.u(5))}


def _barcode(rd):
    sym = rd.u(6)
    return "event", {"type": "barcode", "code": _symbol(sym),
                     "cmd": _name(NAVS, rd.u(3))}


def _hump(rd):
    return "event", {"type": "hump", "n": rd.u(8), "peak_mm": rd.u(11) / 10.0,
                     "max_mm": rd.u(11) / 10.0}


def _obstacle(rd):
    o = {"type": "obstacle", "found": rd.u(1), "dist": _range(rd.u(12)),
         "ang": rd.u(8), "width": rd.u(12), "centre": rd.s(13),
         "cl": rd.u(12), "cr": rd.u(12), "act": _name(ACTIONS, rd.u(3))}
    return "event", o


def _impact(rd):
    return "event", {"type": "impact", "n": rd.u(8)}


def _reply(rd):
    req_id, kind, err = rd.u(16), rd.u(4), rd.u(4)
    o = {"type": "cmd", "id": req_id, "cmd": _name(COMMANDS, kind),
         "ok": 1 if err == 0 else 0}
    if err:
        o["err"] = _name(ERRORS, err)
    return "event", o


def _gains(rd):
    pid = [rd.u(20) / 1000.0 for _ in range(3)]
    ff = [rd.u(20) / 1000.0, rd.u(17) / 1000.0]
    line = [rd.u(20) / 1000.0 for _ in range(3)]
    return "event", {"type": "gains", "pid": pid, "ff": ff, "line": line,
                     "speed": rd.u(9), "rate": rd.u(13)}


def _timing(rd):
    o = {"type": "timing", "win_ms": rd.u(16), "load_pct": rd.u(7),
         "jit_max_us": rd.u(16), "periods": rd.u(16)}
    for name in TIMING_NAMES:
        o[name] = [rd.u(16)]                 # max_us; averages: console
    return "event", o


def _status(rd):
    return "status", "online" if rd.u(1) else "offline"


def _command(rd):
    kind = rd.u(4)
    req_id = rd.u(16)
    name = _name(COMMANDS, kind)
    if name == "speed":
        args = [rd.u(9)]
    elif name == "rate":
        args = [rd.u(13)]
    elif name == "ff":
        args = [rd.u(20) / 1000.0, rd.u(17) / 1000.0]
    elif name in ("pid", "line"):
        args = [rd.u(20) / 1000.0 for _ in range(3)]
    else:
        args = []
    return "command", {"cmd": name, "id": req_id, "args": args}


_DECODERS = {TELEMETRY: _telemetry, HEARTBEAT: _heartbeat, STATE: _state,
             BARCODE: _barcode, HUMP: _hump, OBSTACLE: _obstacle,
             IMPACT: _impact, REPLY: _reply, GAINS: _gains, TIMING: _timing,
             STATUS: _status, COMMAND: _command}


# ---------------------------------------------------------------------------
# Commands: text (as typed on the console) -> frame, with the same limits
# as app_program/subsystem1/command.c
# ---------------------------------------------------------------------------
def parse_text(text):
    """'#7 pid=0.1,0.6,0' -> (name, args, req_id); ValueError if invalid."""
    text = text.strip()
    req_id = 0
    if text.startswith("#"):
        head, _, body = text[1:].partition(" ")
        if not head.isdigit() or not body.strip():
            raise ValueError("bad request id (use #1..#999999999 then a space)")
        req_id, text = int(head), body.strip()
    name, _, rest = text.partition("=")
    name = name.strip().lower()
    if name not in ARG_COUNT:
        raise ValueError("unknown command (try help)")
    try:
        args = [float(v) for v in rest.split(",")] if rest.strip() else []
    except ValueError:
        raise ValueError("bad number")
    if len(args) != ARG_COUNT[name]:
        raise ValueError("wrong number of values")
    if name in ("pid", "line") and any(not 0 <= v <= 1000 for v in args):
        raise ValueError("gain out of range 0..1000")
    if name == "ff" and not (0 <= args[0] <= 1000 and 0 <= args[1] <= 100):
        raise ValueError("ff: kf 0..1000, offset 0..100")
    if name == "speed" and not 60 <= args[0] <= 300:
        raise ValueError("speed: 60..300 mm/s")
    if name == "rate" and not (round(args[0]) == 0
                               or 50 <= round(args[0]) <= 5000):
        raise ValueError("rate: 0 (off) or 50..5000 ms")
    return name, args, req_id


def command_frame(name, args=(), req_id=0):
    wr = Writer(COMMAND)
    wr.u(COMMANDS.index(name), 4)
    wr.u(req_id & 0xFFFF, 16)
    if name == "speed":
        wr.u(args[0], 9)
    elif name == "rate":
        wr.u(args[0], 13)
    elif name == "ff":
        wr.u(args[0] * 1000, 20)
        wr.u(args[1] * 1000, 17)
    elif name in ("pid", "line"):
        for v in args:
            wr.u(v * 1000, 20)
    return wr.bytes()


def command_from_text(text):
    """Typed command -> binary frame for the MQTT cmd topic."""
    name, args, req_id = parse_text(text)
    return command_frame(name, args, req_id)


# ---------------------------------------------------------------------------
# Encoding (used by sim_car.py to publish what the firmware publishes)
# ---------------------------------------------------------------------------
def encode_telemetry(o):
    wr = Writer(TELEMETRY)
    wr.u(o.get("t", 0), 24)
    wr.u(_index(STATES, o.get("st")), 3)
    wr.u(_index(LINES, o.get("ls")), 2)
    wr.u(_index(NAVS, o.get("nav")), 3)
    wr.u(_index(NAVS, o.get("pend")), 3)
    for key, bits in (("spd", 11), ("tgt", 11), ("pwm", 8)):
        for v in o.get(key, [0, 0]):
            wr.s(v, bits)
    for v in o.get("enc", [0, 0]):
        wr.u(int(v) & 0xFFFF, 16)
    wr.s(o.get("odo", 0), 20)
    wr.s(o.get("hdg", 0), 12)
    for v in o.get("ir", [0, 0, 0]):
        wr.u(v, 10)
    wr.s(o.get("le", 0), 11)
    wr.u(o.get("nbc", 0), 8)
    bc = o.get("bc") or ""
    wr.u(CODE39.index(bc[0]) if bc and bc[0] in CODE39 else SYMBOL_NONE, 6)
    wr.s(o.get("pitch", 0), 11)
    wr.s(o.get("roll", 0), 11)
    wr.u(o.get("yaw", 0), 12)
    wr.s(o.get("rate", 0), 13)
    hump = o.get("hump", {})
    wr.s(hump.get("h", 0) * 10, 12)
    wr.u(hump.get("pk", 0) * 10, 11)
    wr.u(hump.get("max", 0) * 10, 11)
    wr.u(hump.get("n", 0), 8)
    wr.u(_index(EVENTS, o.get("ev")), 3)
    front = o.get("front", -1)
    wr.u(RANGE_NONE if front is None or front < 0 else min(front, 4094), 12)
    wr.u(_index(ACTIONS, o.get("act")), 3)
    wr.u(o.get("nobs", 0), 8)
    wr.u(sum(1 << FAULTS.index(f) for f in o.get("faults", [])), 5)
    return wr.bytes()


def encode_heartbeat(o):
    wr = Writer(HEARTBEAT)
    wr.u(o.get("up", 0), 20)
    wr.u(o.get("seq", 0) & 0xFFFF, 16)
    wr.u(_index(STATES, o.get("st")), 3)
    wr.u(_index(LINKS, o.get("link")), 3)
    for key in ("pub", "drop", "rx"):
        wr.u(o.get(key, 0) & 0xFFFF, 16)
    for key in ("recon", "rejoin", "wreset"):
        wr.u(o.get(key, 0), 8)
    wr.u(_index(WIFI_TEXTS, o.get("wifi")), 3)
    parts = [int(p) for p in o.get("ip", "0.0.0.0").split(".")]
    wr.u(sum(p << (8 * i) for i, p in enumerate(parts)), 32)
    wr.u(o.get("tries", 0), 8)
    wr.u(_index(WHY_TEXTS, o.get("why")), 4)
    wr.u(o.get("imu", 0), 1)
    wr.u(o.get("i2cerr", 0), 8)
    wr.u(sum(1 << FAULTS.index(f) for f in o.get("faults", [])), 5)
    for v in o.get("fcount", [0] * len(FAULTS)):
        wr.u(v, 4)
    return wr.bytes()


def encode_event(e):
    t = e.get("type")
    if t == "state":
        wr = Writer(STATE)
        wr.u(_index(STATES, e.get("st")), 3)
        wr.u(_index(REASONS, e.get("why")), 5)
    elif t == "barcode":
        wr = Writer(BARCODE)
        code = e.get("code") or ""
        wr.u(CODE39.index(code[0]) if code and code[0] in CODE39
             else SYMBOL_NONE, 6)
        wr.u(_index(NAVS, e.get("cmd")), 3)
    elif t == "hump":
        wr = Writer(HUMP)
        wr.u(e.get("n", 0), 8)
        wr.u(e.get("peak_mm", 0) * 10, 11)
        wr.u(e.get("max_mm", 0) * 10, 11)
    elif t == "obstacle":
        wr = Writer(OBSTACLE)
        wr.u(e.get("found", 0), 1)
        dist = e.get("dist", -1)
        wr.u(RANGE_NONE if dist < 0 else min(dist, 4094), 12)
        wr.u(e.get("ang", 0), 8)
        wr.u(e.get("width", 0), 12)
        wr.s(e.get("centre", 0), 13)
        wr.u(e.get("cl", 0), 12)
        wr.u(e.get("cr", 0), 12)
        wr.u(_index(ACTIONS, e.get("act")), 3)
    elif t == "impact":
        wr = Writer(IMPACT)
        wr.u(e.get("n", 0), 8)
    elif t == "cmd":
        wr = Writer(REPLY)
        wr.u(e.get("id", 0) & 0xFFFF, 16)
        wr.u(_index(COMMANDS, e.get("cmd")), 4)
        wr.u(0 if e.get("ok") else _index(ERRORS, e.get("err")) or 1, 4)
    elif t == "gains":
        wr = Writer(GAINS)
        for v in e.get("pid", [0, 0, 0]):
            wr.u(v * 1000, 20)
        ff = e.get("ff", [0, 0])
        wr.u(ff[0] * 1000, 20)
        wr.u(ff[1] * 1000, 17)
        for v in e.get("line", [0, 0, 0]):
            wr.u(v * 1000, 20)
        wr.u(e.get("speed", 0), 9)
        wr.u(e.get("rate", 0), 13)
    else:
        raise ValueError("no frame for event type %r" % t)
    return wr.bytes()


def encode_status(online):
    wr = Writer(STATUS)
    wr.u(1 if online else 0, 1)
    return wr.bytes()


if __name__ == "__main__":
    import sys
    for arg in sys.argv[1:]:
        print(decode(bytes.fromhex(arg.lstrip("@"))))
