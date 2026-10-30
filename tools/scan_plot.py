#!/usr/bin/env python3
"""
Plot TEST_ULTRASONIC scans and compare them with the real obstacle (Buddy 5
evidence for REPORT.md section 4.5).

Flash the TEST_ULTRASONIC firmware, press START once per scan and save the
serial console output to a text file (copy it from the terminal, or use the
dashboard's logs/<run>/console.log).  Then:

    python3 tools/scan_plot.py scan.log
    python3 tools/scan_plot.py scan.log --box 60,250,100 --out docs/img

--box X,Y,W[,D] is the tape-measured obstacle: centre X mm to the right of
the sensor (negative = left), front face Y mm ahead of it, width W and depth
D (default 100) in mm.  Each scan is saved as scan_<n>.png and a table of
estimated against true values is printed.

Coordinates match avoidance.c: servo 0 deg = right, 90 = ahead, 180 = left;
x = d cos(a) (right positive), y = d sin(a) (ahead).  Needs matplotlib
(pip install -r tools/dashboard/requirements.txt).
"""

import argparse
import math
import os
import re
import sys

SUMMARY = re.compile(
    r"# found=(\d) closest=(-?\d+)@(-?\d+) edges L=(-?\d+) R=(-?\d+) "
    r"width=(-?\d+) clear L=(-?\d+) R=(-?\d+) -> (\w+) offset=(-?\d+)")
POINT = re.compile(r"^pt,(-?\d+),(-?\d+)\s*$")
CONFIG = os.path.join(os.path.dirname(__file__), "..", "app_program",
                      "car_config.h")


def config_value(name, default):
    """A numeric #define from car_config.h, or the default."""
    try:
        with open(CONFIG, encoding="utf-8") as cfg:
            match = re.search(r"#define\s+%s\s+\(([-\d.]+)f?u?\)" % name,
                              cfg.read())
        return float(match.group(1)) if match else default
    except OSError:
        return default


def read_scans(lines):
    """Scans in the log: summary fields plus their (angle, mm) points."""
    scans = []
    for line in lines:
        line = line.strip()
        summary = SUMMARY.search(line)
        point = POINT.match(line)
        if summary:
            vals = summary.groups()
            scans.append({
                "found": vals[0] == "1", "closest": int(vals[1]),
                "closest_angle": int(vals[2]), "left": int(vals[3]),
                "right": int(vals[4]), "width": int(vals[5]),
                "clear_left": int(vals[6]), "clear_right": int(vals[7]),
                "action": vals[8], "offset": int(vals[9]), "points": []})
        elif point and scans:
            scans[-1]["points"].append((int(point.group(1)),
                                        int(point.group(2))))
    return scans


def to_xy(angle_deg, dist_mm):
    rad = math.radians(angle_deg)
    return dist_mm * math.cos(rad), dist_mm * math.sin(rad)


def true_values(box):
    """Edges, width and closest range of the real box's front face."""
    x, y, w = box[0], box[1], box[2]
    left, right = x - w / 2.0, x + w / 2.0
    nearest_x = 0.0 if left <= 0.0 <= right else min(abs(left), abs(right))
    return left, right, w, math.hypot(nearest_x, y)


def plot_scan(plt, scan, number, box, geometry, out_dir):
    car_w, car_l, corridor = geometry
    fig, axes = plt.subplots(figsize=(6, 6))
    axes.add_patch(plt.Rectangle((-car_w / 2, -car_l), car_w, car_l,
                                 fill=False, color="grey"))
    for side in (-corridor, corridor):
        axes.axvline(side, color="grey", linestyle=":", linewidth=1)
    echoes = [to_xy(a, d) for a, d in scan["points"] if d >= 0]
    misses = [to_xy(a, 600) for a, d in scan["points"] if d < 0]
    if echoes:
        axes.plot([p[0] for p in echoes], [p[1] for p in echoes], "o",
                  color="tab:blue", label="echo")
    for mx, my in misses:
        axes.plot([0, mx], [0, my], color="tab:blue", alpha=0.15)
    if scan["found"]:
        cx, cy = to_xy(scan["closest_angle"], scan["closest"])
        axes.plot([scan["left"], scan["right"]], [cy, cy], "-",
                  color="tab:red", linewidth=3, label="estimated width")
        axes.plot(cx, cy, "x", color="tab:red", markersize=10,
                  label="closest point")
    if box:
        x, y, w, depth = box
        axes.add_patch(plt.Rectangle((x - w / 2, y), w, depth, fill=False,
                                     color="tab:green", linewidth=2,
                                     label="real box"))
    axes.set_title("Scan %d: %s, offset %d mm" % (number, scan["action"],
                                                    scan["offset"]))
    axes.set_xlabel("lateral x (mm, right +)")
    axes.set_ylabel("ahead y (mm)")
    axes.set_aspect("equal")
    axes.set_xlim(-450, 450)
    axes.set_ylim(-car_l - 20, 650)
    axes.grid(alpha=0.3)
    axes.legend(loc="upper right", fontsize=8)
    path = os.path.join(out_dir, "scan_%d.png" % number)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", help="console log ('-' = stdin)")
    parser.add_argument("--box", help="real obstacle X,Y,W[,D] in mm")
    parser.add_argument("--out", default=".", help="folder for the PNGs")
    args = parser.parse_args()

    box = None
    if args.box:
        vals = [float(v) for v in args.box.split(",")]
        if len(vals) not in (3, 4):
            parser.error("--box needs X,Y,W or X,Y,W,D")
        box = vals + [100.0] if len(vals) == 3 else vals

    stream = sys.stdin if args.log == "-" else open(args.log,
                                                   encoding="utf-8")
    with stream:
        scans = read_scans(stream)
    if not scans:
        sys.exit("no '# found=...' summary lines in the log "
                 "(is it TEST_ULTRASONIC output?)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    car_w = config_value("CAR_WIDTH_MM", 160.0)
    car_l = config_value("CAR_LENGTH_MM", 200.0)
    corridor = car_w / 2 + config_value("OBST_MARGIN_MM", 60.0)
    os.makedirs(args.out, exist_ok=True)

    print("scan  found  closest  left  right  width  action")
    if box:
        t_left, t_right, t_width, t_near = true_values(box)
        print("true      -  %7.0f %5.0f %6.0f %6.0f" % (t_near, t_left,
                                                         t_right, t_width))
    for number, scan in enumerate(scans, 1):
        print("%4d  %5s  %7d %5d %6d %6d  %s" % (
            number, "yes" if scan["found"] else "no", scan["closest"],
            scan["left"], scan["right"], scan["width"], scan["action"]))
        if box and scan["found"]:
            print("      error  %+7.0f %+5.0f %+6.0f %+6.0f" % (
                scan["closest"] - t_near, scan["left"] - t_left,
                scan["right"] - t_right, scan["width"] - t_width))
        plot_scan(plt, scan, number, box, (car_w, car_l, corridor), args.out)
    print("saved %d plot(s) in %s" % (len(scans), os.path.abspath(args.out)))


if __name__ == "__main__":
    main()
