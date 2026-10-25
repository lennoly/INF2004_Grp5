"""Compile and run only Buddy 3 tests; Python 3 + a C99 GCC compiler required."""
from pathlib import Path
import os
import sys
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "app_program"
HOST = Path(__file__).resolve().parent / "host"


def main():
    compiler = os.environ.get("CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("GCC not found. Install GCC or set CC to its executable path.")
    flags = [
        "-std=c99", "-Wall", "-Wextra", "-Wpedantic", "-Wsign-conversion",
        "-Wshadow", "-Wfloat-equal", "-Wdouble-promotion", "-Werror", "-O1",
        f"-I{HOST}", f"-I{APP}",
        *[f"-I{APP / f'subsystem{i}'}" for i in range(1, 6)],
    ]
    sources = [
        HOST / "test_subsystem3.c", HOST / "fakes.c",
        APP / "car_math.c", APP / "subsystem2/pid.c",
        APP / "app_tests.c",
        *sorted((APP / "subsystem3").glob("*.c")),
    ]
    with tempfile.TemporaryDirectory(prefix="picocar-s3-") as folder:
        for mode in (3, 4):
            for polarity in (0, 1):
                binary = Path(folder) / f"s3_{mode}_{polarity}.exe"
                print(f"\nTEST mode={mode} black_is_high={polarity}", flush=True)
                subprocess.run([
                    compiler, *flags, f"-DAPP_MODE={mode}",
                    f"-DIR_BLACK_IS_HIGH={polarity}", *map(str, sources),
                    "-lm", "-o", str(binary),
                ], check=True)
                subprocess.run([str(binary)], check=True)
        print("\nMISSION navigation and telemetry compile checks", flush=True)
        subprocess.run([
            compiler, *flags, "-fsyntax-only", str(APP / "vehicle.c"),
            str(APP / "subsystem1/telemetry.c"),
        ], check=True)
        subprocess.run([
            compiler, *flags, f"-I{ROOT / 'tests/host'}", "-DTM_WIFI_MQTT=1",
            "-fsyntax-only", str(APP / "subsystem1/telemetry.c"),
        ], check=True)
        binary = Path(folder) / "navigation.exe"
        subprocess.run([
            compiler, *flags, "-O2", "-fwhole-program",
            str(HOST / "test_navigation.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
        print("\nBuddy 3 driving integration", flush=True)
        binary = Path(folder) / "driving.exe"
        subprocess.run([
            compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
            "-fwhole-program", "-flto", "-DAPP_MODE=APP_MODE_TEST_BUDDY3",
            f"-I{HOST}", f"-I{APP}",
            *[f"-I{APP / f'subsystem{i}'}" for i in range(1, 6)],
            str(HOST / "test_driving.c"), str(APP / "car_math.c"),
            str(APP / "subsystem2/pid.c"),
            str(APP / "subsystem3/line_follow.c"),
            str(APP / "subsystem3/barcode_decode.c"), "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
        subprocess.run([sys.executable, str(ROOT / "testing/subsystem3/review_probes.py")],
                       check=True)


if __name__ == "__main__":
    main()
