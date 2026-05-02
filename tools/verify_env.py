from __future__ import annotations

import argparse
import glob
import importlib
import os
import subprocess
import sys

import _bootstrap  # noqa: F401


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print local environment diagnostics.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Expected default serial device")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    print(f"Python: {sys.version.split()[0]}")

    try:
        uv_version = subprocess.run(
            ["uv", "--version"],
            check=False,
            capture_output=True,
            text=True,
        )
        print(f"uv: {uv_version.stdout.strip() or uv_version.stderr.strip() or 'not available'}")
    except FileNotFoundError:
        print("uv: not found")

    print(f"ROS_DISTRO: {os.getenv('ROS_DISTRO', 'unset')}")

    serial_devices = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    print(f"Serial devices: {serial_devices or 'none found'}")
    print(f"Default expected port: {args.port}")

    modules = [
        "serial",
        "pynmea2",
        "matplotlib",
        "pandas",
        "gps_coverage_core.protocol",
        "gps_coverage_core.geo",
        "gps_coverage_core.planner",
    ]
    missing = []
    for module_name in modules:
        try:
            importlib.import_module(module_name)
            print(f"import OK: {module_name}")
        except Exception as exc:  # pragma: no cover - environment diagnostics
            missing.append(module_name)
            print(f"import FAIL: {module_name}: {exc}")

    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
