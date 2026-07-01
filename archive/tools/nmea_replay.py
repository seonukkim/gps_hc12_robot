from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import _bootstrap  # noqa: F401

from gps_coverage_core.nmea import iter_nmea_file
from gps_coverage_core.protocol import encode_frame


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay NMEA or parsed GPS logs offline.")
    parser.add_argument("input", help="Path to .nmea/.txt or CSV log")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Unused placeholder for CLI consistency")
    parser.add_argument("--interval", type=float, default=0.2, help="Delay between samples in seconds")
    parser.add_argument("--emit-frames", action="store_true", help="Emit protocol-formatted GPS frames")
    return parser


def replay_csv(path: Path, emit_frames: bool, interval: float) -> None:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader, start=1):
            lat = float(row["lat"])
            lon = float(row["lon"])
            alt_m = float(row.get("alt_m", 0.0) or 0.0)
            sats = int(row.get("satellites", 0) or 0)
            hdop = float(row.get("hdop", 99.9) or 99.9)
            fix_valid = int(row.get("fix_valid", 1) or 1)
            print(f"CSV {index:04d}: lat={lat:.6f} lon={lon:.6f} sats={sats} hdop={hdop:.2f}")
            if emit_frames:
                frame = encode_frame("GPS", 500 + index, lat, lon, alt_m, sats, hdop, fix_valid)
                print(frame.decode("ascii").rstrip())
            time.sleep(interval)


def replay_nmea(path: Path, emit_frames: bool, interval: float) -> None:
    for index, parsed in enumerate(iter_nmea_file(path), start=1):
        lat = parsed.point.lat
        lon = parsed.point.lon
        alt_m = parsed.altitude_m if parsed.altitude_m is not None else 0.0
        sats = parsed.satellites if parsed.satellites is not None else 0
        hdop = parsed.hdop if parsed.hdop is not None else 99.9
        fix_valid = int(lat != 0.0 or lon != 0.0)
        print(f"{parsed.sentence_type} {index:04d}: lat={lat:.6f} lon={lon:.6f} sats={sats} hdop={hdop:.2f}")
        if emit_frames:
            frame = encode_frame("GPS", 500 + index, lat, lon, alt_m, sats, hdop, fix_valid)
            print(frame.decode("ascii").rstrip())
        time.sleep(interval)


def main() -> int:
    args = build_parser().parse_args()
    path = Path(args.input)
    if not path.exists():
        raise FileNotFoundError(path)

    if path.suffix.lower() == ".csv":
        replay_csv(path, emit_frames=args.emit_frames, interval=args.interval)
    else:
        replay_nmea(path, emit_frames=args.emit_frames, interval=args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
