from __future__ import annotations

import argparse
import csv
import datetime as dt
from pathlib import Path

import serial
import _bootstrap  # noqa: F401

from gps_coverage_core.protocol import decode_frame
from gps_coverage_core.telemetry import GPSTelemetry


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Log rover GPS frames to CSV.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--output",
        default=f"data/nmea_logs/gps_frames_{dt.datetime.now():%Y%m%d_%H%M%S}.csv",
        help="CSV output path",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} @ {args.baud}")
    print(f"Logging GPS frames to {output_path}")

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser, output_path.open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["timestamp_utc", "seq", "lat", "lon", "alt_m", "satellites", "hdop", "fix_valid"],
        )
        writer.writeheader()

        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    msg_type, seq, payload = decode_frame(raw)
                except ValueError as exc:
                    print(f"RX parse error: {exc}: {raw!r}")
                    continue
                if msg_type != "GPS":
                    continue

                fix = GPSTelemetry.from_payload(seq, payload)
                row = {"timestamp_utc": dt.datetime.now(dt.UTC).isoformat(), **fix.as_csv_row()}
                writer.writerow(row)
                handle.flush()
                print(
                    f"GPS seq={fix.seq} lat={fix.lat:.6f} lon={fix.lon:.6f} "
                    f"sats={fix.satellites} hdop={fix.hdop:.2f}"
                )
            except KeyboardInterrupt:
                print("\nExiting GPS logger.")
                return 0


if __name__ == "__main__":
    raise SystemExit(main())
