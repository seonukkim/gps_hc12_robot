from __future__ import annotations

import argparse
import datetime as dt
import time
from pathlib import Path

import serial
import _bootstrap  # noqa: F401

from gps_coverage_core.protocol import decode_frame, encode_frame


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe HC-12 heartbeat and telemetry test.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument("--heartbeat-hz", type=float, default=5.0, help="Heartbeat rate in Hz")
    parser.add_argument(
        "--log-dir",
        default="data/hc12_logs",
        help="Directory for raw TX/RX line logs",
    )
    return parser


def log_line(handle, direction: str, data: bytes) -> None:
    stamp = dt.datetime.now(dt.UTC).isoformat()
    handle.write(f"{stamp},{direction},{data.decode('ascii', errors='replace').rstrip()}\n")
    handle.flush()


def print_frame(msg_type: str, seq: int, payload: list[str]) -> None:
    if msg_type in {"GPS", "STAT", "ACK", "ERR"}:
        print(f"{msg_type:<4} seq={seq} payload={payload}")


def main() -> int:
    args = build_parser().parse_args()
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"station_hc12_test_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
    hb_period = 1.0 / args.heartbeat_hz if args.heartbeat_hz > 0 else 0.2

    print(f"Opening {args.port} @ {args.baud}")
    print(f"Logging raw traffic to {log_path}")

    seq = 100
    last_hb = 0.0
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, log_path.open(
        "w", encoding="utf-8"
    ) as log_handle:
        while True:
            try:
                now = time.monotonic()
                if now - last_hb >= hb_period:
                    seq += 1
                    frame = encode_frame("HB", seq, "STATION")
                    ser.write(frame)
                    log_line(log_handle, "TX", frame)
                    last_hb = now

                raw = ser.readline()
                if raw:
                    log_line(log_handle, "RX", raw)
                    try:
                        msg_type, rx_seq, payload = decode_frame(raw)
                    except ValueError as exc:
                        print(f"RX parse error: {exc}: {raw!r}")
                    else:
                        print_frame(msg_type, rx_seq, payload)
            except KeyboardInterrupt:
                stop_frame = encode_frame("CMD", seq + 1, "STOP", "0", "0")
                ser.write(stop_frame)
                log_line(log_handle, "TX", stop_frame)
                print("\nExiting heartbeat test after sending STOP.")
                return 0


if __name__ == "__main__":
    raise SystemExit(main())
