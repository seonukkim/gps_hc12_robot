"""Safe station-side HC-12 link probe.

Sends simple PING frames over the HC-12-USB bridge and logs every response to a
timestamped file. This tool is for link validation only:

* It only ever emits ``PING`` frames (and an optional ``CLEAR`` on exit).
* It never sends motor-driving frames (no ``CMD,MANUAL`` / ``CMD,AUTO``).

It pairs with ``firmware/hc12_link_probe`` (which answers ``PONG``) or with the
main controller. Frames use the shared ``@TYPE,SEQ,PAYLOAD*CS`` format from
``gps_coverage_core.protocol``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.protocol import decode_frame, encode_frame

LINK_OK_MAX_AGE_S = 3.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Safe HC-12 link probe. Sends PING frames and logs responses. "
            "Never sends motor commands."
        )
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="USB serial device for the HC-12-USB bridge (e.g. /dev/ttyUSB0 or /dev/cu.usbserial-*)",
    )
    parser.add_argument("--baud", type=int, default=9600, help="HC-12 serial baudrate")
    parser.add_argument(
        "--ping-interval-ms", type=int, default=1000, help="Milliseconds between PING frames"
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Optional run duration in seconds; runs until Ctrl-C if omitted",
    )
    parser.add_argument(
        "--log-dir", default="outputs/logs", help="Directory for the timestamped probe log"
    )
    return parser


def ping_frame(seq: int, payload: str = "PROBE") -> bytes:
    """Build a single PING frame. PING is the only frame type this tool sends."""
    return encode_frame("PING", seq, payload)


def link_status(rx_count: int, last_rx_age_s: float | None) -> str:
    """Summarize link health from receive activity."""
    if rx_count == 0 or last_rx_age_s is None:
        return "NO_RX_YET"
    return "LINK_OK" if last_rx_age_s <= LINK_OK_MAX_AGE_S else "LINK_STALE"


def _log_line(handle, direction: str, data: bytes) -> None:
    stamp = dt.datetime.now(dt.UTC).isoformat()
    text = data.decode("ascii", errors="replace").rstrip()
    handle.write(f"{stamp},{direction},{text}\n")
    handle.flush()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    import serial  # imported here so --help works without pyserial installed

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"hc12_link_probe_{dt.datetime.now():%Y%m%d_%H%M%S}.log"

    ping_period_s = max(args.ping_interval_ms, 1) / 1000.0
    print(f"Opening {args.port} @ {args.baud} for HC-12 link probe (PING only).")
    print(f"Logging to {log_path}")

    seq = 0
    tx_count = 0
    rx_count = 0
    parse_ok = 0
    parse_err = 0
    pong_rx = 0
    last_rx_monotonic: float | None = None
    last_ping = 0.0
    start = time.monotonic()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, log_path.open(
        "w", encoding="utf-8"
    ) as log_handle:
        try:
            while True:
                now = time.monotonic()
                if args.duration_s is not None and now - start >= args.duration_s:
                    break

                if now - last_ping >= ping_period_s:
                    seq += 1
                    frame = ping_frame(seq)
                    ser.write(frame)
                    _log_line(log_handle, "TX", frame)
                    tx_count += 1
                    last_ping = now

                raw = ser.readline()
                if raw:
                    _log_line(log_handle, "RX", raw)
                    rx_count += 1
                    last_rx_monotonic = time.monotonic()
                    try:
                        frame = decode_frame(raw)
                    except ValueError as exc:
                        parse_err += 1
                        print(f"RX parse_error: {exc}: {raw!r}")
                    else:
                        parse_ok += 1
                        if frame["type"] == "PONG":
                            pong_rx += 1
                        print(
                            f"RX type={frame['type']} seq={frame['seq']} "
                            f"payload={frame['payload']}"
                        )

                age = None if last_rx_monotonic is None else time.monotonic() - last_rx_monotonic
                age_text = "NA" if age is None else f"{age * 1000:.0f}"
                print(
                    f"tx_count={tx_count} rx_count={rx_count} pong_rx={pong_rx} "
                    f"parse_ok={parse_ok} parse_error={parse_err} "
                    f"last_rx_age_ms={age_text} link_status={link_status(rx_count, age)}",
                    end="\r",
                    flush=True,
                )
        except KeyboardInterrupt:
            print("\nStopping HC-12 link probe.")

        # Best-effort tidy frame; CLEAR carries no motor command.
        seq += 1
        clear_frame = encode_frame("CLEAR", seq, "PROBE")
        ser.write(clear_frame)
        _log_line(log_handle, "TX", clear_frame)

    print(f"\nDone. tx_count={tx_count} rx_count={rx_count} pong_rx={pong_rx} log={log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
