"""Station-side raw serial reader for OpenRB-150 UART mapping diagnostics.

Reads raw bytes from the USB-Serial station adapter (e.g. the HC-12-USB bridge)
and prints them. It pairs with ``firmware/uart_port_sweep_probe``: whichever
OpenRB UART the HC-12 TX is wired to is the one whose ``@UART,<n>,TX_TEST`` frames
appear here, so this tool also reports which UART port number(s) it detected.

Read-only: it never writes to the port and sends no motor commands.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import re
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.station_serial import DTR_RTS_MODES, safe_open_serial

# macOS USB-serial bridges enumerate as /dev/cu.usbserial-*; keep a couple of
# common fallbacks for other adapter chipsets.
# macOS exposes both /dev/cu.* and /dev/tty.* for the same device; use cu.* only
# (the callout/non-blocking node) to avoid double-matching one physical adapter.
DEFAULT_PORT_GLOBS = (
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
)

UART_FRAME_RE = re.compile(r"@UART,(\d+),TX_TEST")


def find_usbserial_ports(globs: Iterable[str] = DEFAULT_PORT_GLOBS) -> list[str]:
    """Return matching serial device paths, de-duplicated, in a stable order."""
    found: list[str] = []
    seen: set[str] = set()
    for pattern in globs:
        for path in sorted(glob.glob(pattern)):
            if path not in seen:
                seen.add(path)
                found.append(path)
    return found


def detect_uart_ports(text: str) -> list[int]:
    """Return the sorted set of UART port numbers seen in ``@UART,<n>,TX_TEST``."""
    return sorted({int(match) for match in UART_FRAME_RE.findall(text)})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Raw serial reader for UART-mapping diagnostics. Auto-detects a "
            "/dev/cu.usbserial* device, reads raw bytes, and reports which "
            "@UART,<n>,TX_TEST frames arrived. Read-only; never sends."
        )
    )
    parser.add_argument(
        "--port",
        default=None,
        help="USB-serial device; auto-detected from /dev/cu.usbserial* if omitted",
    )
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--duration-s", type=float, default=10.0, help="How long to read before exiting"
    )
    parser.add_argument(
        "--log-dir",
        default=None,
        help="Optional directory for a timestamped raw capture log",
    )
    parser.add_argument("--dtr", choices=DTR_RTS_MODES, default="low", help="DTR after open")
    parser.add_argument("--rts", choices=DTR_RTS_MODES, default="low", help="RTS after open")
    parser.add_argument("--write-timeout-s", type=float, default=1.0)
    return parser


def _resolve_port(requested: str | None) -> str:
    if requested:
        return requested
    candidates = find_usbserial_ports()
    if not candidates:
        raise SystemExit(
            "No /dev/cu.usbserial* device found. Plug in the HC-12-USB bridge, or "
            "pass --port explicitly (list with: ls /dev/cu.*)."
        )
    if len(candidates) > 1:
        print(f"Multiple serial devices found {candidates}; using {candidates[0]}.")
    return candidates[0]


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    port = _resolve_port(args.port)
    print(
        f"Reading {port} @ {args.baud} for {args.duration_s:.1f}s (raw, read-only; "
        f"dtr={args.dtr} rts={args.rts} rtscts=False dsrdtr=False)."
    )

    log_handle = None
    if args.log_dir:
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"serial_raw_read_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
        log_handle = log_path.open("w", encoding="utf-8")
        print(f"Logging raw capture to {log_path}")

    total_bytes = 0
    captured = ""
    start = time.monotonic()
    ser, ctl_state = safe_open_serial(
        port, args.baud, dtr=args.dtr, rts=args.rts, write_timeout_s=args.write_timeout_s
    )
    if ctl_state.get("warnings"):
        print("control_line_warning " + ",".join(ctl_state["warnings"]))
    try:
        while time.monotonic() - start < args.duration_s:
            chunk = ser.read(256)
            if not chunk:
                continue
            total_bytes += len(chunk)
            text = chunk.decode("ascii", errors="replace")
            captured += text
            sys.stdout.write(text)
            sys.stdout.flush()
            if log_handle is not None:
                log_handle.write(text)
                log_handle.flush()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        if log_handle is not None:
            log_handle.close()

    detected = detect_uart_ports(captured)
    print("\n--- summary ---")
    print(f"total_bytes={total_bytes}")
    print(f"detected_uart_ports={detected if detected else 'none'}")
    if detected:
        names = ", ".join(f"Serial{n}" for n in detected)
        print(f"verdict: HC-12 TX appears wired to {names} (frames received here).")
    elif total_bytes > 0:
        print("verdict: bytes received but no @UART frame; check baud or framing.")
    else:
        print("verdict: 0 bytes. HC-12 is not on a swept UART, or wiring/baud is off.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
