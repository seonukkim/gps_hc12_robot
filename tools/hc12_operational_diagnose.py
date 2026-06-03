"""Operational HC-12 station-side diagnosis for the current fixed hardware setup.

This is the field-repeatable station tool. It does NOT require unplugging the
HC-12, a direct loopback, or AT mode. It opens the station USB-Serial bridge and
exercises one of four modes, then prints a concise structured status with a
verdict. It is read/write of diagnostic text frames only: no motor commands.

Modes:
  read-only   read raw bytes, print repr(), detect @UART,<n> sweep frames
  write-only  send @STATION,<seq>,TX_TEST repeatedly (check OpenRB sweep RX)
  ping-pong   send PING, read PONG (compatible with tools/hc12_link_probe.py)
  stability   open + poll in_waiting, never send PING, prove the USB bridge alive

Interpretation note: with the counterpart HC-12 side powered off or its firmware
not transmitting, NO_RX / total_bytes=0 is EXPECTED and does not prove a code or
firmware failure. Mark it with --station-off to record an explicitly invalid run.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.protocol import decode_frame, encode_frame
from tools.hc12_link_probe import link_status, ping_frame
from tools.serial_raw_read import detect_uart_ports
from tools.station_serial import DTR_RTS_MODES, control_line_state, safe_open_serial

# Station USB-serial adapters only. /dev/cu.usbmodem* is the OpenRB itself and is
# explicitly excluded so we never read the OpenRB USB-CDC port by mistake.
STATION_PORT_GLOBS = (
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
)
MODES = ("read-only", "write-only", "ping-pong", "stability")


def filter_station_ports(paths: Sequence[str]) -> list[str]:
    """Drop OpenRB (usbmodem) ports and de-duplicate, preserving order."""
    out: list[str] = []
    seen: set[str] = set()
    for path in paths:
        if "usbmodem" in path:  # OpenRB USB-CDC, not the station bridge
            continue
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def find_station_ports(globs: Sequence[str] = STATION_PORT_GLOBS) -> list[str]:
    paths: list[str] = []
    for pattern in globs:
        paths.extend(sorted(glob.glob(pattern)))
    return filter_station_ports(paths)


def classify_serial_error(exc: BaseException) -> str:
    """Classify a serial/OS error into a short stable code (no pyserial needed)."""
    errno = getattr(exc, "errno", None)
    msg = str(exc)
    if errno == 6 or "Errno 6" in msg or "Device not configured" in msg:
        return "DEVICE_NOT_CONFIGURED"
    if errno == 2 or "Errno 2" in msg or "No such file or directory" in msg:
        return "PORT_NOT_FOUND"
    if errno == 5 or "Errno 5" in msg or "Input/output error" in msg:
        return "IO_ERROR"
    if type(exc).__name__ == "SerialException":
        return "SERIAL_EXCEPTION"
    if isinstance(exc, OSError):
        return "OS_ERROR"
    return "UNKNOWN_ERROR"


def diagnose_verdict(
    *,
    mode: str,
    opened: bool,
    serial_error_count: int,
    total_bytes: int,
    detected_uart_ports: Sequence[int],
    pong_rx: int,
    tx_count: int = 0,
    rx_count: int = 0,
    station_off: bool = False,
) -> str:
    """Single-run verdict from observations. Strings match the report vocabulary."""
    if station_off:
        return "TEST_INVALID_STATION_OFF"
    if not opened:
        return "STATION_USB_UNSTABLE"
    if mode == "ping-pong" and pong_rx > 0:
        return "HC12_LINK_OK"
    if detected_uart_ports:
        if 3 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL3"
        if 1 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL1"
        if 2 in detected_uart_ports:
            return "UART_SWEEP_RECEIVED_ON_SERIAL2"
    no_rx = total_bytes == 0 and rx_count == 0
    # A clean write (frames sent, no serial errors) is NOT an unstable USB bridge.
    # It means the station TX path is fine but nothing came back over RF.
    if tx_count > 0 and serial_error_count == 0 and no_rx:
        return "STATION_TX_OK_NO_RX" if mode == "write-only" else "HC12_NO_RF_RX"
    # Genuinely unstable only when nothing ever worked AND errors occurred.
    if serial_error_count > 0 and tx_count == 0 and total_bytes == 0 and not detected_uart_ports:
        return "STATION_USB_UNSTABLE"
    if mode == "write-only":
        return "STATION_WRITE_ONLY_SENT_CHECK_OPENRB_RX"
    if total_bytes > 0:
        return "RF_BYTES_RECEIVED_UNRECOGNIZED"
    if mode in ("stability", "read-only"):
        return "USB_SERIAL_STABLE_NO_RF_BYTES"
    return "HC12_NO_RF_RX"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Operational HC-12 station diagnosis (no unplugging required)."
    )
    parser.add_argument("--port", default="auto", help="'auto' or an explicit device path")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--mode", choices=MODES, default="read-only")
    parser.add_argument("--log-dir", default="outputs/logs")
    parser.add_argument("--raw-log", action="store_true", help="also dump raw received bytes")
    parser.add_argument("--reconnect", action="store_true", help="reopen the port after errors")
    parser.add_argument("--reconnect-delay-s", type=float, default=1.0)
    parser.add_argument(
        "--station-off",
        action="store_true",
        help="mark the counterpart HC-12 side as off/idle -> run is recorded as invalid",
    )
    parser.add_argument("--status-period-s", type=float, default=5.0)
    parser.add_argument(
        "--dtr",
        choices=DTR_RTS_MODES,
        default="low",
        help="DTR line after open: low (default, matches the known-good manual test)",
    )
    parser.add_argument(
        "--rts",
        choices=DTR_RTS_MODES,
        default="low",
        help="RTS line after open: low (default, matches the known-good manual test)",
    )
    parser.add_argument(
        "--write-timeout-s", type=float, default=1.0, help="serial write timeout in seconds"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    ctl = control_line_state(args.dtr, args.rts)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"hc12_operational_diagnose_{stamp}.log"
    raw_path = log_dir / f"hc12_operational_diagnose_{stamp}_raw.bin" if args.raw_log else None
    log_handle = log_path.open("w", encoding="utf-8")
    raw_handle = raw_path.open("wb") if raw_path is not None else None

    def emit(line: str) -> None:
        print(line)
        log_handle.write(line + "\n")
        log_handle.flush()

    emit(
        f"hc12_operational_diagnose mode={args.mode} baud={args.baud} "
        f"duration_s={args.duration_s} reconnect={args.reconnect} station_off={args.station_off} "
        f"dtr_mode={ctl['dtr_mode']} rts_mode={ctl['rts_mode']} rtscts={ctl['rtscts']} "
        f"dsrdtr={ctl['dsrdtr']} write_timeout_s={args.write_timeout_s}"
    )

    # Mutable run state.
    active_port = None
    opened = False
    opened_once = False
    serial_error_count = 0
    reconnect_count = 0
    last_error = "none"
    total_bytes = 0
    tx_count = 0
    rx_count = 0
    pong_rx = 0
    parse_ok = 0
    parse_error = 0
    stability_alive_ticks = 0
    captured = ""
    rx_line = b""
    last_rx_monotonic: float | None = None
    seq = 0
    last_send = 0.0
    last_status = 0.0
    send_period = 1.0

    def status_fields() -> str:
        ports = detect_uart_ports(captured)
        return (
            f"active_port={active_port} total_bytes={total_bytes} tx_count={tx_count} "
            f"rx_count={rx_count} pong_rx={pong_rx} parse_ok={parse_ok} "
            f"parse_error={parse_error} serial_error_count={serial_error_count} "
            f"reconnect_count={reconnect_count} stability_alive_ticks={stability_alive_ticks} "
            f"detected_uart_ports={ports if ports else 'none'} last_error={last_error} "
            f"dtr_mode={ctl['dtr_mode']} rts_mode={ctl['rts_mode']} "
            f"rtscts={ctl['rtscts']} dsrdtr={ctl['dsrdtr']}"
        )

    ser = None
    start = time.monotonic()
    deadline = start + args.duration_s
    try:
        while time.monotonic() < deadline:
            # (Re)open the port as needed.
            if ser is None:
                try:
                    port = args.port
                    if port == "auto":
                        candidates = find_station_ports()
                        if not candidates:
                            raise OSError(2, "No such file or directory (no station port found)")
                        port = candidates[0]
                    ser, ctl_state = safe_open_serial(
                        port,
                        args.baud,
                        dtr=args.dtr,
                        rts=args.rts,
                        write_timeout_s=args.write_timeout_s,
                    )
                    active_port = port
                    if ctl_state.get("warnings"):
                        emit("control_line_warning " + ",".join(ctl_state["warnings"]))
                    if opened_once:
                        reconnect_count += 1
                        emit(f"reconnect ok port={port} reconnect_count={reconnect_count}")
                    else:
                        emit(
                            f"opened port={port} baud={args.baud} dtr_mode={args.dtr} "
                            f"rts_mode={args.rts} rtscts=False dsrdtr=False"
                        )
                    opened = True
                    opened_once = True
                except Exception as exc:  # noqa: BLE001 - field robustness
                    serial_error_count += 1
                    last_error = classify_serial_error(exc)
                    emit(f"open_error {last_error} serial_error_count={serial_error_count}")
                    if args.reconnect:
                        time.sleep(args.reconnect_delay_s)
                        continue
                    break

            # One iteration of work for the selected mode.
            try:
                now = time.monotonic()
                if args.mode in ("write-only", "ping-pong") and now - last_send >= send_period:
                    seq += 1
                    if args.mode == "write-only":
                        ser.write(encode_frame("STATION", seq, "TX_TEST"))
                    else:
                        ser.write(ping_frame(seq))
                    tx_count += 1
                    last_send = now

                if args.mode == "stability":
                    waiting = ser.in_waiting  # may raise OSError 6 if unconfigured
                    stability_alive_ticks += 1
                    if waiting:
                        chunk = ser.read(waiting)
                        total_bytes += len(chunk)
                        if raw_handle is not None:
                            raw_handle.write(chunk)
                    time.sleep(0.5)
                elif args.mode == "write-only":
                    time.sleep(0.05)  # do not read
                else:  # read-only, ping-pong
                    chunk = ser.read(256)
                    if chunk:
                        total_bytes += len(chunk)
                        rx_count += len(chunk)
                        last_rx_monotonic = time.monotonic()
                        if raw_handle is not None:
                            raw_handle.write(chunk)
                        text = chunk.decode("ascii", errors="replace")
                        captured += text
                        if args.mode == "read-only":
                            emit(f"rx repr={chunk!r}")
                        # Parse complete frames for ping-pong accounting.
                        rx_line += chunk
                        while b"\n" in rx_line:
                            raw_line, rx_line = rx_line.split(b"\n", 1)
                            line = raw_line.strip()
                            if not line:
                                continue
                            try:
                                frame = decode_frame(line)
                            except ValueError:
                                parse_error += 1
                            else:
                                parse_ok += 1
                                if frame["type"] == "PONG":
                                    pong_rx += 1
            except Exception as exc:  # noqa: BLE001 - field robustness
                serial_error_count += 1
                last_error = classify_serial_error(exc)
                emit(f"io_error {last_error} serial_error_count={serial_error_count}")
                try:
                    if ser is not None:
                        ser.close()
                except Exception:  # noqa: BLE001
                    pass
                ser = None
                opened = False
                if args.reconnect:
                    time.sleep(args.reconnect_delay_s)
                    continue
                break

            if now - last_status >= args.status_period_s:
                emit("status " + status_fields())
                last_status = now
    except KeyboardInterrupt:
        emit("interrupted")
    finally:
        try:
            if ser is not None:
                ser.close()
        except Exception:  # noqa: BLE001
            pass
        if raw_handle is not None:
            raw_handle.close()

    detected = detect_uart_ports(captured)
    verdict = diagnose_verdict(
        mode=args.mode,
        opened=opened_once,
        serial_error_count=serial_error_count,
        total_bytes=total_bytes,
        detected_uart_ports=detected,
        pong_rx=pong_rx,
        tx_count=tx_count,
        rx_count=rx_count,
        station_off=args.station_off,
    )
    link = link_status(rx_count, None if last_rx_monotonic is None else time.monotonic() - last_rx_monotonic)
    emit("summary " + status_fields())
    emit(f"link_status={link}")
    emit(f"verdict={verdict}")
    log_handle.close()
    print(f"log={log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
