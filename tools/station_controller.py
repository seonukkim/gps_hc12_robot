from __future__ import annotations

import argparse
import datetime as dt
import time
from pathlib import Path

import serial
import _bootstrap  # noqa: F401

from gps_coverage_core.protocol import (
    clamp_unit,
    decode_frame,
    encode_frame,
    format_command_float,
    manual_command_fields,
)

DEFAULT_HEARTBEAT_HZ = 5.0
DEFAULT_STOP_PERIOD = 1.0


class StationController:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud: int = 9600,
        *,
        timeout: float = 0.2,
        log_dir: str = "data/hc12_logs",
    ) -> None:
        self._seq = 100
        self._serial = serial.Serial(port, baud, timeout=timeout)
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        self.log_file = (
            log_path / f"station_controller_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
        ).open("w", encoding="utf-8")

    def close(self) -> None:
        self.log_file.close()
        self._serial.close()

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _log(self, direction: str, data: bytes) -> None:
        stamp = dt.datetime.now(dt.UTC).isoformat()
        self.log_file.write(
            f"{stamp},{direction},{data.decode('ascii', errors='replace').rstrip()}\n"
        )
        self.log_file.flush()

    def _send_frame(self, msg_type: str, *fields: object) -> int:
        seq = self._next_seq()
        frame = encode_frame(msg_type, seq, *fields)
        self._serial.write(frame)
        self._log("TX", frame)
        return seq

    def send_heartbeat(self) -> int:
        return self._send_frame("HB", "STATION")

    def send_stop(self) -> int:
        return self._send_frame("CMD", "STOP", "0", "0")

    def send_auto(self, left: float | int = 0, right: float | int = 0) -> int:
        return self._send_frame(
            "CMD",
            "AUTO",
            format_command_float(clamp_unit(float(left))),
            format_command_float(clamp_unit(float(right))),
        )

    def send_manual(
        self,
        steer: float,
        throttle: float,
        *,
        deadman: bool,
        estop: bool = False,
        limit: float = 1.0,
    ) -> int:
        return self._send_frame(
            "CMD",
            *manual_command_fields(steer, throttle, deadman, estop, limit=limit),
        )

    def send_cmd(self, command: str, *fields: object) -> int:
        return self._send_frame("CMD", command, *fields)

    def poll(self) -> list[tuple[str, int, list[str]]]:
        decoded: list[tuple[str, int, list[str]]] = []
        while True:
            raw = self._serial.readline()
            if not raw:
                break
            self._log("RX", raw)
            try:
                decoded.append(decode_frame(raw))
            except ValueError as exc:
                print(f"RX parse error: {exc}: {raw!r}")
        return decoded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe station controller loop.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--heartbeat-hz",
        type=float,
        default=DEFAULT_HEARTBEAT_HZ,
        help="Heartbeat rate in Hz",
    )
    parser.add_argument(
        "--stop-period",
        type=float,
        default=DEFAULT_STOP_PERIOD,
        help="STOP cadence in seconds",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    hb_period = 1.0 / args.heartbeat_hz if args.heartbeat_hz > 0 else 1.0 / DEFAULT_HEARTBEAT_HZ
    controller = StationController(port=args.port, baud=args.baud)
    print(f"Running safe controller on {args.port} @ {args.baud}")
    print("Default behavior sends heartbeat and STOP only.")

    last_hb = 0.0
    last_stop = 0.0
    try:
        while True:
            now = time.monotonic()
            if now - last_hb >= hb_period:
                controller.send_heartbeat()
                last_hb = now
            if now - last_stop >= args.stop_period:
                controller.send_stop()
                last_stop = now

            for msg_type, seq, payload in controller.poll():
                print(f"{msg_type:<4} seq={seq} payload={payload}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCtrl+C received; sending STOP five times.")
        for _ in range(5):
            controller.send_stop()
            time.sleep(0.1)
        return 0
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
