from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

import _bootstrap  # noqa: F401

from station_controller import DEFAULT_HEARTBEAT_HZ, StationController

MANUAL_TX_HZ = 20.0
AXIS_HOLD_S = 0.35
MAX_FIRST_TEST_SPEED = 0.25
ARROW_TOKENS = {
    b"\x1b[A": "UP",
    b"\x1b[B": "DOWN",
    b"\x1b[C": "RIGHT",
    b"\x1b[D": "LEFT",
}


def max_speed_arg(text: str) -> float:
    value = float(text)
    if value <= 0 or value > MAX_FIRST_TEST_SPEED:
        raise argparse.ArgumentTypeError(
            f"--max-speed must be in (0, {MAX_FIRST_TEST_SPEED:.2f}] for first tests"
        )
    return value


class RawTerminal:
    def __init__(self, fd: int) -> None:
        self._fd = fd
        self._saved: list[int] | None = None

    def __enter__(self) -> "RawTerminal":
        self._saved = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Keyboard manual driving over HC-12.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--max-speed",
        type=max_speed_arg,
        default=MAX_FIRST_TEST_SPEED,
        help="Absolute steering/throttle limit for manual test frames",
    )
    return parser


def read_input(fd: int, buffer: bytearray) -> list[str]:
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        buffer.extend(chunk)

    tokens: list[str] = []
    while buffer:
        if buffer[0] == 0x1B:
            if len(buffer) < 3:
                break
            seq = bytes(buffer[:3])
            del buffer[:3]
            token = ARROW_TOKENS.get(seq)
            if token is not None:
                tokens.append(token)
            continue

        byte = buffer.pop(0)
        if byte == 3:
            tokens.append("CTRL_C")
        elif byte in {10, 13}:
            tokens.append("ENTER")
        elif 32 <= byte <= 126:
            tokens.append(chr(byte))
    return tokens


def render_status(
    *,
    deadman: bool,
    estop: bool,
    steer: float,
    throttle: float,
    seq: int | None,
) -> None:
    sys.stdout.write(
        "\r"
        f"deadman={'ON ' if deadman else 'OFF'} "
        f"estop={'ON ' if estop else 'OFF'} "
        f"steer={steer:+.2f} "
        f"throttle={throttle:+.2f} "
        f"last_seq={seq if seq is not None else 'NA'}    "
    )
    sys.stdout.flush()


def print_instructions(max_speed: float) -> None:
    print(f"Opening keyboard manual control with max speed {max_speed:.2f}")
    print("Controls: WASD or arrow keys drive, space toggles deadman, x sets E-STOP, c clears E-STOP.")
    print("Neutral manual frames are sent continuously. Press n to zero axes, q to quit.")
    print("Terminal key repeat is used for hold-like motion; releasing keys returns to neutral shortly.")


def main() -> int:
    args = build_parser().parse_args()
    controller = StationController(port=args.port, baud=args.baud)
    stdin_fd = sys.stdin.fileno()
    input_buffer = bytearray()
    heartbeat_period = 1.0 / DEFAULT_HEARTBEAT_HZ
    manual_period = 1.0 / MANUAL_TX_HZ

    deadman = False
    estop = False
    steer_cmd = 0.0
    throttle_cmd = 0.0
    steer_until = 0.0
    throttle_until = 0.0
    last_hb = 0.0
    last_manual = 0.0
    last_seq: int | None = None

    print_instructions(args.max_speed)
    try:
        with RawTerminal(stdin_fd):
            while True:
                now = time.monotonic()
                for token in read_input(stdin_fd, input_buffer):
                    lower = token.lower()
                    if token == "CTRL_C" or lower == "q":
                        raise KeyboardInterrupt
                    if token == "UP" or lower == "w":
                        throttle_cmd = args.max_speed
                        throttle_until = now + AXIS_HOLD_S
                    elif token == "DOWN" or lower == "s":
                        throttle_cmd = -args.max_speed
                        throttle_until = now + AXIS_HOLD_S
                    elif token == "LEFT" or lower == "a":
                        steer_cmd = -args.max_speed
                        steer_until = now + AXIS_HOLD_S
                    elif token == "RIGHT" or lower == "d":
                        steer_cmd = args.max_speed
                        steer_until = now + AXIS_HOLD_S
                    elif lower == " ":
                        deadman = not deadman
                    elif lower == "x":
                        estop = True
                        deadman = False
                        steer_until = 0.0
                        throttle_until = 0.0
                    elif lower == "c":
                        estop = False
                    elif lower == "n":
                        steer_until = 0.0
                        throttle_until = 0.0

                steer = steer_cmd if now < steer_until else 0.0
                throttle = throttle_cmd if now < throttle_until else 0.0
                if estop:
                    steer = 0.0
                    throttle = 0.0

                if now - last_hb >= heartbeat_period:
                    controller.send_heartbeat()
                    last_hb = now

                if now - last_manual >= manual_period:
                    last_seq = controller.send_manual(
                        steer,
                        throttle,
                        deadman=deadman and not estop,
                        estop=estop,
                        limit=args.max_speed,
                    )
                    last_manual = now

                for msg_type, seq, payload in controller.poll():
                    print(f"\rRX {msg_type:<4} seq={seq} payload={payload}    ")

                render_status(
                    deadman=deadman and not estop,
                    estop=estop,
                    steer=steer,
                    throttle=throttle,
                    seq=last_seq,
                )
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting manual keyboard control.")
        return 0
    finally:
        print("Sending STOP on exit.")
        for _ in range(5):
            try:
                controller.send_stop()
            except Exception:
                break
            time.sleep(0.05)
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
