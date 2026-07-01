from __future__ import annotations

import argparse
import sys
import time

import serial


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive HC-12 AT terminal.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial device")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baudrate")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    print(f"Opening {args.port} @ {args.baud} for HC-12 AT commands only.")
    print("This tool refuses rover protocol frames and should not be used for motor commands.")

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        while True:
            try:
                command = input("AT> ").strip()
            except EOFError:
                return 0
            except KeyboardInterrupt:
                print()
                return 0

            if not command:
                continue
            if command.startswith("@"):
                print("Refusing rover protocol frame input in AT mode.")
                continue
            if not command.upper().startswith("AT"):
                print("HC-12 AT mode expects commands beginning with AT.")
                continue

            ser.write((command + "\r\n").encode("ascii"))
            time.sleep(0.2)
            response = ser.read_all()
            if response:
                sys.stdout.write(response.decode("ascii", errors="replace"))
                if not response.endswith(b"\n"):
                    sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
