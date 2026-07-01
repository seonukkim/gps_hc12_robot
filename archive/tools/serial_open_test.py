import argparse
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="/dev/ttyACM0")
parser.add_argument("--baud", type=int, default=9600)
args = parser.parse_args()

print(f"Opening {args.port} @ {args.baud}")

with serial.Serial(args.port, args.baud, timeout=1) as ser:
    print("Opened successfully.")
    ser.write(b"HELLO_FROM_WSL\r\n")
    time.sleep(0.5)
    data = ser.read_all()
    print("RX:", data)
