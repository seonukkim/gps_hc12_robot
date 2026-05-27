# PC Readline Reference

The legacy PC-side material is useful mainly because it confirms the original
HC-12 experiments used simple `pyserial` loops at `9600` baud. It should not be
treated as production station code.

## Useful Pattern

The common pattern is:

```python
import serial

ser = serial.Serial(port="COM4", baudrate=9600, timeout=0.1)

while True:
    if ser.in_waiting > 0:
        line = ser.readline()
        print(line)
```

For this repository, production station tools should improve that pattern by
using an explicit `--port`, escaping raw bytes safely, handling decode errors,
and exiting cleanly on shutdown.

## Reviewed Legacy Scripts

| Source file | Side | Baudrate | Port assumption | Useful part | Known issues | Safe to use |
|---|---|---|---|---|---|---|
| `hc12_T.py` | PC-side | `9600` | `COM4` | Basic receive loop using `readline()` | Chat transmit section is commented out; hardcoded Windows port; no CLI; infinite loop | Reference only |
| `hc12_T2.py` | PC-side | `9600` | `COM4` | Receive loop with `in_waiting` and UTF-8 decode handling | Hardcoded Windows port; no transmit path; infinite loop | Reference only |
| `test3-HC12.py` | PC-side with embedded RP2040 notes | `9600` | `COM4` | Basic `readline()` receive loop | Mixes Python and Arduino notes in one file; hardcoded port | Reference only |
| `test4-HC12.py` | PC-side with embedded RP2040 notes | `9600` | `/dev/cu.usbserial-024446CC` | macOS USB-serial receive example | Hardcoded adapter path; no clean shutdown; embedded Arduino notes | Reference only |
| `test01-HC12py.py` | PC-side | `9600` | `COM4` | Simple `readline()` receiver after startup delay | Contains commented code with stale variable names; hardcoded port | Reference only |
| `test02-HC12py.py` | PC-side | `9600` | `COM4` | Minimal receive/print loop | Hardcoded port; blocks forever; no decode guard | Reference only |
| `test02-HC12ad/test02-HC12py.py` | PC-side | `9600` | `COM8` | Receives ADC lines from a transmitter | Hardcoded port; raw bytes only; embedded old Arduino note | Reference only |
| `nano_hc12test-adc-tx/nano_hc12test-adc-tx.py` | PC-side | `9600` | `COM10` | Parses comma-separated joystick/ADC values | Parser assumes exactly two integers while embedded Arduino note can emit more fields; no robust framing | Reference only |
| `HC12test4/HC12test4.py` | PC-side | `9600` | `COM4` | Intended receive-only HC-12 USB script | Bug: opens `HC12USB` but prints `HC12.is_open`; would fail as written | Do not run as-is |
| `OR150-HC12_TEST/OR_HC12_T/hc12_T2.py` | PC-side | `9600` | `COM4` | Receive loop matching an old `Serial3` transmitter | Same hardcoded-port and infinite-loop limitations | Reference only |
| `HC12python.txt` and `HC12arduino.txt` | PC/Raspberry Pi style | `9600` | `/dev/ttyAMA0` | Shows Linux UART serial settings | Syntax issue: missing comma before `timeout`; both files are Python despite one name saying Arduino; send and receive loops are separate examples jammed together | Do not run as-is |

## Current Project Guidance

- Keep the active station tools under `tools/` and expose `--port`.
- Default station behavior must stay heartbeat/`STOP` only.
- Use receive-only scripts for HC-12 bring-up until the current rover-side HC-12
  wiring is audited.
- Avoid old hardcoded ports such as `COM4` and `/dev/cu.usbserial-*` in active
  code.
