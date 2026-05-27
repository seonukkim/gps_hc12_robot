# Legacy HC-12 References

This directory is an audit of the old local HC-12 material from
`~/Desktop/project-lab/hc12`. These files are reference notes only. They are not
production station code and must not be copied into rover firmware without a new
review.

The legacy folder contains useful HC-12 bring-up patterns, but it also contains
hardcoded serial ports, blocking loops, inconsistent variable names, unverified
UART assumptions, and examples that can drive motors. Keep these materials out
of the active rover path until each assumption is re-tested.

## Files In This Reference Directory

| File | What it is | Side | Baudrate | Known issues | Safe to use |
|---|---|---|---|---|---|
| `README.md` | Index and safety classification for the legacy HC-12 audit | Reference documentation | N/A | Summarizes old files, not executable code | Yes, documentation only |
| `pc_readline_reference.md` | Summary of the old Python `pyserial` receive/chat examples | PC-side | Mostly `9600` | Old scripts hardcode `COM4`, `COM8`, `COM10`, or `/dev/cu.usbserial-*`; some are receive-only and some have variable-name bugs | Safe as reference only |
| `arduino_uart_reference.md` | Summary of old Arduino, RP2040, Nano, and OpenRB-style UART examples | Arduino-side, RP2040-side, OpenRB-side | Mostly `9600`; one RP2040 USB console uses `115200` | Old examples use blocking `String` reads, unverified `Serial3`, `SoftwareSerial` pin assumptions, and sometimes motor outputs | Safe as reference only |
| `legacy_source_inventory.md` | Inventory of useful source groups and skipped artifacts from the old folder | Mixed | Mostly `9600` | Does not preserve every archive/video/image; records why some material was excluded | Safe as reference only |

## Source Material Reviewed

Useful legacy material came from these groups under `~/Desktop/project-lab/hc12`:

- PC Python read loops: `hc12_T.py`, `hc12_T2.py`, `test3-HC12.py`,
  `test4-HC12.py`, `test01-HC12py.py`, `test02-HC12py.py`,
  `HC12test4/HC12test4.py`, and `OR150-HC12_TEST/OR_HC12_T/hc12_T2.py`.
- RP2040 bridge notes: `RP2040 Arduino Code.txt`.
- Arduino/Nano HC-12 bridge examples: `nano-hc12test.ino`,
  `nano_hc12test-adc-tx`, `RX/RX.ino`, `TX/TX.ino`, and the serial/RF practice
  folders.
- OpenRB or Mega-style `Serial3` examples: `OR150-HC12_TEST/OR_HC12_T`,
  `HC12test4`, `test02-HC12ad`, and `test-hc12tx`.
- Joystick RC examples: `arduino_hc12_remote_controller.ino` and
  `arduino_rc_robot.ino`.
- HC-12 AT/channel examples: `LED_Two/LED_Two.ino`.

Large archives, videos, images, `.DS_Store`, `desktop.ini`, and zip duplicates
were not copied into this repository. They are not needed for the current fixed
wiring plan.

## Use Rules

- Use these notes to understand old HC-12 baudrate and framing experiments.
- Do not run legacy motor receiver sketches on the rover.
- Do not use old scripts as station production tools without replacing
  hardcoded ports with explicit CLI options.
- Do not assume old `Serial3` examples match the current OpenRB fixed wiring.
- Do not change HC-12 AT settings unless the module is isolated on a bench and
  the expected channel, baudrate, and mode are documented first.
