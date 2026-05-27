# Legacy Source Inventory

This inventory records what was useful from `~/Desktop/project-lab/hc12` and
what was intentionally excluded from the repository.

## Useful Summarized Material

| Legacy material | Category | Why it is useful | Why it was summarized instead of copied |
|---|---|---|---|
| `hc12_T.py`, `hc12_T2.py`, `test3-HC12.py`, `test4-HC12.py`, `test01-HC12py.py`, `test02-HC12py.py` | PC-side PySerial examples | Confirms `9600` baud and simple `readline()` receive loops | Hardcoded ports, infinite loops, no CLI, and mixed notes |
| `HC12test4/HC12test4.py` | PC-side PySerial example | Shows intended receive-only USB-serial use | Contains a variable-name bug and should not be run as-is |
| `RP2040 Arduino Code.txt` | RP2040 bridge plus PC chat notes | Shows USB console to HC-12 bridge at `Serial1`/`9600` | Mixed Arduino and Python in one file; not OpenRB firmware |
| `nano-hc12test.ino` and practice serial/RF folders | Arduino Nano/UNO bridge examples | Shows basic HC-12 text bridge at `9600` | Tutorial pins and blocking `String` reads are not suitable for rover code |
| `RX/RX.ino` and `TX/TX.ino` | Arduino Nano/UNO one-byte examples | Shows minimal analog byte transmit and PWM receive | Receiver has a read-twice bug; no framing or safety |
| `test02-HC12ad`, `test-hc12tx`, `HC12test4`, `OR150-HC12_TEST/OR_HC12_T` | OpenRB/Mega-style UART examples | Shows old `Serial3` transmit experiments | Current fixed wiring cannot assume HC-12 is on `Serial3` |
| `arduino_hc12_remote_controller.ino` and `arduino_rc_robot.ino` | Joystick RC examples | Shows a comma-separated joystick protocol idea | Transmitter has a compile bug; receiver directly drives motors with no rover safety model |
| `LED_Two/LED_Two.ino` | HC-12 AT/channel example | Shows SET pin command flow | Can change radio settings and should stay isolated from rover bring-up |

## Excluded Material

| Legacy material | Reason excluded |
|---|---|
| Zip archives | Duplicate source or archived tutorials; not needed in repo |
| MP4 tutorial video | Large binary reference; not needed for active project workflow |
| JPG schematic image | Binary reference; current wiring must be documented in `docs/wiring.md` instead |
| `.DS_Store` and `desktop.ini` | OS metadata |
| Tutorial duplicates | Repeated variants of the same `SoftwareSerial` bridge pattern |

## Audit Result

The old material is valuable for HC-12 bring-up history, especially the repeated
`9600` baud assumption and simple line-based testing style. It is not a stable
software baseline. Active code should continue to live in this repository's
`tools/`, `firmware/`, and `gps_coverage_core/` areas, with explicit safety
defaults and documented wiring assumptions.
