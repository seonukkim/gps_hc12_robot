# Arduino UART Reference

The old Arduino-side examples are useful for understanding HC-12 wiring
patterns and simple text framing. They should not be uploaded to the rover
without a new review.

## Reviewed Legacy Arduino Groups

| Source file or group | Side | Baudrate | UART or pin assumption | Useful part | Known issues | Safe to use |
|---|---|---|---|---|---|---|
| `RP2040 Arduino Code.txt` | RP2040-side plus PC-side notes | HC-12 `Serial1` at `9600`; USB console `115200` | RP2040 `Serial1`, noted as GP0/GP1 | Clean USB-to-HC-12 bridge shape | Not OpenRB firmware; bundled with a PC Python chat script; needs board-specific UART verification | Bench reference only |
| `nano-hc12test/nano-hc12test.ino` | Arduino Nano/UNO-side | `9600` | `SoftwareSerial HC12(2, 3)` | Bidirectional serial monitor to HC-12 bridge | Uses blocking `readString()` and no structured protocol | Bench reference only |
| Serial/RF practice folders | Arduino Nano/UNO-side | `9600` | Usually `SoftwareSerial HC12(2, 3)` | Basic HC-12 text exchange and LED demos | Blocking strings, tutorial-specific pins, no failsafe | Bench reference only |
| `RX/RX.ino` and `TX/TX.ino` | Arduino Nano/UNO-side | `9600` | `SoftwareSerial HC12(10, 11)` | One-byte analog-to-PWM demo | Receiver reads twice: stores `val = HC12.read()` but prints a second `HC12.read()`; not robust | Do not copy as-is |
| `nano_hc12test-adc-tx` | Arduino Nano-side plus PC-side | `9600` | `SoftwareSerial HC12(2, 3)` | Sends joystick/ADC comma-separated text | Transmits very quickly; PC parser may expect only two values; no framing checksum | Bench reference only |
| `test02-HC12ad/test02-HC12ad.ino` | OpenRB/Mega-style Arduino-side | `9600` | `Serial3` | Sends comma-separated ADC payloads | Old `Serial3` assumption is not verified for current fixed wiring | Reference only |
| `test-hc12tx/test-hc12tx.ino` | OpenRB/Mega-style Arduino-side | `9600` | `Serial3` | Repeated ADC transmit example | Old `Serial3` assumption; no receiver validation; not tied to current rover safety | Reference only |
| `HC12test4/HC12test4.ino` | OpenRB/Mega-style or RP2040-labeled Arduino-side | `9600` | `Serial3` | Periodic counter transmit over HC-12 UART | Comments mention RP2040 GP0/GP1 but code uses `Serial3`; board assumption is ambiguous | Reference only |
| `OR150-HC12_TEST/OR_HC12_T/OR_HC12_T.ino` | OpenRB-side candidate | `9600` | `Serial3` | Periodic counter transmit over `Serial3` | Does not prove current physical HC-12 wiring; do not change fixed wiring based on this alone | Reference only |
| `OR150-HC12_T3/OR150-HC12_T3.ino` | OpenRB-side candidate | `9600` | `SoftwareSerial HC12(2, 3)` | Shows a simple HC-12 write on serial input | SoftwareSerial support on OpenRB must be compiled/verified first | Reference only |
| `LED_Two/LED_Two.ino` | Arduino Nano/UNO-side | `9600` | `SoftwareSerial HC12(2, 3)`, SET pin on D6 | Shows HC-12 AT/channel command flow | Can change HC-12 configuration; should only be used on an isolated bench | Not safe for rover |
| `arduino_hc12_remote_controller.ino` | Arduino transmitter | `9600` | `SoftwareSerial HC12(10, 11)` | Sends joystick fields as comma-separated text | Compile bug: prints `br` but only `bf` is declared; no explicit frame integrity | Do not run as-is |
| `arduino_rc_robot.ino` | Arduino receiver/robot | `9600` | `SoftwareSerial HC12(10, 11)` | Demonstrates comma-separated joystick parsing | Drives motors directly, no heartbeat/failsafe, no STOP override; one backward log says "Moving Forward" | Unsafe for rover |

## Lessons For This Repository

- The common HC-12 baudrate in old examples is `9600`.
- The common PC-side pattern is line-oriented text over USB serial.
- The common Arduino pattern is either `SoftwareSerial` on Nano/UNO or a board
  hardware UART such as `Serial1`/`Serial3`.
- None of the old UART assumptions override the current fixed wiring: GPS is
  confirmed on OpenRB `Serial2`, and HC-12 wiring remains to be audited.
- Any future HC-12 diagnostic should be receive-only or STOP-only until the
  current rover-side wiring is proven safe.
