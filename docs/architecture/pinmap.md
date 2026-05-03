# Pin Map And Interface Map

This file records the current hardware interface mapping used by the
pre-ROS2/OpenRB prototype. Confirm physical wiring before any powered test.
Motor and ESC tests remain wheel-off-ground only.

## Implemented In Firmware

| Interface | OpenRB / firmware mapping | Baud or signal | Status |
| --- | --- | --- | --- |
| USB debug | `Serial` | `115200` baud | Implemented and used for debug logs |
| HC-12 radio UART | `Serial2` through `HC12_SERIAL` | `9600` baud | Firmware parser implemented; station-side link pending |
| GPS UART | `Serial3` through `GPS_SERIAL` | `9600` baud | GPS communication and FIX verified |
| RC receiver PPM | `PPM_PIN = 6` | PPM pulse input | RC manual verified |
| Left ESC | `ESC_LEFT_PIN = 4` | Servo PWM, neutral `1500 us` | Implemented; wheel-off-ground only |
| Right ESC | `ESC_RIGHT_PIN = 5` | Servo PWM, neutral `1500 us` | Implemented; wheel-off-ground only |

## GPS Wiring

Verified GPS read-only test wiring:

| GPS pin | OpenRB-150 connection | Status |
| --- | --- | --- |
| VCC / UCC | `+5V` | Verified in GPS Serial3 test |
| GND | `GND` | Verified in GPS Serial3 test |
| TX | `D13 / RX` for `Serial3` receive | Verified in GPS Serial3 test |
| RX | Not connected | Read-only GPS test |
| PPS | Not connected | Not used |

Observed GPS test evidence includes `chars_1s > 0` and `status=FIX` in the
Serial3 test notes. Later USB debug status also recorded valid GPS fix fields.

## RC Channel Map

| PPM channel | Firmware use | Firmware index | Status |
| --- | --- | ---: | --- |
| CH1 | Steering joystick horizontal | `0` | Verified |
| CH2 | Throttle joystick vertical | `1` | Verified |
| CH5 | Manual/Auto mode switch | `4` | Verified |
| CH7 | Reserved / unused | none | Not used for mode |

Mode interpretation:

- `CH5 <= 1600 us`: Manual mode.
- `CH5 > 1600 us`: Auto-ready state only.
- CH5 high by itself must not drive motors.
- Explicit station `AUTO` is required before autonomous motor commands are
  accepted.

## HC-12 Wiring Notes

Current documentation lists the intended HC-12 wiring as:

| HC-12 pin | OpenRB side | Status |
| --- | --- | --- |
| TX | OpenRB RX for the selected UART | Pending final board-pin confirmation |
| RX | OpenRB TX for the selected UART | Pending final board-pin confirmation |
| GND | Common ground | Required |

The firmware uses `Serial2` for HC-12. Exact OpenRB-150 UART pin mapping and
logic-level compatibility must be confirmed against board documentation and the
actual module before final powered tests.

## ESC And Motor Safety

Firmware constants:

- neutral pulse: `1500 us`
- range: `+/- 300 us`
- output command clamp: `-1.0..1.0`
- first station manual test cap: `0.25`

Safety notes:

- Motor testing is wheel-off-ground only.
- Startup calls `motorStop()`.
- STOP clears station manual and auto commands.
- RC invalid or link-loss conditions return motor output to neutral.

## Pending Hardware Interfaces

- Magnetic wheel adhesion hardware.
- Cleaning or painting actuator/payload.
- Paint or cleaning-fluid flow control.
- Surface-contact sensing.
- Hull-edge or obstacle sensors.
- Dedicated emergency-stop hardware beyond current firmware command handling.

## Planned Report Use

Use this file to build a concise hardware table in the final report. Keep the
distinction between firmware mapping, verified wiring, and pending physical
integration.
