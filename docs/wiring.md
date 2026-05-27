# Wiring

## HC-12

- HC-12 `TX` -> OpenRB `RX`
- HC-12 `RX` <- OpenRB `TX`
- common `GND` between HC-12, OpenRB, GPS, and power domains
- HC-12 appears to be mounted under or behind the OpenRB board; verify its
  actual UART wiring separately before changing integrated firmware.
- Fixed Wiring Plan: HC-12 cannot be moved right now. Audit the current wiring
  from code and diagnostics before assuming whether it shares `Serial2` with
  GPS.

## GPS

2026-05-26 probe result:

- GPS connected to the central OpenRB connector.
- That wiring is confirmed as `Serial2` at `9600` baud.
- Probe output showed readable `$GPRMC`, `$GPVTG`, and `$GPGGA` NMEA.
- Final probe output reached `fix=true` with latitude around `35.57107`,
  longitude around `129.1860`, about `5` satellites, and HDOP around
  `1.61-1.62`.
- `Serial3` D13/D14 tests failed because the current GPS wiring was not on
  D13/D14.

GPS must stay on the current central OpenRB connector. Confirmed GPS path:

- GPS physical connector: central OpenRB connector
- GPS UART: `Serial2`
- GPS baud: `9600`

Previous Option A and Option B UART-rewiring plans are superseded by the Fixed
Wiring Plan. Do not move GPS. Do not move HC-12.

## Fixed Wiring Decision Table

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

## IMU / Purple Module

- The purple module appears to be an IMU on an I2C-style connection.
- Do not treat the purple module as UART wiring.
- Do not use it as evidence for HC-12 or GPS serial pin assignment.

Next wiring action:

- Do not rewire GPS or HC-12.
- Audit current HC-12 data-line routing from board inspection, code, and
  receive-only diagnostics where safe.
- Add an integrated GPS `Serial2` diagnostic firmware mode.
- Run receive-only station telemetry testing only if safe.

## Voltage / Logic

- Confirm `5V` / `3.3V` logic compatibility before powering the link.
- Do not assume the HC-12 or attached GPS module tolerates every OpenRB UART pin without level validation.

## Notes

- The exact OpenRB-150 UART pin mapping must be confirmed against board documentation before final firmware bring-up.
- Keep antenna placement and motor power wiring separated to reduce noise on the HC-12 and GPS links.
