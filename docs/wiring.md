# Wiring

## HC-12

- HC-12 `TX` -> OpenRB `RX`
- HC-12 `RX` <- OpenRB `TX`
- common `GND` between HC-12, OpenRB, GPS, and power domains

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

Integrated firmware currently assigns `Serial2` to HC-12 and `Serial3` to GPS.
The project chose Option B: preserve HC-12 on `Serial2` and move GPS to a
verified physical `Serial3` RX/TX path. Do not leave GPS and HC-12 competing for
`Serial2`.

Next wiring action:

- Locate actual `Serial3` RX/TX pins using loopback and pin-finder tests.
- Move GPS only after `Serial3` RX/TX is physically verified.
- Rerun the GPS probe on `Serial3` at the confirmed `9600` baud.

## Voltage / Logic

- Confirm `5V` / `3.3V` logic compatibility before powering the link.
- Do not assume the HC-12 or attached GPS module tolerates every OpenRB UART pin without level validation.

## Notes

- The exact OpenRB-150 UART pin mapping must be confirmed against board documentation before final firmware bring-up.
- Keep antenna placement and motor power wiring separated to reduce noise on the HC-12 and GPS links.
