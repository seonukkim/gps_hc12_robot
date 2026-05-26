# Wiring

## HC-12

- HC-12 `TX` -> OpenRB `RX`
- HC-12 `RX` <- OpenRB `TX`
- common `GND` between HC-12, OpenRB, GPS, and power domains
- HC-12 appears to be mounted under or behind the OpenRB board; verify its
  actual UART wiring separately before changing integrated firmware.
- Final plan: move HC-12 data lines to verified physical `Serial3` RX/TX pins.

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

GPS should stay on the current central OpenRB connector. Final plan:

- `GPS_SERIAL=Serial2`
- `HC12_SERIAL=Serial3`, after Serial3 physical pin verification and HC-12 echo
  test

The previous Option B decision is superseded. Do not leave GPS and HC-12
competing for `Serial2`.

## IMU / Purple Module

- The purple module appears to be an IMU on an I2C-style connection.
- Do not treat the purple module as UART wiring.
- Do not use it as evidence for HC-12 or GPS serial pin assignment.

Next wiring action:

- Locate actual `Serial3` RX/TX pins using loopback and pin-finder tests.
- Run Serial3 TX-to-RX loopback.
- Move HC-12 data lines to verified `Serial3` RX/TX.
- Run an HC-12 Serial3 echo test.
- Only then update `openrb_robot_controller` mapping.

## Voltage / Logic

- Confirm `5V` / `3.3V` logic compatibility before powering the link.
- Do not assume the HC-12 or attached GPS module tolerates every OpenRB UART pin without level validation.

## Notes

- The exact OpenRB-150 UART pin mapping must be confirmed against board documentation before final firmware bring-up.
- Keep antenna placement and motor power wiring separated to reduce noise on the HC-12 and GPS links.
