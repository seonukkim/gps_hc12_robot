# Wiring

## HC-12

- HC-12 `TX` -> OpenRB `RX`
- HC-12 `RX` <- OpenRB `TX`
- common `GND` between HC-12, OpenRB, GPS, and power domains

## Voltage / Logic

- Confirm `5V` / `3.3V` logic compatibility before powering the link.
- Do not assume the HC-12 or attached GPS module tolerates every OpenRB UART pin without level validation.

## Notes

- The exact OpenRB-150 UART pin mapping must be confirmed against board documentation before final firmware bring-up.
- Keep antenna placement and motor power wiring separated to reduce noise on the HC-12 and GPS links.
