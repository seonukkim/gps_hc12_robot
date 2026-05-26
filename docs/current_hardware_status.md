# Current Hardware Status

> 2026-05-26 note: GPS UART receive is now confirmed on `Serial2` at `9600`
> with the GPS connected to the central OpenRB connector. The earlier `Serial3`
> `D13` / `D14` checks failed because the current wiring is not on those pins.
> GPS fix also succeeded on this `Serial2` path. The final UART plan is now
> Option A: keep GPS on `Serial2` and move HC-12 to verified `Serial3` pins
> after Serial3 loopback and HC-12 echo testing.

## Confirmed working

- OpenRB-150 USB debug: working
- RC receiver PPM input: working
- RC manual mode: working
- Failsafe STOP behavior: working
- GPS UART receive: working on `Serial2` at `9600` with current central
  connector wiring
- GPS module baudrate: confirmed `9600`
- GPS NMEA output: working; `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, and
  `$GPGLL` observed
- GPS FIX: working on the current `Serial2` probe path
- Purple module: appears to be an IMU on an I2C-style connection; not a UART
  path

## Confirmed RC debug state

Observed from USBDBG:

- `mode=MANUAL`
- `rc_ok=true`
- `auto_sw=false`
- `control_source=RC_MANUAL`
- `steer_norm=0.000`
- `throttle_norm=0.000`
- `left_cmd=0.000`
- `right_cmd=0.000`

## Latest GPS Probe State

Observed from `firmware/gps_uart_probe/gps_uart_probe.ino`:

```text
selected_port=Serial2 baud=9600
chars_1s roughly 350-520
raw_preview contains $GPRMC, $GPVTG, $GPGGA, $GPGSV, $GPGLL
tinygps_chars increases
fix=true
lat around 35.57107
lon around 129.1860
sats around 5
hdop around 1.61-1.62
```

Interpretation:

- GPS UART communication is working.
- GPS satellite fix is working on the `Serial2` probe path.
- GPS should stay on the current central connector.
- Next action is verifying HC-12 on `Serial3`, not more GPS UART probing.

## Historical GPS Debug State

Observed from USBDBG:

- `gps_fix=true`
- `gps_lat≈35.573188`
- `gps_lon≈129.239825`
- `gps_sats=8`
- `gps_hdop≈1.14`

## Current GPS Wiring

| GPS pin | OpenRB-150 pin |
|---|---|
| VCC / UCC | +5V |
| GND | GND |
| TX | central OpenRB connector; confirmed as `Serial2` receive path |
| RX | not connected |
| PPS | not connected |

Current status:

- Central OpenRB connector is confirmed as `Serial2`.
- `Serial3` physical RX/TX pin mapping is unresolved.
- Historical `Serial3` D13/D14 wiring notes in older docs must be treated as a
  different wiring setup until revalidated.
- Purple module appears to be I2C-style IMU wiring and should not be treated as
  UART.

## Firmware mapping

- Integrated controller HC-12 serial: `Serial2`
- Integrated controller GPS serial: `Serial3`
- GPS probe confirmed current physical GPS path: `Serial2` at `9600`
- UART allocation conflict: current GPS wiring and integrated HC-12 both point
  at `Serial2`
- Previous selected resolution Option B is superseded.
- Final target mapping after hardware verification:
  - `GPS_SERIAL=Serial2`
  - `HC12_SERIAL=Serial3`
- USB debug baud: `115200`

## Pending

- Locate actual `Serial3` RX/TX pins using loopback and pin-finder tests.
- Run Serial3 TX-to-RX loopback.
- Move HC-12 data lines to verified `Serial3` RX/TX.
- Run an HC-12 Serial3 echo test.
- Only then update `openrb_robot_controller` mapping.
- Next software milestone: add a non-motion GPS diagnostic integrated firmware
  mode after HC-12 is verified on `Serial3` and GPS remains verified on
  `Serial2`.
- Station-side HC-12-USB device is not confirmed.
- `/dev/ttyUSB*` is not visible yet on the station/development side.
- Need to confirm whether station HC-12-USB is installed and connected to MPC.
