# Current Hardware Status

## Confirmed working

- OpenRB-150 USB debug: working
- RC receiver PPM input: working
- RC manual mode: working
- Failsafe STOP behavior: working
- GPS module: working
- GPS FIX: confirmed

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

## Confirmed GPS debug state

Observed from USBDBG:

- `gps_fix=true`
- `gps_lat≈35.573188`
- `gps_lon≈129.239825`
- `gps_sats=8`
- `gps_hdop≈1.14`

## Confirmed GPS wiring

| GPS pin | OpenRB-150 pin |
|---|---|
| VCC / UCC | +5V |
| GND | GND |
| TX | D13 / RX |
| RX | not connected |
| PPS | not connected |

## Firmware mapping

- GPS serial: `Serial3`
- GPS baud: `9600`
- USB debug baud: `115200`

## Pending

- Station-side HC-12-USB device is not confirmed.
- `/dev/ttyUSB*` is not visible yet on the station/development side.
- Need to confirm whether station HC-12-USB is installed and connected to MPC.
