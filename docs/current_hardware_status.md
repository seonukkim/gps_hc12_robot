# Current Hardware Status

> 2026-05-27 note: GPS UART receive is now confirmed on `Serial2` at `9600`
> with the GPS connected to the central OpenRB connector. The earlier `Serial3`
> `D13` / `D14` checks failed because the current wiring is not on those pins.
> GPS fix also succeeded on this `Serial2` path, including the integrated
> `FIXED_WIRING_GPS_SERIAL2_DIAG` build with HC-12 disabled and motors neutral.
> Previous Option A and Option B rewiring plans are superseded by the Fixed
> Wiring Plan: GPS cannot be moved, HC-12 cannot be moved, and the current
> wiring must be audited.

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
- GPS FIX: working on the current `Serial2` probe path and in the integrated
  GPS `Serial2` diagnostic build
- Integrated GPS `Serial2` diagnostic build: confirmed `gps_chars` increase,
  `gps_fix=true`, valid latitude/longitude, valid satellites/HDOP, HC-12
  disabled, and motors neutral
- Purple module: appears to be an IMU on an I2C-style connection; not a UART
  path
- Fixed Wiring Plan: GPS remains on the central connector / `Serial2`; HC-12
  remains physically mounted as-is until its current wiring is audited

## Confirmed RC debug state

Latest default firmware restore check:

```text
fixed_wiring_gps_serial2_diag=false
hc12_enabled=true
gps_chars=0
mode=AUTO_READY
auto_sw=true
mode_us≈2000
station_deadman=false
control_source=STOP
```

Interpretation:

- Default `openrb_robot_controller` was restored successfully.
- `gps_chars=0` is expected in the default build under current fixed wiring
  because default firmware still reads GPS from `Serial3` while GPS is fixed on
  `Serial2`.
- Manual driving appears stopped because the RC mode switch is currently in
  AUTO. To validate manual control, switch RC mode out of AUTO and verify
  `control_source=RC_MANUAL`.
- `FIXED_WIRING_GPS_SERIAL2_DIAG` is for GPS testing only; manual driving does
  not work in that diagnostic build by design.

Historical manual-state example:

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
- Next action is auditing current HC-12 wiring, not rewiring either module.

## Latest Integrated GPS Diagnostic State

Observed from `openrb_robot_controller` with
`FIXED_WIRING_GPS_SERIAL2_DIAG=1`:

```text
fixed_wiring_gps_serial2_diag=true
hc12_enabled=false
gps_chars increased continuously
gps_fix=true
gps_lat/gps_lon appeared
gps_sats valid
gps_hdop valid
```

Safety observation:

- Motors remained disarmed/neutral.
- Manual driving does not work in this build by design.

Interpretation:

- The integrated controller can read GPS from the fixed `Serial2` wiring in the
  diagnostic build.
- Outdoor/open-sky placement is required for reliable first fix. Indoor or
  window-side tests can show increasing `gps_chars` while `gps_fix=false`.

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
- HC-12 appears mounted under or behind the OpenRB board and cannot be moved
  right now.
- Whether HC-12 shares GPS `Serial2` is not yet proven.
- Historical `Serial3` D13/D14 wiring notes in older docs must be treated as a
  different wiring setup until revalidated.
- Purple module appears to be I2C-style IMU wiring and should not be treated as
  UART.

## Firmware mapping

- Integrated controller HC-12 serial: `Serial2`
- Integrated controller GPS serial: `Serial3`
- GPS probe confirmed current physical GPS path: `Serial2` at `9600`
- Possible UART allocation conflict: GPS is confirmed on `Serial2`; HC-12 may
  or may not share that path and must be audited before assumptions are made
- Previous Option A and Option B rewiring plans are superseded by the Fixed
  Wiring Plan
- USB debug baud: `115200`

## Fixed Wiring Decision Table

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

## Pending

- Use the integrated GPS `Serial2` diagnostic firmware mode only for GPS USB
  debug with motors neutral and HC-12 ignored.
- Audit current HC-12 wiring from code, board inspection, and non-motion
  diagnostics.
- If safe, run receive-only station telemetry testing.
- Keep station-side path planning dry-run only.
- Future fixed-wiring architecture should support GPS `Serial2` plus RC mode
  switch behavior:
  - Auto OFF: RC manual drive
  - Auto ON: onboard GPS mission/autonomy after separate safety design
  - HC-12 unused in that mode until hardware can be revised or proven
    independent from GPS `Serial2`
- Do not update `openrb_robot_controller` motor behavior, autonomy, STOP,
  heartbeat, failsafe, manual override, or RC safety.
- Station-side HC-12-USB device is not confirmed.
- `/dev/ttyUSB*` is not visible yet on the station/development side.
- Need to confirm whether station HC-12-USB is installed and connected to MPC.
