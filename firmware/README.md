# Firmware Notes

- Confirm actual OpenRB-150 UART pin mapping before flashing any sketch.
- Confirm logic-level compatibility for HC-12 and GPS UART wiring.
- Treat all motor / ESC tests as wheel-off-ground only.
- Rover firmware owns low-level safety, RC override, and link-loss stopping behavior.

## Integrated Rover Controller

The active rover sketch is:

```text
firmware/openrb_robot_controller/openrb_robot_controller.ino
```

It prints this USB startup marker when the expected firmware is running:

```text
Firmware: openrb_robot_controller station-manual rc-cardinal-remap 2026-05-26
```

Exact macOS Arduino CLI path used in this repo:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
```

Default build compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Fixed-wiring GPS Serial2 diagnostic compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_DIAG=1' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

In this diagnostic build, GPS uses `Serial2` at `9600`, HC-12 commands are
disabled/ignored to avoid a `Serial2` conflict, and motor outputs are forced
neutral while USB debug reports GPS and RC status.

Latest GPS sky-fix validation:

- `fixed_wiring_gps_serial2_diag=true`
- `hc12_enabled=false`
- `gps_chars` increased continuously
- `gps_fix=true` after moving the external GPS antenna farther outside into
  open sky
- `gps_lat`, `gps_lon`, `gps_sats`, and `gps_hdop` became valid
- motors remained disarmed/neutral

Interpretation checklist:

- `gps_chars` increasing means the GPS UART path is OK.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet.
- Move the antenna outside/open sky before suspecting firmware.
- Protect electronics and antenna connectors from rain even if the GPS can fix
  during rainy open-sky testing.

Unified fixed-wiring RC + GPS autonomy dry-run compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

In this dry-run build, GPS uses `Serial2` at `9600`, HC-12 is disabled/ignored,
RC MANUAL mode keeps current manual driving behavior, and RC AUTO mode forces
neutral motor outputs while printing GPS, placeholder target, distance, bearing,
and readiness fields. This is not real waypoint following.

Expected dry-run USB debug additions:

```text
autonomy_dryrun=true target_lat=35.571120 target_lon=129.186050 target_distance_m=... target_bearing_deg=... gps_ready=... target_ready=... autonomy_ready=...
```

The dry-run distance/bearing helpers are Arduino-side only. Validate them from
USB debug output: with `gps_fix=true`, `target_distance_m` should be finite,
`target_bearing_deg` should remain in `0..360`, and AUTO mode must still keep
`left_cmd=0` and `right_cmd=0`.

When uploading a compile-time variant, upload the matching build directory.

On Linux/WSL station hosts, the upload port may instead look like
`/dev/ttyACM0`. Confirm the actual port before upload.

Full manual-control bring-up notes are in
[`docs/manual_control.md`](../docs/manual_control.md).

## GPS UART Probe

Use this standalone sketch when validating GPS UART wiring and baudrate:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
```

It does not attach motor outputs. Full procedure and per-variant compile/upload
commands are in [`docs/gps_bringup.md`](../docs/gps_bringup.md).

## Serial3 Pin Verification

These standalone sketches are retained as historical/safe UART pin tools:

```text
firmware/pin_finder_test/pin_finder_test.ino
firmware/serial3_loopback_test/serial3_loopback_test.ino
```

Under the Fixed Wiring Plan, do not move GPS or HC-12. They do not attach motor
outputs.
