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

## Firmware Modes

| Mode | Compile flag | Purpose | GPS | HC-12 | Motors |
|---|---|---|---|---|---|
| Default `openrb_robot_controller` | none | HC-12/manual legacy mode | default firmware reads `Serial3`; current fixed GPS wiring is not available here | enabled | normal safety-gated behavior |
| GPS-only diagnostic | `FIXED_WIRING_GPS_SERIAL2_DIAG=1` | fixed GPS `Serial2` debug over USB | `Serial2` at `9600` | disabled/ignored | forced neutral |
| MANUAL RC + AUTO GPS dry-run | `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1` | RC manual driving plus AUTO GPS distance/bearing computation | `Serial2` at `9600` | disabled/ignored | MANUAL can drive; AUTO forced neutral |
| Single-waypoint experiment | `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1` | guarded one-target candidate-command experiment | `Serial2` at `9600` | disabled/ignored | MANUAL can drive; AUTO is neutral unless `AUTO_MOTION_ARMED=1` |

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

Completed unified dry-run validation:

- Running build identified by USBDBG:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`.
- MANUAL mode: `mode=MANUAL`, `auto_sw=false`,
  `control_source=RC_MANUAL`, and RC stick input changes manual command and
  left/right command fields.
- AUTO mode: `mode=AUTO_READY`, `auto_sw=true`,
  `control_source=STOP`, `left_cmd=0.000`, and `right_cmd=0.000`.
- GPS: `gps_chars` increases continuously, `gps_fix=true` appears with
  open-sky antenna placement, and target distance/bearing fields are computed.
- This build still has no autonomous motor output.

Single-waypoint experiment compile/upload/monitor with motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Compile with the nearby target override and motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_TARGET_LAT=35.5716800 -DSINGLE_WP_TARGET_LON=129.1866516' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected single-waypoint USB debug additions:

```text
single_waypoint_experiment=true target_override_enabled=... target_source=... target_lat_macro=... target_lon_macro=... auto_motion_armed=false auto_motor_inhibit=true gps_ready=... target_ready=... timeout_ok=... max_target_distance_m=30.0 arrival_radius_m=2.5 distance_allowed=... safety_ready=... arrived=... target_lat=... target_lon=... target_distance_m=... target_bearing_deg=... candidate_left_cmd=... candidate_right_cmd=... final_left_cmd=0.000 final_right_cmd=0.000
```

Target override rule:

- Runtime USBDBG `target_lat` and `target_lon` are the source of truth.
- Verify those fields before interpreting `target_distance_m`,
  `distance_allowed`, `safety_ready`, or candidate command values.
- With override macros provided, USBDBG should print
  `target_override_enabled=true`, `target_source=compile_time`,
  `target_lat_macro=35.5716800`, `target_lon_macro=129.1866516`,
  `target_lat=35.571680`, and `target_lon=129.186652`.
- Without override macros, USBDBG should print `target_override_enabled=false`
  and `target_source=fallback`.
- The previous nearby target run was safe because `AUTO_MOTION_ARMED=0` kept
  final outputs at zero, but it was not a successful nearby candidate-command
  test because runtime target fields still showed the old placeholder.
- Latest check: target override plumbing is verified with
  `SINGLE_WP_TARGET_LAT=35.5710210` and
  `SINGLE_WP_TARGET_LON=129.1864016`; USBDBG printed
  `target_override_enabled=true`, `target_source=compile_time`, matching macro
  strings, and runtime `target_lat=35.571021`, `target_lon=129.186402`.
- That run was still blocked because current GPS was about `380` to `392` m
  away, greater than `max_target_distance_m=30.0`, so `distance_allowed=false`
  and `safety_ready=false` were expected. Recompute a nearby target from the
  current GPS position before the next inhibited run.
- Next-day retest: target override still worked, but GPS moved to approximately
  `35.571310,129.188630` while the previous target remained
  `35.567560,129.186792`, making `target_distance_m≈448.9`.
  `distance_allowed=false` and `safety_ready=false` were expected.
- `gps_fix=true` alone is not enough; check `gps_age_ms`, `gps_hdop`, and
  `gps_sats` before treating GPS as ready.

Safety gates:

- GPS location valid.
- GPS age no more than `SINGLE_WAYPOINT_GPS_STALE_MS` (`2000` ms).
- HDOP valid and no more than `SINGLE_WAYPOINT_MAX_HDOP` (`2.5`).
- Target available.
- RC input valid.
- RC AUTO switch on.
- Target distance above `SINGLE_WAYPOINT_ARRIVAL_RADIUS_M` (`2.5` m).
- Target distance no more than `SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M` (`30` m).
- AUTO state age no more than `SINGLE_WAYPOINT_AUTO_TIMEOUT_MS` (`15000` ms).

`AUTO_MOTION_ARMED=1` is reserved for a later explicit wheel-off-ground bench
test. The current inhibited build computes candidate commands but forces final
left/right outputs to zero. This mode does not load `mission.json`, does not
run multi-waypoint missions, and does not implement coverage/lawnmower driving.
Candidate commands are straight low-speed placeholders; target bearing is
printed for inspection, but heading control is not implemented yet.

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

## I2C Scanner Test

Use this standalone sketch to verify whether a device responds on the OpenRB
default hardware `Wire` I2C bus:

```text
firmware/i2c_scanner_test/i2c_scanner_test.ino
```

OpenRB-150 variant files confirm the current fixed IMU wiring matches the
board's default `Wire` pins:

- Arduino D11 = SDA = PA08
- Arduino D12 = SCL = PA09
- `PIN_WIRE_SDA = 11`
- `PIN_WIRE_SCL = 12`

It uses `Wire`, USB Serial at `115200`, scans addresses `0x03` through `0x77`,
and does not attach motors or Servo outputs. The scanner prints startup lines
before `Wire.begin()`, reads D11/D12 pullup states before and after
`Wire.begin()`, and prints `scan_pass`, `found_count`, found addresses, and
`stable_valid_address` every 2 seconds. Addresses such as `0x68`, `0x69`, or
`0x76` are common for some IMU/sensor modules, but address alone is not a
device identification.

A valid IMU result requires one stable address. If all addresses or more than 8
addresses are found, treat the pass as invalid (`INVALID_SCAN_TOO_MANY_ADDRESSES`)
and do not treat it as success. If D11/D12 read LOW with pullups enabled, treat
that as an electrical or bus issue such as power, GND, pullups, or a stuck bus;
do not treat it as a pin mapping issue.

Latest observed result:

- The robust default `Wire` scanner runs and prints repeated scan passes.
- Every pass shows `pre_scan_sda=LOW` and `pre_scan_scl=LOW`.
- Every pass prints `BUS_STUCK_LOW_BEFORE_SCAN`.
- Every pass reports `found_count=0` and `stable_valid_address=NA`.
- The scanner is not hanging; it is correctly refusing to scan because the bus
  is stuck low before address probing.
- IMU remains unverified and must not be used for autonomy yet.
- Continue the GPS+RC safety-gated workflow without IMU for now.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-scanner firmware/i2c_scanner_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-scanner firmware/i2c_scanner_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

If the monitor is blank, press the OpenRB reset button while the monitor is
open. The scanner should print again every 2 seconds even if the first startup
messages were missed.

## D11/D12 Bit-Bang I2C Scanner

Use this standalone sketch only as a secondary diagnostic. The current fixed IMU
wiring is on OpenRB default `Wire` pins, so `firmware/i2c_scanner_test` is the
primary scanner.

```text
firmware/i2c_d11_d12_bitbang_scanner/i2c_d11_d12_bitbang_scanner.ino
```

Current fixed IMU wiring:

- SDA: OpenRB D11 / PA08 / SDA(SC2)
- SCL: OpenRB D12 / PA09 / SCL(SC2)

This sketch implements open-drain style I2C in software:

- release line: `pinMode(pin, INPUT_PULLUP)`
- drive low: `pinMode(pin, OUTPUT); digitalWrite(pin, LOW)`
- never drive HIGH directly

It prints the released SDA/SCL state, reports `SDA stuck low` or `SCL stuck
low` if either line remains low, attempts bus recovery before skipping stuck
passes, scans addresses `0x03` through `0x77` every 2 seconds, and prints
`scan_pass`, `bus_stuck_low`, raw/valid found counts, found addresses, and
`stable_valid_address`.

If a pass reports many addresses or every address, treat it as scanner/bus
failure (`INVALID_SCAN_ACK_STUCK_LOW`), not as many devices. The IMU remains
unverified until a stable single address is detected in at least three
consecutive valid scan passes.

Latest observed result:

- The original bit-bang scanner produced impossible all-address detection; that
  was invalid and must not be treated as success.
- The hardened scanner was tested with SDA=D11/SCL=D12 and with swapped
  SDA=D12/SCL=D11.
- Both variants repeatedly reported `released_sda=LOW`, `released_scl=LOW`,
  `SDA stuck low`, `SCL stuck low`, `raw_found_count=0`, `valid_found_count=0`,
  and `stable_valid_address=NA`.
- IMU presence remains unverified.
- Because D11/D12 are OpenRB default `Wire` pins, the next diagnostic is the
  robust default `Wire` scanner in `firmware/i2c_scanner_test`, not a custom
  SERCOM2 scanner.
- If the robust default `Wire` scanner also fails, continue GPS+RC workflow
  without IMU support.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Compile with explicit pin override:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang --build-property 'compiler.cpp.extra_flags=-DI2C_BITBANG_SDA_PIN=11 -DI2C_BITBANG_SCL_PIN=12' firmware/i2c_d11_d12_bitbang_scanner
```

Compile with swapped D12/D11 assignment, without moving wires:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d12-d11-bitbang --build-property 'compiler.cpp.extra_flags=-DI2C_BITBANG_SDA_PIN=12 -DI2C_BITBANG_SCL_PIN=11' firmware/i2c_d11_d12_bitbang_scanner
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Upload the swapped D12/D11 build:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d12-d11-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

## Serial3 Pin Verification

These standalone sketches are retained as historical/safe UART pin tools:

```text
firmware/pin_finder_test/pin_finder_test.ino
firmware/serial3_loopback_test/serial3_loopback_test.ino
```

Under the Fixed Wiring Plan, do not move GPS or HC-12. They do not attach motor
outputs.
