# GPS Bring-Up

This page is the working procedure for recovering and validating the rover GPS
UART link. It is intentionally separate from the integrated rover controller.

## Safety Boundary

- Do not modify `firmware/openrb_robot_controller/openrb_robot_controller.ino`
  for this probe.
- Use `firmware/gps_uart_probe/gps_uart_probe.ino`; it does not include Servo,
  attach ESC pins, or drive motor outputs.
- Keep motor power disconnected when possible. If motor power must stay
  connected, keep the rover wheel-off-ground.
- Test one UART/baud candidate at a time and record the exact output before
  moving to the next candidate.

## Current Results

Date: 2026-05-26

Final status:

- GPS UART bring-up succeeded.
- Current physical GPS wiring is the central OpenRB connector.
- The confirmed GPS UART is `Serial2` at `9600` baud.
- GPS module baudrate is confirmed as `9600`.
- Valid NMEA is received.
- GPS fix eventually became valid.
- Latest GPS-only `Serial2/9600` probe still receives continuous NMEA, but GPS
  fix is intermittent rather than stable.
- Integrated GPS `Serial2` diagnostic sky-fix validation succeeded after moving
  the external GPS antenna farther outside into open sky.
- Previous `gps_sats=0` and `gps_hdop=99.99` was poor satellite acquisition
  from indoor/window-side placement, not UART or firmware failure.
- Final UART decision: Fixed Wiring Plan.
- Keep GPS on the current central OpenRB connector as `Serial2`.
- Keep HC-12 physically as-is and audit its current wiring before assuming
  whether it shares GPS `Serial2`.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as a UART device.

### Serial3 D13/D14 Failure

Uploaded GPS-only test result:

```text
chars_1s=0 total_chars=0 tinygps_chars=0 status=NO_FIX sats=NA hdop=NA age_ms=NA
```

Configuration assumed by that test:

- GPS port: `Serial3`
- OpenRB-150 RX pin assumption: `D13`
- GPS baud: `9600`
- USB monitor baud: `115200`

Interpretation:

- The sketch is running.
- USB serial monitor is working.
- `chars_1s=0`, `total_chars=0`, and `tinygps_chars=0` mean no bytes reached
  the configured GPS serial input.
- This is not merely a satellite-fix problem. It points to wiring, selected
  UART, baudrate, module power, or module output configuration.

Follow-up probe result:

- `Serial3` D13/D14 tests failed because the current GPS wiring is not on
  D13/D14.
- Do not interpret the D13/D14 failure as a failed GPS module while the module
  is plugged into the central OpenRB connector.
- Earlier Serial3 pin-finding work is historical. Under the Fixed Wiring Plan,
  do not move GPS or HC-12; audit the current HC-12 wiring instead.

### Serial2 Success On Current Wiring

Final successful probe result:

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

- GPS UART communication is working on `Serial2` at `9600` with the current
  central OpenRB connector wiring.
- GPS satellite fix was acquired.
- The Serial3 D13/D14 zero-byte result is invalid for the current wiring; it
  does not indicate GPS module failure.
- GPS should stay on the current central connector.
- The next problem is the current HC-12 wiring audit and integrated firmware
  diagnostics.

### Latest Serial2 Probe: UART OK, Fix Intermittent

Latest GPS-only probe setup:

```text
GPS_PROBE_MODE=2
GPS_PROBE_BAUD=9600
```

Observed:

- `Serial2` at `9600` printed continuous NMEA characters.
- This confirms GPS UART receive and baudrate on the current central connector
  are still likely correct.
- Most lines showed no usable fix:
  - RMC status `V`
  - GGA fix quality `0`
  - `sats=0`
  - `hdop=99.99`
  - `fix=false`, or stale cached TinyGPS++ fix state
  - `lat` / `lon` as `NA`, or stale cached coordinates
- A few short bursts showed valid data:
  - RMC status `A`
  - valid latitude/longitude
  - `sats=4..5`
  - `hdop≈1.77..2.48`
- The valid state lasted only briefly, then returned to RMC `V`, GGA quality
  `0`, `sats=0`, and `hdop=99.99`.

Interpretation:

- GPS UART is alive.
- GPS satellite acquisition is intermittent.
- This is not a target override, RC, or timeout issue.
- TinyGPS++ can keep cached coordinates after RMC returns to `V`; do not use
  cached lat/lon as a stable current rover position.
- Do not proceed to AUTO dry-run, bench test, or floor driving until stable
  GPS fix is confirmed.

### Latest Serial2 Probe: Moving Farther Outdoors Recovered Stable Fix

Follow-up placement change:

- The rover/GPS was moved farther outdoors with clearer sky view.
- The same `firmware/gps_uart_probe` configuration was used:
  - `GPS_PROBE_MODE=2`
  - `GPS_PROBE_BAUD=9600`

Observed:

- The probe transitioned from `NO_FIX` to `INTERMITTENT_FIX`.
- Valid fields appeared with RMC `A`, GGA fix quality `>=1`, valid lat/lon
  around `35.5708,129.1870`, satellites around `5`, and HDOP around `4.0`.
- The latest stable lines reached:
  - `gps_probe_state=STABLE_FIX`
  - `current_valid_fix=true`
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `lat≈35.570284..35.570296`
  - `lon≈129.187078`
  - `age_ms≈85..89`
  - `sats=9`
  - `hdop=3.56`
  - `valid_fix_seconds_consecutive=58..60`

Interpretation:

- GPS UART and module operation are confirmed on `Serial2/9600`.
- The previous persistent no-fix was primarily a placement / sky-view problem.
- Indoor, near-building, window-side, or partially covered positions can
  receive NMEA continuously while still staying in `NO_FIX`.
- Before autonomy dry-run, place the rover/GPS farther outdoors with clear sky
  view and wait for `gps_probe_state=STABLE_FIX`.

Minimum stable-fix rule:

- `current_valid_fix=true`
- RMC `A` or GGA fix quality `>=1`
- fresh lat/lon age `<=2000 ms`
- `sats >= 4`
- `hdop <= 5.0`
- `valid_fix_seconds_consecutive >= 30`

This validates standalone GPS stability only. It does not approve floor
driving.

### Main Controller Outdoor GPS Recovery

After the standalone probe reached stable fix, the main controller was tested
farther outdoors with:

```text
FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1
AUTO_MOTION_ARMED=0
```

Observed GPS quality in USBDBG:

- `gps_location_valid=true`
- `gps_location_fresh=true`
- `gps_age_ok=true`
- `gps_sats_ok=true`
- `gps_hdop_ok=true`
- `gps_solution_valid=true`
- `gps_dryrun_ready=true`
- `gps_motion_ready=true`
- `gps_ready=true`
- `gps_block_reason=OK`
- `last_rmc_status=A`
- `last_gga_fix_quality=2`
- `gps_sats≈9..11`
- `gps_hdop≈1.46`

Interpretation:

- Moving the rover farther outdoors fixed GPS acquisition in both the standalone
  probe and the main controller.
- The remaining dry-run blocker is not GPS. In the latest main-controller run,
  the compile-time target was stale and `target_distance_m≈41`, which exceeded
  `max_target_distance_m=30.0`.
- Do not proceed to floor driving. Recompute a nearby target from the current
  GPS position and rerun `AUTO_MOTION_ARMED=0`.

### Main Controller No-Motion AUTO Waypoint Dry-Run Success

The follow-up main-controller dry-run used a nearby compile-time target:

```text
target_lat_macro=35.5705010
target_lon_macro=129.1872696
```

Observed GPS quality:

- In MANUAL, GPS readiness was good enough for dry-run:
  - `gps_location_valid=true`
  - `gps_location_fresh=true`
  - `gps_solution_valid=true`
  - `gps_dryrun_ready=true`
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `gps_sats≈7..9`
  - `gps_hdop≈1.28..1.56`
- On good samples, motion-level fields also passed:
  - `gps_motion_ready=true`
  - `gps_ready=true`

Observed dry-run target and AUTO gates:

- `target_distance_m≈8.4..15.2`
- `distance_allowed=true`
- `mode=AUTO_READY`
- `auto_sw=true`
- `mode_us≈2001`
- `timeout_source=auto_entry`
- `timeout_ok=true`
- `safety_ready=true`
- `candidate_left_cmd=0.100`
- `candidate_right_cmd=0.100`
- `AUTO_MOTION_ARMED=0`
- `auto_motor_inhibit=true`
- `final_left_cmd=0.000`
- `final_right_cmd=0.000`

Interpretation:

- Main-controller GPS and AUTO gates are recovered outdoors.
- No-motion waypoint candidate command generation is validated.
- Some AUTO lines can still report motion-level `gps_ready=false` /
  `gps_block_reason=BAD_HDOP` because the dry-run and motion GPS gates are
  intentionally different. This is acceptable only for `AUTO_MOTION_ARMED=0`
  when `gps_dryrun_ready=true` / `active_gps_ready=true`.
- This does not approve floor driving. The next step is a wheel-off-ground
  bench test with strict safety procedure.

## Architecture Decision: Fixed Wiring Plan

The integrated rover firmware currently defines:

```cpp
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3
```

The current hardware probe confirms GPS data on `Serial2`, not on `Serial3`
D13/D14. GPS cannot be moved. HC-12 appears mounted under or behind the OpenRB
board and cannot be moved right now. Proceed with the current physical wiring.

Decision:

- Previous Option A and Option B UART-rewiring plans are superseded.
- GPS stays on the current central OpenRB connector as `Serial2` at `9600`.
- HC-12 stays physically as-is until the current wiring is audited.
- Do not assume GPS and HC-12 can be used simultaneously on `Serial2`.
- Do not modify motor control, autonomy, STOP, heartbeat timeout, failsafe,
  manual override, or RC safety.

Decision table:

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

Next milestone:

1. Add an integrated GPS `Serial2` diagnostic firmware mode.
2. Audit current HC-12 wiring from code, board inspection, and non-motion
   diagnostics.
3. Run receive-only station telemetry testing only if safe.
4. Continue station-side path planning as dry-run only.

## Probe Sketch

Standalone sketch:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
```

Compile-time options:

| Macro | Meaning |
|---|---|
| `GPS_PROBE_MODE=3` | `Serial3`, expected OpenRB RX `D13` |
| `GPS_PROBE_MODE=2` | `Serial2`, board UART candidate |
| `GPS_PROBE_MODE=89` | `SoftwareSerial` RX `D8` / TX `D9` candidate |
| `GPS_PROBE_BAUD=9600` | GPS candidate baudrate |

Every report line prints:

- selected port
- selected baud
- pin assumption
- `chars_1s`
- `total_chars`
- TinyGPS++ `charsProcessed`
- `last_rmc_status`
- `last_gga_fix_quality`
- `sats`
- `hdop`
- operational `lat` / `lon` only when the current fix is valid
- `age_ms`
- `current_valid_fix`
- `gps_probe_state`
- `valid_fix_seconds_consecutive`
- `no_fix_seconds_consecutive`
- `valid_fix_seconds_total`
- `no_fix_seconds_total`
- truncated `rmc_preview` and `gga_preview`

Stability states:

| State | Meaning |
|---|---|
| `NO_FIX` | No current valid fix. NMEA may still be arriving. |
| `INTERMITTENT_FIX` | Current valid fix is present, but consecutive valid time is less than `30` seconds. |
| `STABLE_FIX` | Current valid fix has lasted at least `30` consecutive seconds. |

Current valid fix criteria:

- RMC status `A` or GGA fix quality `>=1`
- TinyGPS++ latitude/longitude valid
- location age `<=2000 ms`
- satellites `>=4`
- HDOP `<=5.0`

## Probe Matrix

The software build check was run against OpenRB-150 core
`OpenRB-150:samd:OpenRB-150`.

| Order | Candidate | Compile Status | Bench Result | Action |
|---:|---|---|---|---|
| 1 | `Serial3` RX `D13` at `9600` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; use loopback/pin-finder tests to locate real Serial3 RX/TX |
| 2 | `Serial3` RX `D13` at `38400` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; GPS baud is already confirmed as 9600 |
| 3 | `Serial3` RX `D13` at `115200` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; GPS baud is already confirmed as 9600 |
| 4 | `Serial2` at `9600` | pass | success: `chars_1s` roughly 350-520, readable NMEA `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, `$GPGLL`, TinyGPS++ chars increasing, `fix=true`, lat around `35.57107`, lon around `129.1860`, sats around `5`, HDOP around `1.61-1.62` | GPS UART bring-up complete; keep GPS on central connector / `Serial2`; audit current HC-12 wiring next |
| 5 | `SoftwareSerial` RX `D8` / TX `D9` at `9600` | not supported in this OpenRB build | not uploaded | `SoftwareSerial.h` is unavailable; do not use this candidate unless the core/library support changes and it recompiles |

## Wiring Checklist

Check these before changing firmware assumptions:

- GPS `VCC` / `UCC` connected to the intended OpenRB supply rail.
- GPS `GND` connected to OpenRB `GND`; USB, OpenRB, GPS, and any external power
  domains must share ground for UART testing.
- GPS `TX` goes to the selected OpenRB `RX`; UART is crossed TX-to-RX.
- For `Serial3`, the earlier repo assumption was GPS `TX` -> OpenRB `D13` /
  RX, but the physical `Serial3` pin mapping is currently unresolved.
- For the current central OpenRB connector wiring, GPS data was confirmed on
  `Serial2` at `9600`.
- Under the Fixed Wiring Plan, GPS remains on the current central connector /
  `Serial2`.
- HC-12 remains physically as-is until current wiring is audited.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as a UART device or HC-12 substitute.
- GPS `RX` is not required for read-only NMEA receive tests. Leave it
  disconnected unless intentionally configuring the GPS module.
- GPS `PPS` is not required for this test.
- Confirm the GPS module is powered and its antenna has sky view. Sky view is
  needed for a fix, but not needed for `chars_1s > 0`.
- Confirm the GPS module still emits NMEA text and was not reconfigured to a
  binary-only protocol.
- Do not connect GPS and HC-12 to the same UART RX/TX pair at the same time
  during this probe.
- The integrated controller currently expects HC-12 on `Serial2`; do not assume
  GPS and HC-12 can share the current `Serial2` path in normal operation.
- Under the Fixed Wiring Plan, do not move GPS or HC-12.
- Confirm logic-level compatibility before moving HC-12 UART wires to another
  OpenRB pin.

## Decision Rules

- `chars_1s=0`: wiring, selected port, selected baudrate, module power, or GPS
  output configuration problem. Do not wait indoors for a GPS fix; no data is
  arriving.
- `chars_1s>0` and `gps_probe_state=NO_FIX`: GPS bytes are arriving. Keep the
  same UART and baudrate, move outdoors or improve antenna placement, and wait
  for satellite acquisition.
- RMC status `V`, GGA fix quality `0`, `sats=0`, and `hdop=99.99` means no
  usable satellite fix even if bytes are arriving.
- A short RMC `A` burst is not enough for autonomy validation. Require stable
  fix quality over repeated lines before interpreting target distance.
- TinyGPS++ cached `fix`, latitude, or longitude after RMC returns to `V` must
  not be treated as a current usable GPS solution.
- `chars_1s>0` with readable `rmc_preview` / `gga_preview`: parser issues are
  unlikely; focus on fix quality and sky view.
- `tinygps_chars=0` while `chars_1s>0`: raw bytes are arriving but may not be
  NMEA text, may be wrong baud gibberish, or may be a binary-only GPS output.

GPS sky-fix checklist:

- `gps_chars` increasing means UART receive and baudrate are OK.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet.
- Move the antenna outside/open sky before suspecting code.
- Window-side indoor placement may receive NMEA but still fail to acquire
  satellites.
- Rain did not prevent fix when the antenna had open-sky exposure, but protect
  electronics and antenna connectors from water.

Current decision from the successful probe:

- `Serial2` at `9600` with readable `$GP...` NMEA means the GPS UART is working.
- `fix=true`, numeric latitude, numeric longitude, and stable HDOP confirm the
  GPS can produce a usable position on the current wiring.
- Do not keep cycling UART ports once `chars_1s>0` and readable NMEA are
  confirmed.
- Do not implement autonomous movement as the next step; resolve UART
  allocation and add diagnostics first.
- Because the Fixed Wiring Plan is selected, the next proof point is not
  another GPS baud scan and not rewiring. It is a current HC-12 wiring audit.

## Test Procedure

For each supported candidate:

1. Wire only the candidate UART path.
2. Compile the candidate into its own build directory.
3. Upload that exact build directory.
4. Open USB serial monitor at `115200`.
5. Record at least 10 report lines.
6. If `chars_1s=0`, stop that candidate and move to the next baud/port.
7. If `chars_1s>0`, keep that UART/baud and debug fix quality separately.

## Commands

The commands below use the Arduino CLI bundled with Arduino IDE on this
machine, FQBN `OpenRB-150:samd:OpenRB-150`, and OpenRB USB port
`/dev/cu.usbmodem12101`. Replace the port only if the board enumerates
differently.

### Serial3 RX D13 at 9600

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-9600 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 RX D13 at 38400

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-38400 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=38400' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-38400 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 RX D13 at 115200

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-115200 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=115200' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-115200 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial2 at 9600

This is the confirmed working probe for the current central OpenRB connector
wiring. GPS should stay on this connector. If HC-12 also shares `Serial2`, do
not run integrated GPS and HC-12 simultaneously.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Outdoor GPS Fix Test

This test has succeeded on `Serial2` at `9600`. Use this command to repeat it
after the `Serial2` probe sketch is already uploaded:

```bash
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200
```

Logging variant:

```bash
mkdir -p outputs/logs
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200 | tee outputs/logs/gps_outdoor_fix_serial2_$(date +%Y%m%d_%H%M%S).log
```

Outdoor pass criteria:

- `chars_1s > 0`
- `rmc_preview` / `gga_preview` continue to show readable `$GP...` or `$GN...`
  NMEA
- `gps_probe_state=STABLE_FIX`
- `valid_fix_seconds_consecutive >= 30`
- `current_valid_fix=true`
- RMC status `A` or GGA fix quality `>=1`
- `lat` and `lon` are numeric
- `sats >= 4`
- HDOP `<=5.0`; lower is better

Do not treat `INTERMITTENT_FIX` as an autonomy-ready state. It is useful
evidence that the receiver can acquire satellites briefly, but the next
autonomy step remains blocked until `STABLE_FIX` is observed.

Observed passing values:

- `chars_1s`: roughly `350-520`
- NMEA: `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, `$GPGLL`
- `fix`: `true`
- `lat`: around `35.57107`
- `lon`: around `129.1860`
- `sats`: around `5`
- `hdop`: around `1.61-1.62`

## Next Software Milestone

The next software milestone should be a GPS diagnostic integrated firmware mode,
not autonomous movement.

Goal:

- Verify the selected final UART allocation inside
  `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
- Keep manual control, STOP, heartbeat timeout, RC priority, and failsafe
  behavior unchanged.
- Print or transmit explicit GPS diagnostics from integrated firmware:
  selected GPS UART, `chars_1s`, TinyGPS++ chars processed, fix state, sats,
  HDOP, lat/lon, and GPS age.
- Diagnostic target: prove integrated GPS reads from `Serial2` without changing
  motor behavior or enabling autonomous motion.
- Do not send or enable live `AUTO` driving commands.

Recommended order:

1. Add a non-motion integrated GPS `Serial2` diagnostic path.
2. Audit current HC-12 wiring from code and diagnostics.
3. If safe, run receive-only station telemetry tests.
4. Keep station-side path planning dry-run only.
5. Only after telemetry and safety remain stable, continue station-side GPS
   display or logging work.

## Integrated GPS Serial2 Diagnostic Mode

Compile-time flag:

```text
FIXED_WIRING_GPS_SERIAL2_DIAG=1
```

Diagnostic behavior:

- `GPS_SERIAL` uses `Serial2`.
- `GPS_BAUD` remains `9600`.
- HC-12 commands are disabled/ignored to avoid a possible `Serial2` conflict.
- Station commands are not processed.
- RC status is still printed in USB debug.
- Motor outputs are forced neutral and command outputs remain zero.
- USB debug prints GPS status:
  - `gps_fix`
  - `gps_lat`
  - `gps_lon`
  - `gps_sats`
  - `gps_hdop`
  - `gps_age_ms`
  - `gps_chars`

Latest sky-fix validation:

- `fixed_wiring_gps_serial2_diag=true`
- `hc12_enabled=false`
- `gps_chars` increased continuously
- `gps_fix=true` after moving the external antenna farther outside/open sky
- `gps_lat`, `gps_lon`, `gps_sats`, and `gps_hdop` became valid
- motors remained disarmed/neutral

Default build expectation under fixed wiring:

- `fixed_wiring_gps_serial2_diag=false`
- `hc12_enabled=true`
- `gps_chars=0` is expected because default firmware reads GPS from `Serial3`
  while the fixed GPS wiring is on `Serial2`
- If USBDBG shows `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2000`, and
  `control_source=STOP`, manual drive is stopped by the RC mode switch state.
  Switch RC mode out of AUTO before validating `control_source=RC_MANUAL`.

Do not repeat:

- Do not treat default-build `gps_chars=0` as GPS failure under current fixed
  wiring.
- Do not expect manual driving in `FIXED_WIRING_GPS_SERIAL2_DIAG`; this mode is
  GPS diagnostic only and forces motor outputs neutral.
- Do not connect both OpenRB USB and station USB-serial during OpenRB upload if
  `arduino-cli` selects the wrong upload port.
- If upload fails because it selected `/dev/cu.usbserial-02444963`, unplug the
  station USB-serial and upload with only OpenRB connected.

Next architecture:

- Future fixed-wiring mode should combine GPS `Serial2` with RC switch control:
  - Auto OFF: RC manual drive
  - Auto ON: onboard GPS mission/autonomy after separate safety design
- HC-12 is not used in this future mode until hardware can be revised or proven
  independent from GPS `Serial2`.
- Station-side path planning remains dry-run until autonomy is explicitly
  implemented and safety-gated.

Default integrated controller compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
```

Fixed-wiring GPS Serial2 diagnostic compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_DIAG=1' firmware/openrb_robot_controller
```

Upload diagnostic build:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag firmware/openrb_robot_controller
```

Monitor diagnostic build:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### SoftwareSerial D8/D9 at 9600

This candidate is documented but currently not supported by this OpenRB build.
Compilation fails because `SoftwareSerial.h` is unavailable. Do not upload it
until support is added and the compile succeeds.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-soft-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=89 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
```
