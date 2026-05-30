# Known Issues

This file lists known technical gaps and recurring mistakes. Read it before
changing control, protocol, planning, or station workflow code.

## Manual Direction Mapping Is Fragile

Status:

- Current firmware uses `rc-cardinal-remap`.
- Neutral USBDBG is verified.
- Full straight up/down/left/right wheel-off-ground validation is still needed.

Current code:

```cpp
steeringOut = (rawSteering + rawThrottle) * 0.70710678f;
throttleOut = (rawSteering - rawThrottle) * 0.70710678f;
```

Known mistakes:

- changing only throttle sign moved the problem to another diagonal
- treating left/right as forward/reverse was wrong
- judging only wheel spin can confuse RC axis mapping with motor/ESC direction

Reference:

- `docs/manual_control.md`
- `docs/field_test_log.md`

## RC Mode Switch Requires Powered Controller/Link

Status:

- A recent outdoor test showed the previous stuck-looking RC issue was caused
  by the station/controller being off.
- After restoring the controller/link, Manual/Auto switching worked again.
- AUTO was verified with `mode=AUTO_READY`, `auto_sw=true`,
  `mode_us≈2001..2002`, and `control_source=STOP`.
- MANUAL was verified with `mode=MANUAL`, `auto_sw=false`,
  `mode_us≈1000..1001`, and `control_source=RC_MANUAL`.
- Manual stick input changed `steer_us` / `throttle_us`, and at least one
  MANUAL line produced nonzero `left_cmd` / `final_left_cmd`.

Diagnostic:

- Use `firmware/rc_channel_probe`.
- Confirm the station/controller is powered on and linked before interpreting
  static channel output.
- Move one stick or switch at a time.
- Record `ch1_us` through `ch8_us`, each channel min/max, and
  `changed_channels`.
- The AUTO candidate is the channel that reaches around `2000 us`.
- Manual/Auto verification requires seeing both ends of the switch travel:
  `mode_us≈1000` for MANUAL and `mode_us≈2000` for AUTO.

Do not repeat:

- Do not assume the physical switch label matches the PPM channel number.
- Do not treat static RC probe channels as a mapping failure until the
  station/controller power and link are confirmed.
- Do not change controller channel mapping based only on panel labels.
- Do not approve AUTO, bench, or floor testing unless USBDBG shows the expected
  Manual/Auto mode values and the relevant safety gates.

## Rover Drifts Left/Right During Long Manual Movement

Status:

- known issue
- not the top priority

Likely causes to investigate later:

- motor/ESC calibration mismatch
- wheel traction or mechanical asymmetry
- steering mix imbalance
- surface friction
- heading correction not implemented

Do not hide this by changing path planning. Treat it as a low-level calibration
and control issue after station dry-run workflow is stable.

## GPS Telemetry Schema Mismatch

Firmware currently emits key/value GPS payloads:

```text
fix=1,lat=...,lon=...,sats=...,hdop=...,age_ms=...
```

Python `GPSTelemetry.from_payload()` expects positional fields:

```text
lat,lon,alt,sats,hdop,fix_valid
```

This must be fixed before relying on live station GPS telemetry.

## GPS And HC-12 UART Allocation Conflict

Status:

- GPS UART receive is confirmed on `Serial2` at `9600` with the GPS connected
  to the current central OpenRB connector.
- GPS module baudrate is confirmed as `9600`.
- The central OpenRB connector is confirmed as `Serial2`.
- The purple module appears to be an IMU on an I2C-style connection and should
  not be treated as UART.
- HC-12 appears to be mounted under or behind the OpenRB board and needs its
  UART wiring verified separately.
- GPS cannot be moved.
- HC-12 cannot be moved right now.
- GPS fix succeeded on that path:
  - `lat` around `35.57107`
  - `lon` around `129.1860`
  - `sats` around `5`
  - `hdop` around `1.61-1.62`
- Integrated GPS `Serial2` diagnostic sky test succeeded:
  - `fixed_wiring_gps_serial2_diag=true`
  - `hc12_enabled=false`
  - `gps_chars` increased continuously
  - `gps_fix=true` after moving the external GPS antenna farther outside into
    open sky
  - latitude/longitude, satellites, and HDOP became valid
  - motors remained disarmed/neutral
- The integrated rover firmware currently defines:

```cpp
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3
```

Risk:

- GPS and HC-12 cannot both own the same `Serial2` UART during normal rover
  operation.
- If HC-12 shares GPS `Serial2`, simultaneous GPS plus HC-12 bidirectional
  communication may not be possible.
- Do not assume HC-12 shares `Serial2`; audit current wiring from code,
  board inspection, and diagnostics first.
- Keeping the integrated firmware unchanged means integrated GPS telemetry will
  still read from `Serial3`, where current GPS wiring has no bytes.

Decision:

- Previous Option A and Option B UART-rewiring plans are superseded.
- Fixed Wiring Plan:
  - keep GPS on the current central connector / `Serial2`
  - keep HC-12 physically as-is
  - do not move either module
  - audit whether HC-12 is independent from or sharing GPS `Serial2`

Decision table:

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

Next software milestone:

- Add an integrated GPS `Serial2` diagnostic firmware mode.
- Audit current HC-12 wiring.
- Run receive-only station telemetry testing only if safe.
- Keep station-side path planning dry-run only.
- The diagnostic mode should report selected GPS UART, raw character counts,
  TinyGPS++ processed characters, fix state, sats, HDOP, lat/lon, and GPS age.
- It must not implement autonomous movement.
- It must not weaken manual control, STOP override, heartbeat timeout, or
  failsafe behavior.

Do not repeat:

- Do not treat `gps_chars=0` in the default controller build as GPS failure
  under current fixed wiring. Default firmware still reads GPS from `Serial3`.
- Do not expect manual driving in `FIXED_WIRING_GPS_SERIAL2_DIAG`; this mode is
  for GPS testing only, disables/ignores HC-12, and forces motors neutral.
- Do not expect HC-12 in fixed-wiring GPS modes. HC-12 remains
  disabled/ignored there because GPS and HC-12 cannot both use the current
  `Serial2` wiring safely.
- Do not connect both OpenRB USB and station USB-serial during OpenRB upload if
  `arduino-cli` selects the wrong upload port.
- If upload fails because it selected `/dev/cu.usbserial-02444963`, unplug the
  station USB-serial and upload with only OpenRB connected.

Next architecture:

- For fixed wiring, implement a future GPS `Serial2` plus RC switch mode:
  - Auto OFF: RC manual drive
  - Auto ON: onboard GPS mission/autonomy after separate safety design
- HC-12 is not used in that mode until hardware can be revised or proven
  independent from GPS `Serial2`.

## GPS Indoor Or Window-Side Fix Can Fail

Status:

- GPS UART bytes can arrive correctly while GPS fix remains false.
- Indoor/window-side tests may show increasing `gps_chars`, valid NMEA
  sentences, and TinyGPS++ character counts, but still fail to acquire enough
  satellites.
- `gps_sats=0` and `gps_hdop=99.99` are consistent with no satellite
  acquisition, not UART or firmware failure by themselves.
- Outdoor/open-sky antenna placement produced `gps_fix=true` in the
  `FIXED_WIRING_GPS_SERIAL2_DIAG` build.
- Rain did not prevent fix when the antenna had open-sky exposure, but the
  electronics, USB adapters, and antenna connectors must be protected from
  water.

Decision rule:

- `gps_chars=0`: wiring, selected UART, baudrate, power, or GPS output problem.
- `gps_chars>0` with `gps_fix=false`: GPS data is arriving, but satellite fix
  quality is not sufficient yet.
- `gps_sats=0` and `gps_hdop=99.99`: no satellite acquisition yet.
- For first fix, place the GPS antenna outdoors with open sky view and wait
  before debugging firmware.

## GPS Antenna Frame Is Not Rover Body Frame

Status:

- Recent GPS fix and single-waypoint candidate dry-run tests used an external
  antenna placed far outside while the rover body remained indoors.
- In that setup, `gps_lat` and `gps_lon` are the antenna position, not the rover
  body position.

Risk:

- Detached-antenna GPS is valid for reception and satellite-fix validation.
- Detached-antenna GPS is invalid for floor navigation and rover body
  localization.
- The IMU cannot fully correct a detached or free-moving GPS antenna into rover
  body position.
- The IMU may help later with heading/rotation sensing, but it does not replace
  a rover-mounted GPS position source.

Required before motion:

- Mount the GPS antenna rigidly on the rover, or define a fixed measured offset
  from the rover body frame.
- Run an IMU I2C scan.
- Verify IMU orientation and axis signs.
- Re-test candidate GPS fields with mounted antenna and open-sky placement.
- Use wheel-off-ground bench testing before any floor test.

Do not repeat:

- Do not approve `AUTO_MOTION_ARMED=1` floor tests while the GPS antenna is
  detached from the rover body.
- Do not assume IMU data can turn detached antenna coordinates into rover body
  coordinates.

## IMU I2C Wiring Is Fixed On D11/D12

Status:

- The IMU wiring cannot be moved casually.
- Current fixed wiring is believed to be:
  - IMU SDA: OpenRB D11 / PA08
  - IMU SCL: OpenRB D12 / PA09
- OpenRB-150 variant files confirm:
  - Arduino D11 = SDA = PA08
  - Arduino D12 = SCL = PA09
  - `PIN_WIRE_SDA = 11`
  - `PIN_WIRE_SCL = 12`
  - `Wire` is constructed using those pins
- Use `firmware/i2c_scanner_test` as the primary diagnostic for the current
  fixed IMU wiring because D11/D12 are the board's default `Wire` pins.
- Use `firmware/i2c_d11_d12_bitbang_scanner` only as a secondary diagnostic.
- If the scanner reports every address, or a very large number of addresses,
  treat it as scanner/bus failure such as ACK stuck low. It is not evidence of
  many I2C devices.
- The original bit-bang scanner produced impossible all-address detection; this
  result is invalid and must not be treated as successful IMU detection.
- The hardened bit-bang scanner was tested with SDA=D11/SCL=D12 and with the
  swapped SDA=D12/SCL=D11 assignment. Both variants repeatedly reported
  `released_sda=LOW`, `released_scl=LOW`, `SDA stuck low`, `SCL stuck low`,
  `raw_found_count=0`, `valid_found_count=0`, and `stable_valid_address=NA`.
- IMU presence remains unverified.
- D11/D12 stuck-low means an electrical or bus issue, not a pin mapping issue.
- Possible causes include IMU power, GND, pullups, or a bus-stuck fault.
- Robust default `Wire` scanner result:
  - scanner runs and prints repeated scan passes
  - every pass shows `pre_scan_sda=LOW` and `pre_scan_scl=LOW`
  - every pass prints `BUS_STUCK_LOW_BEFORE_SCAN`
  - every pass reports `found_count=0`
  - every pass reports `stable_valid_address=NA`
- `pre_scan_sda=LOW` and `pre_scan_scl=LOW` mean the bus is stuck low before
  address probing.
- `stable_valid_address=NA` means no valid IMU address has been verified.
- Continue GPS+RC work without relying on IMU data for now.

Do not repeat:

- Do not ask to move IMU wires as a routine software step.
- Do not conclude the IMU is absent only because `firmware/i2c_scanner_test`
  finds no devices; verify the startup output and D11/D12 line states first.
- Do not treat all-address detection as valid.
- Do not infer device type from I2C address alone.
- Do not rely on IMU heading or acceleration for autonomy until a stable I2C
  device address and orientation/axis checks are confirmed.

## Single-Waypoint Target Override Must Be Verified In USBDBG

Status:

- A nearby single-waypoint candidate retest was compiled/uploaded with:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5716800`
  - `SINGLE_WP_TARGET_LON=129.1866516`
- Runtime USBDBG still printed the old placeholder:
  - `target_lat=35.571120`
  - `target_lon=129.186050`
- GPS fix was achieved, and at least one log line reached `gps_hdop=1.19` and
  `gps_ready=true`.
- Because the target remained the old placeholder, `target_distance_m` stayed
  around `40` to `60` m, `distance_allowed=false`, and `safety_ready=false`.
- `AUTO_MOTION_ARMED=0` correctly kept final motor output at zero.
- MANUAL still returned to `control_source=RC_MANUAL`.
- Firmware source now supports compile-time target override through:
  - `SINGLE_WP_TARGET_LAT`
  - `SINGLE_WP_TARGET_LON`
- If both macros are provided, USBDBG should print
  `target_override_enabled=true` and `target_source=compile_time`.
- If the macros are not provided, USBDBG should print
  `target_override_enabled=false` and `target_source=fallback`.
- A later check verified target override plumbing with:
  - `SINGLE_WP_TARGET_LAT=35.5710210`
  - `SINGLE_WP_TARGET_LON=129.1864016`
  - `target_override_enabled=true`
  - `target_source=compile_time`
  - `target_lat_macro=35.5710210`
  - `target_lon_macro=129.1864016`
  - `target_lat=35.571021`
  - `target_lon=129.186402`
- That same run had current GPS around `gps_lat≈35.56752..35.56756` and
  `gps_lon≈129.18688`, so `target_distance_m≈380..392`.
- Since `max_target_distance_m=30.0`, `distance_allowed=false` and
  `safety_ready=false` were expected.
- A next-day retest showed the same rule again:
  - runtime GPS moved to approximately `gps_lat=35.571310`,
    `gps_lon=129.188630`
  - firmware still used previous compile-time target
    `target_lat=35.567560`, `target_lon=129.186792`
  - `target_override_enabled=true` and `target_source=compile_time`, so target
    override was working
  - `target_distance_m≈448.9`, above `max_target_distance_m=30.0`
  - `distance_allowed=false` and `safety_ready=false`
- In that next-day run, `gps_fix=true` appeared but `gps_ready=false` remained
  because `gps_age_ms` was often very large, `gps_sats` fluctuated, and
  `gps_hdop` was often `99.99` and only occasionally around `4.7`.
- A later nearby attempt showed why the target must be calculated from the
  actual runtime fix:
  - override target was `target_lat=35.571310`, `target_lon=129.188542`
  - actual GPS fix was `gps_lat=35.571384`, `gps_lon=129.187514`
  - `gps_sats=4`
  - `gps_hdop=3.39..4.12`
  - `gps_age_ms` was initially fresh but later grew stale
  - `target_distance_m=93.3`, above `max_target_distance_m=30.0`
  - `distance_allowed=false` and `safety_ready=false`
- A window/outside-antenna attempt showed another blocked case:
  - target override was valid for `target_lat=35.571384`,
    `target_lon=129.187426`
  - GPS fix appeared around `gps_lat≈35.571284`, `gps_lon≈129.188456`
  - `gps_sats` often became `0`
  - `gps_hdop` often became `99.99`
  - `gps_age_ms` grew very large
  - `target_distance_m≈93.9`, above `max_target_distance_m=30.0`
  - `distance_allowed=false`, `gps_ready=false`, and `safety_ready=false`
- Latest outdoor Manual/Auto recovery showed the RC path is recovered and GPS
  can be usable outdoors, but the autonomy dry-run was still target-blocked:
  - AUTO verified with `mode_us≈2001..2002`, `mode=AUTO_READY`, and
    `control_source=STOP`
  - MANUAL verified with `mode_us≈1000..1001` and
    `control_source=RC_MANUAL`
  - runtime GPS was around `35.5716,129.1875`
  - target remained stale at `35.570675,129.186769`
  - `target_distance_m≈100..131`, above `max_target_distance_m=30.0`
  - `distance_allowed=false` and `safety_ready=false`
  - `AUTO_MOTION_ARMED=0` / `auto_motor_inhibit=true` kept AUTO final commands
    at zero
- Latest outdoor nearby dry-run showed partial progress but remained blocked:
  - target override worked with `target_lat=35.570768`,
    `target_lon=129.186791`
  - outdoor GPS readiness was repeatedly good (`gps_ready=true`,
    `gps_sats=7..8`, `gps_hdop≈0.95..1.98`, fresh `gps_age_ms`)
  - `target_distance_m` decreased below `30.0` m, and
    `distance_allowed=true` was observed
  - mode stayed mostly `MANUAL`
  - `auto_sw=false`
  - `timeout_ok=false`
  - `safety_ready=false`
  - `candidate_left_cmd=0.000` and `candidate_right_cmd=0.000`
- A brief PPM/failsafe-like glitch was observed with `mode=FAILSAFE`,
  `rc_ok=false`, `steer_us≈495`, `throttle_us≈2504`, and
  `control_source=STOP`.

Interpretation:

- This was a safe failed validation, not a successful nearby candidate-command
  test.
- Safety gates and AUTO motor inhibit worked correctly.
- Compile-time target override is now verified, but target override success must
  be checked separately from `distance_allowed`, `safety_ready`, or candidate
  command values.
- Runtime GPS can move far from a previously computed target. Recompute the
  nearby target from the current GPS position before each nearby candidate run.
- The target must be recalculated from the actual runtime GPS fix, not from an
  assumed antenna location.
- Outdoor GPS readiness and RC mode recovery do not make a stale target valid.
  Recompute the target from the current runtime GPS fix before interpreting
  `distance_allowed` or candidate commands.
- `distance_allowed=true` in MANUAL is progress, but it is not sufficient for a
  candidate dry-run success.
- `safety_ready` must be verified in `AUTO_READY` with the active GPS tier
  ready (`active_gps_ready=true`; `dryrun_ready=true` for inhibited dry-run or
  `motion_ready=true` for future armed motion), `distance_allowed=true`,
  `timeout_ok=true`, valid RC, and the AUTO switch on.
- Previous `timeout_ok=false` observations showed that the experiment timeout
  could expire while waiting in MANUAL for GPS/target setup.
- Firmware has been updated so the single-waypoint experiment timeout starts on
  AUTO entry, resets when leaving AUTO, and no longer consumes time during
  MANUAL waiting.
- USBDBG must now be checked for `timeout_source=auto_entry`,
  `auto_entry_ms`, `auto_elapsed_ms`, `timeout_limit_ms`, and `timeout_ok`.
- Brief PPM/failsafe glitches can occur; the safe response is
  `control_source=STOP` with no autonomous output.
- A window-thrown or window/outside antenna position is not equivalent to the
  rover body position. It can validate that GPS reception is possible, but it
  cannot validate rover localization.
- Compile-time nearby targets are only valid for the GPS location used when
  calculating them. If the antenna is moved on another day, the target must be
  recalculated.
- Compile-time targets also become stale whenever the rover itself moves. A
  target that was nearby before moving the rover can immediately exceed
  `max_target_distance_m`.
- The latest successful no-motion dry-run used compile-time target
  `35.5705010,129.1872696` and target distance around `8.4..15.2` m. If the
  rover moves before the next test, recompute the target again.
- `distance_allowed=false` is expected when `target_distance_m` exceeds
  `max_target_distance_m`.
- If GPS quality is good (`gps_ready=true`, RMC `A`, GGA quality `>=1`, enough
  satellites, acceptable HDOP) but `distance_allowed=false`, the usual cause is
  that the compile-time target is too far or stale, not that GPS is broken.
- `gps_fix=true` alone is not enough. `gps_age_ms`, `gps_hdop`, and `gps_sats`
  must also satisfy readiness gates before interpreting a run as GPS-ready.
- `gps_chars` increasing does not imply `gps_fix=true`; it only proves that
  NMEA/serial input is alive.
- GPS fix can be lost between outdoor attempts. If `gps_sats=0` and
  `gps_hdop=99.99`, treat the run as GPS no-fix and do not proceed to AUTO
  candidate validation.
- Latest GPS-only `Serial2/9600` probe showed continuous NMEA input but
  intermittent fix. Most lines reported RMC status `V`, GGA fix quality `0`,
  `sats=0`, and `hdop=99.99`; a few short bursts reached RMC `A`, valid
  lat/lon, `sats=4..5`, and `hdop≈1.77..2.48`, then returned to no-fix.
- Moving the rover/GPS farther outdoors recovered stable fix in the standalone
  probe. The latest stable lines showed `gps_probe_state=STABLE_FIX`,
  `current_valid_fix=true`, RMC `A`, GGA quality `2`, `sats=9`, `hdop=3.56`,
  and `valid_fix_seconds_consecutive=58..60`.
- Main-controller outdoor validation also recovered GPS and AUTO gates:
  `gps_dryrun_ready=true`, `gps_motion_ready=true`, `gps_ready=true`,
  `gps_block_reason=OK`, RMC `A`, GGA quality `2`, `gps_sats≈9..11`,
  `gps_hdop≈1.46`, `mode=AUTO_READY`, `auto_sw=true`, and `timeout_ok=true`.
  It was still blocked because stale target distance was about `41` m, greater
  than `max_target_distance_m=30.0`.
- Main-controller no-motion AUTO waypoint dry-run is now validated with
  `AUTO_MOTION_ARMED=0`: `safety_ready=true`,
  `candidate_left_cmd=0.100`, `candidate_right_cmd=0.100`,
  `auto_motor_inhibit=true`, and final commands still `0.000`.
- Treat the previous persistent `NO_FIX` primarily as a placement/sky-view
  issue. Indoor, near-building, window-side, or partially covered positions can
  keep reporting RMC `V`, GGA quality `0`, `sats=0`, and `hdop=99.99` even
  while UART data is continuous.
- `gps_chars` increasing only proves serial/NMEA input. It does not prove a
  usable GPS position.
- RMC status `V`, GGA fix quality `0`, `sats=0`, and `hdop=99.99` means no
  current usable fix.
- RMC status `A` or GGA quality `>=1` for one second is not enough for
  autonomy validation; require sustained stable fix. In `gps_uart_probe`, use
  `gps_probe_state=STABLE_FIX` or `valid_fix_seconds_consecutive >= 30`.
- TinyGPS++ cached lat/lon after RMC returns to `V` must not be used for target
  distance, safety gates, or candidate commands.
- The current blocker is unstable GPS satellite acquisition, not target
  override, RC mode mapping, or timeout semantics.
- The latest post-timeout-fix run confirmed `timeout_source=auto_entry` and
  target override, but was blocked by `gps_fix=false`, `gps_lat=NA`,
  `gps_lon=NA`, `gps_sats=0`, `gps_hdop=99.99`, and `gps_age_ms=NA`.
- The firmware now separates cached TinyGPS location validity from a usable GPS
  solution:
  - `gps_location_valid` means TinyGPS has a cached location.
  - `gps_location_fresh` / `gps_age_ok` mean the cached location is recent.
  - `gps_sats_ok` requires enough satellites.
  - `gps_hdop_ok` requires acceptable HDOP.
  - `gps_ready` is the stricter motion-level usable-position gate.
- GPS readiness is tiered:
  - `gps_solution_valid` requires valid/fresh location plus RMC `A` or GGA fix
    quality at least `1` when those NMEA statuses are available.
  - `gps_dryrun_ready` is for no-motion candidate calculation and allows HDOP
    up to `6.0`.
  - `gps_motion_ready` is for any future armed motion and keeps stricter HDOP
    and satellite thresholds.
- Dry-run GPS and motion GPS gates are intentionally different.
  `gps_ready=false` / `gps_block_reason=BAD_HDOP` can coexist with
  `safety_ready=true` only in `AUTO_MOTION_ARMED=0` when
  `gps_dryrun_ready=true` / `active_gps_ready=true`. It is not acceptable for
  real motion or floor driving.
- If motion-level `gps_ready=false`, operational `gps_lat` and `gps_lon`
  should be `NA`. In the single-waypoint experiment with
  `AUTO_MOTION_ARMED=0`, target distance and bearing may still be computed from
  `gps_dryrun_ready=true` for no-motion diagnostics.
- Cached coordinates may still be visible as `gps_cached_lat`,
  `gps_cached_lon`, and `gps_cached_age_ms`, but they are debug-only and must
  not be used for target distance or safety decisions.
- `gps_block_reason` identifies the first blocker, such as `NO_LOCATION`,
  `STALE_LOCATION`, `NO_SATS`, `BAD_HDOP`, `NOT_READY`, or `OK`.
- NMEA status fields `last_rmc_status` and `last_gga_fix_quality` are
  diagnostics only; armed-motion safety must still require `gps_motion_ready`
  / motion-level `gps_ready`.
- HDOP around `5` is acceptable only for no-motion dry-run candidate
  calculation when `gps_dryrun_ready=true`. It is not approved for floor
  driving or armed motion.
- `target_distance_m` must also be checked even when `gps_fix=true` and target
  override are both valid.
- `safety_ready` is the combined gate to inspect before any later motion work.
- `timeout_ok` can expire while waiting too long after upload/reset or AUTO
  entry; candidate validation should be done promptly after the run is ready.
- Runtime `target_lat` and `target_lon` are the source of truth.

Do not repeat:

- Do not assume a compile command changed the target unless USBDBG confirms the
  runtime `target_lat` / `target_lon`.
- Do not treat `target_override_enabled=true` as proof that the target is nearby
  enough. Check `target_distance_m` against `max_target_distance_m`.
- Do not treat `distance_allowed=true` in MANUAL as a successful AUTO candidate
  dry-run.
- Do not approve bench testing until `AUTO_READY`, `active_gps_ready=true`,
  `dryrun_ready=true`, `distance_allowed=true`, `timeout_ok=true`,
  `safety_ready=true`, and nonzero candidate commands are observed while final
  outputs remain inhibited.
- Do not reuse a previously computed nearby target after moving the antenna or
  on a later day without recalculating it from the current GPS position.
- Do not reuse a compile-time target from an earlier outdoor location; even
  100 m of target error correctly keeps `distance_allowed=false`.
- Do not compute the target from an assumed antenna location; use the actual
  runtime GPS fix printed by USBDBG.
- Do not treat a window/outside antenna toss as equivalent to moving the rover
  outdoors with the GPS fixed to the rover body.
- Do not treat `gps_fix=true` as sufficient when `gps_age_ms` is stale,
  `gps_hdop` is high, or satellites are unstable.
- Do not treat increasing `gps_chars` as a valid GPS position fix.
- Do not treat `gps_location_valid=true` as a usable GPS fix if age, satellites,
  or HDOP fail.
- Do not treat `gps_dryrun_ready=true` as approval for floor driving. Armed
  motion must require `gps_motion_ready=true`.
- Do not use `gps_cached_lat` / `gps_cached_lon` for target distance,
  waypoint acceptance, or safety decisions.
- Do not proceed to AUTO candidate validation when `gps_sats=0` and
  `gps_hdop=99.99`.
- Do not ignore `timeout_ok`; if it expires, rerun the validation promptly from
  a fresh AUTO entry state.
- Do not interpret `timeout_ok` without checking `timeout_source=auto_entry`.
  In MANUAL, `auto_entry_ms` / `auto_elapsed_ms` should be `NA`; after switching
  to AUTO, they should become numeric.
- Do not ignore PPM/failsafe glitches; any `rc_ok=false` or invalid pulse values
  must keep the rover stopped.
- Do not approve bench testing or floor driving from a run where the target
  override did not take effect or where `distance_allowed=false`.

## Station HC-12 Device Still Needs Confirmation

The repository defaults to `/dev/ttyACM0`, but the actual station HC-12 USB
adapter must be confirmed on the target station host. Do not hard-code a new
port. Use `--port`.

## Path Planning Is Offline/Mock

The planner generates a locally planar A/B lawnmower path. It does not yet:

- capture live A/B points from a station UI
- account for hull curvature
- account for obstacles or edge margins
- account for paint/cleaning process constraints
- command rover motion

Path generation must remain dry-run until a mission approval and safety state
machine exists.

## GPS Fix Loss Policy Is Not Fully Enforced For Autonomy

Autonomous GPS-dependent motion is not implemented. Before adding it:

- define valid GPS fix requirements
- define stale GPS timeout
- define station UI warnings
- define rover-side rejection behavior
- log GPS fix state alongside command requests

## RC + GPS Dry-Run Is Not Real Autonomy

The build flag `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1` is a safe
integration step, not waypoint following.

Validated behavior:

- USBDBG build identity for the completed validation:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`.
- This is the first firmware mode where MANUAL and GPS dry-run coexist in one
  firmware.
- MANUAL mode works with `control_source=RC_MANUAL`, and stick input changes
  `manual_steer_cmd`, `manual_throttle_cmd`, `left_cmd`, and `right_cmd`.
- AUTO mode prints `autonomy_dryrun=true`, GPS fields, and target
  distance/bearing fields.
- AUTO mode must keep `control_source=STOP`, `left_cmd=0`, and `right_cmd=0`.
- No motor movement occurs in AUTO dry-run.
- GPS may show increasing `gps_chars` indoors or near a window while still
  failing to fix; open-sky antenna placement is required before treating this
  as firmware failure.

Known boundaries:

- HC-12 is disabled/ignored in this mode.
- RC MANUAL mode may drive using the existing RC manual behavior.
- AUTO mode must force `left_cmd=0` and `right_cmd=0`.
- `autonomy_ready=true` only means RC, GPS, and target readiness checks passed.
- The placeholder target `35.571120,129.186050` is for distance/bearing
  calculation only.
- AUTO is still computation-only.
- Real motion is not enabled yet.
- Do not convert this mode into motor-driving autonomy without a separate
  safety design and test plan.

## Armed AUTO Output Reached But No Motion (Motor Deadband)

On 2026-05-29 the armed single-waypoint build (`AUTO_MOTION_ARMED=1`) reached
firmware-side final output for the first time: `mode=AUTO_RUNNING`,
`auto_motor_inhibit=false`, motion-grade GPS, all gates passing, and
`final_left_cmd=0.100` / `final_right_cmd=0.100`. No visible rover movement
occurred. Returning to MANUAL drove final commands to `0.000`.

Diagnosis:

- This is NOT a GPS problem and NOT an AUTO-gate problem; the firmware produced
  the commanded output correctly.
- `0.100` maps to only ≈1530 µs (30 µs above the 1500 µs neutral), almost
  certainly below the motor/ESC/friction deadband.

Rule — do not raise the AUTO command without the latch stop:

- Do NOT raise the armed AUTO command magnitude in an ungated, time-unbounded
  build. That is a runaway risk.
- The next motion test must use the guarded ground crawl build
  (`GROUND_CRAWL_TEST_MODE=1`). Armed motion is now gated to zero in any build
  without it.
- The crawl harness clamps to ±`GROUND_CRAWL_MAX_CMD` (default `0.08`, which is
  intentionally below the observed deadband) and latches a hard stop after
  `GROUND_CRAWL_MAX_AUTO_MS` (default `1200` ms; clears only on MANUAL). Step the
  command up only by setting `-DSINGLE_WP_CRAWL_BASE_CMD` for the candidate
  command and `-DGROUND_CRAWL_MAX_CMD` for the final clamp, under latch
  protection, wheels-off-ground or open-area-with-kill-switch.
- Confirm the deadband interpretation with `unclamped_final_left_cmd` /
  `unclamped_final_right_cmd` and `ground_crawl_block_reason` in USBDBG before
  raising the cap.

## Guarded Crawl 0.08 Validated; Current Target Too Close

The guarded ground crawl 0.08 test validated the safety harness:

- `GROUND_CRAWL_TEST_MODE=1` and `AUTO_MOTION_ARMED=1` were active.
- A good GPS window reached `AUTO_RUNNING` with `gps_motion_ready=true`,
  `gps_sats=5`, `gps_hdop≈1.34`, and `gps_block_reason=OK`.
- `ground_crawl_ready=true` and `ground_crawl_block_reason=OK` were observed.
- `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100` were clamped to
  `final_left_cmd=0.080` / `final_right_cmd=0.080`.
- `ground_crawl_latched_stop=true` stopped output after the duration limit.

Remaining cautions:

- The compile-time target later became too close (`target_distance_m≈3.9..4.4`),
  below `GROUND_CRAWL_MIN_TARGET_DISTANCE_M=5.0`, so the harness correctly
  blocked as `DISTANCE_OUT_OF_RANGE`.
- Intermittent GPS degradation can still block as `GPS_NOT_MOTION_READY`.
- This validates the guarded crawl safety behavior, not full autonomous
  driving.
- Before a 0.12 retry, reacquire current GPS and compute a fresh target
  `10..12` m away. Compile with both `SINGLE_WP_CRAWL_BASE_CMD=0.12` and
  `GROUND_CRAWL_MAX_CMD=0.12`; raising only the clamp does not raise the
  `0.100` candidate command. Do not reuse stale or too-close target coordinates.

## GPS-Gated Crawl Is Noisy For Motor Deadband Calibration

GPS remains intermittent enough that GPS-gated guarded crawl tests can be slow
and noisy when the immediate question is only motor deadband or drivetrain
response.

Use `MOTOR_PULSE_TEST_MODE=1` for GPS-independent deadband calibration:

- It does not use GPS readiness or waypoint target distance.
- It disables HC-12.
- It preserves RC MANUAL behavior.
- AUTO emits one neutral-stick pulse and then latches stop until MANUAL.
- It intentionally skips GPS initialization and GPS byte processing. Do not use
  motor pulse USBDBG lines to validate GPS.

Do not use this mode as autonomy proof. It only answers whether a given
`MOTOR_PULSE_CMD` produces physical drivetrain response.

Expected motor pulse GPS-looking fields:

- `gps_chars=0`
- `last_rmc_status=NA`
- `last_gga_fix_quality=NA`
- `gps_block_reason=NO_LOCATION`

These are expected in `MOTOR_PULSE_TEST_MODE=1` and do not mean the GPS module
or `Serial2/9600` wiring failed. Validate GPS separately with
`firmware/gps_uart_probe` or with the no-motion main-controller GPS build:
`FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1` and
`AUTO_MOTION_ARMED=0`, with no `MOTOR_PULSE_TEST_MODE`.

## Drivetrain Asymmetry Suspected

Current calibration observations:

- `MOTOR_PULSE_CMD=0.180` produced valid software output but no visible motion.
- `MOTOR_PULSE_CMD=0.220` produced visible motion.
- The 0.22 log showed symmetric software output:
  - `left_cmd=0.220`
  - `right_cmd=0.220`
  - `motor_pulse_ready=true`
  - `motor_pulse_block_reason=OK`
- The observed physical motion looked more like rotation than straight forward
  motion.
- Manual RC driving is also asymmetric:
  - forward tends to drift/curve left;
  - backward tends to drift/curve right.

Working hypothesis:

- The right-side drive may be stronger than the left-side drive, or the left
  side may have higher friction/deadband.
- Causes may be mechanical, electrical, motor/ESC-related, or software scaling.

Do not hide this in GPS/path planning. Use differential left/right motor pulse
calibration (`MOTOR_PULSE_LEFT_CMD`, `MOTOR_PULSE_RIGHT_CMD`) to characterize
the drivetrain, then apply corrections through the shared drive calibration
layer (`DRIVE_CALIBRATION_ENABLE=1`) used by both MANUAL and AUTO command
paths. Keep defaults identity/off until measured values justify a change.

Latest differential pulse observations:

- Left-only `+0.22`: left wheel rotates forward.
- Right-only `+0.22`: right wheel rotates forward, and the rover curves left as
  expected for right-only drive.
- Both `+0.22/+0.22`: both wheels appear to rotate forward, but the rover
  curves/rotates right instead of going straight.
- Both `-0.22/-0.22`: both wheels appear to rotate backward, but the rover
  curves left while reversing.

Interpretation:

- Basic motor polarity is likely not completely inverted.
- The motor pulse path bypasses RC stick angle remapping, so this is an
  actuator/drivetrain observation rather than a manual-stick remap artifact.
- Symmetric motor pulse output now points toward left/right drivetrain
  asymmetry, likely left side stronger or right side weaker under equal command.
- Manual RC curvature may still involve the RC angle remap or stick mixing, but
  do not use that as the primary explanation for motor pulse results.

Next debugging step: code-path inspection plus right-side compensation tests
through `DRIVE_CALIBRATION_ENABLE=1`.

Latest critical retest:

- `MOTOR_PULSE_LEFT_CMD=+0.25`, `MOTOR_PULSE_RIGHT_CMD=0.00` made the physical
  left wheel rotate forward while the physical right wheel rotated backward.
- `MOTOR_PULSE_LEFT_CMD=0.00`, `MOTOR_PULSE_RIGHT_CMD=+0.25` made the physical
  left wheel rotate backward while the physical right wheel rotated forward.

This result must be treated as a direct-output-path validation failure, not as a
simple left/right scale problem. Do not run compensation tests until USBDBG shows
the staged command path clearly:

- `motor_pulse_left_cmd` / `motor_pulse_right_cmd`
- `logical_left_cmd` / `logical_right_cmd`
- `calibrated_left_cmd` / `calibrated_right_cmd`
- `output_left_cmd` / `output_right_cmd`
- `motor_output_swap_lr=false` unless explicitly testing a physical output swap

The firmware now separates those stages. `MOTOR_PULSE_LEFT_CMD` and
`MOTOR_PULSE_RIGHT_CMD` are intended to be direct logical wheel commands, not
steering/throttle commands.

## Heading / BMI160 Is Not Integrated

The rover likely needs heading from GPS plus an IMU, but BMI160/IMU support is
not implemented in the current repo and the fixed D11/D12 IMU wiring is not yet
verified. Do not build waypoint following as if heading is already available.
The robust default `Wire` D11/D12 scanner currently shows the bus stuck low
before scanning, so no IMU address is verified. If the IMU remains unavailable,
continue GPS+RC workflow without IMU-dependent autonomy.

## ROS2 Is Skeleton-Only

`ros2_ws/src/` packages exist but are not functional runtime nodes. Do not claim
ROS2 integration is complete. Do not introduce ROS2 behavior until simple HC-12
telemetry, STOP, and dry-run station workflow are stable.

## micro-ROS Should Wait

Do not introduce micro-ROS on the rover now. The OpenRB firmware has a simple
HC-12 protocol that should be stabilized first.

## Generated Artifacts Policy Is Unsettled

Some mock mission outputs and generated figures are useful report evidence, but
logs and local test outputs can grow quickly. Decide per artifact whether it is:

- tracked report evidence
- reproducible generated output
- ignored local run data

## Firmware Source Must Match Board Firmware

The rover previously had firmware that printed:

```text
STAT,...,MANUAL_CENTER_STOP,...
```

That firmware source was not the active repo integrated firmware. Always confirm
the USB startup marker after upload before debugging behavior.
