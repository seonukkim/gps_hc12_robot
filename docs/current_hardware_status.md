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
- GPS FIX: confirmed on the current `Serial2/9600` probe path when the
  rover/GPS is moved farther outdoors with clearer sky view. Latest
  `gps_uart_probe` logs reached `gps_probe_state=STABLE_FIX`,
  `current_valid_fix=true`, RMC `A`, GGA fix quality `2`, `sats=9`,
  `hdop=3.56`, and `valid_fix_seconds_consecutive=58..60`.
- Main-controller GPS readiness is also recovered outdoors. In the
  single-waypoint experiment with `AUTO_MOTION_ARMED=0`, USBDBG showed
  `gps_location_valid=true`, `gps_location_fresh=true`, `gps_solution_valid=true`,
  `gps_dryrun_ready=true`, `gps_motion_ready=true`, `gps_ready=true`,
  `gps_block_reason=OK`, RMC `A`, GGA fix quality `2`, `gps_sats≈9..11`, and
  `gps_hdop≈1.46`.
- Integrated GPS `Serial2` diagnostic build: confirmed `gps_chars` increase,
  `gps_fix=true`, valid latitude/longitude, valid satellites/HDOP, HC-12
  disabled, and motors neutral
- Unified fixed-wiring RC + GPS autonomy dry-run build: compiles with
  `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1`; GPS uses `Serial2`, HC-12 is
  disabled, RC MANUAL can drive, and AUTO forces neutral while computing
  readiness only
- Unified dry-run validation: MANUAL mode works with
  `control_source=RC_MANUAL` and stick-controlled `left_cmd` / `right_cmd`;
  AUTO mode prints `autonomy_dryrun=true`, GPS fields, target
  distance/bearing, keeps `left_cmd=0` / `right_cmd=0`, and does not move motors
- Final unified dry-run USBDBG identification:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`; this confirms the running build is the unified
  MANUAL RC + AUTO GPS dry-run build, not default HC-12 mode and not GPS-only
  diagnostic mode
- Final unified dry-run AUTO observation: `mode=AUTO_READY`, `auto_sw=true`,
  `control_source=STOP`, `left_cmd=0.000`, and `right_cmd=0.000`
- Final unified dry-run MANUAL observation: `mode=MANUAL`, `auto_sw=false`,
  `control_source=RC_MANUAL`, and RC stick input changes manual command and
  left/right command fields
- Latest outdoor Manual/Auto recovery: the previous stuck-looking RC issue was
  caused by the station/controller being off. After restoring the
  controller/link, AUTO produced `mode=AUTO_READY`, `auto_sw=true`,
  `mode_us≈2001..2002`, and `control_source=STOP`; MANUAL produced
  `mode=MANUAL`, `auto_sw=false`, `mode_us≈1000..1001`, and
  `control_source=RC_MANUAL`. Manual stick input changed `steer_us` /
  `throttle_us`, and at least one MANUAL line produced nonzero
  `left_cmd` / `final_left_cmd`.
- Latest main-controller outdoor gate recovery: AUTO entry is verified again
  with `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001..2002`,
  `timeout_source=auto_entry`, and `timeout_ok=true`. `AUTO_MOTION_ARMED=0` and
  `auto_motor_inhibit=true` kept `final_left_cmd=0.000` and
  `final_right_cmd=0.000`.
- Latest no-motion AUTO waypoint dry-run is successful. With compile-time
  target `35.5705010,129.1872696`, MANUAL showed nearby
  `target_distance_m≈8.4..15.2` and `distance_allowed=true`; AUTO entry showed
  `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001`, `timeout_ok=true`,
  `safety_ready=true`, `candidate_left_cmd=0.100`, and
  `candidate_right_cmd=0.100`. Because `AUTO_MOTION_ARMED=0`,
  `auto_motor_inhibit=true` kept `final_left_cmd=0.000` and
  `final_right_cmd=0.000`.
- Latest armed AUTO output reached, but no visible motion (motor deadband
  suspected). With `AUTO_MOTION_ARMED=1` and motion-grade GPS, USBDBG showed
  `mode=AUTO_RUNNING`, `control_source=AUTO`, `auto_motor_inhibit=false`,
  `gps_motion_ready=true`, `gps_block_reason=OK`, RMC `A`, GGA quality `2`,
  `gps_sats≈7..9`, `gps_hdop≈1.0..1.4`, `distance_allowed=true`,
  `safety_ready=true`, `candidate_left_cmd=0.100`, `candidate_right_cmd=0.100`,
  and for the first time `final_left_cmd=0.100` / `final_right_cmd=0.100`.
  **No visible rover movement occurred.** Returning to MANUAL drove final
  commands to `0.000`. This is firmware-side armed output success but
  **not** a physical ground crawl. `0.100` maps to only ≈1530 µs (30 µs above
  the 1500 µs neutral), almost certainly below the motor/ESC/friction deadband.
  The next motion test must use the guarded ground crawl build
  (`GROUND_CRAWL_TEST_MODE=1`); the AUTO command must not be raised without the
  crawl clamp + latch stop. Floor driving remains **not** approved.
- Latest guarded ground crawl 0.08 safety validation: with
  `GROUND_CRAWL_TEST_MODE=1`, `AUTO_MOTION_ARMED=1`, and
  `GROUND_CRAWL_MAX_CMD=0.08`, a good GPS window reached `AUTO_RUNNING` with
  `gps_motion_ready=true`, `gps_sats=5`, `gps_hdop≈1.34`, and
  `gps_block_reason=OK`. USBDBG showed `ground_crawl_ready=true`,
  `ground_crawl_block_reason=OK`, `candidate_left_cmd=0.100`,
  `candidate_right_cmd=0.100`, and clamped final commands
  `final_left_cmd=0.080` / `final_right_cmd=0.080`. The latch stop then
  asserted `ground_crawl_latched_stop=true` and forced zero output. Later
  target distance dropped to `≈3.9..4.4` m, below
  `GROUND_CRAWL_MIN_TARGET_DISTANCE_M=5.0`, and the harness correctly blocked
  as `DISTANCE_OUT_OF_RANGE`. Intermittent GPS degradation also blocked motion
  as `GPS_NOT_MOTION_READY` or `LATCHED_STOP`. This validates the guarded crawl
  safety harness, but it is still **not** full autonomous driving.
- Latest guarded crawl 0.12 cap-only observation: `GROUND_CRAWL_MAX_CMD=0.120`
  allowed the harness to cap up to 0.120, but the candidate command was still
  `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100`, so final commands
  remained `final_left_cmd=0.100` / `final_right_cmd=0.100`. The latch still
  worked. Firmware now separates candidate speed from final clamp with
  `SINGLE_WP_CRAWL_BASE_CMD` (default `0.100`). A future 0.12 retry must set
  both `SINGLE_WP_CRAWL_BASE_CMD=0.12` and `GROUND_CRAWL_MAX_CMD=0.12`.
- Motor pulse calibration mode is now available for GPS-independent motor
  deadband checks. Compile with `MOTOR_PULSE_TEST_MODE=1`; HC-12 is disabled,
  GPS readiness/target distance are not used, RC MANUAL is preserved, and AUTO
  emits one neutral-stick pulse for `MOTOR_PULSE_MS` before latching stop until
  MANUAL.
- In `MOTOR_PULSE_TEST_MODE=1`, GPS is intentionally not initialized or read.
  USBDBG fields such as `gps_chars=0`, `last_rmc_status=NA`,
  `last_gga_fix_quality=NA`, and `gps_block_reason=NO_LOCATION` are expected in
  that mode and must not be interpreted as GPS hardware failure. Use
  `gps_uart_probe` or the no-motion single-waypoint main-controller build
  (`FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`) for GPS validation.
- Final unified dry-run GPS observation: `gps_chars` increases continuously,
  open-sky antenna placement produced `gps_fix=true`, and
  `target_distance_m` / `target_bearing_deg` were computed
- Latest outdoor single-waypoint safety state: GPS was usable at several
  points (`gps_fix=true`, `gps_ready=true` when HDOP was good), but the
  compile-time target `35.570675,129.186769` was stale while runtime GPS was
  around `35.5716,129.1875`. `target_distance_m≈100..131` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false`,
  `safety_ready=false`, candidate commands stayed zero, and
  `AUTO_MOTION_ARMED=0` / `auto_motor_inhibit=true` kept AUTO final commands at
  zero.
- Latest outdoor nearby dry-run: target override worked with
  `SINGLE_WP_TARGET_LAT=35.5707680` and
  `SINGLE_WP_TARGET_LON=129.1867906`; runtime printed
  `target_lat=35.570768`, `target_lon=129.186791`. Outdoor GPS quality was good
  in many lines (`gps_ready=true`, `gps_sats=7..8`, `gps_hdop≈0.95..1.98`), and
  `target_distance_m` dropped below `30.0` m, so `distance_allowed=true` was
  observed. The run was still blocked because mode stayed mostly MANUAL,
  `auto_sw=false`, `timeout_ok=false`, `safety_ready=false`, and candidate
  commands stayed zero.
- Latest outdoor dry-run after timeout fix: timeout diagnostics printed
  `timeout_source=auto_entry`, `auto_entry_ms=NA`, `auto_elapsed_ms=NA`,
  `timeout_limit_ms=15000`, and `timeout_ok=true`, so the MANUAL-wait timeout
  issue appears improved. Target override was confirmed with
  `target_lat=35.570834`, `target_lon=129.186958`, and `target_ready=true`.
  RC remained in MANUAL (`mode_us≈1000..1001`, `control_source=RC_MANUAL`).
  GPS UART was alive (`gps_chars` increased), but no fix was acquired:
  `gps_fix=false`, `gps_lat=NA`, `gps_lon=NA`, `gps_sats=0`,
  `gps_hdop=99.99`, and `gps_age_ms=NA`. Therefore
  `target_distance_m=NA`, `distance_allowed=false`, `safety_ready=false`, and
  candidate/final commands stayed zero.
- GPS sky-fix validation: previous `gps_sats=0` and `gps_hdop=99.99` was poor
  indoor/window-side reception, not UART or firmware failure; moving the
  external antenna farther outside into open sky produced fix
- Purple module: fixed wiring is believed to be SDA on OpenRB D11 / PA08 and
  SCL on OpenRB D12 / PA09; OpenRB-150 variant files confirm D11/D12 are the
  board's default `Wire` pins, but IMU I2C presence remains unverified
- Fixed Wiring Plan: GPS remains on the central connector / `Serial2`; HC-12
  remains physically mounted as-is until its current wiring is audited

## GPS Antenna Frame Vs Rover Body Frame

Observed sensor-frame issue:

- During sky-view and single-waypoint candidate dry-run tests, the external GPS
  antenna was placed far outside while the rover body remained indoors.
- Therefore `gps_lat` / `gps_lon` represented the antenna location, not the
  rover body location.

Interpretation:

- This setup is valid for GPS reception, UART, satellite fix, and
  distance/bearing computation validation.
- This setup is not valid for floor navigation or rover body localization.
- An IMU cannot fully correct a detached GPS antenna into rover body position.
- The IMU may help later with heading and rotation sensing, but it does not
  replace a rover-mounted GPS position source.

Requirement for real navigation:

- Mount the GPS antenna rigidly on the rover, or use a fixed, measured antenna
  offset from the rover body frame.
- Do not approve `AUTO_MOTION_ARMED=1` floor testing until this is resolved.

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
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet.
- Rain did not prevent the observed fix once the antenna had open-sky exposure,
  but electronics and antenna connectors must be protected from water.

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
- Purple module uses fixed D11/D12 I2C-style wiring and should not be treated as
  UART.
- IMU SCL is believed to be connected to OpenRB D12 / PA09.
- IMU SDA is believed to be connected to OpenRB D11 / PA08.
- OpenRB-150 variant files confirm:
  - Arduino D11 = SDA = PA08
  - Arduino D12 = SCL = PA09
  - `PIN_WIRE_SDA = 11`
  - `PIN_WIRE_SCL = 12`
  - `Wire` is constructed using `PIN_WIRE_SDA` and `PIN_WIRE_SCL`
- Do not ask to move the IMU wires to hardware SDA/SCL.
- The default `Wire.begin()` scanner is the correct primary scanner for the
  current D11/D12 wiring.
- D11/D12 and swapped D12/D11 assignments can be tested with compile-time pin
  override in the bit-bang scanner without moving wires, but that scanner is now
  secondary.
- All-address detection is scanner/bus failure, not evidence of many devices.
- Hardened bit-bang tests with both SDA=D11/SCL=D12 and SDA=D12/SCL=D11
  repeatedly reported `released_sda=LOW`, `released_scl=LOW`, `SDA stuck low`,
  `SCL stuck low`, `raw_found_count=0`, `valid_found_count=0`, and
  `stable_valid_address=NA`.
- D11/D12 stuck-low should be treated as an electrical or bus issue, such as
  IMU power, GND, pullups, or a stuck device, not as a pin mapping issue.
- Robust default `Wire` scanner result:
  - scanner runs and prints repeated scan passes
  - `pre_scan_sda=LOW`
  - `pre_scan_scl=LOW`
  - `BUS_STUCK_LOW_BEFORE_SCAN`
  - `found_count=0`
  - `stable_valid_address=NA`
- The scanner is not hanging; it is correctly refusing to scan while the bus is
  stuck low before address probing.
- IMU presence and exact device identity remain unverified.
- Continue the GPS+RC workflow without IMU support for now.

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

- Unified RC + GPS dry-run validation is complete. This is the first mode where
  MANUAL and GPS dry-run coexist in one firmware. AUTO is still
  computation-only and real motion is not enabled yet.
- Single-waypoint candidate dry-run with `AUTO_MOTION_ARMED=0` confirms
  candidate-command safety. The earliest nearby retest was blocked by target
  override plumbing: the compile command attempted
  `SINGLE_WP_TARGET_LAT=35.5716800` and `SINGLE_WP_TARGET_LON=129.1866516`,
  while runtime USBDBG still printed `target_lat=35.571120` and
  `target_lon=129.186050`. Later builds fixed and verified target override.
- Earliest nearby candidate retest status: safe failed validation. GPS reached ready
  state on at least one line (`gps_hdop=1.19`, `gps_ready=true`), but the old
  placeholder target kept `target_distance_m` around `40` to `60` m,
  `distance_allowed=false`, `safety_ready=false`, candidate commands at zero,
  and final outputs at zero.
- Runtime `target_lat` / `target_lon` are the source of truth before
  interpreting `distance_allowed` or approving any bench test.
- Firmware source now supports `SINGLE_WP_TARGET_LAT` /
  `SINGLE_WP_TARGET_LON`; USBDBG target override was verified with
  `SINGLE_WP_TARGET_LAT=35.5710210` and
  `SINGLE_WP_TARGET_LON=129.1864016`. Runtime printed
  `target_override_enabled=true`, `target_source=compile_time`,
  `target_lat_macro=35.5710210`, `target_lon_macro=129.1864016`,
  `target_lat=35.571021`, and `target_lon=129.186402`.
- Nearby candidate command remains incomplete. GPS fix was true, but current
  GPS was around `gps_lat≈35.56752..35.56756`, `gps_lon≈129.18688`, so
  `target_distance_m≈380..392` exceeded `max_target_distance_m=30.0`.
  Therefore `distance_allowed=false`, `safety_ready=false`, candidate commands
  stayed zero, and final outputs stayed zero.
- Next-day GPS retest remained safely blocked. The antenna was placed outside
  again and runtime GPS moved to approximately `gps_lat=35.571310`,
  `gps_lon=129.188630`, while firmware still used the previous compile-time
  target `target_lat=35.567560`, `target_lon=129.186792`.
  `target_override_enabled=true` and `target_source=compile_time`, so target
  override itself is working, but `target_distance_m≈448.9` exceeded
  `max_target_distance_m=30.0`. `distance_allowed=false` and
  `safety_ready=false` were expected.
- In the next-day retest, `gps_fix=true` appeared but `gps_ready=false` remained
  because GPS freshness/quality was not stable: `gps_age_ms` was very large in
  many lines, `gps_sats` fluctuated, and `gps_hdop` was often `99.99` and only
  occasionally around `4.7`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept final outputs at
  zero.
- Latest nearby attempt remained safely blocked. Target override was verified
  with `SINGLE_WP_TARGET_LAT=35.5713100` and
  `SINGLE_WP_TARGET_LON=129.1885416`; runtime printed
  `target_lat=35.571310`, `target_lon=129.188542`. GPS UART was alive and GPS
  fix was eventually acquired in MANUAL at `gps_lat=35.571384`,
  `gps_lon=129.187514`, with `gps_sats=4` and `gps_hdop=3.39..4.12`.
  However, `target_distance_m=93.3` exceeded `max_target_distance_m=30.0`, so
  `distance_allowed=false`, `safety_ready=false`, candidate commands stayed
  zero, and final outputs stayed zero. `gps_age_ms` was initially fresh but
  later grew stale.
- Latest window/outside-antenna attempt remained safely blocked. Target
  override was verified with `SINGLE_WP_TARGET_LAT=35.5713840` and
  `SINGLE_WP_TARGET_LON=129.1874256`; runtime printed
  `target_lat=35.571384`, `target_lon=129.187426`. GPS fix appeared around
  `gps_lat≈35.571284`, `gps_lon≈129.188456`, but reception was unstable:
  `gps_sats` often became `0`, `gps_hdop` often became `99.99`, and
  `gps_age_ms` grew very large. `target_distance_m≈93.9` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false`,
  `gps_ready=false`, `safety_ready=false`, candidate commands stayed zero, and
  final outputs stayed zero.
- Latest outdoor Manual/Auto recovery is complete. The station/controller must
  be powered on for meaningful RC mode tests; controller-off can make the RC
  stream appear stuck or failsafe-like. With the link restored, MANUAL and AUTO
  switch positions were confirmed using `mode_us≈1000..1001` and
  `mode_us≈2001..2002`.
- Latest outdoor candidate dry-run remained safely blocked by stale target:
  runtime GPS was around `35.5716,129.1875`, while the target remained
  `35.570675,129.186769`. `target_distance_m≈100..131` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false` and
  `safety_ready=false` were expected.
- Latest outdoor nearby dry-run made partial progress. With target
  `35.570768,129.186791`, GPS was repeatedly ready outdoors and
  `target_distance_m` decreased through `27.1`, `25.4`, `23.5`, `21.1`,
  `18.8`, and `18.5` m. `distance_allowed=true` was observed once the target
  was within `max_target_distance_m=30.0`.
- The same nearby dry-run is still not a successful AUTO candidate validation:
  `mode` stayed mostly `MANUAL`, `auto_sw=false`, `timeout_ok=false`,
  `safety_ready=false`, and `candidate_left_cmd=0.000` /
  `candidate_right_cmd=0.000`. Safety must be verified in `AUTO_READY`, not
  only in MANUAL.
- Single-waypoint timeout semantics have been updated in firmware. The timeout
  now starts on AUTO entry, resets when leaving AUTO, and no longer consumes
  time while waiting in MANUAL for GPS/target setup. USBDBG reports
  `timeout_source=auto_entry`, `auto_entry_ms`, `auto_elapsed_ms`,
  `timeout_limit_ms`, and `timeout_ok`.
- Latest post-timeout-fix dry-run was blocked by GPS no-fix, not timeout:
  `gps_chars` increased continuously, but `gps_fix=false`, `gps_sats=0`, and
  `gps_hdop=99.99`. Reacquire stable GPS fix before attempting AUTO_READY
  candidate validation.
- Latest GPS-only `Serial2` probe confirms the current blocker is still GPS
  satellite fix stability, not UART, RC, target override, or timeout. The probe
  was built with `GPS_PROBE_MODE=2` and `GPS_PROBE_BAUD=9600`; NMEA characters
  streamed continuously, but most lines showed RMC status `V`, GGA fix quality
  `0`, `sats=0`, and `hdop=99.99`. Short bursts reached RMC `A`, valid
  lat/lon, `sats=4..5`, and `hdop≈1.77..2.48`, then fell back to no-fix.
  Treat this as an intermittent GPS acquisition failure. TinyGPS++ cached
  coordinates after fallback to RMC `V` are not a stable current rover position.
- GPS fix recovery was confirmed after moving the rover/GPS farther outdoors.
  The same `Serial2/9600` probe then transitioned through
  `INTERMITTENT_FIX` and reached `STABLE_FIX` with
  `valid_fix_seconds_consecutive=58..60`, RMC `A`, GGA quality `2`, `sats=9`,
  `hdop=3.56`, `age_ms≈85..89`, and lat/lon around
  `35.57029,129.187078`. This attributes the previous no-fix primarily to
  placement/sky view, not firmware or UART.
- Main-controller GPS/AUTO gate recovery was confirmed in a prior run, but that
  nearby waypoint candidate dry-run was incomplete. The compile-time target
  `35.5702838,129.1869899` was stale while current GPS was around
  `35.57050,129.18736`, so `target_distance_m≈41` exceeded
  `max_target_distance_m=30.0`. `distance_allowed=false`,
  `safety_ready=false`, and candidate commands stayed zero.
- That stale-target blocker has been resolved for no-motion dry-run by
  recompiling with target `35.5705010,129.1872696`; candidate command
  generation has been observed while final motor outputs remained inhibited.
- Some AUTO dry-run lines may show motion-level `gps_ready=false` /
  `gps_block_reason=BAD_HDOP` while `safety_ready=true`. This is valid only in
  `AUTO_MOTION_ARMED=0` when the active dry-run gate is ready. It is not
  acceptable for any future armed motion.
- GPS readiness diagnostics have been hardened. USBDBG now separates
  `gps_location_valid` from `gps_ready`, prints `gps_age_ok`, `gps_sats_ok`,
  `gps_hdop_ok`, readiness constants, and `gps_block_reason`, and only prints
  operational `gps_lat` / `gps_lon` when `gps_ready=true`. Cached coordinates
  are available only as debug fields: `gps_cached_lat`, `gps_cached_lon`, and
  `gps_cached_age_ms`.
- Single-waypoint target distance and bearing are now computed only when
  `gps_ready=true`; stale cached coordinates no longer produce operational
  target distance/bearing values. The experiment also prints `gps_coord_sane`
  and blocks safety when a ready coordinate is implausibly far from the
  compile-time target.
- GPS readiness is now tiered for single-waypoint dry-run work:
  `gps_solution_valid` checks valid/fresh location plus NMEA fix status when
  available, `gps_dryrun_ready` allows no-motion candidate calculations with
  `GPS_DRYRUN_MIN_SATS=4` and `GPS_DRYRUN_MAX_HDOP=6.0`, and
  `gps_motion_ready` keeps stricter motion gating with
  `GPS_MOTION_MIN_SATS=5` and `GPS_MOTION_MAX_HDOP=2.5`. `gps_ready` remains
  the stricter motion-level field.
- In `AUTO_MOTION_ARMED=0`, target distance/bearing and candidate commands may
  be computed from `gps_dryrun_ready`; final outputs remain inhibited. In any
  future `AUTO_MOTION_ARMED=1` build, `gps_motion_ready` is required.
- A brief PPM/failsafe-like glitch was observed with `mode=FAILSAFE`,
  `rc_ok=false`, `steer_us≈495`, `throttle_us≈2504`, and
  `control_source=STOP`. This is the correct safe response.
- Window/outside-thrown antenna placement is not equivalent to rover body
  localization.
- Target override success must be interpreted separately from
  `distance_allowed` / `safety_ready`.
- Next required validation before any armed motion:
  - use the guarded ground crawl build (`GROUND_CRAWL_TEST_MODE=1`) for the next
    motion test; armed motion is now gated to zero in any build without it
  - prepare a strict wheel-off-ground bench procedure
  - keep the rover physically lifted
  - verify RC manual override, STOP, and failsafe before any armed variant
  - confirm outdoor GPS readiness and nearby target gates again
  - repeat the no-motion `AUTO_MOTION_ARMED=0` validation if the rover or target
    changes
  - do not enable floor driving until wheel-off-ground logs prove the expected
    behavior
  - do not raise the AUTO command past the deadband except through the guarded
    crawl harness; use `-DSINGLE_WP_CRAWL_BASE_CMD` for candidate speed and
    `-DGROUND_CRAWL_MAX_CMD` for the final clamp, in small steps, under the
    crawl latch stop
  - IMU is optional for the current GPS+RC single-waypoint preparation stage;
    do not block candidate dry-run work on IMU availability
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
- Do not add real waypoint following or weaken STOP, heartbeat, failsafe,
  manual override, RC safety, or wheel-off-ground motor-test rules.
- Station-side HC-12-USB device is not confirmed.
- `/dev/ttyUSB*` is not visible yet on the station/development side.
- Need to confirm whether station HC-12-USB is installed and connected to MPC.
