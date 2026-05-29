# Autonomy Plan

This project does not have real autonomous waypoint following yet. The current
autonomy work is limited to a safe fixed-wiring dry-run mode that combines RC
manual driving with GPS readiness calculations.

## Status

Unified fixed-wiring RC + GPS dry-run validation is complete. Single-waypoint
candidate dry-run with `AUTO_MOTION_ARMED=0` confirms candidate-command safety,
and compile-time target override is now verified in USBDBG. Nearby candidate
command validation is still incomplete. The next-day GPS retest was safely
blocked because the antenna was placed at a new location, making the previous
compile-time target stale and about `448.9` m away; GPS freshness/quality also
did not satisfy readiness. A later nearby attempt acquired GPS fix, but the
target was still `93.3` m from the actual runtime fix, so the distance gate
blocked it. The latest window/outside-antenna attempt also remained safely
blocked because GPS quality was unstable and the antenna position was not the
rover body position. The latest outdoor Manual/Auto recovery test confirmed
that the previous RC issue was caused by the station/controller being off:
MANUAL and AUTO switching now work again, and outdoor GPS can become usable.
That run was still safety-blocked because the compile-time target was stale
(`target_distance_m≈100..131` m, above `max_target_distance_m=30.0`). The
latest outdoor nearby dry-run then showed real progress: target override worked,
outdoor GPS readiness was repeatedly good, and `distance_allowed=true` was
observed after `target_distance_m` dropped below `30.0` m. It was still blocked
because the rover stayed mostly in MANUAL, `timeout_ok=false`,
`safety_ready=false`, and no candidate commands were produced. The
single-waypoint timeout semantics have now been improved so the timeout starts
on AUTO entry instead of being consumed during long MANUAL GPS waiting. The
latest post-timeout-fix outdoor attempt confirmed the new timeout fields and
target override, but it was blocked by `gps_fix=false`, `gps_sats=0`, and
`gps_hdop=99.99`. Bench testing and floor waypoint driving are not approved yet.

Full coverage driving from `mission.json` / `mission.csv` is intentionally not
the next step. The rover must first prove one carefully bounded waypoint motion
with explicit safety gates, GPS validity policy, heading plan, manual override,
and wheel-off-ground checks.

Staged plan:

1. Dry-run complete: MANUAL RC and AUTO GPS distance/bearing computation coexist
   with AUTO motor output forced to zero.
2. Single-waypoint candidate-command dry-run: compute candidate commands behind
   safety gates while `AUTO_MOTION_ARMED=0` forces final motor output to zero.
3. Target override diagnostics complete: USBDBG now confirms
   `target_override_enabled=true`, `target_source=compile_time`, and runtime
   `target_lat` / `target_lon` matching the compile-time macros.
4. RC recovery complete: with the station/controller powered and linked, MANUAL
   shows `mode_us≈1000..1001` and `control_source=RC_MANUAL`; AUTO shows
   `mode_us≈2001..2002`, `mode=AUTO_READY`, and `control_source=STOP`.
5. Outdoor GPS/distance progress: GPS readiness is now good outdoors, and
   `distance_allowed=true` has been observed in MANUAL with a nearby target.
   This is not enough for success because `safety_ready` must be verified in
   AUTO_READY.
6. Full outdoor fixed-frame dry-run: take the rover and GPS outside together,
   acquire a fresh fix in MANUAL, recompute the target from that actual fix, and
   rerun with `AUTO_MOTION_ARMED=0`. Do not proceed until
   `target_distance_m <= max_target_distance_m`, GPS freshness/quality gates
   pass, and `timeout_ok` has not expired.
7. Timeout semantics improvement complete in firmware: the single-waypoint
   experiment tracks AUTO entry time, resets the AUTO timeout when leaving
   AUTO, and reports `timeout_source=auto_entry`, `auto_entry_ms`,
   `auto_elapsed_ms`, `timeout_limit_ms`, and `timeout_ok`.
8. GPS reacquisition required: the latest attempt showed `gps_chars` increasing
   but no valid GPS fix. Reacquire stable outdoor GPS in MANUAL before the next
   AUTO_READY candidate validation.
9. Sensor-frame validation: keep the GPS antenna fixed to the rover body in open
   sky. IMU diagnostics remain useful, but IMU is optional for the current
   GPS+RC single-waypoint preparation stage and must not block candidate dry-run
   work.
10. Bench test with wheels lifted: compile the same experiment with
   `AUTO_MOTION_ARMED=1` only after explicit approval and verify low-speed
   output, timeout, arrival stop, GPS rejection, and manual override.
11. Low-speed floor test: only after wheel-off-ground behavior and sensor-frame
   assumptions are validated.
12. Multi-waypoint motion: only after single-waypoint behavior is proven.
13. Coverage path / lawnmower driving: last step, after mission sequencing,
   heading control, logging, and safety policy are complete.

## GPS Antenna Frame Vs Rover Body Frame

GPS coordinates are antenna coordinates. During the recent single-waypoint
candidate dry-run, the GPS antenna was placed far outside while the rover body
remained indoors. That setup is acceptable for validating GPS reception,
`gps_fix`, distance/bearing computation, and AUTO motor inhibition, but it is
not valid for floor navigation.

A detached antenna means `gps_lat` / `gps_lon` represents the antenna location,
not the rover body location. An IMU cannot fully correct a detached or
free-moving GPS antenna into rover body position. The IMU may help later with
heading, rotation, tilt, and short-term motion sensing, but it does not replace
a rover-mounted GPS position source.

Real outdoor navigation requires one of these conditions:

- the GPS antenna is rigidly mounted on the rover; or
- the antenna offset from the rover body frame is fixed, measured, and modeled.

Until that is true, do not proceed to floor waypoint driving and do not approve
`AUTO_MOTION_ARMED=1` floor tests.

## Next Required Validation Before Motion

- Go fully outdoors with the rover and GPS fixed together, acquire a fresh GPS
  fix in MANUAL, recompute a nearby target from that actual fix, then rerun the
  single-waypoint experiment with `AUTO_MOTION_ARMED=0`.
- Confirm the station/controller is powered on and linked before interpreting
  RC mode values; controller-off can make the PPM stream appear stuck or
  failsafe-like.
- Verify Manual/Auto switch positions before the dry-run: MANUAL should show
  `mode_us≈1000` and `control_source=RC_MANUAL`; AUTO should show
  `mode_us≈2000`, `mode=AUTO_READY`, and `control_source=STOP`.
- Confirm the target override fields still match the intended target, then
  interpret `target_distance_m`, `distance_allowed`, and `safety_ready`.
- `distance_allowed=true` in MANUAL is not enough; verify the same nearby-target
  condition in `AUTO_READY` with `gps_ready=true`, `timeout_ok=true`, and
  `safety_ready=true`.
- Confirm GPS readiness using `gps_age_ms`, `gps_hdop`, and `gps_sats`; do not
  rely on `gps_fix=true` alone.
- `gps_chars` increasing only proves serial/NMEA input is alive. Do not proceed
  to AUTO candidate validation when `gps_fix=false`, `gps_lat=NA`,
  `gps_lon=NA`, `gps_sats=0`, or `gps_hdop=99.99`.
- Run candidate validation after entering AUTO; MANUAL GPS waiting no longer
  consumes the single-waypoint AUTO timeout in the updated firmware.
- In MANUAL, expect `auto_entry_ms=NA` and `auto_elapsed_ms=NA`; after switching
  to AUTO, expect `timeout_source=auto_entry`, a numeric `auto_entry_ms`, a
  small `auto_elapsed_ms`, and `timeout_ok=true` until the AUTO timeout limit is
  exceeded.
- Run wheel-off-ground bench testing only after safety gates and sensor-frame
  assumptions are clear.
- Keep `AUTO_MOTION_ARMED=0` for floor or indoor tests.
- IMU status: optional for the current GPS+RC single-waypoint stage. The
  default `Wire` scanner currently shows D11/D12 stuck low before scanning, so
  no IMU address is verified and no IMU data may be used for autonomy yet.

## Current Fixed-Wiring Dry-Run Mode

Compile-time flag:

```text
FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1
```

Behavior:

- GPS uses the fixed central OpenRB connector on `Serial2` at `9600`.
- HC-12 is disabled/ignored because it may conflict with the fixed GPS
  `Serial2` wiring.
- RC PPM remains enabled and is the safety/manual interface.
- MANUAL mode preserves current RC manual driving behavior and reports
  `control_source=RC_MANUAL`.
- AUTO switch position does not drive motors. It forces `left_cmd=0` and
  `right_cmd=0`, then prints readiness/debug fields over USB.
- STOP, RC failsafe, and manual override must not be weakened.

AUTO dry-run debug fields:

```text
autonomy_dryrun=true
target_lat=35.571120
target_lon=129.186050
target_distance_m=...
target_bearing_deg=...
gps_ready=...
target_ready=...
autonomy_ready=...
```

The current placeholder target is hard-coded in the firmware as:

```text
lat=35.571120
lon=129.186050
```

The placeholder target is only used to compute distance and bearing from the
current GPS position. It must not cause motor movement.

## Onboard Mission Dry-Run Implementation

The current onboard mission dry-run is implemented inside
`firmware/openrb_robot_controller/openrb_robot_controller.ino` and is enabled
only by:

```text
FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1
```

The dry-run target is a small placeholder config section in the firmware:

- `DRYRUN_TARGET_AVAILABLE`
- `DRYRUN_TARGET_LAT`
- `DRYRUN_TARGET_LON`
- `DRYRUN_GPS_READY_MAX_AGE_MS`

The Arduino-side geodesy helpers are:

- `dryrunDistanceMeters(...)`: haversine distance in meters.
- `dryrunBearingDegrees(...)`: initial bearing in degrees, normalized to
  `0..360`.

These helpers are currently Arduino-side only, so there is no Python unit test
for them. Manual validation is by USB debug output in the dry-run build:

- verify `target_distance_m` is finite when `gps_fix=true`;
- verify `target_bearing_deg` stays in `0..360`;
- verify `gps_ready=true` only when GPS has a valid recent location;
- verify AUTO mode still reports `left_cmd=0` and `right_cmd=0`;
- verify switching RC back to MANUAL restores `control_source=RC_MANUAL`.

This is the onboard preparation step for future mission execution. It is not
real waypoint following.

## Validation Result

The unified fixed-wiring dry-run firmware is complete and was tested with
`FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1`.

Observed:

- USBDBG identified the build as unified dry-run, not default and not GPS-only
  diagnostic:
  - `fixed_wiring_gps_serial2_diag=false`
  - `hc12_enabled=false`
  - `autonomy_dryrun=true`
- `GPS_SERIAL=Serial2`.
- HC-12 was disabled/ignored.
- MANUAL mode worked with RC control.
- `control_source=RC_MANUAL` in MANUAL mode.
- Stick input changed `manual_steer_cmd`, `manual_throttle_cmd`, `left_cmd`,
  and `right_cmd`.
- AUTO mode printed `autonomy_dryrun=true`.
- AUTO mode printed GPS fields, `target_lat`, `target_lon`, and target
  distance/bearing fields.
- With the antenna outside/open sky, `gps_fix=true` was observed.
- Earlier `gps_sats=0` / `gps_hdop=99.99` was poor antenna placement, not UART
  failure.
- AUTO mode used `control_source=STOP` and kept `left_cmd=0` and
  `right_cmd=0`.
- No motor movement occurred in AUTO dry-run.

Rule:

- This is the first firmware mode where MANUAL and GPS dry-run coexist in one
  firmware.
- AUTO is still computation-only.
- Real motion is not enabled yet.
- MANUAL remains the recovery/manual override path.
- HC-12 remains disabled in this fixed-wiring GPS mode.

## Readiness Logic

- `gps_ready=true` when GPS has a valid, recent location.
- `target_ready=true` when the placeholder waypoint is available.
- `autonomy_ready=true` only when RC is valid, the RC AUTO switch is on, GPS is
  ready, and the target is ready.

`autonomy_ready=true` is still only a dry-run state. It is not permission to
move.

## Single-Waypoint Experiment

Compile-time flag:

```text
FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1
```

Default motion arming flag:

```text
AUTO_MOTION_ARMED=0
```

Behavior:

- GPS uses fixed `Serial2` at `9600`.
- HC-12 is disabled/ignored.
- RC PPM remains enabled.
- MANUAL mode preserves existing RC manual driving and reports
  `control_source=RC_MANUAL`.
- AUTO mode uses one placeholder target only.
- AUTO mode computes `target_distance_m` and `target_bearing_deg`.
- AUTO mode prints candidate left/right commands.
- Candidate commands are straight low-speed placeholders. Target bearing is
  printed for inspection, but heading control is not implemented yet.
- With `AUTO_MOTION_ARMED=0`, final `left_cmd` and `right_cmd` remain zero even
  when candidate commands are nonzero.
- With `AUTO_MOTION_ARMED=1`, low-speed candidate output is allowed only if all
  safety gates pass. This is reserved for a later explicit wheel-off-ground
  bench test.

Target override rule:

- Runtime USBDBG `target_lat` and `target_lon` are the source of truth.
- Compile-time target override flags must be verified in USBDBG before using
  `target_distance_m`, `distance_allowed`, or `safety_ready` to approve any next
  step.
- With both override macros provided, USBDBG should print
  `target_override_enabled=true` and `target_source=compile_time`.
- Without override macros, USBDBG should print `target_override_enabled=false`
  and `target_source=fallback`, using the existing placeholder target.
- USBDBG also prints `target_lat_macro`, `target_lon_macro`,
  `max_target_distance_m`, and `arrival_radius_m` for diagnostics.
- The latest nearby retest attempted to compile with
  `SINGLE_WP_TARGET_LAT=35.5716800` and `SINGLE_WP_TARGET_LON=129.1866516`, but
  runtime USBDBG still printed `target_lat=35.571120` and
  `target_lon=129.186050`.
- Therefore the nearby retest was blocked by target override plumbing and must
  not be counted as a successful nearby candidate-command test.
- A later target override check compiled with
  `SINGLE_WP_TARGET_LAT=35.5710210` and `SINGLE_WP_TARGET_LON=129.1864016`.
  USBDBG correctly printed `target_override_enabled=true`,
  `target_source=compile_time`, matching macro strings, and runtime
  `target_lat=35.571021`, `target_lon=129.186402`.
- That proves target override plumbing is fixed. It does not prove nearby
  candidate command behavior, because the current GPS position was around
  `35.56752..35.56756,129.18688`, making `target_distance_m≈380..392`.
- A next-day retest kept target override working
  (`target_override_enabled=true`, `target_source=compile_time`) but the antenna
  position changed to about `35.571310,129.188630` while the firmware target was
  `35.567560,129.186792`, producing `target_distance_m≈448.9`.
- That next-day run was also blocked by GPS readiness: `gps_fix=true` appeared,
  but `gps_ready=false` remained because `gps_age_ms` was often very large,
  `gps_sats` fluctuated, and `gps_hdop` was often `99.99` with only occasional
  readings around `4.7`.
- A later nearby attempt compiled with `SINGLE_WP_TARGET_LAT=35.5713100` and
  `SINGLE_WP_TARGET_LON=129.1885416`; USBDBG confirmed the override target, and
  GPS fix was acquired at `gps_lat=35.571384`, `gps_lon=129.187514`. The target
  was still `93.3` m away from the actual fix, so `distance_allowed=false` with
  `max_target_distance_m=30.0`.
- The latest window/outside-antenna test compiled with
  `SINGLE_WP_TARGET_LAT=35.5713840` and `SINGLE_WP_TARGET_LON=129.1874256`;
  USBDBG confirmed the override, but the antenna position was around
  `35.571284,129.188456`, the target was still about `93.9` m away, and GPS
  quality was unstable (`gps_sats` often `0`, `gps_hdop` often `99.99`,
  stale `gps_age_ms`).

Safety constants:

- `SINGLE_WAYPOINT_MAX_AUTO_THROTTLE=0.10`
- `SINGLE_WAYPOINT_ARRIVAL_RADIUS_M=2.5`
- `SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M=30.0`
- `SINGLE_WAYPOINT_GPS_STALE_MS=2000`
- `SINGLE_WAYPOINT_MAX_HDOP=2.5`
- `SINGLE_WAYPOINT_AUTO_TIMEOUT_MS=15000`

Safety gates:

- GPS location valid.
- GPS age below stale threshold.
- GPS HDOP valid and below threshold.
- Target valid.
- RC input valid.
- RC AUTO switch on.
- Target distance above arrival radius.
- Target distance below max allowed distance.
- AUTO timeout not exceeded.

USB debug fields:

```text
single_waypoint_experiment=true
target_override_enabled=...
target_source=...
target_lat_macro=...
target_lon_macro=...
auto_motion_armed=...
auto_motor_inhibit=...
gps_ready=...
target_ready=...
timeout_source=auto_entry
auto_entry_ms=...
auto_elapsed_ms=...
timeout_limit_ms=...
timeout_ok=...
max_target_distance_m=...
arrival_radius_m=...
distance_allowed=...
safety_ready=...
arrived=...
target_lat=...
target_lon=...
target_distance_m=...
target_bearing_deg=...
candidate_left_cmd=...
candidate_right_cmd=...
final_left_cmd=...
final_right_cmd=...
```

This mode does not load `mission.json`, does not run multiple waypoints, and
does not implement coverage/lawnmower driving.

Observed candidate dry-run status:

- `single_waypoint_experiment=true`
- `auto_motion_armed=false`
- `auto_motor_inhibit=true`
- `gps_fix=true` eventually with open-sky antenna placement
- `target_distance_m` and `target_bearing_deg` printed
- AUTO kept final `left_cmd` / `right_cmd` at `0`
- MANUAL returned to `RC_MANUAL`, and stick input changed motor command fields

This confirms candidate dry-run safety and basic distance/bearing computation,
but it does not validate rover body localization while the GPS antenna is
detached from the rover.

Latest nearby candidate retest:

- Build/upload succeeded with
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`, `SINGLE_WP_TARGET_LAT=35.5716800`, and
  `SINGLE_WP_TARGET_LON=129.1866516`.
- Runtime USBDBG still printed the old placeholder target:
  `target_lat=35.571120`, `target_lon=129.186050`.
- GPS fix was achieved, and at least one log line reached `gps_hdop=1.19` and
  `gps_ready=true`.
- Because the target remained the old placeholder, `target_distance_m` stayed
  around `40` to `60` m, `distance_allowed=false`, and `safety_ready=false`.
- `candidate_left_cmd=0.000`, `candidate_right_cmd=0.000`,
  `final_left_cmd=0.000`, and `final_right_cmd=0.000`.
- MANUAL still returned to `control_source=RC_MANUAL`.
- This was a safe failed validation. Safety gates and motor inhibit worked, but
  target override plumbing is the next blocker.

Latest target override check:

- Build/upload succeeded with
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`, `SINGLE_WP_TARGET_LAT=35.5710210`, and
  `SINGLE_WP_TARGET_LON=129.1864016`.
- USBDBG correctly printed `target_override_enabled=true`,
  `target_source=compile_time`, `target_lat_macro=35.5710210`,
  `target_lon_macro=129.1864016`, `target_lat=35.571021`, and
  `target_lon=129.186402`.
- GPS fix was true, but current GPS was around `gps_lat≈35.56752..35.56756`
  and `gps_lon≈129.18688`.
- `target_distance_m` remained around `380` to `392` m while
  `max_target_distance_m=30.0`.
- `distance_allowed=false`, `safety_ready=false`,
  `candidate_left_cmd=0.000`, and `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` correctly kept `final_left_cmd=0.000` and
  `final_right_cmd=0.000`.
- This verifies target override plumbing, but nearby candidate command remains
  incomplete. Recompute a nearby target from the current GPS position and rerun
  with `AUTO_MOTION_ARMED=0`.

Latest next-day GPS retest:

- The GPS antenna was placed outside again on the next day.
- Runtime GPS moved to approximately `gps_lat=35.571310`,
  `gps_lon=129.188630`.
- Firmware still used the previous compile-time target:
  `target_lat=35.567560`, `target_lon=129.186792`.
- Target override was still working:
  `target_override_enabled=true`, `target_source=compile_time`.
- The target was no longer nearby: `target_distance_m≈448.9` while
  `max_target_distance_m=30.0`.
- `distance_allowed=false` and `safety_ready=false`.
- `gps_fix=true` appeared, but `gps_ready=false` remained because `gps_age_ms`
  was often very large, `gps_sats` fluctuated, and `gps_hdop` was often
  `99.99` and only occasionally around `4.7`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept
  `final_left_cmd=0.000` and `final_right_cmd=0.000`.
- This was a safe blocked validation, not candidate-command success. Recompute
  the target from the current GPS location and rerun with `AUTO_MOTION_ARMED=0`.

Latest nearby attempt:

- Build/upload succeeded with
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`, `SINGLE_WP_TARGET_LAT=35.5713100`, and
  `SINGLE_WP_TARGET_LON=129.1885416`.
- USBDBG correctly printed `target_override_enabled=true`,
  `target_source=compile_time`, `target_lat_macro=35.5713100`,
  `target_lon_macro=129.1885416`, `target_lat=35.571310`, and
  `target_lon=129.188542`.
- GPS UART was alive and `gps_chars` increased.
- GPS fix was eventually acquired in MANUAL:
  `gps_fix=true`, `gps_lat=35.571384`, `gps_lon=129.187514`,
  `gps_sats=4`, and `gps_hdop=3.39..4.12`.
- `gps_age_ms` was initially fresh but later grew stale.
- The target was not nearby relative to the actual fix:
  `target_distance_m=93.3`, greater than `max_target_distance_m=30.0`.
- `distance_allowed=false`, `safety_ready=false`,
  `candidate_left_cmd=0.000`, and `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept
  `final_left_cmd=0.000` and `final_right_cmd=0.000`.
- This is a safe blocked result, not nearby candidate-command success.
  Recompute the target from the actual GPS fix position
  `35.571384,129.187514` and rerun with `AUTO_MOTION_ARMED=0`.

Latest window/outside-antenna attempt:

- The user tested by placing or throwing the GPS antenna outside from indoors.
- Build/upload succeeded with
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`, `SINGLE_WP_TARGET_LAT=35.5713840`, and
  `SINGLE_WP_TARGET_LON=129.1874256`.
- USBDBG correctly printed `target_override_enabled=true`,
  `target_source=compile_time`, `target_lat_macro=35.5713840`,
  `target_lon_macro=129.1874256`, `target_lat=35.571384`, and
  `target_lon=129.187426`.
- GPS fix was eventually seen, but not stable enough for candidate validation:
  `gps_fix=true` appeared, `gps_lat≈35.571284`, `gps_lon≈129.188456`,
  `gps_sats` often became `0`, `gps_hdop` often became `99.99`, and
  `gps_age_ms` grew very large.
- The target was not nearby relative to the actual GPS position:
  `target_distance_m≈93.9`, greater than `max_target_distance_m=30.0`.
- `distance_allowed=false`, `gps_ready=false`, `safety_ready=false`,
  `candidate_left_cmd=0.000`, and `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept
  `final_left_cmd=0.000` and `final_right_cmd=0.000`.
- This is a safe blocked result, not candidate-command success. Go fully
  outdoors with the rover and GPS fixed together, acquire a fresh MANUAL fix,
  recompute the target from that actual fix, and rerun with
  `AUTO_MOTION_ARMED=0`.

Latest outdoor Manual/Auto recovery and stale-target block:

- The previous RC issue was caused by the station/controller being off.
- After restoring the controller/link, AUTO was verified with
  `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001..2002`, and
  `control_source=STOP`.
- MANUAL was verified with `mode=MANUAL`, `auto_sw=false`,
  `mode_us≈1000..1001`, and `control_source=RC_MANUAL`.
- Manual stick input changed `steer_us` / `throttle_us`, and at least one
  MANUAL line produced nonzero `left_cmd` / `final_left_cmd`.
- Outdoor GPS was usable at several points: `gps_fix=true` and `gps_ready=true`
  appeared when HDOP was good.
- The compile-time target was stale: `target_lat=35.570675`,
  `target_lon=129.186769`, while runtime GPS was around `35.5716,129.1875`.
- `target_distance_m≈100..131`, greater than `max_target_distance_m=30.0`.
- `distance_allowed=false`, `safety_ready=false`,
  `candidate_left_cmd=0.000`, and `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept AUTO final commands
  at zero.
- This is a successful RC recovery and a safety-blocked autonomy dry-run, not a
  nearby candidate-command success. Recompute a nearby target from the current
  runtime GPS fix and rerun with `AUTO_MOTION_ARMED=0`.

Latest outdoor nearby dry-run:

- Build/upload used `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`, `SINGLE_WP_TARGET_LAT=35.5707680`, and
  `SINGLE_WP_TARGET_LON=129.1867906`.
- USBDBG confirmed target override:
  `target_override_enabled=true`, `target_source=compile_time`,
  `target_lat_macro=35.5707680`, `target_lon_macro=129.1867906`,
  `target_lat=35.570768`, and `target_lon=129.186791`.
- Outdoor GPS readiness improved: `gps_fix=true`, repeated `gps_ready=true`,
  `gps_sats=7..8`, `gps_hdop≈0.95..1.98`, and fresh `gps_age_ms` appeared in
  many lines.
- The rover/GPS became close enough to the target. `target_distance_m` decreased
  from around `45..55` m to `27.1`, `25.4`, `23.5`, `21.1`, `18.8`, and
  `18.5` m. `distance_allowed=true` was observed once below
  `max_target_distance_m=30.0`.
- The run was still blocked: mode stayed mostly `MANUAL`, `auto_sw=false`,
  `timeout_ok=false`, `safety_ready=false`, and candidate commands remained
  zero.
- A brief PPM/failsafe-like glitch appeared with `mode=FAILSAFE`,
  `rc_ok=false`, `steer_us≈495`, `throttle_us≈2504`, and
  `control_source=STOP`; keeping `STOP` is the correct safe behavior.
- This is partial progress and a safe blocked validation, not a successful AUTO
  candidate dry-run.

Timeout semantics update:

- The single-waypoint experiment now starts the timeout window on AUTO entry
  rather than at boot or during MANUAL waiting.
- Leaving AUTO resets the AUTO entry timestamp.
- MANUAL no longer consumes the AUTO candidate timeout.
- USBDBG now prints `timeout_source=auto_entry`, `auto_entry_ms`,
  `auto_elapsed_ms`, `timeout_limit_ms`, and `timeout_ok`.
- Next outdoor dry-run should verify `AUTO_READY`, `gps_ready=true`,
  `distance_allowed=true`, `timeout_ok=true`, `safety_ready=true`, and nonzero
  candidate commands while `AUTO_MOTION_ARMED=0` keeps final outputs at zero.

Latest post-timeout-fix GPS no-fix attempt:

- Firmware printed `timeout_source=auto_entry`, `auto_entry_ms=NA`,
  `auto_elapsed_ms=NA`, `timeout_limit_ms=15000`, and `timeout_ok=true`, so the
  previous MANUAL-wait timeout issue appears improved.
- Target override was confirmed with `target_override_enabled=true`,
  `target_source=compile_time`, `target_lat_macro=35.5708340`,
  `target_lon_macro=129.1869576`, `target_lat=35.570834`,
  `target_lon=129.186958`, and `target_ready=true`.
- RC was in MANUAL: `mode=MANUAL`, `auto_sw=false`, `mode_us≈1000..1001`, and
  `control_source=RC_MANUAL`.
- GPS UART was alive because `gps_chars` increased continuously.
- GPS fix was not acquired: `gps_fix=false`, `gps_lat=NA`, `gps_lon=NA`,
  `gps_sats=0`, `gps_hdop=99.99`, and `gps_age_ms=NA`.
- Therefore `target_distance_m=NA`, `distance_allowed=false`,
  `safety_ready=false`, candidate commands were zero, and final outputs were
  zero.
- This is a safe blocked validation. The current blocker is GPS satellite fix,
  not timeout, target override, or RC mode mapping.

## Safety Rules

- No real autonomous motion in this mode.
- AUTO computes readiness only.
- Switching RC back to MANUAL must immediately return to RC manual behavior.
- If RC is invalid, firmware must stop outputs and enter failsafe behavior.
- HC-12 is not used in this fixed-wiring dry-run mode.
- Motor tests remain wheel-off-ground.

## Future Real Autonomy Requirements

Next milestone:

- Go fully outdoors with the rover and GPS fixed together, acquire a fresh
  MANUAL fix, recompute a nearby target from that actual fix, and rerun
  `AUTO_MOTION_ARMED=0` promptly after upload/reset or AUTO entry. Continue IMU
  electrical diagnostics separately, but do not block GPS+RC candidate dry-run
  on IMU availability.
- Keep the waypoint target small and explicit when motion work resumes.
- Require GPS readiness, known GPS body-frame placement, heading/attitude plan,
  RC override, STOP/failsafe checks, and wheel-off-ground validation before any
  ground-contact test.
- Do not jump directly to full coverage/lawnmower execution.

Before implementing waypoint following:

- define GPS fix and age requirements;
- define heading source and BMI160 integration plan;
- define STOP/override behavior for every state;
- define path and waypoint acceptance criteria;
- define low-speed test procedure;
- log all GPS, RC, mode, command, target, distance, and bearing fields;
- verify behavior with wheels off ground before any ground-contact test.

Real waypoint following must be introduced as a separate milestone, not by
expanding the dry-run mode directly into motion.
