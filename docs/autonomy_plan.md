# Autonomy Plan

This project does not have real autonomous waypoint following yet. The current
autonomy work is limited to a safe fixed-wiring dry-run mode that combines RC
manual driving with GPS readiness calculations.

## Status

Unified fixed-wiring RC + GPS dry-run validation is complete. Single-waypoint
candidate dry-run with `AUTO_MOTION_ARMED=0` confirms candidate-command safety,
but the latest nearby candidate retest is blocked by target override plumbing:
the compile command attempted to set a nearby target, while runtime USBDBG still
printed the old placeholder target. Bench testing and floor waypoint driving
are not approved yet.

Full coverage driving from `mission.json` / `mission.csv` is intentionally not
the next step. The rover must first prove one carefully bounded waypoint motion
with explicit safety gates, GPS validity policy, heading plan, manual override,
and wheel-off-ground checks.

Staged plan:

1. Dry-run complete: MANUAL RC and AUTO GPS distance/bearing computation coexist
   with AUTO motor output forced to zero.
2. Single-waypoint candidate-command dry-run: compute candidate commands behind
   safety gates while `AUTO_MOTION_ARMED=0` forces final motor output to zero.
3. Target override diagnostics: compile with `SINGLE_WP_TARGET_LAT` /
   `SINGLE_WP_TARGET_LON`, verify USBDBG prints `target_override_enabled=true`
   and `target_source=compile_time`, then verify runtime `target_lat` /
   `target_lon` before interpreting `distance_allowed` or `safety_ready`.
4. Sensor-frame validation: retest GPS with the antenna mounted on the rover
   body in open sky. IMU diagnostics remain useful, but IMU is optional for the
   current GPS+RC single-waypoint preparation stage and must not block
   candidate dry-run work.
5. Bench test with wheels lifted: compile the same experiment with
   `AUTO_MOTION_ARMED=1` only after explicit approval and verify low-speed
   output, timeout, arrival stop, GPS rejection, and manual override.
6. Low-speed floor test: only after wheel-off-ground behavior and sensor-frame
   assumptions are validated.
7. Multi-waypoint motion: only after single-waypoint behavior is proven.
8. Coverage path / lawnmower driving: last step, after mission sequencing,
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

- Validate compile-time target override diagnostics on the rover. The runtime
  `target_override_enabled`, `target_source`, `target_lat_macro`,
  `target_lon_macro`, `target_lat`, and `target_lon` fields printed by USBDBG
  must match the intended nearby target before `target_distance_m`,
  `distance_allowed`, or `safety_ready` are interpreted as a nearby-target
  result.
- Re-test candidate GPS fields with the GPS antenna mounted on the rover and
  placed in open sky.
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

## Safety Rules

- No real autonomous motion in this mode.
- AUTO computes readiness only.
- Switching RC back to MANUAL must immediately return to RC manual behavior.
- If RC is invalid, firmware must stop outputs and enter failsafe behavior.
- HC-12 is not used in this fixed-wiring dry-run mode.
- Motor tests remain wheel-off-ground.

## Future Real Autonomy Requirements

Next milestone:

- Validate compile-time target override diagnostics on hardware, then prepare
  GPS antenna/body-frame validation for the GPS+RC single-waypoint workflow.
  Continue IMU electrical diagnostics separately, but do not block GPS+RC
  candidate dry-run on IMU availability.
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
