# Autonomy Plan

This project does not have real autonomous waypoint following yet. The current
autonomy work is limited to a safe fixed-wiring dry-run mode that combines RC
manual driving with GPS readiness calculations.

## Status

Unified fixed-wiring RC + GPS dry-run validation is complete. The next
autonomy milestone is single-waypoint controlled motion preparation, not full
coverage/lawnmower driving.

Full coverage driving from `mission.json` / `mission.csv` is intentionally not
the next step. The rover must first prove one carefully bounded waypoint motion
with explicit safety gates, GPS validity policy, heading plan, manual override,
and wheel-off-ground checks.

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

## Safety Rules

- No real autonomous motion in this mode.
- AUTO computes readiness only.
- Switching RC back to MANUAL must immediately return to RC manual behavior.
- If RC is invalid, firmware must stop outputs and enter failsafe behavior.
- HC-12 is not used in this fixed-wiring dry-run mode.
- Motor tests remain wheel-off-ground.

## Future Real Autonomy Requirements

Next milestone:

- Prepare single-waypoint controlled motion only.
- Keep the waypoint target small and explicit.
- Require GPS readiness, heading/attitude plan, RC override, STOP/failsafe
  checks, and wheel-off-ground validation before any ground-contact test.
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
