# Autonomy Plan

This project does not have real autonomous waypoint following yet. The current
autonomy work is limited to a safe fixed-wiring dry-run mode that combines RC
manual driving with GPS readiness calculations.

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
target=35.571120,129.186050
target_distance_m=...
target_bearing_deg=...
gps_ready=...
target_ready=...
autonomy_ready=...
```

The placeholder target is only used to compute distance and bearing from the
current GPS position. It must not cause motor movement.

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
