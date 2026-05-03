# Control Flow

This document describes current control behavior and the planned mission flow.
It separates implemented code paths from pending station and ROS2 integration.

## Implemented Flow

### Station Safe Loop

Current station tools open a serial device, defaulting to `/dev/ttyACM0` at
`9600` baud. The safe controller loop sends heartbeat frames and periodic `STOP`
frames. It does not send live motor-driving `AUTO` commands at startup.

Implemented station commands include:

- heartbeat: `HB,STATION`
- stop: `CMD,STOP,0,0`
- manual command helper: `CMD,MANUAL,steer,throttle,deadman,estop`
- auto command helper: `CMD,AUTO,left,right`

The auto helper exists in code, but safe station startup does not call it.

### Rover Firmware Flow

1. Firmware starts USB debug, HC-12 UART, GPS UART, PPM input, and ESC outputs.
2. Startup calls `motorStop()`.
3. RC channels are read from PPM.
4. GPS bytes are parsed through TinyGPS++.
5. HC-12 frames are decoded when full lines arrive.
6. Commands are accepted or rejected according to current safety state.
7. ESC outputs are updated only after safety checks.
8. Status, GPS telemetry, and USB debug fields are emitted periodically.

### Planner Flow

1. Python receives A/B coordinates, lane spacing, and lane count.
2. A/B coordinates are converted to local meters.
3. A-to-B becomes the reference lane direction.
4. Perpendicular offsets define parallel lanes.
5. Lane direction alternates to reduce turnaround travel.
6. Waypoints are saved as JSON/CSV and previewed as figures.

This planner flow is currently offline/mock. It is not yet connected to a live
HC-12 or ROS2 mission executor.

## Verified Flow Elements

- RC manual control has been verified.
- GPS FIX has been verified.
- Failsafe STOP behavior has been verified.
- Mock planner output has been verified through tests and figures.

## Pending Flow Elements

- Station-side HC-12 USB device confirmation.
- End-to-end HC-12 frame exchange between station and rover.
- Live A/B point recording through station UI or ROS2.
- Live waypoint execution.
- ROS2 topic/service orchestration.
- Cleaning/painting actuator control.

## Planned Mission Flow

1. Operator powers the rover with wheels off ground for bench tests or attaches
   it to a safe representative surface for later field tests.
2. Station starts in heartbeat/STOP mode.
3. Operator confirms RC manual control and STOP behavior.
4. Operator records A and B points.
5. Planner generates a rectangular planar coverage path.
6. Operator reviews path preview and mission summary.
7. Station sends mission commands only after explicit approval.
8. Rover applies commands only when RC mode, link state, and failsafe state allow
   motion.
9. Logs are saved for reproducible validation.

## Safety Priority

Safety priority from highest to lowest:

1. Physical safety setup and wheel-off-ground rule for motor tests.
2. RC manual override and valid RC signal.
3. STOP command and station E-stop state.
4. Link freshness and station command timeout.
5. Autonomous or station command request.

The station is not treated as the final safety authority. The rover firmware must
stop motion when local safety conditions are not satisfied.

## Protocol Flow

Frame format:

```text
@TYPE,SEQ,PAYLOAD*CS
```

Key frame types:

- `HB`: station heartbeat.
- `CMD`: `STOP`, `MANUAL`, `AUTO`, or `START`.
- `GPS`: rover GPS telemetry.
- `STAT`: rover mode, RC, and link status.
- `ACK`: accepted command or heartbeat.
- `ERR`: rejected command or bad frame.

## Known Integration Risk

Firmware currently emits GPS telemetry as key/value fields such as
`fix=1,lat=...,lon=...`, while `GPSTelemetry.from_payload()` expects positional
fields. This schema mismatch should be fixed before claiming complete station
GPS telemetry integration.

## Report Figure

Recommended figure:

- `docs/figures/generated/fig_control_flow.png`

Use it to explain separation between offline planning, safe station defaults,
and rover-side safety ownership.
