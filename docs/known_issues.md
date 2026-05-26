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

## Heading / BMI160 Is Not Integrated

The rover likely needs heading from GPS plus BMI160 IMU, but BMI160 support is
not implemented in the current repo. Do not build waypoint following as if
heading is already available.

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
