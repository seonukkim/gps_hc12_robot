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
