# Project Context

This project is a GPS + HC-12 controlled rover prototype for coverage work on a
locally planar surface. The course/instructor goal is moving from manual rover
control toward a station-side workflow where the user selects two points, enters
a sweep or coverage width, receives GPS telemetry, generates a path plan, and
eventually sweeps the selected area.

## Current Status

Implemented and recently verified:

- RC manual rover control.
- Station keyboard manual over HC-12 protocol.
- Safe station defaults: heartbeat plus STOP until explicitly armed.
- OpenRB firmware parsing for heartbeat, STOP, MANUAL, AUTO/START, GPS, status,
  ACK, and ERR frames.
- Rover-side STOP/failsafe behavior and neutral startup.
- GPS reading on rover GPS UART (physically `Serial2`, center 5-pin connector;
  the legacy default build's `Serial3` reads nothing) and USB debug GPS fields.
- Long-antenna GPS `Serial2@9600` outdoor validation with `STABLE_FIX`,
  `gps_block_reason=OK`, satellites present, HDOP reasonable, and lat/lon present.
- BMI160 IMU detection and normal-mode integrated diagnostics at I2C `0x68`
  (`chip_id=0xD1`, PMU normal, plausible accel/gyro values).
- Integrated no-motion rover build with physical motor output compile-gated off.
- Offline/mock A/B lawnmower path generation.
- Path preview and mock mission JSON/CSV/summary generation.
- Python tests for protocol, geodesy, and planner behavior.

Not implemented yet:

- live station UI for selecting points
- live GPS point capture workflow
- mission approval state machine
- autonomous path execution
- closed-loop waypoint following
- trusted heading fusion from GPS plus BMI160 IMU for motion control
- ROS2 runtime bridge or planner behavior
- micro-ROS on the rover

## Operating Principle

The station PC handles high-level decisions:

- user input
- planning
- logging
- previews
- dry-run checks
- high-level command requests

The rover handles low-level authority:

- motor output
- RC/manual priority
- command parsing
- GPS/IMU sensor ownership
- telemetry
- local failsafe
- STOP enforcement

The rover should never move simply because the station generated a path.

## Hardware And Ports

Known assumptions:

- Rover: OpenRB-150.
- Rover HC-12: legacy default uses `Serial2`; diagnostics/dry-run move HC-12 to
  `Serial3`/`Serial1` so GPS keeps `Serial2`. Current no-motion integrated
  controller selects HC-12 on `Serial3`, but RF link is deferred/unproven.
- Rover GPS: physically `Serial2`, `9600` (center 5-pin connector, known-good).
  Only the legacy default build defines `GPS_SERIAL=Serial3` (reads nothing);
  do not assume GPS is on Serial3 `D13`/`D14` unless explicitly rewired.
- Rover IMU: BMI160 on `Wire` (`D11` SDA / `D12` SCL), detected at `0x68`;
  the legacy `firmware/imu_probe` is MPU/ICM-only and is not a BMI160 health
  check.
- Rover USB debug: `115200`.
- Station HC-12 USB default: `/dev/ttyACM0`, `9600`, with `--port` exposed.
- RC PPM input: OpenRB `D6`.
- ESC output pins: left `D4`, right `D5`.
- RC Manual/Auto mode: PPM CH5.
- PPM CH7 is reserved/unused for mode.

## Current Manual Direction Work

Manual forward/backward driving works, but the integrated station joystick axes
have required careful remapping. The current active firmware marker is:

```text
openrb_robot_controller station-manual rc-arcade-manual-fwdneg 2026-05-30
```

The current intent is:

- physical stick straight up -> forward
- physical stick straight down -> reverse
- physical stick left/right -> steering
- current sign convention: `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`

Use `docs/manual_control.md` before modifying manual direction code. Previous
mistakes included treating diagonal input as forward/reverse and moving the bug
between axes by changing only signs.

## Near-Term Instructor Goal

The next development objective is remote station-side operation:

1. Display or collect current rover GPS telemetry.
2. Let the user select or record point A and point B.
3. Let the user input sweep width or lane spacing.
4. Generate a local planar coverage path.
5. Preview and log the plan.
6. Keep the station dry-run by default.
7. Require explicit approval before any motion command.
8. Keep STOP available and authoritative at every step.

## Development Boundary

Do not implement physical autonomy yet. The immediate work is documentation,
stable protocol, station dry-run workflow, telemetry schema cleanup, sensor
diagnostics, and logs. Any integrated validation must keep
`PHYSICAL_PATH_FOLLOWING_ENABLE=0` and `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
unless the user explicitly requests a separate motion test.

ROS2 may be useful later on the station side, but it should not be introduced
until the simple HC-12 protocol and station workflow are stable. micro-ROS should
not be introduced on the rover unless the simple firmware protocol becomes a
clear blocker.
