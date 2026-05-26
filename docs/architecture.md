# Architecture

This repository is a pre-autonomy rover stack for a GPS + HC-12 controlled
coverage robot. The current working baseline is manual control plus safe serial
protocol handling and offline coverage planning. Autonomy, ROS2 runtime
orchestration, and micro-ROS are not implemented yet.

## Current System Split

The system is split into a station PC and an OpenRB-150 rover.

Station PC responsibilities:

- operator workflow and UI, currently command-line tools only
- safe serial bring-up over HC-12 USB
- heartbeat and STOP transmission
- station-side manual keyboard command generation
- A/B point and sweep-width entry for offline/mock planning
- path preview, JSON/CSV export, and logs
- future high-level mission command generation

Rover responsibilities:

- final low-level motor safety decision
- RC manual input and mode switch handling
- ESC output generation
- HC-12 frame parsing and acknowledgements
- GPS reading from the selected rover GPS UART
- rover telemetry and USB debug output
- failsafe handling for stale link, invalid RC, STOP, and station E-stop

## Current Hardware Assumptions

- Rover controller: OpenRB-150.
- Integrated controller HC-12 rover UART: `Serial2`, `9600` baud.
- Integrated controller GPS rover UART: `Serial3`, `9600` baud.
- Current GPS probe result: central OpenRB connector works as `Serial2`,
  `9600` baud, with readable NMEA and GPS fix.
- Current architecture issue: the confirmed physical GPS path and integrated
  HC-12 path both point at `Serial2`.
- Selected resolution: Option B. Preserve HC-12 on `Serial2`, keep
  `GPS_SERIAL=Serial3`, and move GPS to a verified physical `Serial3` RX/TX
  path.
- Open issue: actual `Serial3` RX/TX pin mapping is unresolved and must be
  found with loopback and pin-finder tests.
- OpenRB USB debug: `115200` baud.
- Station serial default: `/dev/ttyACM0`, override with `--port`.
- RC receiver PPM input: OpenRB `D6`.
- ESC outputs: left `D4`, right `D5`.
- RC mode input: receiver PPM CH5, not CH7.

Do not change pin mappings, serial ports, or channel assumptions without
updating `docs/rc_channel_map.md`, `docs/manual_control.md`, and this document.

UART decision:

- Option B is selected.
- Preserve the known-working HC-12 manual-control path on `Serial2`.
- Move GPS to verified `Serial3` RX/TX pins before relying on integrated GPS
  telemetry.

## Implemented Components

Python core:

- `gps_coverage_core.protocol`: ASCII frame encode/decode, XOR checksum,
  clamping, command formatting, and manual command fields.
- `gps_coverage_core.planner`: A/B rectangular lawnmower path generation.
- `gps_coverage_core.geo`: WGS84 to local East/North conversion helpers.
- `gps_coverage_core.nmea`: GGA/RMC parsing helpers.
- `gps_coverage_core.telemetry`: GPS telemetry model, currently not aligned
  with firmware key/value GPS payloads.

Station tools:

- `tools/station_controller.py`: safe heartbeat plus periodic STOP loop.
- `tools/station_hc12_test.py`: heartbeat and telemetry receive smoke test.
- `tools/station_keyboard_manual.py`: armed keyboard manual control with
  deadman, local E-stop, startup STOP, and STOP on exit.
- `tools/station_mock_mission.py`: offline A/B path artifact generation.
- `tools/path_preview.py`: path preview figure generation.
- `tools/analyze_gps_log.py` and `tools/analyze_safety_log.py`: USB debug log
  analysis.

Rover firmware:

- `firmware/openrb_robot_controller/openrb_robot_controller.ino`: integrated
  rover sketch for RC manual control, station manual control, STOP, AUTO command
  gating, link timeout, GPS telemetry, status frames, and USB debug.
- Focused bring-up sketches under `firmware/` for GPS, HC-12 echo, PPM, and RC
  mix testing.

ROS2:

- `ros2_ws/src/` contains package skeletons only.
- The nodes currently initialize `rclpy` and log periodic alive messages.
- No ROS2 topics, services, launch files, bridge behavior, mission state
  machine, or waypoint follower exists yet.

## Current Data Flow

Safe station serial flow:

1. Station opens HC-12 USB serial.
2. Station sends heartbeat and STOP only by default.
3. Rover decodes frames, updates link freshness, and ACKs valid frames.
4. Rover emits `STAT` and `GPS` frames and USB debug.
5. Logs are written under `data/hc12_logs/` or relevant tool output folders.

Manual control flow:

1. RC manual remains available through receiver PPM.
2. Station keyboard manual starts disarmed.
3. Pressing `e` arms station manual; space enables deadman.
4. Rover drives from station manual frames only while frames are fresh and
   `deadman=1`.
5. `STOP`, local E-stop, stale station frames, stale link, invalid RC, or rover
   failsafe returns outputs to neutral.

Offline planning flow:

1. User enters point A, point B, lane spacing, and lane count.
2. Planner generates a locally planar lawnmower path.
3. Tools save JSON/CSV/summary/preview artifacts.
4. No rover motion is triggered by path generation.

## Target Remote Station Flow

The next target is still station-side and HC-12 based:

1. Station UI lets the user record or enter two points.
2. Station accepts sweep/coverage width or lane spacing.
3. Station receives rover GPS telemetry.
4. Station generates and previews a path plan.
5. Operator explicitly approves a dry-run or armed mission.
6. Station sends high-level command requests over HC-12.
7. Rover executes only after local safety gates allow motion.
8. Station logs telemetry, commands, state transitions, and STOP events.

Path generation must not automatically move the rover.

## Safety Boundary

The station may request motion. The rover decides whether motion is allowed.

Safety precedence:

1. Physical setup and wheel-off-ground rule during bench motor tests.
2. STOP command and station E-stop.
3. Rover-side failsafe and invalid RC handling.
4. Station link freshness and heartbeat timeout.
5. GPS fix and localization validity for any future autonomous motion.
6. Operator approval for any non-manual mission command.

## ROS2 Boundary

ROS2 may be used later on the station side for localization, planning, control,
and UI orchestration. Do not introduce ROS2 runtime behavior until the simple
HC-12 protocol, telemetry schema, STOP behavior, and station dry-run workflow are
stable. Do not introduce micro-ROS on the rover until there is a clear need and
the simple firmware protocol is proven.
