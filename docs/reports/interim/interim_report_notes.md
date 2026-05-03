# Interim Report Notes

Date: 2026-05-03 KST

These notes are source material for a later Korean academic report. They use clear
technical English and avoid claims that have not been verified.

## Project Framing

The project is a ship exterior cleaning and painting robot for an Industrial
Engineering course. Ship hull surface work is labor-intensive, repetitive, and
difficult to keep consistent across large work areas. The project frames this as
a workflow automation problem: define a repeatable work region, generate a
coverage path, and validate safety behavior before moving toward autonomous
operation.

The current surface model assumes a locally planar ship exterior surface.
Magnetic wheels are the planned adhesion method for a metal hull, but magnetic
adhesion has not yet been designed, built, or validated in this repository.

## Current Concept Of Operation

1. The operator manually drives or positions the rover near the target surface.
2. The operator records point A and point B.
3. The A/B points define the reference edge of a rectangular planar work region.
4. The planner offsets lanes from the A-to-B edge using a selected lane spacing.
5. The planner outputs an alternating lawnmower waypoint sequence.
6. The operator reviews generated waypoints and preview figures offline.
7. Future station, HC-12, and ROS2 layers will execute or monitor the mission.

Current tools support offline or mock A/B mission generation. They do not yet
record A/B points through a completed station operator interface.

## Implemented

- Python project managed with `uv`, targeting Python 3.12 on Ubuntu 24.04 style
  environments.
- ROS-independent protocol utilities in `gps_coverage_core.protocol`.
- ROS-independent planar planner in `gps_coverage_core.planner`.
- WGS84/local coordinate helper modules and NMEA parsing helpers.
- Station tools with conservative defaults:
  - default port `/dev/ttyACM0`
  - default baudrate `9600`
  - heartbeat and repeated `STOP` in safe loops
  - explicit `--port` arguments in serial tools
- Mock mission generation through `tools/station_mock_mission.py`.
- Path preview generation through `tools/path_preview.py` and figure scripts.
- OpenRB firmware for RC manual priority, STOP handling, link timeout handling,
  USB debug output, GPS Serial3 input, and HC-12 frame parsing.
- ROS2 Jazzy package skeletons under `ros2_ws/src/`.

## Verified

- RC manual control has been verified on the rover.
- GPS communication and GPS FIX have been verified.
- Rover-side failsafe STOP behavior has been verified.
- The mock lawnmower planner has been verified through unit tests and generated
  path previews.
- Generated report figures exist for:
  - system overview
  - control flow
  - A/B region definition
  - lawnmower path preview
  - waypoint sequence
  - GPS log summaries
  - manual control and failsafe log summaries

## Pending

- Station-side HC-12 USB device confirmation.
- End-to-end station-to-rover HC-12 link test.
- GPS telemetry schema alignment between firmware key/value payloads and Python
  positional telemetry parsing.
- ROS2 runtime behavior beyond skeleton nodes.
- Magnetic wheel adhesion design and validation.
- Cleaning or painting payload integration.
- Autonomous field test on a ship hull or representative metal surface.

## Planned

- ROS2 Jazzy transition while preserving ROS-independent core modules.
- `hc12_bridge_node` for serial frame bridging.
- `station_mission_node` for operator workflow and mission state.
- `coverage_planner_node` for ROS2 access to the planar planner.
- `waypoint_follower_node` for future waypoint execution.
- Curvature-aware hull modeling, obstacle handling, edge exclusion zones, and
  coating-process constraints.

## Technical Method Notes

The current planner treats A and B as a line segment in latitude/longitude,
converts B into local meters relative to A, and creates parallel lanes using an
equirectangular local approximation. Each lane has two endpoints. Lane direction
alternates to create a lawnmower pattern.

The current rectangular work region is therefore represented implicitly by:

- the A-to-B lane length
- the lane spacing
- the number of lanes
- the perpendicular lane offsets

This is useful for an interim report because it shows a clear coverage-planning
method, but it should be described as a planar mock planner, not as validated
hull navigation.

## Industrial Engineering Relevance

- Workflow automation: manual selection of a work region can become a repeatable
  setup procedure.
- Labor-intensive work: ship hull cleaning and painting require large-area,
  repetitive surface coverage.
- Coverage planning: the lawnmower pattern gives a systematic method for
  reducing missed areas and repeated passes.
- Safety/failsafe operation: RC manual priority, STOP behavior, and conservative
  station defaults are core requirements before autonomy.
- Reproducible validation: code tests, generated figures, and USB debug log
  analysis create evidence that can be repeated and reported.

## Suggested Claim Wording

- Acceptable: "The current prototype implements a Python-based pre-ROS2 planar
  coverage planner from A/B points."
- Acceptable: "RC manual operation, GPS FIX, failsafe STOP behavior, and mock
  planner output have been verified."
- Acceptable: "HC-12 station integration and ROS2 runtime integration remain
  pending."
- Avoid: "The robot autonomously cleaned or painted a ship hull."
- Avoid: "ROS2 controls the rover."
- Avoid: "The HC-12 station link has been completed."
- Avoid: "Magnetic wheel adhesion has been validated."

## Figure Sources

Use captions from `docs/figures/generated/figure_captions.md`. Generated
schematics and mock path figures are useful for method explanation. GPS and
safety figures can support validation sections when the report text explains
that they summarize selected logs, not full autonomous performance.
