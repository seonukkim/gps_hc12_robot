# System Overview

This document summarizes the current and planned architecture for the ship
exterior cleaning and painting robot. It is written for report support and
should be kept consistent with `README.md`, `docs/architecture.md`, and current
code.

## System Goal

The target system is a mobile robot that can support ship exterior cleaning or
painting work on a metal hull. The long-term concept uses magnetic wheels for
adhesion and a coverage planner for systematic surface traversal.

The current implementation is a Python-based, pre-ROS2 prototype. It focuses on
safe rover bring-up, protocol utilities, GPS/log handling, and mock planar
coverage planning.

## Current Assumption

The ship exterior surface is currently modeled as locally planar. A and B are
operator-selected reference points. The A/B segment defines one edge of a
rectangular planar work region. Lane spacing and lane count define the region
width. The planner generates alternating lawnmower lane endpoints for coverage.

This assumption is useful for an Industrial Engineering prototype because it
turns manual work setup into a repeatable workflow. It is not yet a hull
curvature model.

## Implemented

| Area | Current implementation |
| --- | --- |
| Core protocol | ASCII frame encode/decode and XOR checksum in `gps_coverage_core.protocol` |
| Planner | Planar A/B lawnmower path generation in `gps_coverage_core.planner` |
| Geodesy | WGS84/local conversion helpers |
| Station tools | Safe serial loops, GPS logging, path preview, mock mission generation |
| Firmware | OpenRB RC/manual control, STOP, GPS Serial3, USB debug, HC-12 frame parser |
| Figure generation | Report figures for system, planning, GPS, and safety summaries |
| ROS2 workspace | Package skeletons under `ros2_ws/src/` |

## Verified

- RC manual operation on the rover.
- GPS module communication and GPS FIX.
- Rover-side failsafe STOP behavior.
- Mock planner behavior and path preview generation.
- Unit tests for protocol, geodesy, and planner behavior.

## Pending

- Station-side HC-12 USB device confirmation.
- End-to-end HC-12 station-to-rover integration.
- GPS telemetry payload schema reconciliation.
- ROS2 runtime integration.
- Magnetic wheel adhesion validation.
- Cleaning/painting payload validation.

## Planned

- ROS2 Jazzy integration on Ubuntu 24.04.
- HC-12 bridge node for serial frames.
- Station mission node for operator workflow.
- Coverage planner node that reuses the ROS-independent planner.
- Waypoint follower node with safety-aware command output.
- Extended planner for hull curvature, obstacles, edge margins, and coating
  process constraints.

## Runtime Boundary Model

The rover firmware owns the final motion safety decision. Station software can
request commands, but the rover must enforce STOP, RC priority, link timeout, and
failsafe behavior.

Current station defaults are intentionally conservative:

- heartbeat frames are allowed
- periodic `STOP` frames are allowed
- live motor-driving `AUTO` commands are not sent at startup
- serial tools expose `--port`
- default USB serial device is `/dev/ttyACM0`

## High-Level Data Flow

Current mock/offline flow:

1. User provides A/B latitude and longitude values to a Python tool.
2. Planner converts the region to local meters.
3. Planner generates a lawnmower waypoint sequence.
4. Tool writes JSON, CSV, summary text, and preview figures.
5. Report figures use generated artifacts or deterministic fallback data.

Planned live flow:

1. Operator records A/B points through station workflow.
2. Planner generates mission waypoints.
3. Station sends approved commands through HC-12.
4. Rover firmware checks RC mode, link freshness, STOP state, and failsafe state.
5. Rover applies motor commands only when safety conditions allow motion.

## Report Figure

Recommended figure:

- `docs/figures/generated/fig_system_overview.png`

Use it as a schematic of architecture and boundaries. Do not describe it as a
completed autonomous execution result.
