# Architecture Notes

Use this directory for expanded architecture notes, diagrams, interface sketches, and future design decisions.

The current high-level architecture summary remains in `docs/architecture.md`. This directory is for supporting material that would make the root-level summary too long.

## Current Implementation

- Python pre-ROS2 core protocol, geodesy, telemetry, NMEA, and planner modules.
- ROS-independent rectangular coverage path generation from manually selected A/B GPS points.
- Station tools that default to heartbeat and `STOP`.
- OpenRB firmware bring-up work for RC priority, GPS Serial3, and low-level safety behavior.

## Planned Or Pending

- ROS2 Jazzy nodes and runtime orchestration.
- Station-side HC-12 USB link confirmation and end-to-end rover communication.
- Magnetic wheel adhesion hardware and validation.
- Full ship hull surface modeling, curvature-aware planning, obstacle handling, edge exclusion, and coating-process constraints.

Keep core protocol and planning material independent from ROS2 so it remains usable from plain Python tools and firmware test harnesses.
