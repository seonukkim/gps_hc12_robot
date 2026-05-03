# Limitations And Next Steps

This file lists technical limits that should be visible in interim and final
reports. It also gives practical next steps for continuing the project.

## Current Limitations

### Surface And Planning

- The ship exterior is currently assumed to be locally planar.
- A/B points define a rectangular work region in the mock planner, not a measured
  3D hull patch.
- The planner does not handle hull curvature.
- The planner does not handle obstacles, ribs, seams, ports, or edge exclusion
  zones.
- The planner does not model cleaning width, spray width, paint overlap, curing
  time, tool wear, or fluid flow rate.
- The current path is a waypoint preview, not validated closed-loop tracking.

### Hardware

- Magnetic wheels are planned but not validated.
- Adhesion force, slip, surface contamination, and hull curvature effects are
  not measured.
- Cleaning/painting payload hardware is not integrated.
- Station-side HC-12 USB hardware is not confirmed.
- Exact HC-12 UART pin mapping and logic-level compatibility need final
  confirmation before powered tests.

### Software

- Current implementation is Python-based and pre-ROS2.
- ROS2 packages are skeletons only.
- GPS telemetry schema is inconsistent between firmware key/value payloads and
  Python positional telemetry parsing.
- Live A/B recording through a station UI is not complete.
- Live autonomous waypoint execution is not complete.
- Station keyboard manual display handling should be reviewed before operator
  use because `StationController.poll()` returns dictionaries.

### Safety And Validation

- RC manual, GPS FIX, failsafe STOP, and mock planner behavior have been
  verified.
- Full autonomous safety validation has not been completed.
- Motor tests remain wheel-off-ground only.
- Reproducible validation currently depends on selected logs and generated
  figures, not long-duration field trials.

## Near-Term Next Steps

1. Align GPS telemetry schema.
2. Confirm station HC-12 USB device on the target station.
3. Run heartbeat/receive testing with `tools/station_hc12_test.py`.
4. Keep station startup heartbeat and STOP only.
5. Re-run `uv run pytest -q` after telemetry or planner edits.
6. Decide which generated mock mission artifacts should be tracked as report
   evidence.
7. Review and fix station keyboard manual receive-display handling before
   operator bench testing.
8. Update protocol documentation after schema changes.

## Medium-Term Next Steps

1. Build a ROS2 bridge node that only sends heartbeat and STOP at first.
2. Publish decoded rover status and GPS telemetry in ROS2.
3. Wrap the existing planner in `coverage_planner_node`.
4. Implement mission state for A/B recording, path generation, and operator
   approval.
5. Add bench tests for command gating and STOP behavior.
6. Add a representative-surface adhesion test plan for magnetic wheels.

## Long-Term Next Steps

1. Validate magnetic wheel adhesion on representative metal hull surfaces.
2. Integrate cleaning or painting actuator hardware.
3. Add hull curvature and edge-exclusion planning.
4. Add coverage quality metrics such as missed area, overlap, and path length.
5. Add localization and tracking validation.
6. Run controlled field trials only after safety and adhesion are verified.

## Report Guidance

Use these phrases:

- "planned magnetic-wheel adhesion"
- "planar surface assumption"
- "mock planar coverage planner"
- "pre-ROS2 Python implementation"
- "HC-12 station-side integration pending"
- "ROS2 integration planned"

Avoid these phrases unless new evidence exists:

- "completed autonomous cleaning"
- "completed autonomous painting"
- "validated magnetic adhesion"
- "fully integrated HC-12 station link"
- "ROS2-controlled rover"
- "curvature-aware hull coverage"
