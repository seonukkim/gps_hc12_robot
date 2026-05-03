# Final Report Notes

Date: 2026-05-03 KST

These notes define what the final report can claim if no additional tests are
completed. Update this file only when new evidence exists.

## Evidence Standard

Final-report statements should be traceable to at least one of these sources:

- project code or tests
- firmware constants and behavior
- generated mock mission artifacts
- USB debug logs
- analysis scripts
- hardware bring-up notes
- sourced external references with license or attribution notes

Do not present mock data, schematic figures, or planned architecture as measured
field performance.

## Implemented

- Python pre-ROS2 core modules for protocol, planner, geodesy, NMEA parsing, and
  GPS telemetry modeling.
- Station tools for safe serial loops, GPS logging, log analysis, mock mission
  generation, NMEA replay, and path preview.
- OpenRB firmware for RC input, manual control, STOP behavior, station command
  parsing, GPS Serial3 integration, status output, and USB debug logs.
- Planar lawnmower path generator from A/B points, spacing, and lane count.
- ROS2 package skeletons for future bridge, mission, planner, and follower
  nodes.

## Verified

- RC manual operation.
- GPS module communication and GPS FIX.
- Failsafe STOP behavior.
- Mock planner behavior through tests and generated figures.
- Safe station defaults in current tools: heartbeat and STOP unless manually
  changed by the operator.

## Pending

- Station-side HC-12 USB hardware confirmation.
- End-to-end station-to-rover HC-12 communication test.
- Reconciled GPS telemetry payload schema across firmware and Python tools.
- Functional ROS2 runtime integration.
- Magnetic wheel adhesion experiment.
- Cleaning/painting mechanism integration.
- Autonomous coverage execution on a representative metal hull surface.

## Planned

- ROS2 Jazzy architecture on Ubuntu 24.04.
- ROS2 bridge from HC-12 serial frames to topics/services.
- Mission state node for A/B point recording and operator workflow.
- ROS2 planner interface that reuses the current ROS-independent planner.
- Waypoint follower and safety-aware command gating.
- More realistic hull surface modeling and coverage constraints.

## Final Report Structure Suggestion

1. Introduction and Industrial Engineering motivation.
2. Related work or reference technology for hull cleaning, painting, mobile
   robots, and magnetic adhesion.
3. System requirements and safety constraints.
4. Hardware configuration and pin map.
5. Software architecture.
6. A/B region definition and coverage-planning method.
7. Implementation status.
8. Verification results.
9. Limitations.
10. Future work and ROS2 transition plan.
11. Conclusion.

## Required Technical Explanations

The final report should explain the following in precise terms:

- The current surface is assumed planar.
- A/B points define the reference edge of a rectangular region.
- Lane spacing and lane count define coverage width.
- The planner outputs alternating lane endpoints in a lawnmower pattern.
- The station defaults are conservative and do not send live `AUTO` commands at
  startup.
- Rover-side safety remains authoritative through RC priority, STOP, link
  timeout, and failsafe behavior.
- Core planning and protocol modules are independent from ROS2 so they can be
  reused during migration.

## Claim Limits

Allowed if no new tests are added:

- "The current prototype is Python-based and pre-ROS2."
- "The planner generates a mock planar lawnmower path from A/B points."
- "RC manual, GPS FIX, failsafe STOP, and mock planner behavior have been
  verified."
- "ROS2 and HC-12 station integration are planned or pending."

Not allowed without new evidence:

- "The system completed autonomous hull cleaning or painting."
- "The HC-12 station link is fully integrated."
- "ROS2 controls the rover."
- "The robot adheres safely to a ship hull using magnetic wheels."
- "The planner handles hull curvature, obstacles, or coating-process physics."

## Validation Material To Preserve

- `docs/figures/generated/figure_captions.md`
- generated system and planner figures
- GPS log figures generated from real logs when available
- manual control and failsafe log figures
- `docs/gps_serial3_test.md`
- `docs/current_hardware_status.md`
- `docs/project_notes/repo_audit.md`
- unit tests under `tests/`

## Final Work Before Submission

- Decide which generated mock mission artifacts should be committed as report
  evidence.
- Re-run figure generation from repository root.
- Re-run `uv run pytest -q`.
- Confirm final statements against the latest `docs/project_notes/progress_summary.md`.
- Add citations for external images or technical references.
- Keep temporary prompts and task notes out of the report directories.
