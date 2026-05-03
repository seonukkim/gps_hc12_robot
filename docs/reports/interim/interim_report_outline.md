# Interim Report Outline

This outline is intended for an Industrial Engineering interim report. It is not
the final prose. The sections below include suggested figure placement and claim
limits.

## 1. Title And Abstract

Working title:

Ship Exterior Cleaning and Painting Robot Using Planar A/B Coverage Planning

Abstract points:

- Ship exterior cleaning and painting are labor-intensive surface workflows.
- The project proposes a magnetic-wheel mobile robot for metal hull adhesion.
- The current work focuses on safe rover bring-up and pre-ROS2 Python planning.
- Manual operation selects A/B points that define a rectangular planar region.
- A lawnmower path is generated for coverage of that region.
- RC manual control, GPS FIX, failsafe STOP, and mock planner output have been
  verified.
- HC-12 station integration and ROS2 integration remain pending.

## 2. Introduction

Purpose:

- Explain why ship hull surface work is suitable for Industrial Engineering
  automation.
- Emphasize repetitive work, worker burden, quality consistency, and safety.
- Define the project scope as a course prototype, not a finished industrial
  product.

Figure suggestion:

- No figure required, or use a later system overview figure after the problem is
  introduced.

## 3. Problem Definition And IE Relevance

Cover these points:

- Current ship exterior cleaning and painting require repeated movement across
  large surfaces.
- Manual work can lead to fatigue, inconsistent coverage, and difficult process
  documentation.
- A robot can support workflow automation by formalizing setup, path generation,
  operation, and validation records.
- Safety/failsafe design must be considered before increasing autonomy.

Suggested table:

| IE topic | Project connection |
| --- | --- |
| Work measurement | Repeatable path and lane spacing |
| Workflow automation | A/B region setup and generated mission files |
| Quality consistency | Systematic lawnmower coverage |
| Safety | STOP, RC manual priority, failsafe behavior |
| Reproducibility | Tests, logs, and generated figures |

## 4. System Concept

Describe the target system:

- Magnetic-wheel rover on a metal ship hull.
- Operator manually records A/B reference points.
- A/B points define a rectangular planar work region.
- Planner generates lawnmower coverage waypoints.
- Future station and ROS2 layers will supervise execution.

Clearly state current limits:

- The current implementation is Python-based and pre-ROS2.
- HC-12 station-side integration is pending.
- Magnetic adhesion is planned, not validated.
- The surface is currently assumed planar.

Figure suggestion:

- `docs/reports/interim/figures/generated/fig_system_overview.png`
- Caption focus: station, radio, rover, RC, GPS, and safety boundaries.
- Do not describe this as completed autonomous operation.

## 5. Hardware And Safety Bring-Up

Implemented or verified:

- OpenRB-150 controller.
- RC receiver PPM input.
- RC manual mode.
- GPS module on OpenRB `Serial3` at `9600`.
- GPS FIX.
- Rover-side failsafe STOP.
- ESC output safety convention: wheel-off-ground only.

Pending:

- Station-side HC-12 USB confirmation.
- Magnetic wheels and adhesion tests.
- Cleaning/painting payload hardware.

Figure suggestions:

- `fig_manual_control_timeline.png` for RC/manual command log summary.
- `fig_failsafe_event_timeline.png` for failsafe-related log conditions.
- `fig_state_machine.png` for safety-state explanation.

Suggested wording:

"The safety figures summarize selected USB debug logs and design state, not a
complete autonomous field safety certification."

## 6. Software Architecture

Describe implemented modules:

- `gps_coverage_core.protocol`: frame encode/decode and checksum.
- `gps_coverage_core.planner`: planar A/B lawnmower path generator.
- `gps_coverage_core.geo`: WGS84/local coordinate conversion helpers.
- `gps_coverage_core.nmea`: NMEA parsing helpers.
- `tools/`: station, mock mission, log analysis, and figure utilities.
- `firmware/`: OpenRB rover control and bring-up sketches.
- `ros2_ws/src/`: ROS2 package skeletons only.

Figure suggestion:

- `fig_control_flow.png`
- Caption focus: offline planning, station defaults, and rover safety boundary.

## 7. A/B Region Definition And Coverage Planning

Explain the planning method:

- The operator-selected A/B points are treated as one edge of a rectangular
  planar region.
- Lane spacing and lane count define the region width.
- The planner converts coordinates to local meters, offsets lanes, and returns
  alternating lane endpoints.
- The output is a lawnmower pattern suitable for coverage preview and later
  mission execution.

Implemented:

- Python function `generate_lawnmower_path()`.
- CLI mock mission artifact generation.
- JSON, CSV, summary text, and PNG preview outputs.

Verified:

- Unit tests check waypoint count, lane alternation, spacing, and invalid input.

Figure suggestions:

- `fig_ab_region_definition.png` immediately after A/B explanation.
- `fig_lawnmower_path_preview.png` after describing generated coverage.
- `fig_waypoint_sequence.png` when explaining waypoint ordering.

## 8. Validation Results To Date

Suggested subsections:

- RC manual operation.
- GPS FIX.
- Failsafe STOP.
- Mock planner and generated mission artifacts.

Figure suggestions:

- `fig_gps_fix_timeline.png`
- `fig_gps_satellites_vs_time.png`
- `fig_gps_hdop_vs_time.png`
- `fig_gps_position_scatter.png`
- `fig_manual_control_timeline.png`
- `fig_control_source_transition.png`

Claim limit:

- GPS figures summarize receiver fix and log quality. They do not validate
  closed-loop hull navigation or path tracking accuracy.

## 9. Current Limitations

Required points:

- Planar surface assumption.
- No curvature-aware hull model.
- No obstacle or edge exclusion handling.
- No coating-process model such as spray width, overlap, curing time, or paint
  flow rate.
- No completed HC-12 station integration.
- ROS2 nodes are skeletons only.
- Magnetic wheel adhesion is pending.
- Full cleaning/painting task execution is pending.

## 10. Next Steps

Near-term:

- Align GPS telemetry schema between firmware and Python.
- Confirm station HC-12 USB device and run heartbeat/receive tests.
- Keep station startup behavior heartbeat and STOP only.
- Preserve wheel-off-ground constraints for motor tests.
- Re-run `uv run pytest -q` after schema or planner changes.

Later:

- Build ROS2 bridge and mission nodes.
- Add waypoint follower behavior.
- Test magnetic adhesion on representative metal surfaces.
- Extend planner for hull curvature and process constraints.

## 11. Conclusion

Conclusion points:

- The project has established the safe pre-ROS2 foundation.
- The current planner demonstrates A/B rectangular coverage generation.
- Verified items support continued development.
- The main remaining risk is integration: station HC-12, ROS2 runtime behavior,
  adhesion, and actual cleaning/painting payload validation.
