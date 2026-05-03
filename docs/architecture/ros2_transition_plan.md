# ROS2 Transition Plan

This plan describes how the current Python pre-ROS2 prototype can move toward
ROS2 Jazzy on Ubuntu 24.04 without overstating current project status.

## Current Status

Implemented:

- ROS-independent Python modules for protocol, planner, geodesy, NMEA parsing,
  and telemetry modeling.
- ROS2 package skeletons under `ros2_ws/src/`.
- Skeleton nodes that initialize `rclpy`, create a timer, and log an alive
  message.

Verified:

- Core Python tests for protocol, geodesy, and planner behavior.
- Mock planner output and report figure generation.

Pending:

- ROS2 topics, services, actions, parameters, launch files, and integration
  tests.
- HC-12 serial bridge behavior inside ROS2.
- Mission workflow state machine.
- Live waypoint following.

Planned:

- ROS2 Jazzy runtime integration on Ubuntu 24.04.
- Reuse of current ROS-independent modules inside ROS2 nodes.

## Design Principles

- Keep `gps_coverage_core` independent from ROS2.
- Keep rover firmware safety authoritative.
- Keep station startup conservative: heartbeat and STOP only.
- Use ROS2 to orchestrate and observe, not to bypass RC priority or failsafe
  behavior.
- Introduce ROS2 behavior in testable phases.

## Planned Nodes

| Node | Purpose | Current status |
| --- | --- | --- |
| `hc12_bridge_node` | Bridge serial frames to ROS2 topics/services | Skeleton only |
| `station_mission_node` | Manage A/B workflow, mission state, and operator approval | Skeleton only |
| `coverage_planner_node` | Expose A/B rectangular coverage planning through ROS2 | Skeleton only |
| `waypoint_follower_node` | Convert approved waypoints into safe command requests | Skeleton only |

## Suggested Interfaces

These interfaces are proposed and should be reviewed before implementation.

Topics:

- `/rover/gps`: decoded GPS telemetry.
- `/rover/status`: decoded mode, RC, link, and safety status.
- `/station/heartbeat`: station heartbeat state.
- `/mission/waypoints`: planned waypoint list.
- `/mission/state`: mission state for operator display.

Services:

- `/mission/set_point_a`
- `/mission/set_point_b`
- `/mission/generate_plan`
- `/mission/approve_plan`
- `/rover/stop`

Parameters:

- `serial_port`, default `/dev/ttyACM0`
- `serial_baud`, default `9600`
- `lane_spacing_m`
- `num_lanes`
- `heartbeat_hz`
- `stop_period_s`

## Phase 1: Bridge Without Motion

Goal:

- Confirm HC-12 station-side serial access and decode rover telemetry in ROS2.

Allowed behavior:

- Send heartbeat.
- Send STOP.
- Decode `GPS`, `STAT`, `ACK`, and `ERR`.
- Publish decoded status topics.

Not allowed:

- Automatic `AUTO` command on startup.
- Autonomous motor-driving command publication.

Exit criteria:

- Station-side HC-12 USB device is confirmed.
- ROS2 bridge logs valid incoming frames or clearly reports parse errors.
- STOP can be sent and acknowledged without live driving.

## Phase 2: Mission Planning In ROS2

Goal:

- Wrap the existing A/B planner in a ROS2 node.

Allowed behavior:

- Accept A/B coordinates from test inputs or operator workflow.
- Generate waypoints using current planner code.
- Publish and save preview artifacts.

Exit criteria:

- ROS2 planner output matches existing Python planner tests.
- Invalid inputs are rejected.
- Generated waypoints are traceable to A/B, spacing, and lane count.

## Phase 3: Operator Workflow

Goal:

- Manage point recording, plan generation, approval, and STOP state.

Allowed behavior:

- Record A/B point candidates from GPS or manual input.
- Require explicit approval before command output.
- Preserve STOP as the default state.

Exit criteria:

- Mission states are logged.
- No drive command is produced without explicit approval.
- STOP remains available in every state.

## Phase 4: Waypoint Following

Goal:

- Convert approved waypoints into command requests for controlled bench or field
  experiments.

Safety constraints:

- Wheel-off-ground for early motor tests.
- RC manual override remains highest priority.
- Link loss forces STOP.
- Reconnection does not resume motion automatically.

Exit criteria:

- Bench tests demonstrate command gating and STOP behavior.
- Field tests are only attempted after adhesion and safety validation.

## Phase 5: Hull And Process Extensions

Goal:

- Move beyond planar mock coverage toward ship-hull cleaning/painting needs.

Extensions:

- Curvature-aware surface representation.
- Edge exclusion and obstacle handling.
- Coverage overlap and process width.
- Paint/cleaning tool state.
- Localization uncertainty margins.
- Adhesion margin monitoring.

## Validation Plan

Use reproducible checks:

- `uv run pytest -q` for core Python modules.
- ROS2 package build and package discovery.
- Bridge-only logs before motion commands.
- USB debug analysis for STOP, RC state, and command outputs.
- Generated report figures from scripts, not hand-edited screenshots.

## Report Claim Limit

Until the phases above are implemented and tested, the report should state:

"ROS2 integration is planned. The current ROS2 packages are skeletons only, and
the current functional implementation is Python-based and pre-ROS2."
