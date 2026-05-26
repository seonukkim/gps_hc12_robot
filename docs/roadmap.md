# Roadmap

The roadmap favors small, testable milestones. Do not start autonomous execution
until manual control, STOP, heartbeat, telemetry schema, path generation, and
station dry-run approval are stable.

## Milestone 0: Documentation Baseline

Goal:

- Make the current architecture, protocol, safety constraints, known issues, and
  field-test history explicit.

Deliverables:

- `docs/project_context.md`
- `docs/architecture.md`
- `docs/roadmap.md`
- `docs/protocol.md`
- `docs/safety_and_failures.md`
- `docs/field_test_log.md`
- `docs/known_issues.md`
- updated `AGENTS.md`

Exit checks:

```bash
uv run --extra dev pytest -q
```

## Milestone 1: HC-12 Telemetry Dry Run

Goal:

- Confirm reliable station-to-rover HC-12 serial communication without motion.

Allowed behavior:

- heartbeat
- STOP
- receive and decode `STAT`, `GPS`, `ACK`, and `ERR`
- log all raw TX/RX lines

Not allowed:

- AUTO commands
- path execution
- movement after station startup

Commands:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0 --heartbeat-hz 5
uv run python tools/station_controller.py --port /dev/ttyACM0
```

Exit criteria:

- station HC-12 USB port is confirmed
- rover ACKs heartbeat/STOP
- logs show valid checksums or clear parse errors
- STOP can be sent and acknowledged repeatedly

## Milestone 2: GPS Telemetry Schema Cleanup

Goal:

- Make firmware GPS telemetry and Python parsing use one documented schema.

Current issue:

- firmware emits key/value GPS payloads
- Python `GPSTelemetry.from_payload()` expects positional GPS payloads

Deliverables:

- updated parser or firmware payload
- protocol tests
- GPS logger update if needed
- `docs/protocol.md` updated

Exit checks:

```bash
uv run --extra dev pytest -q
uv run python -m py_compile tools/gps_logger.py gps_coverage_core/telemetry.py
```

## Milestone 3: Station Path Planning Dry Run

Goal:

- Provide station-side workflow for point A, point B, and sweep width without
  moving the rover.

Allowed behavior:

- manual input of A/B coordinates
- optional selection from recent GPS telemetry
- sweep width or lane spacing input
- path generation
- preview artifact generation
- logs and mission summary

Not allowed:

- drive command output
- implicit arming after plan generation

Existing command:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir data/mock_runs/example
```

Exit criteria:

- generated JSON/CSV/summary/preview are reproducible
- invalid inputs are rejected
- plan generation never sends motion commands

## Milestone 4: Station Mission State Machine Dry Run

Goal:

- Add a station-side state model for point selection, plan preview, approval,
  STOP, and logging.

Required states:

- `DISARMED`
- `COLLECTING_POINTS`
- `PLAN_READY`
- `APPROVED_DRY_RUN`
- `ARMED_MANUAL_ONLY`
- `STOPPED`
- `FAULT`

Allowed behavior:

- state transitions
- logs
- preview
- heartbeat plus STOP

Not allowed:

- autonomous motor command output

Exit criteria:

- no state can bypass STOP
- plan approval is explicit
- command log ties user action to sequence numbers

## Milestone 5: Heading And Localization Inputs

Goal:

- Establish data sources for latitude, longitude, and heading.

Planned sources:

- GPS latitude/longitude from rover
- BMI160 IMU heading or yaw estimate, if hardware and driver are confirmed

Exit criteria:

- sensor availability documented
- heading units and frame convention documented
- loss-of-fix and stale-heading behavior documented
- no control loop depends on unverified heading

## Milestone 6: Controlled Motion Experiments

Goal:

- Move from dry-run to limited command execution only after prior milestones pass.

Rules:

- wheel-off-ground first
- small output limits
- STOP available from station and rover
- heartbeat timeout tested
- GPS fix loss behavior tested
- logs captured for every run

Exit criteria:

- command request, rover state, GPS, and STOP events are logged
- no automatic resume after link loss
- manual control remains available

## Later: ROS2 Station Integration

ROS2 may be introduced on the station side after Milestones 1-4 are stable.
Start with a bridge that sends heartbeat and STOP only. Do not introduce
micro-ROS on the rover until the simple HC-12 protocol is proven insufficient.
