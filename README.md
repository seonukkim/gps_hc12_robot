# Ship Hull Coverage Robot

Pre-ROS2 Python prototype for an Industrial Engineering project: a ship exterior
cleaning and painting robot intended to operate on the outer hull of a ship. The
current planning assumption is a locally planar surface. The adhesion concept is
magnetic wheels, but magnetic adhesion and full cleaning/painting operation have
not yet been validated.

The repository currently focuses on safe rover bring-up, GPS/log handling,
protocol utilities, and mock planar coverage planning. ROS2 Jazzy integration is
planned, but the current ROS2 packages are skeletons only.

## Safety Defaults

Station-side defaults are intentionally conservative:

- Default USB serial device: `/dev/ttyACM0`.
- Default baudrate: `9600`.
- Serial tools expose `--port` so the device can be changed explicitly.
- Station loops default to heartbeat and `STOP` only.
- Station startup must not send live motor-driving `AUTO` commands.
- Rover motor testing is wheel-off-ground only.
- RC manual override and rover-side failsafe logic remain authoritative.

## Industrial Engineering Motivation

Ship hull exterior work is labor-intensive, repetitive, and difficult to keep
consistent over large surfaces. This project frames that problem as an
Industrial Engineering workflow:

- reduce manual burden in hull cleaning and painting tasks;
- define repeatable coverage regions from operator-selected points;
- generate systematic lawnmower-style paths for surface coverage;
- automate the workflow from manual setup to planned execution;
- validate safety behavior before expanding toward autonomous field operation.

## System Concept

The intended operation concept is:

1. Drive the rover manually on or near the target hull surface.
2. Record operator-selected A/B reference points.
3. Define a rectangular planar work region from those references.
4. Generate a lawnmower-style coverage path with configurable lane spacing.
5. Preview or export the mock mission offline.
6. Later, send mission commands through the station, HC-12 radio, and ROS2 stack.

Current code supports the Python-side protocol and mock planning pieces. It does
not yet implement completed autonomous ROS2 execution or confirmed end-to-end
station HC-12 operation.

## Hardware Overview

- Target surface: outer hull of a ship, currently approximated as planar.
- Adhesion concept: magnetic wheels, pending design and validation.
- Rover controller: OpenRB-150.
- Manual control: RC receiver with PPM input; RC manual mode has been verified.
- GPS: GPS module on OpenRB `Serial3`, `9600` baud; GPS FIX has been verified.
- Radio link: HC-12 UART is the intended station-to-rover link. Station-side
  HC-12 USB confirmation is still pending.
- Actuation: ESC/motor outputs are managed by rover firmware. Bench motor tests
  must remain wheel-off-ground.
- Station/development OS: Ubuntu 24.04. WSL2 Ubuntu 24.04 and Jetson are target
  station environments.

See [docs/current_hardware_status.md](docs/current_hardware_status.md),
[docs/wiring.md](docs/wiring.md), and [firmware/README.md](firmware/README.md).

## Software Architecture

Current implementation is Python-based and pre-ROS2:

- `gps_coverage_core.protocol`: serial frame encoding/decoding and checksums.
- `gps_coverage_core.geo`: WGS84/local coordinate conversion helpers.
- `gps_coverage_core.planner`: simple planar A/B lawnmower path generation.
- `gps_coverage_core.nmea`: supported NMEA parsing helpers.
- `gps_coverage_core.telemetry`: GPS telemetry model.
- `tools/`: station-side utilities for safe serial loops, GPS logging, log
  analysis, NMEA replay, path previews, and mock mission generation.
- `firmware/`: OpenRB sketches for integrated rover control and focused bring-up
  tests.
- `ros2_ws/src/`: ROS2 Jazzy package skeletons for future bridge, mission,
  planner, and waypoint follower nodes.

Core protocol and planning modules are intentionally independent from ROS2 so
they can be used by plain Python tools now and reused during the ROS2 migration.

Planned ROS2 migration:

- `hc12_bridge_node`: bridge HC-12 serial frames into ROS2 topics/services.
- `station_mission_node`: manage operator workflow and mission state.
- `coverage_planner_node`: expose coverage path generation through ROS2.
- `waypoint_follower_node`: consume planned paths and produce rover commands.

These ROS2 nodes are not complete runtime behavior yet.

## Verified Progress

Verified or implemented so far:

- RC manual control on the rover.
- GPS module communication and GPS FIX.
- Failsafe STOP behavior.
- Safe station defaults that send heartbeat and `STOP`, not live `AUTO`.
- Mock lawnmower path planning and preview generation.
- Unit tests for protocol, geodesy, and planner behavior.

Pending:

- Station-side HC-12 USB device confirmation and end-to-end link test.
- Reconciliation of GPS telemetry payload schema across firmware and Python
  tools.
- ROS2 node integration beyond skeleton packages.
- Magnetic wheel adhesion validation.
- Full autonomous field test on the target surface.

## Directory Structure

```text
gps_coverage_core/        ROS-independent Python protocol, geo, telemetry, NMEA, planner
tools/                    Station utilities, log analyzers, mock mission generators
firmware/                 OpenRB/Arduino sketches and bring-up tests
ros2_ws/src/              ROS2 Jazzy package skeletons, not completed runtime nodes
tests/                    Python tests for protocol, geo, and planner modules
scripts/                  Environment, requirements, and ROS2 workspace helper scripts
docs/                     Architecture, protocol, safety, wiring, reports, figures
docs/project_notes/       Repository audit and cleanup notes
docs/figures/             Shared generated/raw/external figure library
docs/reports/interim/     Interim report workspace
docs/reports/final/       Final report workspace
data/                     Local logs and generated mock mission outputs
outputs/                  Local generated outputs
```

## Setup

Use `uv` for dependency and task execution:

```bash
uv sync --extra dev --extra web
```

Useful checks:

```bash
uv run pytest -q
uv run python tools/verify_env.py --port /dev/ttyACM0
./scripts/export_requirements.sh
```

## Running Current Components

Analyze GPS USB debug logs:

```bash
uv run python tools/analyze_gps_log.py data/gps_logs/*.log
```

Analyze safety USB debug logs:

```bash
uv run python tools/analyze_safety_log.py data/safety_logs/*.log
```

Generate a mock station-side coverage mission without HC-12 or ROS2:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir data/mock_runs/example
```

Generate a standalone path preview figure:

```bash
uv run python tools/path_preview.py \
  --lat-a 35.123456 --lon-a 129.123456 \
  --lat-b 35.123456 --lon-b 129.124556 \
  --spacing 5.0 \
  --output docs/figures/generated/path_preview.png
```

Generate report-ready mock mission artifacts in the shared figure area:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir docs/figures/generated/mock_mission_example
```

When the station HC-12 USB device is connected, run only safe heartbeat/receive
testing first:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0
```

The safe controller loop also defaults to heartbeat and periodic `STOP`:

```bash
uv run python tools/station_controller.py --port /dev/ttyACM0
```

Manual keyboard station testing exists, but it sends manual command frames and
must be treated as wheel-off-ground motor testing only:

```bash
uv run python tools/station_keyboard_manual.py --port /dev/ttyACM0 --max-speed 0.25
```

The keyboard tool starts in heartbeat-plus-`STOP` mode. Press `e` to arm station
manual control, press space to enable the deadman, then use `WASD` or arrow keys
for short manual pulses. Press `x` for local E-stop/disarm and `q` to exit; exit
sends repeated `STOP` frames.

See [docs/manual_control.md](docs/manual_control.md) for the current rover
firmware upload steps, RC direction mapping, USB debug checks, and station
manual-control procedure.

## Figure Gallery

Shared generated figures belong in
[docs/figures/generated/](docs/figures/generated/). This directory may be empty
until the figure-generation commands above are run.

Figure source classes are documented in [docs/figures/README.md](docs/figures/README.md):

- `generated/`: reproducible project-generated figures from scripts, logs, mock
  missions, diagrams, or processed data.
- `raw/`: original captures such as photos, screenshots, serial captures, or
  unedited exported plots.
- `external/`: third-party or reference figures with source and license notes.
- `thumbnails/`: small convenience previews derived from another source.

Report-specific figures can also be placed under:

- [docs/reports/interim/figures/generated/](docs/reports/interim/figures/generated/)
- [docs/reports/final/figures/generated/](docs/reports/final/figures/generated/)

## Report Material Map

- [docs/reports/interim/](docs/reports/interim/): interim report workspace,
  including generated/raw/external figure folders and tables.
- [docs/reports/final/](docs/reports/final/): final report workspace for
  verified claims and final evidence.
- [docs/project_notes/repo_audit.md](docs/project_notes/repo_audit.md):
  current repository audit and risk notes.
- [docs/project_notes/directory_structure_summary.md](docs/project_notes/directory_structure_summary.md):
  documentation and report folder map.
- [docs/project_notes/cleanup_summary.md](docs/project_notes/cleanup_summary.md):
  cleanup status and remaining artifact-policy decisions.

Final report claims should stay tied to verified evidence. Do not describe HC-12
station operation, ROS2 autonomy, magnetic adhesion, or full cleaning/painting
operation as complete until those items are implemented and tested.

## Limitations And Next Steps

Current limitations:

- Planner assumes a locally planar rectangular work region.
- No hull curvature handling, obstacle handling, edge exclusion zones, coating
  process constraints, or localization uncertainty margins yet.
- Magnetic wheel adhesion is a concept, not a validated subsystem.
- Station HC-12 USB and end-to-end radio communication remain pending.
- ROS2 packages are skeletons only.
- GPS payload schema needs alignment between firmware and Python telemetry
  parsing before relying on all station-side GPS tools.

Next steps:

- Confirm station-side HC-12 USB attachment and safe heartbeat/telemetry link.
- Align and document one GPS telemetry payload schema.
- Keep expanding mock mission evidence without claiming autonomous field
  completion.
- Implement ROS2 bridge, mission, planner, and follower nodes around the
  existing ROS-independent core modules.
- Validate magnetic wheel adhesion and safety behavior before any live hull test.
