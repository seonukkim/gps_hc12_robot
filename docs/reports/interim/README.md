# Interim Report Workspace

Use this directory for interim report material for the ship hull exterior cleaning and painting robot course project.

The current implementation is a Python-based, pre-ROS2 prototype focused on safe station tooling, protocol utilities, GPS/log handling, and planar rectangular lawnmower coverage planning from manually selected A/B points.

## Current Scope

Implemented or partially implemented:

- Python core modules for protocol handling, GPS/geodesy helpers, telemetry parsing, and simple planar coverage planning.
- Station-side tools with safe defaults: heartbeat and `STOP` unless an operator explicitly changes behavior.
- Offline path preview and mock mission artifact generation.
- OpenRB bring-up documentation for RC, GPS, safety, and wiring.

Planned or pending:

- ROS2 Jazzy integration beyond the current package skeletons.
- End-to-end station HC-12 link confirmation.
- Magnetic wheel adhesion design and validation.
- Full ship hull curvature handling, obstacle handling, edge exclusion, coating-process constraints, and field-ready cleaning or painting payload integration.

## Layout

- `figures/generated/`: figures produced from project scripts, logs, simulations, diagrams, or processed data.
- `figures/raw/`: original captures such as photos, screenshots, scope traces, or unedited exported plots.
- `figures/external/`: third-party, vendor, course, or reference figures. Keep source and license notes with each item.
- `tables/`: report tables, CSV exports, and table source files used by the interim report.

Do not store temporary Codex task prompts, one-off task logs, or legacy `codex_task*.md` files here.
