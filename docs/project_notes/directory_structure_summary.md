# Directory Structure Summary

Summary date: 2026-05-03 KST

## Project Context

This repository supports an Industrial Engineering course project for a ship hull exterior cleaning and painting robot. The current implementation is Python-based and pre-ROS2. The intended workflow is manual A/B point selection, planar rectangular work-region definition, and lawnmower-style coverage path planning.

## Current Implementation Status

Implemented or partially implemented:

- ROS-independent Python core modules for protocol framing, geodesy, telemetry, NMEA parsing, and coverage planning.
- A simple planar lawnmower path planner from manually supplied A/B GPS points.
- Safe station-side tools that default to heartbeat and `STOP`.
- Offline path preview and mock mission output generation.
- OpenRB bring-up work and documentation for RC manual priority, GPS Serial3, safety behavior, wiring, and testing.

Planned or pending:

- ROS2 Jazzy runtime integration. Current ROS2 content is skeleton-only.
- Confirmed end-to-end station HC-12 communication. Station-side HC-12 hardware confirmation remains pending in existing notes.
- Magnetic wheel adhesion design, mounting, and validation.
- Hull-curvature-aware planning, obstacle handling, edge exclusion zones, coating process constraints, and full cleaning/painting payload validation.

## Documentation Directories

Report workspaces:

```text
docs/reports/interim/
docs/reports/interim/figures/generated/
docs/reports/interim/figures/raw/
docs/reports/interim/figures/external/
docs/reports/interim/tables/
docs/reports/final/
docs/reports/final/figures/generated/
docs/reports/final/figures/raw/
docs/reports/final/figures/external/
docs/reports/final/tables/
```

Shared figure library:

```text
docs/figures/generated/
docs/figures/raw/
docs/figures/external/
docs/figures/thumbnails/
```

Architecture, notes, and reusable Codex support:

```text
docs/architecture/
docs/project_notes/
docs/codex/prompts/
```

`docs/codex/tasks/` was intentionally not created. Temporary Codex task prompts and one-off task logs should stay outside the repository. Reusable future prompt templates may be stored under `docs/codex/prompts/`.

## Figure Separation

- `generated/`: project-produced figures created from scripts, diagrams, logs, simulations, mock missions, or processed data. These are appropriate for reproducible report figures.
- `raw/`: unedited source evidence such as hardware photos, screenshots, exported plots, serial captures, or lab images. Treat these as original records.
- `external/`: figures not produced by this project, including vendor diagrams, course material, papers, standards, and web references. Keep source and license or attribution notes with each file.
- `thumbnails/`: small previews derived from another figure source. Thumbnails are convenience assets and should not be treated as primary evidence.

Report-specific generated, raw, and external figures can live inside the matching report folder. Shared or reusable figures can live under `docs/figures/`.

## Report Claim Guidance

Use current evidence for report wording:

- It is accurate to describe the project as a pre-ROS2 Python implementation with planned ROS2 migration.
- It is accurate to describe the current planner as planar, rectangular, and lawnmower-style.
- It is accurate to describe A/B selection as the intended manual workflow.
- It is not accurate to claim completed ROS2 operation unless future code and tests demonstrate it.
- It is not accurate to claim completed HC-12 station-to-rover integration until station hardware and link tests are confirmed.
- It is not accurate to claim validated magnetic wheel adhesion or full cleaning/painting operation until hardware evidence exists.

Keep station-side safety defaults visible in documentation: heartbeat and `STOP` only unless explicitly changed, no startup behavior that sends live motor-driving `AUTO` commands, and wheel-off-ground motor testing.
