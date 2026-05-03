# Experiment Log

This log records report-relevant project evidence. Add new entries when a test
or generated artifact changes a report claim.

## Status Categories

- Implemented: code, firmware, scripts, or documentation exist.
- Verified: behavior has supporting test, log, generated output, or hardware
  observation.
- Pending: required integration or validation has not been completed.
- Planned: future design direction.

## Log Entries

| Date | Area | Evidence | Result | Report use |
| --- | --- | --- | --- | --- |
| 2026-05-03 | Repository audit | `docs/project_notes/repo_audit.md` | Current code, docs, data, and risks summarized | Use for accurate status claims |
| 2026-05-03 | Cleanup | `docs/project_notes/cleanup_summary.md` | Legacy temporary prompt files removed; no `codex_task*.md` files remain | Use for repository hygiene statement only if needed |
| 2026-05-03 | GPS Serial3 | `docs/gps_serial3_test.md` | GPS bytes received and `status=FIX` observed | Supports GPS communication and FIX verification |
| 2026-05-03 | Hardware status | `docs/current_hardware_status.md` | RC manual, GPS FIX, and failsafe STOP listed as confirmed | Supports implemented/verified status table |
| 2026-05-03 | Mock mission | `data/mock_runs/example_20260503_1801/` | Waypoint CSV/JSON, summary, and preview figure generated | Supports A/B planar coverage planning demonstration |
| 2026-05-03 | Figure captions | `docs/figures/generated/figure_captions.md` | Captions generated with source and claim limits | Use for report figure placement and wording |
| 2026-05-03 | Planner tests | `tests/test_planner.py` | Tests cover waypoint count, lane alternation, spacing, and invalid inputs | Supports mock planner verification |
| 2026-05-03 | Protocol tests | `tests/test_protocol.py` | Tests cover valid frames, checksum errors, malformed frames, and payload commas | Supports protocol utility verification |

## Implemented Evidence Summary

- OpenRB firmware contains RC/manual, STOP, HC-12 parser, GPS Serial3, and USB
  debug behavior.
- Python tools contain safe station loops, mock mission generation, path preview,
  and log analysis.
- ROS2 packages exist as skeletons only.

## Verified Evidence Summary

- RC manual operation verified.
- GPS FIX verified.
- Failsafe STOP verified.
- Mock planner verified through tests and generated artifacts.

## Pending Evidence

- Station-side HC-12 USB confirmation log.
- End-to-end station-to-rover HC-12 command and telemetry log.
- ROS2 bridge runtime test.
- Magnetic adhesion force or slip test.
- Cleaning/painting payload bench test.
- Autonomous coverage execution log.

## Planned Experiments

### HC-12 Bridge Smoke Test

Goal:

- Confirm station HC-12 USB device and basic heartbeat/telemetry exchange.

Safe command:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0
```

Expected evidence:

- Log under `data/hc12_logs/`.
- `ACK`, `STAT`, or `GPS` frames when rover link is active.
- STOP sent on exit.

### GPS Telemetry Schema Test

Goal:

- Confirm firmware and Python tools parse the same GPS payload format.

Expected evidence:

- Updated protocol documentation.
- Passing parser tests.
- GPS logger output with valid rows.

### Mock Planner Reproducibility Test

Goal:

- Regenerate A/B rectangular coverage artifacts for report figures.

Safe command:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir docs/figures/generated/mock_mission_example
```

Expected evidence:

- `waypoints.json`
- `waypoints.csv`
- `mission_summary.txt`
- `path_preview.png`

### ROS2 Bridge Phase 1

Goal:

- Run ROS2 bridge behavior with heartbeat and STOP only.

Expected evidence:

- ROS2 node logs.
- Decoded rover status or clear parse errors.
- No startup `AUTO` command.

### Magnetic Wheel Adhesion Test

Goal:

- Measure whether the planned magnetic wheel concept can support rover mass and
  expected surface orientation.

Expected evidence:

- Test surface description.
- Robot or wheel mass/load data.
- Adhesion force or slip observation.
- Safety setup notes.

## Notes For Report Writers

- Treat generated path figures as mock planner evidence.
- Treat GPS figures as GPS log summaries, not navigation accuracy validation.
- Treat safety figures as selected bench-log summaries, not certification.
- Do not claim HC-12 station integration or ROS2 integration until new entries
  provide evidence.
