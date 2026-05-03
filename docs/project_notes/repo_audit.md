# Repository Audit

Audit date: 2026-05-03 KST

## Scope

This audit inspected the current pre-ROS2 Python/OpenRB repository for the ship exterior cleaning and painting robot course project. The current project assumption is a planar ship outer-hull work surface with magnetic-wheel adhesion planned later. The intended workflow is manual driving, A/B point recording, rectangular work-region definition, and lawnmower-style coverage-path generation.

## Current Directory Structure

- `README.md`, `pyproject.toml`, `uv.lock`, `uv.toml`, `Makefile`: project setup, dependency, and task entry points.
- `.github/workflows/`: Python test workflow using `uv`, `pytest`, `verify_env.py`, and requirements export.
- `gps_coverage_core/`: ROS-independent Python protocol, geodesy, NMEA, telemetry, and planner modules.
- `tools/`: station-side Python utilities for serial testing, safe HC-12 loops, GPS logging, path previews, mock missions, NMEA replay, environment checks, and log analysis.
- `firmware/`: OpenRB/Arduino sketches for the integrated rover controller and focused GPS, HC-12 echo, PPM, and RC-mix tests.
- `ros2_ws/src/`: ROS2 Jazzy package skeletons only. No functional ROS2 bridge, mission, planner, or follower behavior is implemented yet.
- `tests/`: Python tests for core protocol, geodesy, and planner behavior.
- `docs/`: architecture, wiring, protocol, safety, hardware status, RC mapping, WSL USB, GPS Serial3 result, and test-plan documentation.
- `data/`: local evidence and generated data, including GPS logs, HC-12 logs, safety logs, and mock mission outputs.
- `outputs/`: ignored local run outputs.

Local generated/cache directories are also present, including `.venv/`, `.pytest_cache/`, `__pycache__/`, and ignored run logs. These are not project documentation.

## Implemented Modules

- `gps_coverage_core.protocol`
  - Encodes and decodes `@TYPE,SEQ,PAYLOAD*CS` frames.
  - Implements XOR checksums, command-value clamping, compact command-float formatting, and manual command field generation.
  - Raises `ValueError` for malformed frames.
- `gps_coverage_core.planner`
  - Generates a simple alternating lawnmower path from A/B GPS points.
  - Uses a local equirectangular approximation and a default of four lanes.
  - Validates positive spacing and non-identical A/B points.
- `gps_coverage_core.geo`
  - Converts between WGS84 lat/lon and local East/North meters with GeographicLib.
- `gps_coverage_core.nmea`
  - Parses supported GGA/RMC NMEA sentences into compact GPS samples.
- `gps_coverage_core.telemetry`
  - Defines a positional GPS telemetry data model for decoded GPS frame payloads.
- `tools/station_controller.py`
  - Safe station loop and reusable `StationController`.
  - Defaults to `/dev/ttyACM0`, 9600 baud, heartbeat plus periodic `STOP`.
  - Sends five `STOP` frames on Ctrl+C.
- `tools/station_hc12_test.py`
  - Safe HC-12 heartbeat and telemetry receive test.
- `tools/station_keyboard_manual.py`
  - Wheel-off-ground manual keyboard test tool with a `0.25` first-test speed cap, neutral frames, deadman, E-stop, and STOP-on-exit behavior.
- `tools/gps_logger.py`
  - Logs decoded GPS frames to CSV, subject to the GPS payload-schema risk noted below.
- `tools/path_preview.py`
  - Generates a matplotlib preview figure for a lawnmower path.
- `tools/station_mock_mission.py`
  - Generates mock mission waypoints, CSV/JSON, summary text, and optional preview figure without HC-12 or ROS.
- `tools/analyze_gps_log.py` and `tools/analyze_safety_log.py`
  - Parse OpenRB `USBDBG` logs and summarize GPS quality and safety behavior.
- `tools/nmea_replay.py`, `tools/hc12_terminal.py`, `tools/serial_open_test.py`, `tools/verify_env.py`
  - Provide offline replay, HC-12 AT command safety guard, serial smoke testing, and environment diagnostics.
- `firmware/openrb_robot_controller/openrb_robot_controller.ino`
  - Integrated OpenRB controller with PPM RC input, RC manual priority, HC-12 frame handling, STOP, manual, AUTO/START command handling, link timeout, failsafe state, USB debug output, and GPS Serial3/TinyGPS++ integration.
  - Uses `HC12_SERIAL Serial2`, `GPS_SERIAL Serial3`, GPS baud 9600, USB debug baud 115200.
  - Maintains the safe startup posture: motor stop at setup, CH5 high enters `AUTO_READY` only, and explicit AUTO is required for autonomous drive.
- Focused firmware sketches
  - `firmware/gps_test/gps_test.ino`
  - `firmware/hc12_echo_test/hc12_echo_test.ino`
  - `firmware/ppm_test/ppm_test.ino`
  - `firmware/rc_mix_test/rc_mix_test.ino`

## Current Documentation

- `README.md`: project overview, safe station defaults, setup, WSL USB attach, serial defaults, safety, layout, and quick-start commands.
- `docs/architecture.md`: ground-station/rover split, safety boundary, and protocol boundary.
- `docs/current_hardware_status.md`: confirmed OpenRB, RC, GPS FIX, failsafe STOP, and pending station HC-12 state.
- `docs/gps_serial3_test.md`: GPS Serial3 wiring and FIX test result. This file is present in the workspace and is currently untracked.
- `docs/protocol.md`: line protocol, examples, command payloads, and safety expectations.
- `docs/rc_channel_map.md`: confirmed CH1 steering, CH2 throttle, CH5 Manual/Auto, CH7 unused.
- `docs/safety_checklist.md`: safe defaults and bench-test constraints.
- `docs/test_plan.md`: bench, station, planner, and ROS2 skeleton validation steps.
- `docs/wiring.md`: HC-12 wiring and voltage caution notes.
- `docs/wsl_usb_guide.md`: USB attach workflow for WSL2.
- `firmware/README.md`: firmware bring-up cautions.

## Data And Generated Figures

- GPS evidence logs are under `data/gps_logs/`.
- HC-12 station logs are under `data/hc12_logs/`.
- Safety logs are under `data/safety_logs/`.
- Mock mission outputs are under `data/mock_runs/`:
  - `waypoints.json`
  - `waypoints.csv`
  - `mission_summary.txt`
  - `path_preview.png`
  - `example_20260503_1801/path_preview.png`
- The generated preview figures are PNG files at 1280 x 960.

Most logs are ignored by `.gitignore`. Some mock mission JSON/TXT/PNG outputs are currently untracked rather than ignored.

## Legacy Codex Prompt Cleanup

- No active `codex_task*.md` files remain.
- No `docs/codex/tasks/*.md` files are present.
- Removed temporary prompt artifacts found during this audit:
  - `codex_integrate_gps_serial3.md`
  - `outputs/codex_cleanup/legacy_codex_task_snapshot.md`
- Re-ran the prompt-file search after cleanup and found no remaining `codex_task`, `*codex*task*`, or `codex_integrate*.md` files.

## Risks Before Report-Material Generation

- HC-12 station integration is still pending. The report should not claim complete station-to-rover HC-12 operation until the station HC-12 USB device and link test are confirmed.
- GPS telemetry payloads need schema reconciliation. The firmware currently emits GPS frames as key/value fields such as `fix=1,lat=...`, while `GPSTelemetry.from_payload()` and some Python tools expect positional fields such as `lat,lon,alt,sats,hdop,fix_valid`.
- ROS2 is skeleton-only. The current ROS2 packages are placeholders and should be described as planned integration, not completed functionality.
- Coverage planning is currently planar and simplified. It does not yet handle ship-hull curvature, obstacles, edge exclusion zones, spray/paint process constraints, adhesion margins, or localization uncertainty.
- Manual keyboard station behavior needs a final bench smoke test before it is used as an operator workflow. In particular, RX display handling should be reviewed because `StationController.poll()` returns dictionaries.
- Generated mock mission outputs and logs need a final artifact policy before report handoff: decide which examples should be tracked, ignored, or regenerated during report build.
- Tracked `firmware/gps_test/*.bak.*` files may confuse readers unless kept intentionally as bring-up history.
- Do not overstate safety validation. Current progress confirms failsafe STOP behavior, but live rover motor testing remains wheel-off-ground only.

## Recommended Next Steps

1. Settle and document one GPS frame payload schema, then update firmware, Python parsers, tests, and `docs/protocol.md` consistently.
2. Confirm station HC-12 hardware attachment and run `tools/station_hc12_test.py --port /dev/ttyACM0` or the actual `--port`.
3. Run `uv run pytest -q` and `uv run python tools/verify_env.py` after the schema cleanup.
4. Compile the OpenRB controller sketch after any firmware edits; do not upload unless explicitly requested.
5. Decide whether `docs/gps_serial3_test.md`, analysis tools, and `station_mock_mission.py` should be staged as final project files.
6. Decide whether generated mock mission artifacts should be tracked as report evidence or left as reproducible local outputs.
7. Prepare report diagrams from verified states only: hardware status, workflow, protocol boundary, safety boundary, and mock coverage path preview.
8. Keep ROS2 claims limited to package skeletons and planned Jazzy integration until nodes implement real behavior.
