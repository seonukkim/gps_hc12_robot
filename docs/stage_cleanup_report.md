# Numbered Stage Cleanup Report

Cleanup date: 2026-06-08

Primary workflow:

```bash
bash scripts/run_physical_path_planner.sh <mode> --out-dir <dir>
```

The active user-facing workflow is now the unified physical path planner. Old
numbered stage launchers, tools, tests, and presets are quarantined under
`legacy/`.

## Deleted Files

None. Files were moved rather than deleted because several contain historical
calibration logic or local edits that may still be useful for forensic review.

## Moved To Legacy

Moved active scripts to `legacy/stage_scripts/`:

- `scripts/run_guarded_motor_sanity_crawl.sh`
- previously quarantined `run_stage*.sh` wrappers
- old GPS/IMU/manual-RC diagnostic wrappers from the fragmented workflow

Moved active tools to `legacy/stage_tools/`:

- `tools/stage*.py`
- `tools/check_stage*.py`
- `tools/build_stage*.py`
- old guarded-crawl helpers used only by historical stage tests
- old GPS/IMU/manual-RC diagnostic tools from the fragmented workflow

Moved active tests to `legacy/stage_tests/`:

- `tests/test_stage*.py`
- old GPS/IMU/manual-RC diagnostic tests from the fragmented workflow

Moved active configs to `legacy/stage_configs/`:

- `configs/stage32_known_good_block1.json`
- `configs/stage32_known_good_two_blocks_left12_right12.json`

Legacy directories now include README files stating that they are deprecated and
that `scripts/run_physical_path_planner.sh` is the active workflow.

## Wrapper / Compatibility

- `legacy/stage_scripts/run_guarded_pulse_calibration.sh` is a neutral internal
  wrapper for the old guarded pulse calibration launcher.
- The unified `calibrate-turn` mode uses that wrapper internally so the public
  command remains functional without restoring old launchers to `scripts/`.

## Remaining Numbered References

These remain by design:

- Firmware macros and telemetry fields such as `STAGE20_*`,
  `stage20_cmd_state`, and `stage16_reject_reason`. These are compatibility
  names emitted by existing firmware and are intentionally not changed in this
  cleanup.
- `tools/physical_path_planning/` compatibility parsing for those firmware
  fields. Renaming them would change runtime protocol behavior, which this
  cleanup explicitly avoids.
- Calibration resolver fallback keys such as `stage22_forward_a_cmd` and output
  paths under historical `outputs/stage...` directories. These are accepted
  input formats for old calibration JSONs and are not user-facing entry points.
- Active tests may include generated telemetry fixture field names so the
  unified parser continues to accept current firmware logs.
- Files under `legacy/` intentionally keep historical names.

## Active Workflow Confirmation

- Active top-level `scripts/` has no `run_stage*.sh` files.
- Active top-level `tools/` has no `stage*.py`, `check_stage*.py`, or
  `build_stage*.py` files.
- Active top-level `tests/` has no `test_stage*.py` files.
- Active `configs/` has no `*stage*` presets.
- Primary docs recommend `scripts/run_physical_path_planner.sh` only.
- `scripts/run_physical_path_planner.sh --help` does not expose numbered stage
  terminology.
