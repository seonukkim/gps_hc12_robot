# Numbered Workflow Cleanup Report

Cleanup date: 2026-06-08

Primary workflow:

```bash
bash scripts/run_physical_path_planner.sh <mode> --out-dir <dir>
```

The active user-facing workflow is now the unified physical path planner. Old
numbered development launchers, tools, tests, and presets are quarantined under
`legacy/`.

## Deleted Files

None. Files were moved rather than deleted because several contain historical
calibration logic or local edits that may still be useful for forensic review.

## Moved To Legacy

Moved active scripts to the legacy scripts directory:

- `scripts/run_guarded_motor_sanity_crawl.sh`
- previously quarantined numbered wrappers
- old GPS/IMU/manual-RC diagnostic wrappers from the fragmented workflow

Moved active tools to the legacy tools directory:

- old numbered tool modules
- old numbered check modules
- old numbered build modules
- old guarded-crawl helpers used only by historical numbered tests
- old GPS/IMU/manual-RC diagnostic tools from the fragmented workflow

Moved active tests to the legacy tests directory:

- old numbered test modules
- old GPS/IMU/manual-RC diagnostic tests from the fragmented workflow

Moved active configs to the legacy configs directory:

- old numbered known-good config presets

Legacy directories now include README files stating that they are deprecated and
that `scripts/run_physical_path_planner.sh` is the active workflow.

## Wrapper / Compatibility

- The legacy guarded-pulse calibration script is a neutral internal
  wrapper for the old guarded pulse calibration launcher.
- The unified `calibrate-turn` mode uses that wrapper internally so the public
  command remains functional without restoring old launchers to `scripts/`.

## Remaining Numbered Internals

These remain by design:

- Firmware compatibility macros and telemetry fields may remain internally when
  changing them would alter runtime protocol behavior.
- The unified parser may still accept those internal compatibility fields.
- Calibration resolver fallback keys and historical output paths may still be
  accepted as input formats for old calibration JSONs; they are not user-facing
  entry points.
- Active tests may include generated telemetry fixture field names so the
  unified parser continues to accept current firmware logs.
- Files under `legacy/` intentionally keep historical names.

## Active Workflow Confirmation

- Active top-level `scripts/` has no numbered launchers.
- Active top-level `tools/` has no numbered tool/check/build modules.
- Active top-level `tests/` has no numbered test modules.
- Active `configs/` has no `*stage*` presets.
- Primary docs recommend `scripts/run_physical_path_planner.sh` only.
- `scripts/run_physical_path_planner.sh --help` does not expose numbered stage
  terminology.
