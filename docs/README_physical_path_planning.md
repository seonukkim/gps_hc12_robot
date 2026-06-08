# Physical Path Planning (Integrated CLI)

One package, one CLI, five modes. `tools/physical_path_planning/` consolidates the
A→B serpentine geometry, calibration resolution, telemetry/safety parsing, guarded
pulse execution, and the continuous-motion controller that used to be scattered
across the Stage30–36 modules. There is exactly one entrypoint:

```
uv run python -m tools.physical_path_planning.cli <mode> [options]
# or the launcher wrapper (adds the firmware flash for run/execute-plan):
scripts/run_physical_path_planner.sh <mode> [options]
```

Modes: `preview | calibrate-turn | execute-plan | run | diagnose`
(`execute-plan` is an alias of `run`).

See also [physical_path_planning_architecture.md](physical_path_planning_architecture.md)
for the module map and [field_test_manual.md](field_test_manual.md) for the
operator step-by-step.

## The one mental model: A→B is the DIAGONAL

`start` (A) and `goal` (B) are **opposite corners of a rectangle's diagonal** — they
are *not* a straight driving line. You also give `--workspace-width-m`, the short
side, which **must be shorter than the A→B diagonal**. The planner derives the long
side as `sqrt(diagonal² − width²)`, lays serpentine lanes `--step-spacing-m` apart,
and inserts connector turns between lanes.

If you genuinely want a straight line from A to B, pass
`--path-shape direct_line` (the escape hatch); width is then not required.

## Safety posture (read first)

- **Nothing here weakens motor-output safety.** For `run`/`execute-plan`, the
  launcher flashes the STAGE20 *guarded-crawl* firmware behind the same 4-flag
  compile gate as `scripts/run_stage20_physical_ab_probe.sh`. That firmware gate —
  not this CLI — is the real motor-output safety. This is guarded, bounded motion,
  **not** full autonomous path following.
- **Every summary carries `ready_for_full_path_following=false`.** It is enforced in
  code (`checks.assert_not_ready_for_full_path_following`) on every emit path —
  preview, controller, and diagnose. No mode can claim full-path-following readiness.
- Physical execution still depends on the field preconditions in
  [current_hardware_status.md](current_hardware_status.md) and the root `README.md`
  (stable outdoor GPS fix, RC AUTO/MANUAL mode channel that actually holds, etc.).
  The hardware modes execute when invoked — the operator is trusted to satisfy those
  preconditions at the field.

## No-hardware paths (fully testable, no serial, no firmware)

You can exercise everything except the actual motor pulses without a board:

- `preview` — never opens serial; pure geometry + render.
- `run --print-plan` / `execute-plan --print-plan` — builds and writes the plan,
  opens **no** serial, flashes **no** firmware.
- `calibrate-turn --print-cmd` — prints the exact Stage20 shell-out it *would* run
  and exits; no firmware, no serial.
- `diagnose --from-log FILE` — summarizes a saved serial log; no serial.

## Modes

### preview — build + render the plan (no motion)

```bash
uv run python -m tools.physical_path_planning.cli preview \
  --start-lat 35.1 --start-lon 129.1 \
  --goal-mode bearing_distance --goal-bearing-deg 90 --goal-distance-m 6 \
  --workspace-width-m 2
```

Goal modes: `absolute` (`--goal-lat/--goal-lon`), `relative_enu`
(`--goal-east-m/--goal-north-m`), `relative_latlon` (`--goal-dlat/--goal-dlon`),
`bearing_distance` (`--goal-bearing-deg/--goal-distance-m`). Writes
`preview_summary.json` (+ `preview.png` unless `--no-png`) under
`outputs/physical_path_planning/preview`. Works with **no calibration at all**:
missing turn-angle calibration falls back to repeated-pulse connectors
(`fallback_to_repeated_pulses=true`) rather than blocking the preview.

A worked example config ships at `configs/field_rectangle_example.json` — every key
is a `build_preview()` kwarg, so it loads directly.

### calibrate-turn — measure a 90° turn (IMU yaw)

```bash
uv run python -m tools.physical_path_planning.cli calibrate-turn \
  --port /dev/ttyACM0 --mode turn_left --save-turn-calibration true
```

This **shells out** to `scripts/run_stage20_physical_ab_probe.sh` and always passes
`--imu-angle-compare true`, which makes that script compile the BMI160 yaw flags
(`-DIMU_ENABLE=1 -DIMU_YAW_DIAG=1`), upload, and measure before/after yaw. The CLI
itself never opens serial here. With `--save-turn-calibration true` and a visually
confirmed turn, it writes
`outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json`
— exactly the file the resolver reads back as the angle-calibrated connector.

### run / execute-plan — guarded continuous motion

```bash
scripts/run_physical_path_planner.sh run \
  --start-lat 35.1 --start-lon 129.1 --goal-lat 35.1006 --goal-lon 129.1006 \
  --workspace-width-m 2
```

Via the launcher, this flashes the guarded-crawl firmware (bounded
`MAX_ABS_A/B`, `MAX_MS`, IMU yaw for heading-hold; all path-following/stage flags
forced to 0), re-resolves the upload port, then drives `controller.run_controller`.
Add `--print-plan` to build the plan with no serial and no flash. Outputs
(`run_summary.json`, `run_rows.csv`, `run_serial.log`) land under
`outputs/physical_path_planning/run`. Exit code is non-zero when the run aborts.

The controller invents no new control law — it reuses the tested pieces: guarded
`executor.send_pulse`, `geometry.compute_b_correction` (steering on the **B axis
only**, clamped to ±0.08; forward A and pulse-ms come straight from calibration and
are never lowered), and `geometry.gps_policy_action`. Loop behavior: hold the
segment heading between pulses from IMU yaw; on GPS degradation continue
dead-reckoned and flag `gps_degraded` (policy configurable); hard-abort on serial
disconnect, `RC_INVALID` during a pulse, or RC manual override; wait up to
`--rc-neutral-wait-s` for neutral sticks before a pulse.

### diagnose — read-only telemetry summary

```bash
# live port:
uv run python -m tools.physical_path_planning.cli diagnose --port /dev/ttyACM0 --duration-s 5
# or from a saved log (no serial):
uv run python -m tools.physical_path_planning.cli diagnose \
  --from-log outputs/physical_path_planning/run/run_serial.log
```

Counts events, reports the last heartbeat's GPS/IMU fields and whether the firmware
exposes the STAGE20 guarded-crawl role. Read-only; never sends a command.

## Ports and baud

Default `--port /dev/ttyACM0` (the Ubuntu station). On the Mac dev machine the
OpenRB enumerates as `/dev/cu.usbmodem*` — pass `--port` explicitly. The USB debug
(USBDBG/telemetry) link is `115200`; the GPS module itself is `9600` on the rover's
internal `Serial2` and is not what the CLI talks to.

## Configs

`configs/physical_path_planning_default.json` documents every runtime knob (mirrors
the argparse defaults; the CLI does not auto-load it). `configs/calibration_default.json`
is the resolver's fallback motion set, kept in lockstep with the code by a parity
test. `configs/field_rectangle_example.json` is a directly-loadable A→B-diagonal
preview example.
