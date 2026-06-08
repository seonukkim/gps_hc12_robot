# Physical Path Planning (Unified CLI)

Use one field-facing entrypoint:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

Modes:

- `diagnose` — read-only guarded pulse heartbeat, GPS, IMU, and RC telemetry.
- `rc-input-diagnose` — read-only receiver input/channel diagnostic; no motors.
- `manual-rc` — upload and validate manual RC recovery.
- `station-hw-diagnose` — read-only physical station hardware link diagnostic.
- `station-hw-manual` — physical station hardware manual rover control.
- `usb-pulse-test` — laptop USB bounded A/B pulse motor validation.
- `guarded-pulse-ready` — upload/check IMU-enabled guarded pulse firmware.
- `calibrate-turn` — run turn angle calibration with IMU yaw comparison.
- `preview` — build + render a rectangle coverage plan without motor output.
- `execute-plan` / `run` — supervised guarded pulse execution.

OpenRB-150 is auto-detected when `--port` is omitted. Use `--port "$PORT"` only
when auto-detection fails.

## Quickstart

```bash
bash scripts/run_physical_path_planner.sh diagnose \
  --out-dir outputs/physical_path_planning/diagnose

bash scripts/run_physical_path_planner.sh rc-input-diagnose \
  --out-dir outputs/physical_path_planning/rc_input_diagnose

bash scripts/run_physical_path_planner.sh manual-rc \
  --out-dir outputs/physical_path_planning/manual_rc

bash scripts/run_physical_path_planner.sh station-hw-diagnose \
  --out-dir outputs/physical_path_planning/station_hw_diagnose

bash scripts/run_physical_path_planner.sh station-hw-manual \
  --out-dir outputs/physical_path_planning/station_hw_manual

bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --out-dir outputs/physical_path_planning/usb_pulse_test

bash scripts/run_physical_path_planner.sh guarded-pulse-ready \
  --out-dir outputs/physical_path_planning/guarded_pulse_ready

bash scripts/run_physical_path_planner.sh calibrate-turn \
  --direction left --b-cmd 0.22 --pulse-ms 1200 \
  --target-angle-deg 90 --angle-tolerance-deg 10 \
  --out-dir outputs/physical_path_planning/calibration/left_022_1200

bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
  --workspace-width-m 1.2 --step-spacing-m 0.25 \
  --out-dir outputs/physical_path_planning/preview_relative_enu

bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --out-dir outputs/physical_path_planning/execute_preview_relative_enu
```

Every command writes:

```text
<out-dir>/summary.md
<out-dir>/summary.json
```

Use `cat <out-dir>/summary.md` or `cat <out-dir>/summary.json` as the first
inspection step after every run.

## Rectangle Geometry

`start` (A) and `goal` (B) are opposite corners of the workspace rectangle's
diagonal, not a direct driving line. `--workspace-width-m` is the short side and
must be shorter than the A-B diagonal. `--step-spacing-m` controls lane spacing.

Use `--path-shape direct_line` only when a literal straight A-B plan is explicitly
wanted.

## Safety Posture

Physical execution uses IMU-enabled guarded pulse firmware. Full path following,
HC-12 path control, and autonomous startup motion remain disabled. Every summary
must carry:

```text
ready_for_full_path_following=false
```

No mode sends motor output during `diagnose`, `station-hw-diagnose`, `manual-rc`
validation monitoring, or `preview`. `station-hw-manual` listens to the separate
physical station hardware and maps station throttle/steering to physical A/B.
Station hardware transport/protocol are reported as `station_transport`,
`station_protocol`, and `station_parser`; the workflow does not assume a fixed
radio, UART, or baud setting. If frames arrive but no parser matches, the result
is `WRONG_STATION_FRAME_PARSER` and the first unmatched frames are written to
`raw_station_frames.txt` and `raw_station_frames_hex.txt`.
`usb-pulse-test` sends only bounded A/B commands over USB after operator
confirmation; it is not physical station hardware control, not RC manual
passthrough, and not autonomous path planning. Guarded pulse execution remains
supervised and aborts on serial disconnect, `REJECT`, `RC_INVALID`, missing
ACK/STOP, nonzero final motor commands, or output still active after STOP.

For station-supervised motor validation, first print the exact commands without
opening serial:

```bash
bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --print-command true \
  --out-dir outputs/physical_path_planning/usb_pulse_test_print
```

The live `usb-pulse-test` flow prints only concise operator status by default:
heartbeat ready, command sent, ACK/ACTIVE/STOP, final zero, and observed motion.
Raw firmware telemetry is saved to `raw_usbdbg.log`; add `--verbose-raw true`
only when debugging the serial stream. By default usb-pulse-test does not require
receiver input, so `RC_INPUT_ABSENT` does not block bounded station pulse testing.

If `manual-rc` reports `reason=RC_INPUT_ABSENT`, all receiver channel inputs are
zero or absent. Check receiver power, transmitter binding, receiver signal wiring,
PPM/SBUS/PWM output mode, firmware input mode, and channel mapping. After wiring or
binding changes, rerun:

```bash
bash scripts/run_physical_path_planner.sh manual-rc \
  --upload false --validate true --diagnose-only true \
  --out-dir outputs/physical_path_planning/manual_rc_diagnose
```

Use only `scripts/run_physical_path_planner.sh` for field work.
