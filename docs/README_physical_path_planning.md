# Physical Path Planning (Unified CLI)

Use one field-facing entrypoint:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

Modes:

- `diagnose` — read-only guarded pulse heartbeat, GPS, IMU, and RC telemetry.
- `gps-wait` — wait for GPS cold/warm start without motor output.
- `rc-input-diagnose` — read-only receiver input/channel diagnostic; no motors.
- `manual-rc` — upload and validate manual RC recovery.
- `manual-control` — upload and monitor PPM physical manual control on D6.
- `station-hw-diagnose` — read-only physical station hardware link diagnostic.
- `station-hw-manual` — deprecated serial-frame monitor; use `manual-control` for the current PPM controller.
- `usb-pulse-test` — laptop USB bounded A/B pulse motor validation.
- `tune-motion` — interactive visual/IMU-assisted motion calibration using USB pulses.
- `guarded-pulse-ready` — upload/check IMU-enabled guarded pulse firmware.
- `calibrate-turn` — run turn angle calibration with IMU yaw comparison.
- `preview` — build + render a rectangle coverage plan without motor output.
- `auto-relative-preview` — wait for GPS, resolve a relative A→B field, write the field config + preview without motor output.
- `execute-plan` / `run` — supervised guarded pulse execution (default `--path-control-mode gps_imu_closed_loop`).
- `auto-relative-run` — wait for GPS, then start closed-loop execution when the physical mode switch is set to AUTO.

OpenRB-150 is auto-detected when `--port` is omitted. Use `--port "$PORT"` only
when auto-detection fails.

The closed-loop GPS/IMU 5 m experiment and the AUTO-switch relative run
(`east=3 m, north=4 m`) are documented step by step in
[field_test_manual.md](field_test_manual.md) (sections 12 and 13).

## Quickstart

```bash
bash scripts/run_physical_path_planner.sh diagnose \
  --out-dir outputs/physical_path_planning/diagnose

bash scripts/run_physical_path_planner.sh gps-wait \
  --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
  --out-dir outputs/physical_path_planning/gps_wait

bash scripts/run_physical_path_planner.sh rc-input-diagnose \
  --out-dir outputs/physical_path_planning/rc_input_diagnose

bash scripts/run_physical_path_planner.sh manual-rc \
  --out-dir outputs/physical_path_planning/manual_rc

bash scripts/run_physical_path_planner.sh manual-control \
  --out-dir outputs/physical_path_planning/manual_control

bash scripts/run_physical_path_planner.sh station-hw-diagnose \
  --out-dir outputs/physical_path_planning/station_hw_diagnose

bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --out-dir outputs/physical_path_planning/usb_pulse_test

bash scripts/run_physical_path_planner.sh tune-motion \
  --primitive forward \
  --out-dir outputs/physical_path_planning/tune_forward

bash scripts/run_physical_path_planner.sh guarded-pulse-ready \
  --out-dir outputs/physical_path_planning/guarded_pulse_ready

bash scripts/run_physical_path_planner.sh calibrate-turn \
  --direction left --b-cmd 0.22 --pulse-ms 1200 \
  --target-angle-deg 90 --angle-tolerance-deg 10 \
  --out-dir outputs/physical_path_planning/calibration/left_022_1200

bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
  --workspace-width-m 1.2 --step-spacing-m 0.25 \
  --print-field-config true \
  --out-dir outputs/physical_path_planning/preview_relative_enu

bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --path-control-mode gps_imu_closed_loop \
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

When `--start-lat` / `--start-lon` are omitted, `preview` waits for the current
rover GPS from OpenRB telemetry. The default wait is up to 300 seconds so GPS
cold/warm start can finish. It uses a fresh cached GPS coordinate if the current
fix is temporarily unavailable. If neither is usable, the summary reports
`reason=NO_USABLE_START_GPS`; move outdoors and wait longer, or pass explicit
start coordinates.

Use `--path-shape direct_line` only when a literal straight A-B plan is explicitly
wanted.

Every `preview` and `run` resolves the field geometry before planning. Add
`--print-field-config true` to print the exact resolved configuration, and inspect:

```text
<out-dir>/field_config_resolved.json
```

For `--goal-mode relative_enu`, A is always local `(0, 0)` and B is exactly
`(goal_east_m, goal_north_m)`. The workspace width is perpendicular to the A-B
diagonal. The resolved field config records start source, start/goal lat/lon,
resolved local goal coordinates, width, step spacing, path shape, lane count,
and segment count. `execute-plan --plan-dir ...` loads the same
`field_config_resolved.json` that preview wrote.

## Execution Control Modes

`execute-plan` and `run` default to closed-loop correction:

```bash
bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --path-control-mode gps_imu_closed_loop \
  --live-chunk-ms 700 --max-segment-chunks 20 \
  --gps-degradation-policy continue --gps-reanchor true \
  --imu-heading-hold true --cross-track-correction true \
  --k-heading 0.006 --k-cross-track 0.20 --max-correction-b 0.08 \
  --out-dir outputs/physical_path_planning/execute_gps_imu_closed_loop
```

Control modes:

- `gps_imu_closed_loop` — GPS reanchors position when valid, IMU holds heading,
  and B correction uses heading plus cross-track error.
- `imu_heading` — IMU heading hold only; GPS is not used for cross-track
  correction.
- `open_loop_chunks` — bounded calibrated chunks with no correction, useful only
  as a baseline comparison.

Execution writes:

```text
closed_loop_trace.csv
planned_vs_actual.csv
raw_usbdbg.log
summary.md
summary.json
```

The summary reports whether closed-loop correction was actually enabled, GPS
degradation/reanchor counts, IMU heading usage, cross-track correction usage,
and heading/cross-track error statistics.

## Safety Posture

Physical execution uses IMU-enabled guarded pulse firmware. Full path following,
HC-12 path control, and autonomous startup motion remain disabled. Every summary
must carry:

```text
ready_for_full_path_following=false
```

No mode sends motor output during `diagnose`, `station-hw-diagnose`, `manual-rc`
validation monitoring, or `preview`. The current physical station/controller
manual path is PPM, not serial station frames:

```text
signal -> OpenRB D6
CH1 steering -> physical B
CH2 throttle -> physical A
CH5 mode/manual-auto
```

Run `manual-control` to upload and monitor that PPM manual path. It keeps the
full RC/GPS/IMU USBDBG display visible: sensors are telemetry-only diagnostics
and do not gate manual drive. If PPM channels are all zero, the result is
`PPM_INPUT_ABSENT` and the issue is wiring, receiver power, binding, or PPM
output mode. `station-hw-manual` is deprecated for this hardware unless a real
serial station frame protocol is confirmed.
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

## Interactive Motion Tuning

Use `tune-motion` after `usb-pulse-test` has confirmed bounded A/B commands move
the rover. The tool runs one candidate USB pulse at a time, reads IMU yaw
telemetry when available, asks for simple visual feedback, and adjusts the next
candidate automatically. It does not ask for manually measured distance.

```bash
bash scripts/run_physical_path_planner.sh tune-motion \
  --primitive forward \
  --out-dir outputs/physical_path_planning/tune_forward

bash scripts/run_physical_path_planner.sh tune-motion \
  --primitive backward \
  --out-dir outputs/physical_path_planning/tune_backward

bash scripts/run_physical_path_planner.sh tune-motion \
  --primitive turn-left-90 \
  --out-dir outputs/physical_path_planning/tune_turn_left_90

bash scripts/run_physical_path_planner.sh tune-motion \
  --primitive turn-right-90 \
  --out-dir outputs/physical_path_planning/tune_turn_right_90
```

Operator feedback is one of:

```text
good weak strong too_short too_long left right none retry approve abort
```

Entering `approve` saves the current candidate to:

```text
outputs/physical_path_planning/calibration/motion_calibration.json
```

`execute-plan` and `run` load that file by default. Approved forward/backward
entries become the straight-pulse base, with IMU heading hold applying only a
small clamped B correction. Approved `turn_left_90` and `turn_right_90` entries
become the preferred connector commands; repeated small turn pulses remain a
fallback when approved 90-degree turn calibration is missing.

During physical execution, straight coverage segments use continuous USB live
drive updates by default rather than dense stop-start micro-pulses. GPS is used
for progress and cross-track correction when valid; temporary GPS degradation
continues on IMU heading hold and calibrated progress.

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
