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
- `reset-motion-calibration` — back up and clear approved motion calibration before a full recalibration. No motor output.
- `calibration-check` — report which motion primitives the current plan requires and whether `stop_correct_go` can run. No motor output; exit code is `0` when ready, `1` when calibration is incomplete.
- `guarded-pulse-ready` — upload/check IMU-enabled guarded pulse firmware.
- `calibrate-turn` — run turn angle calibration with IMU yaw comparison.
- `preview` — build + render a rectangle coverage plan without motor output.
- `auto-relative-preview` — wait for GPS, resolve a relative A→B field, write the field config + preview without motor output.
- `align-heading` — point the rover at the first lane heading via a GPS displacement probe + IMU-feedback turn.
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

## ㄹ Coverage Field Runbook (turn_step_turn)

This is the recommended end-to-end workflow for a small ㄹ/lawnmower coverage
field (example: 2.4 m x 2.4 m, 1.2 m lane spacing). Corners are planned as
`pivot -> step-over straight -> pivot` and executed with angle-aware repeated
turn pulses (see Execution Control Modes below).

```bash
RUN_ID=$(date +%Y%m%d_%H%M%S)

# 0. Store the REAL per-pulse turn angle once (example: pulses turn ~30 deg).
#    Skip if motion_calibration.json already holds honest target_angle_deg.
bash scripts/run_physical_path_planner.sh set-motion-calibration \
  --primitive turn-left-90 --a 0.00 --b 0.24 --ms 700 \
  --target-angle-deg 30 --source field_measured_small_pulse \
  --out-dir outputs/physical_path_planning/recalib_left_$RUN_ID
bash scripts/run_physical_path_planner.sh set-motion-calibration \
  --primitive turn-right-90 --a 0.00 --b -0.08 --ms 600 \
  --target-angle-deg 30 --source field_measured_small_pulse \
  --out-dir outputs/physical_path_planning/recalib_right_$RUN_ID

# 1. Confirm calibration (prints per-direction target angles + warnings).
bash scripts/run_physical_path_planner.sh calibration-check \
  --out-dir outputs/physical_path_planning/calibration_check_$RUN_ID

# 2. GPS ready.
bash scripts/run_physical_path_planner.sh gps-wait \
  --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
  --out-dir outputs/physical_path_planning/gps_wait_$RUN_ID

# 3. Preview the ㄹ plan (default --connector-style turn_step_turn).
bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 2.4 --goal-north-m 2.4 \
  --workspace-width-m 2.4 --step-spacing-m 1.2 \
  --path-shape coverage_lawnmower --print-field-config true \
  --out-dir outputs/physical_path_planning/preview_$RUN_ID

# 4. Execute when the preview PNG shows the expected ㄹ.
bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_$RUN_ID \
  --path-control-mode stop_correct_go \
  --initial-heading-align none \
  --move-chunk-ms 800 --max-segment-chunks 160 \
  --settle-after-move-ms 300 --telemetry-stabilize-ms 500 \
  --heading-correction-threshold-deg 15 \
  --heading-correction-tolerance-deg 8 \
  --cross-track-correction-threshold-m 0.35 \
  --sensor-trust-mode imu_gps_first --allow-calibration-fallback true \
  --gps-degradation-policy continue --gps-reanchor true \
  --max-correction-b 0.08 \
  --out-dir outputs/physical_path_planning/execute_$RUN_ID

cat outputs/physical_path_planning/execute_$RUN_ID/summary.md
```

Notes:

- Align the rover with the first lane heading before `execute-plan` when using
  `--initial-heading-align none`; the mission heading frame is captured there.
- Keep `--cross-track-correction-threshold-m` well under the lane spacing
  (0.35 m for 1.2 m lanes); a threshold larger than the spacing disables the
  recovery that pulls the rover back onto its lane.
- Inspect corners in `stop_correct_go_trace.csv`: `phase=connector` rows show
  `requested_turn_angle_deg`, `calibration_target_angle_deg`,
  `turn_pulse_index/turn_pulse_budget`, `applied_turn_delta_deg`,
  `remaining_turn_error_deg`, and `connector_turn_completed`.

## AUTO/MANUAL Switch Workflow (auto-relative-run)

`auto-relative-run` arms the station against the physical PPM mode switch
(CH5): flipping MANUAL -> AUTO starts ONE execution of the planned
relative-ENU path anchored at the current GPS fix; flipping back to MANUAL
stops the rover immediately (worst case one move chunk), records
`stop_reason=USER_SWITCHED_TO_MANUAL`, and leaves RC manual in control.

```bash
# Optional: pre-plan the field, then arm against the switch.
bash scripts/run_physical_path_planner.sh auto-relative-run \
  --goal-east-m 2.4 --goal-north-m 2.4 \
  --workspace-width-m 2.4 --step-spacing-m 1.2 \
  --path-control-mode stop_correct_go \
  --gps-timeout-s 300 --auto-switch-timeout-s 300 \
  --out-dir outputs/physical_path_planning/auto_relative_run_$RUN_ID
```

To run the same designated path again after a MANUAL stop, re-run the command:
each AUTO flip executes the path once, re-anchored at the rover's current
position. `--allow-keyboard-start true` substitutes Enter for the switch when
no PPM receiver is attached.

## Rectangle Geometry

The default `--path-shape` is `coverage_lawnmower`: a local-ENU ㄹ/lawnmower
coverage sweep. It is not a direct A-B follower. The generated plan alternates
straight lane segments and explicit `path_connector` turns so the rover sweeps
the full rectangular area.

For `--goal-mode relative_enu`, A is always local `(0, 0)` and B is
`(goal_east_m, goal_north_m)`. `--workspace-width-m` is the coverage rectangle
width and `--step-spacing-m` controls lane spacing. The preview writes both the
resolved geometry and mandatory images:

```text
field_config_resolved.json
planned_segments.csv
planned_segments.json
preview_current_goal_rectangle_path.png
preview_overview.png
```

If an image cannot be written, preview fails with
`reason=PREVIEW_IMAGE_NOT_WRITTEN`.

When `--start-lat` / `--start-lon` are omitted, `preview` waits for the current
rover GPS from OpenRB telemetry. The default wait is up to 300 seconds so GPS
cold/warm start can finish. It uses a fresh cached GPS coordinate if the current
fix is temporarily unavailable. If neither is usable, the summary reports
`reason=NO_USABLE_START_GPS`; move outdoors and wait longer, or pass explicit
start coordinates.

Use `--path-shape diagonal_rectangle_serpentine` only when the older A-B diagonal
frame is intentionally requested; it prints a warning because it is not the ㄹ
coverage path. Use `--path-shape direct_line` only when a literal straight A-B
plan is explicitly wanted.

Every `preview` and `run` resolves the field geometry before planning. Add
`--print-field-config true` to print the exact resolved configuration, and inspect:

```text
<out-dir>/field_config_resolved.json
```

The resolved field config records start source, start/goal lat/lon, resolved
local goal coordinates, width, step spacing, path shape, lane count, connector
count, segment count, expected sweep style, and image paths. `execute-plan
--plan-dir ...` loads the same `field_config_resolved.json` and
`planned_segments.json` that preview wrote; it does not regenerate a different
shape.

Inspect a saved plan without motion:

```bash
bash scripts/run_physical_path_planner.sh inspect-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu
```

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
- `stop_correct_go` — discrete move/stop/sense/correct cycles: drive a bounded
  calibrated chunk, stop, read a stabilized GPS/IMU heartbeat, apply a bounded
  heading correction (and small cross-track trim) only while stopped, then
  continue. Sensor priority follows `--sensor-trust-mode` (`imu_gps_first` by
  default); GPS degradation dead-reckons on the IMU. This mode requires real
  forward motion calibration (and backward for multi-lane serpentine plans);
  `run` and `auto-relative-run` abort **before any motion** with
  `reason=CALIBRATION_INCOMPLETE` when a required primitive is still the
  repeated-pulses fallback. Run `calibration-check` first to confirm readiness.

Connector turns in `stop_correct_go` are angle-aware: the calibration's
`target_angle_deg` is the rotation ONE pulse really produces (often 15-45
degrees on this rover despite the `turn_*_90` key name), so each planned
corner pulses repeatedly and stops on the measured IMU yaw delta:

- `--max-connector-pulses-per-turn` (default 6) — rotation-loop guard.
- `--connector-turn-tolerance-deg` (default 10) — IMU stop window.
- `--turn-calibration-angle-policy from_json|assume_90` (default `from_json`).
- `--turn-angle-deg-override` — substitute a measured per-pulse angle without
  editing the calibration JSON.
- `--heading-reference mission|per_lane` (default `mission`) — mission chains
  one yaw frame across the whole run so connector under-turns stay visible on
  the next lane and get corrected; `per_lane` is the legacy re-capture behavior
  that silently absorbed them (the historical cause of rounded ㄹ corners).

Execution writes:

```text
closed_loop_trace.csv
planned_vs_actual.csv
raw_usbdbg.log
summary.md
summary.json
```

`stop_correct_go` additionally writes `stop_correct_go_trace.csv` (per-chunk
move/correction trace) and `heading_correction_trace.csv` (only the stopped
correction cycles).

The summary reports whether closed-loop correction was actually enabled, GPS
degradation/reanchor counts, IMU heading usage, cross-track correction usage,
and heading/cross-track error statistics. The `stop_correct_go` summary adds
`sensor_trust_mode`, heading-correction counts/successes, cross-track-trim
count, and sensor-fallback count.

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

## Manual Motion Calibration Override

When visual tuning is unstable for one primitive, use `set-motion-calibration`
to write an approved manual preset or a single primitive override. This is a
local file operation: it backs up the existing `motion_calibration.json`,
preserves primitives that are not being overwritten, validates the A/B signs,
and writes the updated calibration back to:

```text
outputs/physical_path_planning/calibration/motion_calibration.json
```

Field preset with strong forward/backward/left and softer right:

```bash
bash scripts/run_physical_path_planner.sh set-motion-calibration \
  --preset field_manual_high_except_soft_right \
  --out-dir outputs/physical_path_planning/calibration_manual_override
```

Explicit soft right-turn override:

```bash
bash scripts/run_physical_path_planner.sh set-motion-calibration \
  --primitive turn-right-90 \
  --a 0.0 --b -0.06 --ms 800 \
  --target-angle-deg 90 \
  --source manual_soft_right_test \
  --out-dir outputs/physical_path_planning/recalib_turn_right_soft
```

Right-turn soft candidates for field testing:

```text
b=-0.06 ms=800
b=-0.08 ms=1000
b=-0.10 ms=1000
```

The physical mapping is unchanged: `A>0` forward, `A<0` backward, `B>0` left,
and `B<0` right. `ready_for_full_path_following` remains `false`.

To recalibrate from scratch without losing the prior values, back up first:

```bash
bash scripts/run_physical_path_planner.sh reset-motion-calibration \
  --out-dir outputs/physical_path_planning/reset_motion_calibration
```

This copies the existing `motion_calibration.json` to a timestamped
`motion_calibration.backup_<stamp>.json` sibling, then clears the original so the
next `tune-motion` approvals start clean. `tune-motion --reset-calibration true`
does the same backup-then-clear at the start of one session.

## Initial Heading Alignment

The IMU reports a relative yaw, not an absolute compass heading, so `align-heading`
derives the absolute starting heading from a short GPS displacement probe and uses
the IMU only as turn feedback.

```bash
bash scripts/run_physical_path_planner.sh align-heading \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --strategy gps_probe \
  --probe-a 0.25 --probe-duration-s 1.0 --min-probe-distance-m 0.30 \
  --heading-tolerance-deg 8 --turn-b-left 0.24 --turn-b-right -0.12 \
  --out-dir outputs/physical_path_planning/preview_relative_enu/alignment
```

Strategies are `gps_probe` (drive a short forward probe, estimate the current ENU
heading from GPS displacement, then turn left `B>0` / right `B<0` by the shortest
heading error and stop on the IMU yaw delta), `user_confirmed` (point the rover by
hand, press Enter, capture the current IMU yaw as the lane reference), and `skip`
(no alignment, no serial). Displacement below `--min-probe-distance-m` reports
`reason=PROBE_GPS_DISPLACEMENT_TOO_SMALL`.

`execute-plan`, `run`, and `auto-relative-run` run the same alignment inline via
`--initial-heading-align none|gps_probe|user_confirmed` (default `none` for
`execute-plan`/`run`, `gps_probe` for `auto-relative-run`). On success the aligned
IMU yaw becomes the first lane reference; per-lane references are still recaptured
after each connector turn. A `gps_probe` failure aborts before any path motion;
`user_confirmed` and `none` never abort on alignment. The A/B mapping is unchanged:
`A>0` forward, `A<0` backward, `B>0` left, `B<0` right.

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
