# Physical Path Planning Field Test Manual

Use the unified launcher for field work:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

OpenRB-150 is auto-detected when `--port` is omitted. Use `--port "$PORT"` only
when auto-detection fails.

## Preconditions

- RC transmitter on.
- MANUAL / AUTO OFF available for recovery.
- GPS antenna mounted for real rover position tests.
- BMI160 IMU telemetry available for turn angle calibration and heading logs.
- Clear test area or wheels off ground for first motion after firmware changes.
- Full path following and HC-12 path control remain disabled.

## 1. Diagnose (Read Only)

```bash
bash scripts/run_physical_path_planner.sh diagnose \
  --out-dir outputs/physical_path_planning/diagnose
```

This watches telemetry only. It sends no motor commands.

## 2. GPS Wait (Read Only)

```bash
bash scripts/run_physical_path_planner.sh gps-wait \
  --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
  --out-dir outputs/physical_path_planning/gps_wait
```

This watches GPS and IMU telemetry only. It sends no motor commands. Use it when
GPS characters are increasing but the solution is not valid yet; a cold/warm
start can take several minutes outdoors. On success it saves a cached start
coordinate for preview/run.

## 3. Manual RC Recovery

If manual RC telemetry shows all receiver channels at zero, run the receiver-only
input diagnostic first:

```bash
bash scripts/run_physical_path_planner.sh rc-input-diagnose \
  --out-dir outputs/physical_path_planning/rc_input_diagnose
```

This uploads a read-only PPM channel probe. It does not initialize motors and
classifies whether receiver input is absent, present but invalid, or valid PPM.

```bash
bash scripts/run_physical_path_planner.sh manual-rc \
  --out-dir outputs/physical_path_planning/manual_rc
```

Follow the printed sequence:

1. neutral 5 seconds
2. slight forward
3. neutral
4. slight backward
5. neutral
6. slight left/right steering
7. neutral

PASS requires `control_source=RC_MANUAL`, nonzero final motor commands while the
sticks move, physical output active or motor writes during manual movement, and
final commands returning to zero at neutral.

If the summary reports `reason=RC_INPUT_ABSENT`, the receiver signal is not reaching
OpenRB. Check receiver power, transmitter binding, signal wiring, PPM/SBUS/PWM
output mode, firmware input mode, and channel mapping. This is not a motor
calibration failure and it does not by itself invalidate GPS or IMU diagnostics.

After wiring or binding changes:

```bash
bash scripts/run_physical_path_planner.sh manual-rc \
  --upload false --validate true --diagnose-only true \
  --out-dir outputs/physical_path_planning/manual_rc_diagnose
```

## 4. PPM Physical Manual Control

The current physical station/controller manual path is PPM into OpenRB D6. It is
not the serial station-frame monitor, and it is not a sensor-disabled mode. GPS
and IMU status remain visible in the manual-control USBDBG/status display when
available, but they do not gate manual motor output.

```bash
bash scripts/run_physical_path_planner.sh manual-control \
  --out-dir outputs/physical_path_planning/manual_control
```

Expected wiring and channel mapping:

- signal -> OpenRB D6
- CH1 steering -> physical B
- CH2 throttle -> physical A
- CH5 mode/manual-auto

If the summary reports `reason=PPM_INPUT_ABSENT`, all PPM channels are zero or
missing. Check station/controller power, receiver binding, PPM output mode, and
the D6 signal wire. GPS, IMU, path packages, and serial station frames are not
required for this mode.

## 5. Deprecated Serial Station-Frame Monitor

```bash
bash scripts/run_physical_path_planner.sh station-hw-diagnose \
  --out-dir outputs/physical_path_planning/station_hw_diagnose
```

`station-hw-diagnose` is no-motion and reports whether serial station hardware
frames, deadman, and estop are arriving. `station-hw-manual` is deprecated for
the current hardware because the working manual path is PPM on D6.

If station frames arrive but never parse, the result is
`WRONG_STATION_FRAME_PARSER`. Inspect `raw_station_frames.txt` and
`raw_station_frames_hex.txt` in the output directory; this means the station
machine is transmitting a protocol the current rover parser does not match.

## 6. USB Pulse Test

```bash
bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --out-dir outputs/physical_path_planning/usb_pulse_test
```

This does not use RC receiver input for command generation. The laptop sends
bounded physical A/B commands over USB after Enter and asks for observed motion:

- forward: `A=+0.30`, `B=0.00`, `800 ms`
- backward: `A=-0.08`, `B=0.00`, `300 ms`
- left: `A=0.00`, `B=+0.26`, `700 ms`
- right: `A=0.00`, `B=-0.08`, `250 ms`

To inspect the exact USB pulse commands without serial or upload:

```bash
bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --print-command true \
  --out-dir outputs/physical_path_planning/usb_pulse_test_print
```

During the live run, the console should show concise operator status only:
heartbeat ready, command sent, ACK seen, ACTIVE seen, STOP seen, final zero, and
observed motion. Raw heartbeat and debug lines are saved to `raw_usbdbg.log`; use
`--verbose-raw true` only when debugging serial telemetry.

Use this only as laptop USB motor validation. It is not autonomous path planning,
not RC manual passthrough, and not physical station hardware control. If
`manual-rc` reports `RC_INPUT_ABSENT` but `usb-pulse-test` passes, the motor path
works and the remaining issue is the RC receiver input path.

## 7. Interactive Motion Tuning

Run this after `usb-pulse-test` confirms that bounded A/B commands physically
move the rover.

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

For each trial, press Enter, observe the rover, then answer:

```text
good weak strong too_short too_long left right none retry approve abort
```

For turns, IMU yaw delta is recorded automatically. Enter `approve` to save the
candidate to `outputs/physical_path_planning/calibration/motion_calibration.json`.
No manually measured observed distance is required.

## 8. Guarded Pulse Readiness

```bash
bash scripts/run_physical_path_planner.sh guarded-pulse-ready \
  --out-dir outputs/physical_path_planning/guarded_pulse_ready
```

This uploads/checks IMU-enabled guarded pulse firmware and confirms the guarded
pulse heartbeat, BMI160 yaw telemetry, RC OK, and neutral sticks. It is still not
full path following.

## 9. Turn Angle Calibration

```bash
bash scripts/run_physical_path_planner.sh calibrate-turn \
  --direction left --b-cmd 0.22 --pulse-ms 1200 \
  --target-angle-deg 90 --angle-tolerance-deg 10 \
  --out-dir outputs/physical_path_planning/calibration/left_022_1200
```

This uses guarded pulse calibration with IMU yaw comparison.

## 10. Preview

```bash
bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
  --workspace-width-m 1.2 --step-spacing-m 0.25 \
  --out-dir outputs/physical_path_planning/preview_relative_enu
```

Preview generates the rectangle coverage plan and images without motor output.
A-B is the diagonal of the workspace rectangle. If `--start-lat` and
`--start-lon` are omitted, preview waits up to 300 seconds for current rover GPS
from OpenRB telemetry and falls back to a fresh cached GPS coordinate. If neither
is available, the summary reports `reason=NO_USABLE_START_GPS`; wait outdoors
longer for GPS or pass explicit start coordinates.

## 11. Execute A Reviewed Plan

```bash
bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --out-dir outputs/physical_path_planning/execute_preview_relative_enu
```

Execution uses guarded pulse commands only. Abort conditions remain serial
disconnect, `REJECT`, `RC_INVALID`, missing ACK/STOP, nonzero final commands after
STOP, or output still active after STOP.

Approved `motion_calibration.json` entries are loaded automatically. If approved
90-degree turn commands exist, connectors use them before falling back to
repeated small turn pulses. Straight coverage segments use continuous USB live
drive updates by default, with approved forward/backward A/B/ms as the base. GPS
degradation is not fatal under the default continue policy when IMU heading hold
can continue.

Every output must keep:

```text
ready_for_full_path_following=false
```

Every command writes:

```text
<out-dir>/summary.md
<out-dir>/summary.json
```

## 12. Closed-Loop Path Following (5 m straight field experiment)

This is the GPS/IMU closed-loop experiment. The default
`--path-control-mode gps_imu_closed_loop` actively steers the B axis from the
IMU heading error and the GPS cross-track error while driving the calibrated
forward/backward A command; `--path-control-mode open_loop_chunks` is kept only
for comparison and does not steer.

1. GPS/IMU readiness:

   ```bash
   bash scripts/run_physical_path_planner.sh gps-wait \
     --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
     --out-dir outputs/physical_path_planning/gps_wait_5m
   ```

2. Preview a 5 m relative path (writes the fully resolved field config):

   ```bash
   bash scripts/run_physical_path_planner.sh preview \
     --goal-mode relative_enu --goal-east-m 5.0 --goal-north-m 0.0 \
     --workspace-width-m 1.5 --step-spacing-m 0.30 \
     --path-shape diagonal_rectangle_serpentine --print-field-config true \
     --out-dir outputs/physical_path_planning/preview_5m_relative_enu
   ```

3. Confirm the resolved field config (A is local `(0,0)`, B is `(5.0, 0.0)`):

   ```bash
   cat outputs/physical_path_planning/preview_5m_relative_enu/field_config_resolved.json
   cat outputs/physical_path_planning/preview_5m_relative_enu/summary.md
   ```

4. Execute with real closed-loop correction:

   ```bash
   bash scripts/run_physical_path_planner.sh execute-plan \
     --plan-dir outputs/physical_path_planning/preview_5m_relative_enu \
     --path-control-mode gps_imu_closed_loop \
     --live-chunk-ms 700 --max-segment-chunks 30 \
     --gps-degradation-policy continue --gps-reanchor true \
     --imu-heading-hold true --cross-track-correction true \
     --k-heading 0.006 --k-cross-track 0.20 --max-correction-b 0.08 \
     --out-dir outputs/physical_path_planning/execute_5m_gps_imu_closed_loop
   ```

   `--plan-dir` loads the planned segments, the approved
   `motion_calibration.json`, and the start coordinate from the plan directory,
   so execution does not require a fresh start GPS fix.

5. Inspect correction evidence:

   ```bash
   cat  outputs/physical_path_planning/execute_5m_gps_imu_closed_loop/summary.md
   head -40 outputs/physical_path_planning/execute_5m_gps_imu_closed_loop/closed_loop_trace.csv
   tail -40 outputs/physical_path_planning/execute_5m_gps_imu_closed_loop/closed_loop_trace.csv
   ```

Each chunk prints one line:

```text
seg=<i> chunk=<j> mode=gps_imu_closed_loop gps=OK|DEGRADED imu=OK|NA heading_err=<deg> cte=<m> progress=<m> remaining=<m> A=<a> B=<b> correction=<heading|cross_track|both|dead_reckon>
```

Reading the evidence:

- `closed_loop_correction_applied=true` confirms a nonzero steering correction
  reached B at least once; `false` means B stayed `0` for every chunk (open-loop
  behavior). `imu_heading_used_count` and `cross_track_correction_used_count`
  count the chunks that used each correction.
- The IMU heading reference is captured per lane from the first heartbeat yaw, so
  `--start-yaw-deg` is optional; pass it only to override the first lane.
- When GPS degrades the run continues on IMU heading hold plus calibrated
  dead-reckoned progress (`gps_degraded=true`, `correction=dead_reckon`); when GPS
  recovers the pose is re-anchored (`gps_reanchored=true`).
- `closed_loop_trace.csv` carries the full per-chunk record (pose, segment
  endpoints, target/IMU heading, heading and cross-track error, the
  `b_heading_correction`/`b_cross_track_correction` components, the final A/B
  command, and the ACK/ACTIVE/STOP handshake flags).

RC/PPM input is ignored for Mac USB supervised drive, so `rc_ok=false` does not
abort execution. Each live chunk is split so its duration never exceeds the
firmware maximum; a firmware `USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX` reject is
classified as `HOST_SENT_DURATION_OVER_MAX` with the offending duration.

## 13. AUTO-Switch Relative Run (3 m east / 4 m north diagonal)

`auto-relative-run` waits for GPS, then watches the physical PPM mode switch and
starts the same closed-loop execution only when AUTO is selected. The first field
target is the 5 m diagonal `east=3 m, north=4 m`.

Preview only (no motion):

```bash
bash scripts/run_physical_path_planner.sh auto-relative-preview \
  --goal-east-m 3.0 --goal-north-m 4.0 \
  --workspace-width-m 1.5 --step-spacing-m 0.30 \
  --out-dir outputs/physical_path_planning/auto_relative_3x4m_preview
```

Wait for AUTO, then run closed-loop:

```bash
bash scripts/run_physical_path_planner.sh auto-relative-run \
  --goal-east-m 3.0 --goal-north-m 4.0 \
  --workspace-width-m 1.5 --step-spacing-m 0.30 \
  --path-shape diagonal_rectangle_serpentine \
  --path-control-mode gps_imu_closed_loop \
  --live-chunk-ms 700 --max-segment-chunks 30 \
  --gps-degradation-policy continue --gps-reanchor true \
  --imu-heading-hold true --cross-track-correction true \
  --k-heading 0.006 --k-cross-track 0.20 --max-correction-b 0.08 \
  --out-dir outputs/physical_path_planning/auto_relative_3x4m
```

Behavior:

- Waits for GPS readiness (default timeout 300 s, min sats 5, max HDOP 2.5) and
  saves the current fix as start A. While waiting it prints a status line every
  second with `mode_switch`, `mode_us`, `rc_ok`, `gps_ready`, `gps_sats`,
  `gps_hdop`, `current_lat`, `current_lon`, `imu_present`,
  `imu_relative_yaw_deg`, and `waiting_for=GPS` then `waiting_for=AUTO_SWITCH`.
- `MANUAL` keeps motors stopped and never starts the path. `AUTO` generates the
  preview from the current GPS start (writes `field_config_resolved.json` and the
  preview image) and starts closed-loop execution.
- If the PPM mode channel is absent, pass `--allow-keyboard-start true` to start
  on Enter instead of the switch.
- Flipping the switch back to `MANUAL` during execution stops safely with
  `stop_reason=USER_SWITCHED_TO_MANUAL` and a final motor zero.
- `--plan-dir` reuses an existing `auto-relative-preview` plan and skips the fresh
  GPS wait.

The run writes `summary.md`, `summary.json`, `field_config_resolved.json`, the
preview image, `closed_loop_trace.csv`, `planned_vs_actual.csv`, and
`raw_usbdbg.log`. Every summary keeps `ready_for_full_path_following=false`.

If alignment is requested (default `--initial-heading-align gps_probe` for
`auto-relative-run`), it runs after the AUTO/keyboard start and before path
execution; see section 14 for the alignment and recalibration details.

## 14. Initial Heading Alignment

The rover starts wherever it is pointing. The IMU reports only a relative yaw, not
an absolute compass heading, so alignment derives the absolute heading from a
short GPS displacement probe and then uses the IMU only as turn feedback.

```bash
bash scripts/run_physical_path_planner.sh align-heading \
  --plan-dir outputs/physical_path_planning/preview_5m_relative_enu \
  --strategy gps_probe \
  --probe-a 0.25 --probe-duration-s 1.0 --min-probe-distance-m 0.30 \
  --heading-tolerance-deg 8 --turn-b-left 0.24 --turn-b-right -0.12 \
  --max-turn-duration-s 3.0 \
  --out-dir outputs/physical_path_planning/preview_5m_relative_enu/alignment
```

Strategies:

- `gps_probe` (automatic): read GPS, drive one short bounded forward probe, read
  GPS again. With at least `--min-probe-distance-m` of displacement it estimates
  the current ENU heading from the GPS displacement, compares it to the first
  segment heading, and turns left (`B>0`) or right (`B<0`) by the shortest error.
  The turn stops on the IMU yaw-delta reaching the heading error (preferred over
  any calibration estimate). Displacement below the threshold reports
  `reason=PROBE_GPS_DISPLACEMENT_TOO_SMALL` (increase `--probe-duration-s` or move
  to open ground).
- `user_confirmed`: point the rover toward the A->B direction by hand, press
  Enter, and the current IMU yaw is captured as the first lane reference. No probe
  and no GPS displacement are required; never aborts.
- `skip`: no alignment, no serial.

The summary records `strategy`, `target_heading_deg`, `probe_start_lat/lon`,
`probe_end_lat/lon`, `probe_distance_m`, `estimated_current_heading_deg`,
`initial_heading_error_deg`, `final_heading_error_deg`, `turn_direction`,
`turn_b_cmd`, `turn_duration_ms`, `alignment_success`, and
`ready_for_execute_plan`, plus `align_heading_trace.csv` and `raw_usbdbg.log`.

A/B mapping is unchanged: `A>0` forward, `A<0` backward, `B>0` left, `B<0` right.

`execute-plan`, `run`, and `auto-relative-run` accept the same alignment inline
via `--initial-heading-align none|gps_probe|user_confirmed` (plus
`--align-heading-tolerance-deg`, `--align-probe-a`, `--align-probe-duration-s`,
`--align-min-probe-distance-m`). When alignment succeeds, its aligned IMU yaw
becomes the first lane reference; per-lane references are still recaptured after
each connector turn. A `gps_probe` failure aborts before any path motion;
`user_confirmed` and `none` never abort on alignment. The default is `none` for
`execute-plan`/`run` and `gps_probe` for `auto-relative-run`.

## 15. Full Recalibration

Recalibration overwrites the approved per-primitive entries. Back up the existing
file first so the previous calibration is preserved:

```bash
bash scripts/run_physical_path_planner.sh reset-motion-calibration \
  --out-dir outputs/physical_path_planning/reset_motion_calibration
```

This copies any existing
`outputs/physical_path_planning/calibration/motion_calibration.json` to a
timestamped `motion_calibration.backup_<stamp>.json` sibling, then removes the
original so `tune-motion` starts clean. (`tune-motion --reset-calibration true`
performs the same backup-then-clear at the start of a single tuning session.)

Then recalibrate each primitive; entering `approve` overwrites that primitive's
entry in the fresh file:

```bash
bash scripts/run_physical_path_planner.sh tune-motion --primitive forward \
  --out-dir outputs/physical_path_planning/tune_forward
bash scripts/run_physical_path_planner.sh tune-motion --primitive backward \
  --out-dir outputs/physical_path_planning/tune_backward
bash scripts/run_physical_path_planner.sh tune-motion --primitive turn-left-90 \
  --out-dir outputs/physical_path_planning/tune_turn_left_90
bash scripts/run_physical_path_planner.sh tune-motion --primitive turn-right-90 \
  --out-dir outputs/physical_path_planning/tune_turn_right_90
```

## 16. Recalibrate + Aligned 3 m east / 4 m north Run

The full field sequence for the 3-4-5 diagonal, from a clean recalibration through
an aligned closed-loop run:

1. Recalibrate everything (section 15): `reset-motion-calibration`, then
   `tune-motion` for forward, backward, turn-left-90, turn-right-90.

2. GPS readiness:

   ```bash
   bash scripts/run_physical_path_planner.sh gps-wait \
     --timeout-s 300 --min-sats 5 --max-hdop 2.5 \
     --out-dir outputs/physical_path_planning/gps_wait_3x4m
   ```

3. Preview the 3 m east / 4 m north diagonal (writes the resolved field config):

   ```bash
   bash scripts/run_physical_path_planner.sh preview \
     --goal-mode relative_enu --goal-east-m 3.0 --goal-north-m 4.0 \
     --workspace-width-m 1.5 --step-spacing-m 0.30 \
     --path-shape diagonal_rectangle_serpentine --print-field-config true \
     --out-dir outputs/physical_path_planning/preview_3x4m
   ```

4. Align to the first lane heading (section 14):

   ```bash
   bash scripts/run_physical_path_planner.sh align-heading \
     --plan-dir outputs/physical_path_planning/preview_3x4m \
     --strategy gps_probe \
     --out-dir outputs/physical_path_planning/preview_3x4m/alignment
   ```

5. Execute closed-loop with inline alignment:

   ```bash
   bash scripts/run_physical_path_planner.sh execute-plan \
     --plan-dir outputs/physical_path_planning/preview_3x4m \
     --initial-heading-align gps_probe \
     --path-control-mode gps_imu_closed_loop \
     --live-chunk-ms 500 --max-segment-chunks 30 \
     --gps-degradation-policy continue --gps-reanchor true \
     --imu-heading-hold true --cross-track-correction true \
     --k-heading 0.006 --k-cross-track 0.20 --max-correction-b 0.08 \
     --out-dir outputs/physical_path_planning/execute_3x4m
   ```

   To skip alignment and keep the prior behavior, pass
   `--initial-heading-align none`. The inline alignment in step 5 makes the
   standalone step 4 optional; run step 4 first only to inspect the alignment
   result before committing to execution.

Every summary in this sequence keeps `ready_for_full_path_following=false`.
