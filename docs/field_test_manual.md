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

## 2. Manual RC Recovery

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

## 3. Physical Station Hardware Manual

```bash
bash scripts/run_physical_path_planner.sh station-hw-diagnose \
  --out-dir outputs/physical_path_planning/station_hw_diagnose

bash scripts/run_physical_path_planner.sh station-hw-manual \
  --out-dir outputs/physical_path_planning/station_hw_manual
```

`station-hw-diagnose` is no-motion and reports whether station hardware frames,
deadman, and estop are arriving. `station-hw-manual` is the separate physical
station controller path; it maps station throttle to physical A and station
steering to physical B.

## 4. USB Pulse Test

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

## 5. Guarded Pulse Readiness

```bash
bash scripts/run_physical_path_planner.sh guarded-pulse-ready \
  --out-dir outputs/physical_path_planning/guarded_pulse_ready
```

This uploads/checks IMU-enabled guarded pulse firmware and confirms the guarded
pulse heartbeat, BMI160 yaw telemetry, RC OK, and neutral sticks. It is still not
full path following.

## 5. Turn Angle Calibration

```bash
bash scripts/run_physical_path_planner.sh calibrate-turn \
  --direction left --b-cmd 0.22 --pulse-ms 1200 \
  --target-angle-deg 90 --angle-tolerance-deg 10 \
  --out-dir outputs/physical_path_planning/calibration/left_022_1200
```

This uses guarded pulse calibration with IMU yaw comparison.

## 6. Preview

```bash
bash scripts/run_physical_path_planner.sh preview \
  --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
  --workspace-width-m 1.2 --step-spacing-m 0.25 \
  --out-dir outputs/physical_path_planning/preview_relative_enu
```

Preview generates the rectangle coverage plan and images without motor output.
A-B is the diagonal of the workspace rectangle.

## 7. Execute A Reviewed Plan

```bash
bash scripts/run_physical_path_planner.sh execute-plan \
  --plan-dir outputs/physical_path_planning/preview_relative_enu \
  --out-dir outputs/physical_path_planning/execute_preview_relative_enu
```

Execution uses guarded pulse commands only. Abort conditions remain serial
disconnect, `REJECT`, `RC_INVALID`, missing ACK/STOP, nonzero final commands after
STOP, or output still active after STOP.

Every output must keep:

```text
ready_for_full_path_following=false
```

Every command writes:

```text
<out-dir>/summary.md
<out-dir>/summary.json
```
