# Outdoor Safety Validation Workflow

This document keeps the outdoor bring-up workflow focused on the current unified
field entrypoint:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

Do not use older numbered experiment scripts for field work. Use the functional
modes in the unified launcher.

## No-Motion Checks

Use `diagnose` for read-only telemetry:

```bash
bash scripts/run_physical_path_planner.sh diagnose \
  --out-dir outputs/physical_path_planning/diagnose
```

This sends no motor commands. Every run writes:

```text
<out-dir>/summary.md
<out-dir>/summary.json
```

## Receiver Passthrough

Use `manual-rc` only to validate receiver stick passthrough:

```bash
bash scripts/run_physical_path_planner.sh manual-rc \
  --out-dir outputs/physical_path_planning/manual_rc
```

If the summary reports `RC_INPUT_ABSENT`, the receiver signal is not reaching the
OpenRB. That is a receiver wiring, binding, input mode, or channel mapping issue.
It does not prove a motor calibration problem.

## Physical Station Hardware Manual

Use `station-hw-diagnose` first when the separate station controller hardware
should drive the rover:

```bash
bash scripts/run_physical_path_planner.sh station-hw-diagnose \
  --out-dir outputs/physical_path_planning/station_hw_diagnose
```

Then use `station-hw-manual` for the live physical station hardware manual path:

```bash
bash scripts/run_physical_path_planner.sh station-hw-manual \
  --out-dir outputs/physical_path_planning/station_hw_manual
```

This path does not use RC receiver passthrough, GPS, IMU, path packages, or laptop
USB pulse commands as the control source. Station throttle maps to physical A and
station steering maps to physical B.

## USB Pulse Test

Use `usb-pulse-test` when the laptop should send bounded physical A/B commands
over USB:

```bash
bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --out-dir outputs/physical_path_planning/usb_pulse_test
```

Print the exact commands without serial, firmware upload, or motion:

```bash
bash scripts/run_physical_path_planner.sh usb-pulse-test \
  --print-command true \
  --out-dir outputs/physical_path_planning/usb_pulse_test_print
```

The calibrated command map is:

```text
FORWARD:
A=+0.300 B=+0.000 ms=800

BACKWARD:
A=-0.080 B=+0.000 ms=300

LEFT:
A=+0.000 B=+0.260 ms=700

RIGHT:
A=+0.000 B=-0.080 ms=250
```

`usb-pulse-test` does not require RC receiver input, station hardware input, GPS,
IMU, path packages, or autonomous path planning. It is bounded laptop USB pulse
movement only.

## Safety Boundary

- Do not enable full path following.
- Do not enable HC-12 path control.
- Do not send autonomous startup motion.
- Keep `ready_for_full_path_following=false` in every summary.
- Abort physical testing if final motor commands do not return to zero.
