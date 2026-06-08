# Manual Control Regression Trace

This trace records the evidence for the old PPM manual-control path that
physically moved the rover, compares it with the current manual-control wrapper,
and documents the minimal restore.

## Exact Working Run

Working log file:

```text
outputs/logs/manual_forward_neg_turn_pos_20260530_141846.log
```

That log is local and ignored, not tracked in git. Its USBDBG rows show:

- `mode=MANUAL`
- `rc_ok=true`
- `auto_sw=false`
- `control_source=RC_MANUAL`
- `manual_forward_sign=-1.0`
- `manual_turn_sign=1.0`
- `old_angle_remap_active=false`
- `physical_a_role=throttle`
- `physical_b_role=turn`
- `wheel_to_physical_mapping=diff_to_throttle_turn`
- nonzero `final_left_cmd`, `final_right_cmd`, `physical_a_cmd`, and
  `physical_b_cmd`

The exact command in shell history that generated the matching filename pattern
was:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && \
PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && \
echo "PORT=$PORT" && \
mkdir -p outputs/logs && \
arduino-cli compile \
  --fqbn OpenRB-150:samd:OpenRB-150 \
  --build-path /private/tmp/openrb-manual-forward-neg-turn-pos \
  --build-property "compiler.cpp.extra_flags=-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0" \
  firmware/openrb_robot_controller && \
arduino-cli upload \
  -v \
  -p "$PORT" \
  --fqbn OpenRB-150:samd:OpenRB-150 \
  --build-path /private/tmp/openrb-manual-forward-neg-turn-pos \
  firmware/openrb_robot_controller && \
sleep 2 && \
PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && \
echo "MONITOR PORT=$PORT" && \
arduino-cli monitor \
  -p "$PORT" \
  --config baudrate=115200 | tee outputs/logs/manual_forward_neg_turn_pos_$(date +%Y%m%d_%H%M%S).log
```

Exact old firmware flags:

```text
-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0
```

Important absence: the working run did not compile with
`-DMANUAL_CONTROL_PPM=1`, `-DMANUAL_RC_RECOVERY=1`, `-DIMU_ENABLE=1`, or
`-DIMU_YAW_DIAG=1`.

## Git History Evidence

Commands used:

```text
git log --all --oneline -- scripts/upload_manual_rc_recovery_firmware.sh scripts/run_manual_rc_passthrough_validation.sh tools/manual_rc_passthrough_validation.py firmware/openrb_robot_controller/openrb_robot_controller.ino
git log --all --oneline -S "manual_forward_neg_turn_pos" -- .
git log --all --oneline -S "rc-arcade-manual-fwdneg" -- .
git log --all --oneline -S "CONTROL_SOURCE_RC_MANUAL" -- firmware/openrb_robot_controller/openrb_robot_controller.ino
git log --all --oneline -S "MANUAL_FORWARD_SIGN" -- firmware/openrb_robot_controller/openrb_robot_controller.ino
git log --all --oneline -S "ppm_age_ms" -- firmware/openrb_robot_controller/openrb_robot_controller.ino
```

`manual_forward_neg_turn_pos` was found in shell history and the local ignored
log, not in git.

Key commits:

| Commit | Link | Evidence |
|---|---|---|
| `64583a6627414395feccac510dc3e7e46c04eaf1` | https://github.com/seonukkim/gps_hc12_robot/commit/64583a6627414395feccac510dc3e7e46c04eaf1 | Contains `rc-arcade-manual-fwdneg`, RISING PPM ISR, old arcade manual path, and USBDBG fields seen in the working log. |
| `a24557bedd1f48069eedf34e38e5736ea1a43c54` | https://github.com/seonukkim/gps_hc12_robot/commit/a24557bedd1f48069eedf34e38e5736ea1a43c54 | Introduces the `MANUAL_FORWARD_SIGN` / `MANUAL_TURN_SIGN` arcade mixer fix. |
| `75706be738c1c89be36ced224872cb7b766ae3b2` | https://github.com/seonukkim/gps_hc12_robot/commit/75706be738c1c89be36ced224872cb7b766ae3b2 | Introduces USB debug status including `ppm_age_ms`. |
| `7c58ce3ebfa71c9ea7569db2295862bb98eb3de0` | https://github.com/seonukkim/gps_hc12_robot/commit/7c58ce3ebfa71c9ea7569db2295862bb98eb3de0 | Later PPM manual-control workflow restore. |
| `0c4074d48507d48bd8771072d24255ba185d2751` | https://github.com/seonukkim/gps_hc12_robot/commit/0c4074d48507d48bd8771072d24255ba185d2751 | Later manual-control telemetry restore. |
| `cf23d09eaaff906b9bf24d7e3e3bfa7cf321ea57` | https://github.com/seonukkim/gps_hc12_robot/commit/cf23d09eaaff906b9bf24d7e3e3bfa7cf321ea57 | Moves older stage scripts/tools into `legacy/`. |
| `77460ec2d1fea9a400889ae9e27d9cc35d3baa55` | https://github.com/seonukkim/gps_hc12_robot/commit/77460ec2d1fea9a400889ae9e27d9cc35d3baa55 | Current pre-restore CLI default compiled the newer `MANUAL_CONTROL_PPM=1` profile. |

File links:

| File | Old/current links |
|---|---|
| `firmware/openrb_robot_controller/openrb_robot_controller.ino` | old: https://github.com/seonukkim/gps_hc12_robot/blob/64583a6627414395feccac510dc3e7e46c04eaf1/firmware/openrb_robot_controller/openrb_robot_controller.ino current: https://github.com/seonukkim/gps_hc12_robot/blob/main/firmware/openrb_robot_controller/openrb_robot_controller.ino |
| `tools/physical_path_planning/cli.py` | current: https://github.com/seonukkim/gps_hc12_robot/blob/main/tools/physical_path_planning/cli.py pre-restore: https://github.com/seonukkim/gps_hc12_robot/blob/77460ec2d1fea9a400889ae9e27d9cc35d3baa55/tools/physical_path_planning/cli.py |
| `scripts/run_physical_path_planner.sh` | current: https://github.com/seonukkim/gps_hc12_robot/blob/main/scripts/run_physical_path_planner.sh |
| `legacy/stage_scripts/upload_manual_rc_recovery_firmware.sh` | current: https://github.com/seonukkim/gps_hc12_robot/blob/main/legacy/stage_scripts/upload_manual_rc_recovery_firmware.sh at move commit: https://github.com/seonukkim/gps_hc12_robot/blob/cf23d09eaaff906b9bf24d7e3e3bfa7cf321ea57/legacy/stage_scripts/upload_manual_rc_recovery_firmware.sh |
| `legacy/stage_scripts/run_manual_rc_passthrough_validation.sh` | current: https://github.com/seonukkim/gps_hc12_robot/blob/main/legacy/stage_scripts/run_manual_rc_passthrough_validation.sh at move commit: https://github.com/seonukkim/gps_hc12_robot/blob/cf23d09eaaff906b9bf24d7e3e3bfa7cf321ea57/legacy/stage_scripts/run_manual_rc_passthrough_validation.sh |
| `legacy/stage_tools/manual_rc_passthrough_validation.py` | current: https://github.com/seonukkim/gps_hc12_robot/blob/main/legacy/stage_tools/manual_rc_passthrough_validation.py at move commit: https://github.com/seonukkim/gps_hc12_robot/blob/cf23d09eaaff906b9bf24d7e3e3bfa7cf321ea57/legacy/stage_tools/manual_rc_passthrough_validation.py |

## Old Versus Current Manual Path

| Item | Old working run | Current firmware | Pre-restore CLI default | Restored default |
|---|---|---|---|---|
| PPM pin | OpenRB D6 | D6 | D6 | D6 |
| Interrupt edge | `RISING` | `RISING` | `RISING` in firmware | `RISING` |
| Sync pulse threshold | `>3000 us` literal | `PPM_SYNC_US=3000` | same firmware | same firmware |
| Channel count | 8 | 8 | 8 | 8 |
| CH1 steering | index 0 | index 0 | index 0 | index 0 |
| CH2 throttle | index 1 | index 1 | index 1 | index 1 |
| CH5 mode | index 4 | `MODE_CHANNEL_INDEX_VALUE=4` by default | `-DMODE_CHANNEL_INDEX=4` | `-DMODE_CHANNEL_INDEX=4` |
| Mode threshold | `>1600 us` AUTO | `>1600 us` AUTO | same firmware | same firmware |
| `rc_ok` logic | recent frame plus valid steering/throttle/mode pulse widths | same core logic, with extra counters | same firmware | same firmware |
| Failsafe | invalid/stale RC forces `STOP`/`FAILSAFE` | same | same | same |
| `MANUAL_FORWARD_SIGN` | `-1` | compile-time configurable | `-1` | `-1` |
| `MANUAL_TURN_SIGN` | `1` | compile-time configurable | `1` | `1` |
| Old angle remap | disabled in final manual path | disabled in final manual path | disabled | disabled |
| Motor output path | `RC_MANUAL -> applyManualOverride -> applyDriveCommand` | same path exists | special `MANUAL_CONTROL_PPM` branch selected | old integrated branch selected |
| Control source | `CONTROL_SOURCE_RC_MANUAL` when RC valid and CH5 manual | same | same, but inside special branch | same old integrated branch |
| Physical A/B conversion | `A=(L+R)/2`, `B=(R-L)/2` | same | same | same |
| HC-12/GPS serial profile | default integrated: HC-12 enabled, GPS `Serial3` | depends on compile flags | `MANUAL_CONTROL_PPM=1` disables HC-12 and moves GPS to `Serial2` | exact old integrated flags |
| IMU profile | no IMU flags in old run | available when compiled | `IMU_ENABLE=1 IMU_YAW_DIAG=1` | no IMU flags, to match old run |

## Likely Regression Point

The motor mapping and PPM ISR in current firmware were already aligned with the
old moving path. The regression is most likely in the wrapper profile used by
`manual-control`: before this restore, the normal CLI command compiled
`MANUAL_CONTROL_PPM=1` plus IMU/GPS/path-disabling flags. That is not the same
as the exact old shell-history command that produced the successful movement
log.

The latest failure log also showed all selected PPM channels at zero with
`ppm_frame_count=0` and `ppm_last_channel_count=0`. That should not be reported
as `MODE_CHANNEL_MISSING`; no usable PPM frame was captured yet. The summary
classification now reports that as PPM absent/no-frame unless the firmware
actually captures enough channels to know that the CH5 mode slot is missing.

## Minimal Restore Implemented

`manual-control` now has named compile profiles:

- `old-working-ppm` (default): exact moving-run flags:
  `-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0`
  plus the explicit `MODE_CHANNEL_INDEX=4` override used by the current docs.
- `full-telemetry-ppm`: preserves the newer `MANUAL_CONTROL_PPM=1`,
  GPS-Serial2, and IMU diagnostic compile flags for sensor-heavy diagnosis.

Default command:

```bash
bash scripts/run_physical_path_planner.sh manual-control --profile old-working-ppm
```

The restore does not change pin mappings, channel mappings, motor mapping,
STOP/failsafe priority, calibrated A/B behavior, or path-planning behavior.
