# Outdoor No-Motion Validation Workflow

This workflow validates GPS, BMI160, RC/manual state, HC-12 status fields, and
motor safety with the integrated no-motion firmware. It does not approve physical
driving.

## Safety Boundary

- Do not enable `PHYSICAL_PATH_FOLLOWING_ENABLE`.
- Do not enable `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT`.
- Do not run guarded crawl.
- Do not run motor pulse tests.
- Do not upload or monitor while another tool is already using the OpenRB serial
  port.
- Keep this workflow as no-motion sensor and signal diagnosis only.

Expected motor safety in every integrated no-motion log:

```text
physical_block_reason=COMPILE_GATE_OFF
physical_output_active=false
final_left_cmd=0.000
final_right_cmd=0.000
```

## Port Detection

```bash
cd /Users/seonuk/Desktop/project-lab/gps_hc12_robot
arduino-cli board list
PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}')
echo "OPENRB_PORT=$PORT"
uv run python -m serial.tools.list_ports -v
```

PASS:

- OpenRB appears as `/dev/cu.usbmodem...`.
- CP2104 station adapter may appear as `/dev/cu.usbserial-02442CA5`.

FAIL / block:

- No OpenRB port appears.
- Another monitor is already open on the OpenRB port.

## Integrated Baseline Monitor

If the integrated no-motion firmware is not already uploaded, upload it with the
dedicated no-motion script. This script refuses motion-enabling flags and compiles
with `PHYSICAL_PATH_FOLLOWING_ENABLE=0` and
`PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`.

```bash
cd /Users/seonuk/Desktop/project-lab/gps_hc12_robot
scripts/run_integrated_dryrun_no_motion.sh -p /dev/cu.usbmodem12101
```

If the OpenRB port is different, replace `/dev/cu.usbmodem12101` with the port
printed by `arduino-cli board list`.

For monitor-only recovery/watch mode, use the command below only if the
integrated no-motion firmware is already uploaded.

```bash
cd /Users/seonuk/Desktop/project-lab/gps_hc12_robot
mkdir -p outputs/logs
PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}')
arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/outdoor_integrated_watch_$(date +%Y%m%d_%H%M%S).log
```

After stopping the monitor with Ctrl-C:

```bash
LOG=$(ls -t outputs/logs/outdoor_integrated_watch_*.log outputs/logs/integrated_dryrun_hc12_deferred_*.log 2>/dev/null | head -1)
echo "$LOG"
scripts/check_integrated_dryrun_log.sh "$LOG"
grep -E "gps_block_reason|gps_sats|gps_hdop|position_source|imu_type|imu_i2c_addr|imu_chip_id|imu_pmu_normal|imu_data_plausible|imu_heading_ready|hc12_enabled|hc12_port|hc12_rx_count|hc12_tx_count|hc12_parse_ok|rc_ok|mode=|control_source|neutral_ok|physical_block_reason|physical_output_active|final_left_cmd|final_right_cmd" "$LOG" | tail -80
```

PASS:

- GPS outdoors: `gps_block_reason=OK`, `gps_sats>=4`, reasonable `gps_hdop`,
  `position_source=gps`.
- BMI160: `imu_type=BMI160`, `imu_i2c_addr=0x68`, `imu_chip_id=0xD1`,
  `imu_pmu_normal=true`, `imu_data_plausible=true`.
- RC with transmitter on: `rc_ok=true`, `mode=MANUAL`, `control_source=RC_MANUAL`,
  sticks neutral, and `neutral_ok=true` where printed.
- HC-12 configured: `hc12_enabled=true`, `hc12_port=Serial3` or
  `hc12_pf_port=Serial3`.
- Motor safety: compile gate off and final outputs zero.

FAIL / block:

- `physical_output_active=true`.
- nonzero final output in no-motion mode.
- `gps_block_reason=NO_LOCATION` outdoors after open-sky wait.
- BMI160 fields missing or implausible.
- RC transmitter on but `rc_ok=false` persists.

Indoor note: `gps_block_reason=NO_LOCATION` is expected indoors and is not a GPS
wiring regression.

## Hand-Carry Heading Validation

Use the same monitor command as above and perform this sequence while logging:

1. 20 s stationary.
2. Hand-carry straight for 20 to 30 m.
3. 10 s stationary.
4. Optional: return 20 to 30 m on the same line.

Then inspect:

```bash
LOG=$(ls -t outputs/logs/outdoor_integrated_watch_*.log outputs/logs/integrated_dryrun_hc12_deferred_*.log 2>/dev/null | head -1)
grep -E "course_displacement_m|estimated_course_deg|heading_ready|heading_source|heading_agreement_diag|heading_agreement_error_deg|imu_relative_yaw_deg|gps_motion_ready|gps_block_reason" "$LOG" | tail -120
uv run python tools/analyze_heading_readiness_log.py "$LOG"
scripts/check_outdoor_heading_log.sh "$LOG"
```

PASS:

- `course_displacement_m` increases during a segment, or the analyzer reports
  enough `bbox_diag_m` / `cumulative_path_m` movement.
- `estimated_course_deg` becomes finite after enough movement.
- `gps_course_last_accepted_valid=true` appears after a course segment is
  accepted.
- `gps_course_output_deg` is non-`NA`, or the analyzer reports
  `HEADING_COURSE_LATCH_AVAILABLE`.
- `heading_agreement_diag` leaves `WAITING_GPS_COURSE` when both GPS course and
  BMI160 yaw diagnostic are available.
- `heading_agreement_error_deg` is finite and not obviously unstable during the
  straight carry.

FAIL / block:

- `heading_ready=false` for the entire carry despite good GPS.
- `course_displacement_m` never increases.
- BMI160 becomes not present or implausible.

This validates heading diagnostics only. It does not approve steering execution.

`course_displacement_m` is segment-local. It can reset after the firmware
accepts a GPS course and re-seeds the course anchor. Do not use only that field
to decide whether the hand-carry moved far enough. Use
`gps_course_total_displacement_m_since_boot` plus the analyzer's
`bbox_diag_m` / `cumulative_path_m` movement summary for total movement.

If a log shows good GPS and tens of meters of `current_lat/current_lon` movement
but `gps_course_deg=NA`, run the analyzer above. A common no-motion diagnostic
case is that the firmware computes a short course segment, immediately re-seeds
the GPS course anchor, and the periodic USBDBG sample then sees
`path_following_block_reason=NO_HEADING`. Diagnostic USBDBG fields such as
`gps_course_anchor_reset_reason`, `gps_course_anchor_reset_count`,
`gps_course_total_displacement_m_since_boot`, and `gps_course_estimated_deg`
make that transient behavior visible without changing any motor-output gate.

The latest diagnostic firmware also prints retained accepted-course fields:

- `gps_course_candidate_deg`: course computed in the current loop, if any.
- `gps_course_last_accepted_valid`
- `gps_course_last_accepted_deg`
- `gps_course_last_accepted_age_ms`
- `gps_course_last_accepted_displacement_m`
- `gps_course_last_accepted_anchor_reset_count`
- `gps_course_output_valid`
- `gps_course_output_deg`
- `gps_course_output_age_ms`
- `gps_course_output_block_reason`

These fields are diagnostic-only. They do not satisfy the physical-output gate
and must not be used as approval for motor output.

Analyzer verdicts:

- `MOTOR_SAFETY_FAIL`: abort immediately and inspect compile gates; no more
  outdoor validation.
- `SENSOR_BASELINE_FAIL`: GPS, BMI160, or RC/manual baseline failed; fix that
  before heading validation.
- `SENSOR_BASELINE_PASS`: GPS/IMU/RC/motor safety are acceptable, but the carry
  segment was not long enough for heading.
- `HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT`: movement was enough, but the
  periodic USBDBG log missed or blocked GPS course readiness; use the diagnostic
  firmware fields and repeat hand-carry.
- `HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT`: the GPS-course candidate crossed the
  displacement threshold and `COURSE_ACCEPTED_RESEED` was observed, but the
  diagnostic latch was not exposed as output; inspect
  `gps_course_output_block_reason`.
- `HEADING_COURSE_LATCH_AVAILABLE`: retained GPS course is exposed for no-motion
  diagnostics. Review `heading_agreement_diag` / `heading_agreement_error_deg`,
  then proceed only to offline path-planning preview.
- `HEADING_READY_PASS`: current-loop heading readiness is observed. Physical
  motor enable remains prohibited.

## RC / Manual Check

Before monitoring:

1. Turn transmitter on.
2. Keep throttle neutral.
3. Set AUTO switch OFF.

PASS:

- `rc_ok=true`
- `mode=MANUAL`
- `control_source=RC_MANUAL`
- `neutral_ok=true` where printed
- motor safety fields remain compile-gated off and final outputs zero

FAIL / block:

- `rc_ok=false` / `FAILSAFE` with transmitter on.
- mode switch does not produce expected MANUAL/AUTO fields.

## HC-12 Safe Validation

List serial ports:

```bash
uv run python -m serial.tools.list_ports -v
```

Station adapter stability:

```bash
uv run python tools/hc12_operational_diagnose.py --port /dev/cu.usbserial-02442CA5 --mode stability --duration-s 20 --reconnect --dtr low --rts low
```

Station write-only diagnostic:

```bash
uv run python tools/hc12_operational_diagnose.py --port /dev/cu.usbserial-02442CA5 --mode write-only --duration-s 30 --reconnect --dtr low --rts low
```

Latest Mac result:

```text
port=/dev/cu.usbserial-02442CA5
DTR=low RTS=low rtscts=False dsrdtr=False
tx_count=16
rx_count=0
serial_error_count=0
verdict=STATION_TX_OK_NO_RX
link_status=NO_RX_YET
```

Interpretation: station serial open/write is OK, but RF RX is not proven. This is
`RF_LINK_DEFERRED` / `DEFERRED_TO_UBUNTU_STATION`, not HC-12 RF success and not a
GPS/BMI160 failure.

Optional RF ping probe if the OpenRB side is running a compatible no-motion HC-12
listener:

```bash
uv run python tools/hc12_link_probe.py --port /dev/cu.usbserial-02442CA5 --baud 9600 --duration-s 60 --dtr low --rts low --log-dir outputs/logs
```

Status split:

- `STATION_PORT_FOUND`: CP2104 appears in port list.
- `STATION_SERIAL_OPEN_OK`: station tool opens with DTR/RTS low and no serial
  error.
- `STATION_TX_OK`: write-only mode transmits without serial error.
- `RF_RX_SEEN`: station or OpenRB RX counters increase.
- `HC12_PARSE_OK`: parsed PING/PONG or command frames are observed.

If the Mac proves only station port/open/write but no RF RX, record
`STATION_TX_OK_NO_RX` and `DEFERRED_TO_UBUNTU_STATION`, not failed.

## Offline Side-Tool Path Preview

This is station-side preview only. It generates side-mounted cleaning-tool
footprint coverage first, then derives the chassis centerline support path. The
chassis footprint is a boundary constraint and does not count as cleaned area.
It does not send HC-12 frames, rover commands, or firmware uploads.

For A/B bounded workspace planning, follow
[docs/side_tool_path_planning.md](side_tool_path_planning.md). In the default
`tool_serpentine_ab` mode, A is the top-left tool-center start and B is the
bottom-right tool-center end. The primary route is the continuous
tool/paint-tank center path; the chassis path is derived afterward. Each emitted
primitive row must be `move_forward`, `move_backward`, `rotate_left`, or
`rotate_right`. Contamination and transition-envelope overlays are optional
legacy diagnostics and are off by default for this reset preview.

```bash
uv run python tools/side_tool_path_preview.py \
  --a-x 0 --a-y 1.2 \
  --b-x 8 --b-y 0 \
  --step-spacing-m 0.25 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --tool-length-m 0.18 \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --out-dir outputs/side_tool_path_preview/simple_serpentine
```

Legacy temporal diagnostic example, not the reset default:

```bash
uv run python tools/side_tool_path_preview.py \
  --advanced \
  --workspace-mode ab_diagonal_center \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 1.2 \
  --tool-side right \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --lane-spacing-m 0.25 \
  --row-count auto \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --robot-radius-m 0.14 \
  --boundary-margin-m 0.03 \
  --coverage-resolution-m 0.05 \
  --rotation-sample-deg 5 \
  --translation-sample-m 0.05 \
  --swept-volume-validation strict \
  --kinematic-model differential_drive \
  --contamination-mode strict \
  --fail-on-contamination-violation true \
  --tool-active-during-transitions false \
  --same-step-tool-before-chassis false \
  --route-order auto_temporal_safe \
  --require-start-at-A true \
  --require-end-at-B true \
  --max-start-error-m 0.05 \
  --max-end-error-m 0.05 \
  --allow-best-effort false \
  --auto-orient-tool-inside true \
  --transition-style auto_internal \
  --emit-geometry-samples true \
  --emit-separated-previews true \
  --emit-timeline-frames true \
  --emit-segment-frames true \
  --timeline-frame-stride 5 \
  --emit-contamination-previews true \
  --fail-on-boundary-violation true \
  --out-dir outputs/side_tool_path_preview/ab_diagonal_temporal_right
```

Compatibility alias for the filename that was accidentally tried earlier:

```bash
uv run python tools/preview_side_tool_path.py \
  --workspace-mode ab_diagonal_center \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 1.2 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --lane-spacing-m 0.25 \
  --row-count auto \
  --first-lane-direction forward \
  --transition-style auto_internal \
  --out-dir outputs/side_tool_path_preview/coverage_auto_internal_left
```

Reset outputs:

- `tool_path.csv`
- `summary.md`
- `primitive_sequence.csv`
- `preview_route_sequence.md`
- `preview_tool_path_primary.png`
- `preview_chassis_derived_from_tool.png`
- `preview_primitive_sequence.png`
- `preview_tool_coverage_only.png`

Legacy diagnostic modes may additionally emit geometry samples, swept-volume
summaries, contamination events, and wet-area overlays when those features are
explicitly enabled.

## Field A/B To No-Motion Path Validation

Field capture uses raw opposite rectangle points, then normalizes them into the
tool path start/end corners:

- raw A/B: arbitrary captured opposite rectangle points;
- `A_prime`: top-left corner of the normalized rectangle;
- `B_prime`: bottom-right corner of the normalized rectangle;
- approach to `A_prime`: tool inactive;
- `A_prime` to `B_prime`: simple tool-centered serpentine path;
- no motion validation: target preview fields only, no motor commands.

## Stage 11 Path Package Bridge Check

The outdoor integrated dry-run sensor baseline can pass while the live target
source is still wrong. In the observed dry-run, GPS position, BMI160, and motor
safety passed:

- `gps_block_reason=OK`, `gps_sats=5`, `gps_hdop≈1.88`,
  `position_source=gps`
- `imu_type=BMI160`, `imu_data_plausible=true`, `imu_calibrated=true`
- `physical_compile_gate=false`, `physical_path_following_enable=false`,
  `allow_motor_output=false`, `physical_output_active=false`

GPS course heading can remain unavailable during USB-tethered no-motion because
the rover cannot be moved far enough for course estimation. Treat
`gps_course_deg=NA`, `gps_course_output_block_reason=NO_ACCEPTED_COURSE_YET`,
and `path_following_block_reason=NO_HEADING` as a no-motion/tethered heading
skip, not a failed sensor baseline.

However, `active_target_source=compile_time` means the generated
`field_ab_serpentine/path_package.json` is not connected to live target
validation. That blocks physical path planning progression. Do not proceed to
ground crawl or motor tests until a package-to-firmware or package-to-station
target bridge is implemented and verified in no-motion mode.

Inspect the generated path package:

```bash
uv run python tools/inspect_path_package.py \
  --path-package latest
```

Expected inspection checks:

- `tool_side_left=True`
- `tool_path_starts_at_A_prime=True`
- `tool_path_ends_at_B_prime=True`
- `tool_path_continuous=True`
- `connectors_inactive=True`
- `sweep_tracks_active=True`
- `primitive_sequence_allowed=True`
- `motor_command_generated_false=True`

Generate a software-side physical preview from the package:

```bash
uv run python tools/physical_path_preview_from_package.py \
  --path-package latest \
  --out-dir outputs/stage11_physical_preview/latest
```

This generates:

- `summary.md`
- `preview_tool_path.png`
- `preview_chassis_path.png`
- `preview_primitive_sequence.png`
- `preview_approach_then_serpentine.png`
- `primitive_sequence_checked.csv`

Analyze an integrated dry-run log for path-package linkage:

```bash
uv run python tools/analyze_integrated_dryrun_for_path_package.py \
  outputs/logs/integrated_dryrun_hc12_deferred_*.log
```

If the analyzer reports `active_target_source=compile_time` and
`live_path_package_connected=false`, the recommendation is:

```text
Do not proceed to motor tests. Validate path package offline and implement package-to-firmware/station target bridge.
```

Stage 11 is still no-motion only. USB tether means hand-carry heading validation
is skipped. Real motion remains prohibited.

## Stage 12 Station-Side Path Package Target Bridge

Stage 11 proves that the package is valid and that the sensor baseline can pass,
but it also proves that the live integrated dry-run is still using
`active_target_source=compile_time`. Stage 12 adds a station-side target tracker
that reads `path_package.json` and computes package targets from a current pose.
This is still not firmware path following and does not generate motor commands.

Offline local-pose target preview:

```bash
uv run python tools/station_path_package_tracker.py \
  --path-package latest \
  --mode offline_pose \
  --current-x 0 \
  --current-y 1.2 \
  --current-heading-deg 0 \
  --out-dir outputs/stage12_station_tracker/offline_pose
```

Replay an integrated dry-run log:

```bash
uv run python tools/station_path_package_tracker.py \
  --path-package latest \
  --mode replay_log \
  --log outputs/logs/integrated_dryrun_hc12_deferred_20260607_105607.log \
  --out-dir outputs/stage12_station_tracker/replay_log
```

Live USBDBG read-only target preview, only when explicitly requested:

```bash
uv run python tools/station_path_package_tracker.py \
  --path-package latest \
  --mode live_usbdbg \
  --port "$PORT" \
  --duration-s 60 \
  --out-dir outputs/stage12_station_tracker/live_usbdbg
```

The tracker writes:

- `station_target_status.csv`
- `summary.md`
- `preview_current_target.png`

Summary fields separate firmware and station target sources:

- `station_package_target_source=path_package`
- `firmware_active_target_source`
- `firmware_still_compile_time`
- `local_pose_available`
- `active_primitive_index`
- `target_distance_m`
- `target_bearing_deg`
- `cross_track_error_m`
- `tool_active_expected`
- `motor_command_generated=false`
- `physical_output_active=false`
- `ready_for_station_side_target_preview`
- `ready_for_motor_test=false`

If the replay or live USBDBG rows contain only latitude/longitude and the path
package has no georeference, the tracker reports
`local_pose_available=false` and
`reason=NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL`. In that case, use
`offline_pose` with local meters or capture georeferenced A/B points. The
firmware `compile_time` target is never treated as the station package target.

## Stage 13 Georeferenced A/B And GPS-To-Local Tracking

Stage 13 adds georeferenced field capture so live or replayed USBDBG
`current_lat/current_lon` can be converted into the same local x/y frame used by
the generated path package. This is still station-side preview/no-motion only.
It does not enable firmware path following and does not generate motor commands.

Manual georeferenced A/B capture:

```bash
uv run python tools/capture_georef_ab_points.py \
  --mode manual_latlon \
  --a-lat 35.xxxxx \
  --a-lon 129.xxxxx \
  --b-lat 35.xxxxx \
  --b-lon 129.xxxxx \
  --out-dir outputs/field_georef_capture/latest
```

Replay-log capture requires explicit row ranges for each point:

```bash
uv run python tools/capture_georef_ab_points.py \
  --mode replay_log \
  --log outputs/logs/integrated_dryrun_hc12_deferred_20260607_105607.log \
  --a-row-range 10:30 \
  --b-row-range 80:100 \
  --out-dir outputs/field_georef_capture/latest
```

Live USBDBG capture reads serial only and prompts for A and B sample windows:

```bash
uv run python tools/capture_georef_ab_points.py \
  --mode live_usbdbg \
  --port "$PORT" \
  --sample-count 10 \
  --out-dir outputs/field_georef_capture/latest
```

Generate a georeferenced path package:

```bash
uv run python tools/field_ab_to_serpentine.py \
  --field-points-georef-json outputs/field_georef_capture/latest/field_points_georef.json \
  --step-spacing-m 0.25 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --tool-length-m 0.18 \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --out-dir outputs/field_ab_serpentine/latest
```

The package includes:

- `georeference_available=true`
- `raw_A_lat`, `raw_A_lon`, `raw_B_lat`, `raw_B_lon`
- `origin_lat`, `origin_lon`
- `local_frame_type=equirectangular_enu`
- `x_axis_source=normalized_rectangle`
- `meters_per_deg_lat`, `meters_per_deg_lon`

Check georeference metadata:

```bash
uv run python tools/check_georef_path_package.py \
  --path-package latest \
  --out-dir outputs/stage13_georef_check/latest
```

Replay a USBDBG log through the station package tracker using GPS lat/lon:

```bash
uv run python tools/station_path_package_tracker.py \
  --path-package latest \
  --mode replay_log \
  --log outputs/logs/integrated_dryrun_hc12_deferred_20260607_105607.log \
  --out-dir outputs/stage13_station_tracker/replay_log
```

If georeference exists, the tracker reports
`local_pose_available=true`, `local_pose_source=gps_georeference`, and computes
station-side package targets from GPS. If firmware still reports
`active_target_source=compile_time`, that remains a firmware target-source
blocker, but it is not confused with the station package target:

- `firmware_active_target_source=compile_time`
- `station_package_target_source=path_package`
- `firmware_still_compile_time=true`
- `ready_for_station_side_target_preview=true` when station GPS-to-local target
  computation is valid
- `ready_for_motor_test=false` always

## Stage 14 Station-Side Virtual Controller Preview

Stage 14 computes diagnostic-only virtual controller values from the
station-side package target. It reuses Stage 12/13 target calculations and emits
only `virtual_*` fields:

- `virtual_forward_cmd`
- `virtual_turn_cmd`
- `virtual_left_cmd`
- `virtual_right_cmd`

These values are not firmware commands, are not written to serial, are not sent
over HC-12, and must not be used for motor testing.

Offline pose preview:

```bash
uv run python tools/station_virtual_path_controller.py \
  --path-package latest \
  --mode offline_pose \
  --current-x 0 \
  --current-y 1.2 \
  --current-heading-deg 0 \
  --out-dir outputs/stage14_virtual_controller/offline_pose
```

Replay-log preview:

```bash
uv run python tools/station_virtual_path_controller.py \
  --path-package latest \
  --mode replay_log \
  --log outputs/logs/integrated_dryrun_hc12_deferred_20260607_105607.log \
  --out-dir outputs/stage14_virtual_controller/replay_log
```

Read-only live USBDBG preview, only when explicitly requested:

```bash
uv run python tools/station_virtual_path_controller.py \
  --path-package latest \
  --mode live_usbdbg \
  --port "$PORT" \
  --duration-s 60 \
  --out-dir outputs/stage14_virtual_controller/live_usbdbg
```

Optional diagnostic saturation parameters:

```bash
--max-virtual-forward-cmd 0.10
--max-virtual-turn-cmd 0.05
--lookahead-m 1.0
```

Outputs:

- `virtual_control.csv`
- `summary.md`
- `preview_virtual_control.png`

If current heading is unavailable, Stage 14 reports
`virtual_heading_status=DIAG_ONLY`; it can still compute bearing-only virtual
diagnostics, but `ready_for_motor_test=false` remains mandatory.

## Stage 15 Guarded Motor Sanity Crawl

Stage 15 is a bounded motor-direction and stop-behavior sanity check only. It is
not path following, not tool-path execution, not HC-12 target control, and not
station virtual-controller execution.

The Stage 15 build uses a separate compile-time gate:

- `STAGE15_GUARDED_CRAWL_TEST=1`
- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- `PATH_FOLLOWING_DRYRUN=0`
- `GROUND_CRAWL_TEST_MODE=0`
- `AUTO_MOTION_ARMED=0`

Only these USB-triggered test primitives are accepted:

- `test_forward_pulse`
- `test_backward_pulse`
- `test_rotate_left_pulse`
- `test_rotate_right_pulse`
- `test_stop`

Defaults are intentionally small: `--max-cmd 0.06`, `--pulse-ms 200`, and
`--require-enter true`. The wrapper refuses `--pulse-ms > 300` unless
`--dangerously-allow-longer-pulse` is present, and refuses `--max-cmd > 0.10`
unless `--dangerously-allow-higher-cmd` is present.

Run only when explicitly performing the guarded sanity check:

```bash
scripts/run_guarded_motor_sanity_crawl.sh \
  --port /dev/ttyACM0 \
  --max-cmd 0.06 \
  --pulse-ms 200 \
  --inter-pulse-ms 2000 \
  --require-enter true \
  --out-dir outputs/stage15_guarded_crawl/latest
```

Use wheels off ground or physically restrain the rover. If ground contact is
unavoidable, use the smallest possible pulse and keep a hand on power/stop.

The checker verifies that every pulse is bounded, a stop follows each pulse,
final commands are zero, path following is inactive, no path package is used,
and no unexpected continuous output remains:

```bash
uv run python tools/check_guarded_crawl_log.py \
  outputs/stage15_guarded_crawl/latest/guarded_crawl.log \
  --max-cmd 0.06 \
  --max-duration-ms 200
```

Stage 15 PASS means the short physical motor sanity pulses and stop behavior
matched the guardrails. It does not authorize autonomous path following.

## Stage 16 USB Guarded Path-Package Crawl

Stage 16 is a USB-tethered, station-supervised micro-crawl from a generated
`path_package.json`. It is not normal firmware path following, does not use
compile-time waypoints, and does not use HC-12.

The OpenRB build accepts only an explicit arm, one bounded USB text pulse, and
STOP:

```text
STAGE16_ARM seq=<int>
STAGE16_CMD seq=<int> left=<float> right=<float> ms=<int>
STAGE16_STOP seq=<int>
```

`STAGE16_ARM` clears the Stage 16 latched stop only when RC is OK, neutral is
OK, and physical output is already zero. A `STAGE16_CMD` is rejected unless the
firmware is armed, and the firmware relatches stop after every pulse by
default. `STAGE16_STOP` immediately forces zero output and relatches stop.

Firmware guardrails:

- `STAGE16_USB_GUARDED_CRAWL=1`
- `STAGE16_MAX_CMD=0.04`
- `STAGE16_MAX_MS=150`
- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- `PATH_FOLLOWING_DRYRUN=0`
- `GROUND_CRAWL_TEST_MODE=0`
- `AUTO_MOTION_ARMED=0`

Dry-run first:

```bash
scripts/run_stage16_guarded_path_crawl.sh \
  --port /dev/ttyACM0 \
  --path-package latest \
  --dry-run true \
  --max-cmd 0.03 \
  --pulse-ms 100 \
  --max-total-distance-m 0.20 \
  --duration-s 30 \
  --require-enter true \
  --out-dir outputs/stage16_guarded_path_crawl/latest
```

Physical micro-crawl, only after dry-run and with an operator at the stop/power:

```bash
scripts/run_stage16_guarded_path_crawl.sh \
  --port /dev/ttyACM0 \
  --path-package latest \
  --dry-run false \
  --max-cmd 0.03 \
  --pulse-ms 100 \
  --max-total-distance-m 0.20 \
  --duration-s 30 \
  --require-enter true \
  --out-dir outputs/stage16_guarded_path_crawl/latest
```

Stage 16 still reports `ready_for_full_path_following=false`. It is only a
guarded bridge check that the station can compute a path-package target and send
one explicitly armed and confirmed, short, low-magnitude USB pulse followed by
STOP.

## Stage 16 Motor Deadband Calibration

If Stage 16 logs show `event=ARM`, `event=ACK`, `stage16_cmd_state=ACTIVE`,
`physical_output_active=true`, bounded wheel commands such as
`stage16_left_cmd=0.020 stage16_right_cmd=0.020 ms=100`, and then `event=STOP`
with final commands back to zero, the USB bridge and safety gates are working.
If the rover still does not visibly move, the likely causes are motor deadband,
insufficient pulse magnitude/duration, motor power, motor-enable wiring, driver
mapping, or mechanical friction. This is not a path-planning issue.

Keep MANUAL/neutral RC behavior correct and keep AUTO/path following OFF:

- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- no HC-12
- no compile-time waypoint following
- no serpentine/lane following
- `ready_for_full_path_following=false`

Run deadband calibration only when explicitly ready to test bounded pulses:

```bash
scripts/run_stage16_motor_deadband_calibration.sh \
  --port "$PORT" \
  --mode forward \
  --cmd-list "0.015,0.020,0.025,0.030,0.035,0.040" \
  --pulse-ms-list "80,100,120,150" \
  --require-enter true \
  --interactive-visible-motion true \
  --out-dir outputs/stage16_deadband_calibration/forward
```

The calibration tool sends `STAGE16_ARM` before every pulse, sends exactly one
bounded `STAGE16_CMD`, waits for `ACK/ACTIVE`, waits for `STOP/PULSE_COMPLETE`,
then sends `STAGE16_STOP` anyway. After each trial, the operator records visible
motion as `none`, `forward`, `backward`, `left`, `right`, `twitch`, or
`unknown`.

Check the calibration output:

```bash
uv run python tools/check_stage16_deadband_calibration.py \
  outputs/stage16_deadband_calibration/forward
```

If no visible motion occurs at `cmd=0.040` and `pulse_ms=150`, record
`no_visible_motion_at_limit=true` and suspect motor power, motor enable, wiring,
driver mapping, or mechanical friction before increasing any limit.

## Stage 17 First-Primitive Path-Package Crawl

Stage 17 is a more realistic path-package micro-crawl, but only for the current
or first active primitive. It is not full serpentine execution, does not advance
through lanes automatically, and still uses the station as supervisor. The
OpenRB executes only bounded USB pulses and still does not read the path package.

Stage 17 exists because Stage 16 proved the command path and physical output can
activate, while visible movement may still be below motor/mechanical deadband.
Stage 17 allows slightly longer bounded pulses:

- default wrapper values: `--max-cmd 0.045`, `--pulse-ms 300`
- compile defaults: `STAGE17_MAX_CMD=0.06`, `STAGE17_MAX_MS=500`
- hard wrapper limits: `--max-cmd <= 0.080`, `--pulse-ms <= 800`
- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- no HC-12
- no AUTO path following
- `ready_for_full_path_following=false`

Dry-run the first-primitive command proposal first:

```bash
scripts/run_stage17_first_primitive_crawl.sh \
  --port "$PORT" \
  --path-package latest \
  --dry-run true \
  --pose-mode manual_local \
  --current-x 0 \
  --current-y 1.2 \
  --current-heading-deg 0 \
  --max-cmd 0.045 \
  --pulse-ms 300 \
  --max-total-distance-m 0.20 \
  --require-enter true \
  --out-dir outputs/stage17_first_primitive_crawl/latest
```

Physical first-primitive micro-crawl, only after dry-run and with an operator at
stop/power:

```bash
scripts/run_stage17_first_primitive_crawl.sh \
  --port "$PORT" \
  --path-package latest \
  --dry-run false \
  --pose-mode manual_local \
  --current-x 0 \
  --current-y 1.2 \
  --current-heading-deg 0 \
  --max-cmd 0.045 \
  --pulse-ms 300 \
  --max-total-distance-m 0.20 \
  --require-enter true \
  --out-dir outputs/stage17_first_primitive_crawl/latest
```

If the station-computed path command is too asymmetric and only tests turning,
use `--force-straight-test true` as a diagnostic motor-deadband check. In that
mode, the output must be labeled diagnostic only and must not be treated as
path following.

Check the output:

```bash
uv run python tools/check_stage17_first_primitive_crawl_log.py \
  outputs/stage17_first_primitive_crawl/latest
```

## Stage 18 Motor Mapping And Power Probe

If Stage 17 reaches `physical_output_active=true` and final commands such as
`0.045/0.045` for `300 ms`, then stops cleanly, but there is still no visible
motion, the station-to-firmware command path is working. The remaining suspects
are motor deadband, motor power, motor driver enable, wiring/ground, backend
mapping, or mechanical friction.

Stage 18 does not use a path package target and does not run path following. It
sends bounded motor mapping probes only:

- `forward`
- `backward`
- `rotate_left`
- `rotate_right`
- `left_wheel_only`
- `right_wheel_only`

Default probe set:

- command values: `0.04,0.06,0.08`
- pulse durations: `300,500,800 ms`
- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- no HC-12
- `ready_for_full_path_following=false`

Run only when explicitly ready to test bounded motor output:

```bash
scripts/run_stage18_motor_mapping_probe.sh \
  --port "$PORT" \
  --mode all \
  --cmd-list "0.04,0.06,0.08" \
  --pulse-ms-list "300,500,800" \
  --require-enter true \
  --interactive-visible-motion true \
  --out-dir outputs/stage18_motor_mapping_probe/latest
```

Each pulse logs requested/logical/physical command mapping:

- `requested_left_cmd`, `requested_right_cmd`
- `logical_left_cmd`, `logical_right_cmd`
- `physical_a_cmd`, `physical_b_cmd`
- `final_left_cmd`, `final_right_cmd`
- `motor_write_called`
- `motor_backend`
- `motor_enable_state`
- `pwm_or_dynamixel_write_status`

After every pulse, record whether the left wheel, right wheel, and rover body
visibly moved. If no visible motion occurs at `cmd=0.08` and `pulse_ms=800`,
record `motor_power_or_mapping_suspect=true` and inspect battery, motor driver
enable, wiring, common ground, and motor backend mapping before increasing any
limit.

Check the output:

```bash
uv run python tools/check_stage18_motor_mapping_probe.py \
  outputs/stage18_motor_mapping_probe/latest
```

## Stage 20 Physical A/B Manual-Equivalent Probe

Manual RC already proved the motor power path and physical output mapping:

- `physical_a_role=throttle`
- `physical_b_role=turn`
- `wheel_to_physical_mapping=diff_to_throttle_turn`
- manual RC path: `manual_forward_sign=-1.0`, `manual_turn_sign=1.0`
- Stage 20 direct physical A/B calibration: positive `a` is forward,
  negative `a` is backward, positive `b` is left turn, and negative `b` is
  right turn for the current probe setup.

Stage 20 uses those physical A/B semantics directly. It does not treat `a` and
`b` as left/right wheel commands. The USB command is:

```text
STAGE20_CMD seq=<n> a=<physical_throttle> b=<physical_turn> ms=<ms>
```

The guarded firmware must be compiled with:

- `STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1`
- `PHYSICAL_PATH_FOLLOWING_ENABLE=0`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`
- `STAGE16_USB_GUARDED_CRAWL=0`
- `STAGE17_FIRST_PRIMITIVE_CRAWL=0`
- `STAGE18_MOTOR_MAPPING_PROBE=0`

Run a forward A-channel probe only when explicitly ready for bounded wheel-off
or restrained motor output:

```bash
scripts/run_stage20_physical_ab_probe.sh \
  --port "$PORT" \
  --mode forward \
  --cmd-list "0.08,0.12,0.16,0.20,0.25" \
  --pulse-ms-list "300,500,800" \
  --forward-sign 1.0 \
  --turn-sign 1.0 \
  --require-enter true \
  --interactive-visible-motion true \
  --out-dir outputs/stage20_physical_ab_probe/forward
```

Run a right-turn B-channel micro probe separately. Start low because recent
field results showed right turn can be over-strong:

```bash
scripts/run_stage20_physical_ab_probe.sh \
  --port "$PORT" \
  --mode turn_right \
  --cmd-list "0.04,0.06,0.08,0.10,0.12" \
  --pulse-ms-list "150,250,350" \
  --forward-sign 1.0 \
  --turn-sign 1.0 \
  --stop-after-big-turn true \
  --require-enter true \
  --interactive-visible-motion true \
  --out-dir outputs/stage20_physical_ab_probe/turn_right_confirm
```

Every pulse still requires `STAGE20_ARM`, is followed by `STAGE20_STOP`, and
must end with `final_left_cmd=0` and `final_right_cmd=0`. Stage 20 always
reports `ready_for_full_path_following=false`.

## Stage 20B Directional Calibration

Stage 20B builds a direction-specific physical A/B calibration from confirmed
Stage 20 probe directories. Use separate forward, backward, left-turn, and
right-turn runs. Do not use a single global minimum forward or turn command;
the observed results are asymmetric.

```bash
uv run python tools/build_stage20_directional_calibration.py \
  --forward-dir outputs/stage20_physical_ab_probe/forward_sign_pos \
  --backward-dir outputs/stage20_physical_ab_probe/backward_sign_pos \
  --turn-left-dir outputs/stage20_physical_ab_probe/turn_left_confirm \
  --turn-right-dir outputs/stage20_physical_ab_probe/turn_right_confirm \
  --out outputs/stage20_physical_ab_probe/calibration/physical_ab_directional_calibration.json
```

If `turn_right_gain_class=overstrong` or `turn_left_gain_class=weak`, Stage 21
must use `--straight-only true` or `--turn-disabled true`. Turning stays blocked
until both left and right turns are calibrated safely.

## Stage 20C Forward/Backward Fine Calibration

The current fine calibration result is forward-specific: backward motion is
visible, but positive-A forward still may only twitch or show no body motion.
That means motor power and the driver path are not globally suspect. Treat this
as `forward_threshold_unresolved=true` until a positive-A command produces
reliable forward body motion.

Run the extended forward probe only when explicitly ready for bounded
wheel-off-ground or restrained output:

```bash
scripts/run_stage20_physical_ab_probe.sh \
  --port "$PORT" \
  --mode forward \
  --cmd-list "0.25,0.28,0.30,0.32,0.35" \
  --pulse-ms-list "650,800,1000" \
  --forward-sign 1.0 \
  --turn-sign 1.0 \
  --max-abs-a 0.35 \
  --max-ms 1000 \
  --require-enter true \
  --interactive-visible-motion true \
  --continue-after-visible false \
  --out-dir outputs/stage20_physical_ab_probe/forward_extended
```

If any trial is rejected with `LATCHED_STOP` or `RC_INVALID`, mark that trial
invalid and stop the sweep. Do not count it as a no-motion sample. Send
`STAGE20_STOP`, correct the RC/arming state, and rerun that point deliberately.

Build the fine calibration after the forward and backward fine runs:

```bash
uv run python tools/build_stage20_fine_motion_calibration.py \
  --forward-fine-dir outputs/stage20_physical_ab_probe/forward_extended \
  --backward-fine-dir outputs/stage20_physical_ab_probe/backward_fine \
  --base-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_directional_calibration.json \
  --out outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json
```

If the generated fine calibration has `forward_threshold_unresolved=true` or
`ready_for_stage21_forward=false`, do not run a physical Stage 21 forward pulse.
Stage 21 may still dry-run the command, but the physical command remains blocked
with `stage21_command_blocked_reason=FORWARD_THRESHOLD_UNRESOLVED`.

## Stage 21 Path Package Physical A/B First Primitive

Stage 21 is one bounded path-package primitive using the Stage 20 physical A/B
transport. The station loads `path_package.json`, computes the current
station-side target, converts virtual forward/turn to physical A/B:

```text
physical_a = forward_sign * scaled_forward_cmd
physical_b = turn_sign * scaled_turn_cmd
```

Then it applies the minimum effective A/B command from Stage 20. It does not run
the full serpentine path and does not advance through lanes.
With asymmetric turn calibration, keep Stage 21 straight-only so the B channel
is exactly zero.

Dry-run first:

```bash
scripts/run_stage21_path_physical_ab_first_primitive.sh \
  --port "$PORT" \
  --path-package latest \
  --pose-mode manual_local \
	  --current-x 0 --current-y 1.2 --current-heading-deg 0 \
	  --calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_directional_calibration.json \
	  --use-recommended-crawl-command false \
	  --straight-only true \
	  --turn-disabled true \
	  --max-abs-a 0.35 \
	  --max-ms 1000 \
	  --max-forward-cmd 0.28 \
	  --pulse-ms 800 \
	  --dry-run true \
	  --out-dir outputs/stage21_path_physical_ab_first_primitive/dry_run
```

If Stage 20 has confirmed safe visible motion and the rover is restrained or
wheel-off-ground, the physical first-primitive command is:

```bash
scripts/run_stage21_path_physical_ab_first_primitive.sh \
  --port "$PORT" \
  --path-package latest \
  --pose-mode manual_local \
	  --current-x 0 --current-y 1.2 --current-heading-deg 0 \
	  --calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_directional_calibration.json \
	  --use-recommended-crawl-command false \
	  --straight-only true \
	  --turn-disabled true \
	  --max-abs-a 0.35 \
	  --max-ms 1000 \
	  --max-forward-cmd 0.28 \
	  --pulse-ms 800 \
	  --dry-run false \
  --require-enter true \
  --out-dir outputs/stage21_path_physical_ab_first_primitive/latest
```

Stage 21 remains `ready_for_full_path_following=false`.

## Stage 22 Straight Segment Repeat

Stage 22 repeats the confirmed straight-only primitive to build a short straight
segment from multiple stable pulses:

- physical A/throttle: `0.30`
- physical B/turn: `0.00`
- pulse: `800 ms`
- `max_abs_a=0.35`
- `max_abs_b=0.25`
- `max_ms=1000`
- `straight_only=true`
- `turn_disabled=true`

Do not increase pulse duration for this stage. Do not enable turning, HC-12,
compile-time waypoints, serpentine execution, or full path following.

Run only when explicitly ready for bounded restrained or wheel-off-ground
physical output:

```bash
scripts/run_stage22_straight_segment.sh \
  --port "$PORT" \
  --path-package outputs/field_ab_serpentine_georef/latest/path_package.json \
  --fine-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json \
  --repeat-count 3 \
  --pulse-ms 800 \
  --max-abs-a 0.35 \
  --max-ms 1000 \
  --require-enter true \
  --out-dir outputs/stage22_straight_segment/latest
```

The runner sends `STAGE20_ARM` before every pulse, sends exactly one
`STAGE20_CMD` with B forced to zero, waits for `ACK`/`ACTIVE`, waits for
`STOP`/pulse completion, and sends `STAGE20_STOP` after every pulse anyway.
It aborts on `REJECT`, nonzero final commands, or output still active after
STOP.

The checker requires:

- all pulses ACKed
- all pulses stopped
- no reject
- final commands zero
- B always zero
- at least two out of three pulses reported as `forward` or `twitch`
- `ready_for_full_path_following=false`

Manual local-meter capture:

```bash
uv run python tools/capture_field_ab_points.py \
  --a-x 8 --a-y 0 \
  --b-x 0 --b-y 1.2 \
  --out-dir outputs/field_ab_capture/latest
```

Generate the field path package:

```bash
uv run python tools/field_ab_to_serpentine.py \
  --field-points-json outputs/field_ab_capture/latest/field_points.json \
  --current-x 8 --current-y 0 --current-heading-deg 0 \
  --step-spacing-m 0.25 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --tool-length-m 0.18 \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --out-dir outputs/field_ab_serpentine/latest
```

Run package-only no-motion validation first. This does not open serial; it only
loads the latest generated `path_package.json` and verifies that target preview
rows can be produced from the package:

```bash
scripts/run_path_no_motion_validation.sh \
  --path-package latest \
  --mode package_check \
  --out-dir outputs/path_no_motion_validation/latest
```

The output should include:

```text
validation_mode=package_check
serial_opened=false
selected_path_package=...
reason=package_check_does_not_use_serial
motor_command_generated=false
physical_output_active=false
```

Package discovery for `--path-package latest` checks:

1. `outputs/field_ab_serpentine/latest/path_package.json`
2. newest `outputs/field_ab_serpentine/*/path_package.json`
3. newest `outputs/field_runs/*/field_ab_serpentine/path_package.json`
4. newest safe `outputs/**/path_package.json`

If a provided path does not exist, the validator prints the missing path and
nearest candidates instead of a Python traceback.

Run live serial no-motion target validation only when you intentionally want to
open the OpenRB serial port and read USBDBG lines:

```bash
scripts/run_path_no_motion_validation.sh \
  --port /dev/ttyACM0 \
  --path-package latest \
  --mode live_serial \
  --duration-s 120 \
  --concise true \
  --no-motion-gps-mode position_only \
  --min-sats 4 \
  --max-hdop 3.0 \
  --out-dir outputs/path_no_motion_validation/latest
```

In concise `position_only` mode, stationary GPS course heading is not required.
The GPS position is accepted when `position_source=gps`,
`gps_sats>=4`, and `gps_hdop<=3.0`. If IMU relative yaw is present, it is
reported as diagnostic-only. If GPS course is unavailable because the rover has
not moved far enough, the validator reports
`heading_status=WAITING_FOR_MOTION_OR_DIAG_ONLY`, not `FAIL`. No-motion target
preview is driven by finite `target_distance_m` and `target_bearing_deg`, not by
GPS course heading.

Concise output keeps only target-preview and safety fields:

- `validation_mode`, `serial_opened`, `selected_path_package`,
  `path_package_loaded`
- `gps_status`, `gps_sats`, `gps_hdop`, `position_source`, `current_lat`,
  `current_lon`
- `imu_status`, `imu_type`, `imu_chip_id`, `imu_data_plausible`
- `rc_status`, `heading_status`
- `active_primitive_index`
- `target_distance_m`, `target_bearing_deg`, `cross_track_error_m`
- `heading_error_deg`
- `motor_command_generated=false`
- `physical_output_active=false`
- `ready_for_outdoor_no_motion_validation`
- `next_action`

Raw RC pulse widths, HC-12 fields, gyro-bias details, GPS course latch details,
desired command repeats, and compile-time waypoint fields belong in verbose
diagnostics, not the concise no-motion target preview.

Check the concise summary with:

```bash
uv run python tools/check_path_no_motion_summary.py \
  outputs/path_no_motion_validation/latest \
  --min-sats 4 \
  --max-hdop 3.0
```

The checker prints one operator action:

- `PASS`: motor output is disabled, the path package is loaded, GPS position
  quality is acceptable, and target distance/bearing are finite.
- `WAIT`: GPS position exists but satellites or HDOP are marginal.
- `FAIL`: GPS position is missing, motor/physical output is active, the path
  package is missing, serial was expected but not opened, or target values are
  NA.

Offline tracking simulation:

```bash
uv run python tools/simulate_side_tool_tracking.py \
  --path-csv outputs/side_tool_path_preview/coverage_auto_internal_left/side_tool_path.csv \
  --out-dir outputs/side_tool_tracking_sim/coverage_auto_internal_left
```

The simulator writes `tracking_errors.csv` and `summary.md`. It produces only
`virtual_desired_forward_cmd` / `virtual_desired_turn_cmd` diagnostics and keeps
`motor_command_generated=False`.

Optional waypoint/bearing export for offline no-motion review:

```bash
uv run python tools/preview_side_tool_waypoints.py \
  --tool-side left \
  --tool-lateral-offset-m 0.35 \
  --tool-width-m 0.30 \
  --lane-spacing-m 0.30 \
  --row-length-m 8.0 \
  --row-count 4 \
  --start-heading-deg 0 \
  --first-lane-direction forward \
  --out-dir outputs/side_tool_waypoint_preview/left_tool_example
```

This produces `side_tool_waypoints.csv` and `waypoint_summary.md`. It lists
preview target bearings, segment labels, expected rover heading, and whether a
reverse-direction lane is expected. It does not generate motor commands.

Preview PASS:

- `workspace_mode=tool_serpentine_ab`.
- `planner_primary_path=tool_center_path`.
- `tool_path_starts_at_A=True`.
- `tool_path_ends_at_B=True`.
- `tool_path_continuous=True`.
- `tool_connector_count = tool_track_count - 1`.
- `chassis_path_derived_from_tool=True`.
- `primitive_sequence_valid=True`.
- every primitive is one of `move_forward`, `move_backward`, `rotate_left`, or
  `rotate_right`.
- `contamination_mode=off`.
- `motor_command_generated=False` for every row.
- simulation rows contain only `virtual_*` command diagnostics.

If legacy strict contamination diagnostics are enabled, treat their failures as
diagnostic blockers for that stricter workflow only. They are not part of the
default reset readiness gate.

Preview FAIL:

- any output contains rover motor commands.
- any tool footprint path is missing or identical to chassis centerline when a
  lateral offset was requested.
- any preview row has `within_boundary=False`.

For the immediate offline planning checklist, hand-carry route validation is not
required if bounded geometry and offline tracking simulation pass. Physical motor
enable remains prohibited.
