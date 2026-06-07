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

Run offline no-motion validation from a sample status log:

```bash
scripts/run_path_no_motion_validation.sh \
  --port /dev/ttyACM0 \
  --path-package outputs/field_ab_serpentine/latest/path_package.json \
  --sample-log outputs/logs/no_motion_status_sample.log \
  --out-dir outputs/path_no_motion_validation/latest
```

The wrapper reports the port for traceability but does not open serial by
default. It writes `no_motion_validation.csv` and `summary.md` with:

- `current_position`
- `current_heading`
- `active_primitive_index`
- `target_point`
- `target_distance_m`
- `target_bearing_deg`
- `heading_error_deg`
- `cross_track_error_m`
- `along_track_progress_m`
- `tool_active_expected`
- `motor_command_generated=false`
- `physical_output_active=false`

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
