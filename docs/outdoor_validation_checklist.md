# Outdoor Validation Checklist

Staged checklist for taking the GPS + IMU + HC-12 navigation stack from the
breadboard to a first guarded physical run. Do not skip a stage. Physical motor
execution stays disabled by default and is only attempted after every prior
stage passes.

Context: the old board/circuit had wiring issues. After rebuilding on a
breadboard, HC-12 and IMU both work. So earlier IMU/HC-12 failures were the old
circuit, not the OpenRB-150 or the code. The remaining blockers before physical
driving are (1) the RC/PPM Manual/Auto mode channel must hold reliably and
(2) a heading source must be validated outdoors.

## Stage 1 — Indoor / breadboard (no motors)

- [ ] IMU probe pass: `firmware/imu_probe` shows `bus_state=RELEASED_HIGH`,
      `i2c_addr=0x69`, `whoami=0x6F`, `imu_present=true`, live `accel_raw_*` /
      `gyro_raw_*`, and (default build) `imu_calibrated=true` with small
      `gyro_bias_*`.
- [ ] IMU yaw axis/sign found: with `IMU_YAW_DIAG=1`, a clockwise yaw increases
      `imu_relative_yaw_deg`; pick `IMU_YAW_AXIS` / `IMU_YAW_SIGN` accordingly.
- [ ] IMU stationary drift acceptable: `imu_relative_yaw_deg` drifts only slowly
      while stationary over ~30–60 s (note the rate; it is expected, not zero).
- [ ] HC-12 PING/PONG pass: `firmware/hc12_link_probe` (Serial3) +
      `tools/hc12_link_probe.py` show `link_status=LINK_OK`, rising
      `hc12_rx_count`/`pong_rx_count`, `hc12_parse_error` low.
- [ ] Path-planning preview pass: `tools/path_planning_preview.py` writes
      `waypoints.csv` + `summary.md`.
- [ ] Path-following dry-run pass: integrated build prints distance/bearing,
      `path_following_block_reason`, and (with `IMU_ENABLE=1`) the IMU fields.
      `physical_block_reason=COMPILE_GATE_OFF`, all final commands `0.000`.

## Stage 2 — Outdoor, NO motors

- [ ] GPS fix OK: `gps_block_reason=OK`, `gps_motion_ready=true`,
      `gps_sats` healthy, `gps_hdop` low.
- [ ] GPS course-over-ground ready: after enough displacement,
      `heading_ready=true`, `heading_source` as configured, `gps_course_deg`
      numeric.
- [ ] IMU yaw diagnostic stable enough for a short run (calibrated, bounded
      drift over the run length).
- [ ] GPS course vs IMU yaw compared: walk/drive a known turn and check
      `heading_agreement_diag=SEEDED_DELTA_COMPARE` with a small
      `heading_agreement_error_deg` (the change in GPS course should track the
      change in IMU yaw). Record the error; do not trust IMU heading until this
      passes repeatedly.
- [ ] HC-12 status still OK at the field site (range, `link_status=LINK_OK`,
      `STATUS` returns dry-run state).
- [ ] RC/PPM Manual/Auto mode channel holds (re-check the CH5/PPM blocker with
      `firmware/ppm_channel_map_probe`).

## Stage 3 — Outdoor, guarded physical (only after Stage 2 passes)

- [ ] All Stage 2 items green and repeatable.
- [ ] Heading source approved: `HEADING_SOURCE_MODE` is GPS_COURSE (0) or
      GPS_COURSE_WITH_IMU_DIAG (2). IMU yaw remains diagnostic-only.
- [ ] Tiny caps: `PATH_FOLLOWING_MAX_FORWARD_CMD` ≤ 0.18,
      `PATH_FOLLOWING_MAX_TURN_CMD` ≤ 0.04.
- [ ] Short timeout: `PATH_FOLLOWING_MAX_AUTO_MS` ≤ 500 ms, latch-stop verified
      (returns to zero, clears only on MANUAL).
- [ ] Manual stop ready: RC operator can take MANUAL / failsafe instantly.
- [ ] Station operator ready with HC-12 `ESTOP`.
- [ ] Wheels-off-ground or open area with kill switch for the first attempt.
- [ ] Only then compile with all gates explicit
      (`PHYSICAL_PATH_FOLLOWING_ENABLE=1`, `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1`,
      `AUTO_MOTION_ARMED=1`, `GROUND_CRAWL_TEST_MODE=1`,
      `PATH_FOLLOWING_MODE_CHANNEL_STABLE=1`). These are never set by default.

## Priorities

- Physical path planning / path-following validation is the priority.
- UI and ROS2 are optional. Do not over-prioritize ROS2: the director plans to
  try ViAM over the summer, which may replace or reduce the value of a ROS2
  integration. Keep the core firmware + station tooling stack independent of
  ROS2 so either path stays open.
