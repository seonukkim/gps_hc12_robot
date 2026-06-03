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

HC-12 status: Mac temporary station RF validation is **DEFERRED** (USB-Serial is
stable but the RF link did not carry bytes; see `docs/current_hardware_status.md`).
HC-12 is **optional** for the USB-monitor dry-run below, and its final validation
moves to Stage G on the Ubuntu Station. HC-12 deferral does NOT block Stages B–F.

## Staged plan (HC-12 deferred) — A through G

Run in order. Motors stay disabled until every prior stage passes.

- [ ] **Stage A — Commit diagnostic tools.** Commit the HC-12 / UART / IMU
      diagnostic tools and docs so the working setup is reproducible
      (`tools/station_serial.py`, `tools/hc12_operational_diagnose.py`,
      `tools/hc12_diagnose_report.py`, `tools/serial_raw_read.py`,
      `firmware/uart_port_sweep_probe`, `scripts/hc12_field_check.sh`).
- [ ] **Stage B — Manual control restore check.** Flash the default
      `openrb_robot_controller`; verify RC MANUAL drives (`control_source=RC_MANUAL`),
      STOP/failsafe works, and AUTO holds neutral. Do not change motor mapping.
- [ ] **Stage C — IMU yaw diagnostic indoors.** `firmware/imu_probe` with
      `IMU_YAW_DIAG=1`. **Strict pass** (check with
      `scripts/check_imu_probe_log.sh`): `imu_mpu_class_present=true` with a
      `0x68`/`0x69` address and a readable WHO_AM_I, then `imu_calibrated=true`,
      pick `IMU_YAW_AXIS`/`IMU_YAW_SIGN`, small stationary drift. `imu_present=true`
      alone is NOT a pass. **CURRENT STATUS: BLOCKED** — the latest indoor run
      showed `i2c_addr=0x03 / UNKNOWN_I2C_DEVICE / imu_mpu_class_present=false`
      (IMU_NOT_MPU_CLASS_DETECTED). Fix the IMU wiring/power/module, or rely on
      GPS course heading at Stage E. This does NOT block Stage D.
- [ ] **Stage D — Integrated GPS+IMU path-following dry-run, motors disabled.**
      Run `scripts/run_integrated_dryrun_no_motion.sh` and inspect with
      `scripts/check_integrated_dryrun_log.sh`. Require
      `physical_block_reason=COMPILE_GATE_OFF`, `physical_output_active=false`,
      `final_left_cmd=0.000`, `final_right_cmd=0.000`. HC-12 not required
      (`hc12_rx_count` may stay 0).
- [ ] **Stage E — Outdoor GPS/heading dry-run (no motors).** Same build outdoors:
      `gps_motion_ready=true`, `heading_ready=true` after displacement,
      `heading_source` as configured, and the GPS-course-vs-IMU-yaw comparison
      (`heading_agreement_diag=SEEDED_DELTA_COMPARE`, small
      `heading_agreement_error_deg`). Still no motor output.
- [ ] **Stage F — Guarded crawl test, only after all gates pass.** Only after
      Stages B–E pass and the RC/PPM mode channel holds: a tiny wheels-off-ground /
      open-area guarded crawl with all gates explicit
      (`PHYSICAL_PATH_FOLLOWING_ENABLE=1`, `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1`,
      `AUTO_MOTION_ARMED=1`, `GROUND_CRAWL_TEST_MODE=1`,
      `PATH_FOLLOWING_MODE_CHANNEL_STABLE=1`), tiny caps, short timeout, manual
      stop ready. Never set by default.
- [ ] **Stage G — Ubuntu Station HC-12 validation (later).** Repeat the HC-12
      diagnosis on the Ubuntu Station + USB-Serial (the intended station) using
      `scripts/hc12_field_check.sh`: expect `UART_SWEEP_RECEIVED_ON_SERIAL3` then
      `HC12_LINK_OK`. This is deferred and independent of Stages B–F.

## Stage 1 — Indoor / breadboard (no motors)

- [ ] IMU probe **strict** pass: `firmware/imu_probe` shows
      `bus_state=RELEASED_HIGH`, `imu_mpu_class_present=true`,
      `imu_probe_pass=true` with `i2c_addr=0x68`/`0x69`, `whoami` readable (e.g.
      `0x6F`), live `accel_raw_*` / `gyro_raw_*`, and `imu_calibrated=true`.
      `imu_present=true` alone is NOT a pass; `i2c_addr=0x03` / `UNKNOWN_I2C_DEVICE`
      / `imu_mpu_class_present=false` is a FAIL (IMU_NOT_MPU_CLASS_DETECTED).
      Verify with `scripts/check_imu_probe_log.sh`. (Currently BLOCKED.)
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
