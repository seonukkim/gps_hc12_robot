# Current Hardware Status

> 2026-05-27 note: GPS UART receive is now confirmed on `Serial2` at `9600`
> with the GPS connected to the central OpenRB connector. The earlier `Serial3`
> `D13` / `D14` checks failed because the current wiring is not on those pins.
> GPS fix also succeeded on this `Serial2` path, including the integrated
> `FIXED_WIRING_GPS_SERIAL2_DIAG` build with HC-12 disabled and motors neutral.
> Previous Option A and Option B rewiring plans are superseded by the Fixed
> Wiring Plan: GPS cannot be moved, HC-12 cannot be moved, and the current
> wiring must be audited.

## Circuit Rebuild Root Cause (2026-06-03)

After meeting the director, the previous board/circuit was found to have
wiring/circuit issues. The circuit was rebuilt on a breadboard, and:

- HC-12 now works and connects to the computer/station successfully.
- The IMU now works and is detected reliably.
- Therefore the earlier IMU and HC-12 failures were most likely caused by the
  old circuit/wiring, not by the OpenRB-150 board or the firmware/code.

What this changes:

- GPS stays on `Serial2`. HC-12 must be on `Serial3` or `Serial1` (not
  `Serial2`); path-following builds reject HC-12 on `Serial2` at compile time.
- Physical path planning is now plausible, but outdoor validation is still
  required first (GPS fix + course-over-ground, IMU yaw drift check, and a
  GPS-course-vs-IMU-yaw agreement comparison). See
  `docs/outdoor_validation_checklist.md`.
- Priority is physical path planning / path-following validation and the safe
  firmware dry-run. UI and ROS2 are optional. Do not over-prioritize ROS2: the
  director plans to try ViAM over the summer, which may replace or reduce the
  value of a ROS2 integration. Keep core firmware/station tooling ROS2-independent.
- Physical motor execution remains disabled by default (all four
  path-following motion gates plus the mode-channel acknowledgement default to
  `0`). IMU yaw is diagnostic-only and is never used to drive motors.

IMU candidate (unchanged): I2C `~0x69`, WHO_AM_I observed `0x6F`; treat as an
MPU/ICM register-map-compatible clone/variant until proven otherwise.

## HC-12 Mac temporary station validation: DEFERRED (2026-06-03)

Decision: HC-12 RF validation on the Mac temporary station is **DEFERRED**. It is
NOT a blocker for GPS / IMU / path-following dry-run work, which runs over the USB
monitor and does not need HC-12.

State at deferral:

- Station USB-Serial (`/dev/cu.usbserial-02442CA5`) is **stable** with DTR/RTS
  forced low. The DTR/RTS-safe station tools work: `read-only` and `write-only`
  open cleanly with `serial_error_count=0`.
- RF link was **not established** in the Mac temporary station setup:
  - station read-only: `total_bytes=0`, `detected_uart_ports=none`
  - station write-only: `tx_count>0`, `serial_error_count=0`,
    verdict `STATION_TX_OK_NO_RX` (station TX path healthy, nothing came back)
  - OpenRB uart sweep: `Serial3_tx` increases but `Serial3_rx` stays `0`
- Interpretation: the station TX and USB bridge are fine; the RF path simply did
  not carry bytes in this temporary setup. This is a physical RF / settings /
  station question, not a firmware/code fault, and not a GPS/IMU blocker.

Required follow-up (do not delete the HC-12 tools):

- Final HC-12 validation must be **repeated on the Ubuntu Station + USB-Serial**
  setup (the intended station), not on the Mac temporary station.
- Until then, HC-12 is **optional** for the current USB-monitor-based dry-run.
  The integrated dry-run build sets `hc12_enabled=true` on Serial3 but does not
  require any HC-12 RX to succeed (`hc12_rx_count` may stay 0).

See `docs/outdoor_validation_checklist.md` (Stage G) for the Ubuntu-station HC-12
re-validation, and the section below for the in-place diagnosis procedure.

## IMU indoor status: BLOCKED (2026-06-03, IMU_NOT_MPU_CLASS_DETECTED)

The latest indoor `firmware/imu_probe` (Stage C) run is **NOT an IMU pass**.
Upload/monitor work and the I2C bus is released high, but the probe repeatedly
sees only:

```text
i2c_addr=0x03 imu_candidate=UNKNOWN_I2C_DEVICE whoami=NA
imu_present=true imu_mpu_class_present=false
```

Interpretation: `imu_present=true` alone is **not** a pass — it is set whenever
any address ACKs, and a lone `0x03` reading `UNKNOWN_I2C_DEVICE` is a bus
artifact / non-IMU device, not the IMU. Current indoor IMU status is therefore
**IMU_BLOCKED / IMU_NOT_MPU_CLASS_DETECTED**.

Strict IMU pass criteria (use these everywhere now):

- `imu_mpu_class_present=true`, AND
- a valid MPU/ICM-style address (`0x68` or `0x69`), AND
- a readable WHO_AM_I (and in continuous mode, gyro calibration completes).

Known-good reference: `i2c_addr=0x69`, `whoami=0x6F`,
`imu_mpu_class_present=true`. The probe now also prints an explicit
`imu_probe_pass` / `imu_probe_block_reason`; check a probe log with
`scripts/check_imu_probe_log.sh`.

Impact:

- The integrated no-motion dry-run still runs for safety/logging (motors stay
  disabled), but **heading validation is NOT passed** until either a valid
  MPU/ICM IMU is detected OR GPS course-over-ground heading is valid
  (`gps_motion_ready=true` and `heading_ready=true`).
- Do not proceed to outdoor heading validation or any guarded crawl until a
  heading source is valid. Fix the IMU wiring/power/module first, or rely on GPS
  course heading outdoors.

## HC-12 operational diagnosis without unplugging the module

The HC-12 is fixed on the board and cannot be removed. The repeatable field loop
uses `tools/hc12_operational_diagnose.py` (station) + `firmware/uart_port_sweep_probe`
(OpenRB) + `tools/hc12_diagnose_report.py` (report). No unplugging, loopback, or
AT mode. Wiring is already verified; `total_bytes=0` is a state to interpret, not
an automatic code bug. HC-12 is expected on Serial3 (D14/D13); GPS stays on
Serial2. Full commands are in `firmware/README.md`.

Five states to distinguish:

1. Station OFF / counterpart off: `NO_RX` / `total_bytes=0` is EXPECTED and does
   not prove firmware failure. Mark with `--station-off` → verdict
   `TEST_INVALID_STATION_OFF`; the link test is invalid until both HC-12 sides
   are powered and one is transmitting.
2. Station USB stability (`--mode stability`): open/close OK + rising
   `stability_alive_ticks` + `serial_error_count=0` means the USB bridge is alive;
   `in_waiting=0` means no bytes, not failure (`USB_SERIAL_STABLE_NO_RF_BYTES`).
3. OpenRB uart sweep: run concurrently with a station `read-only` test; the
   detected `@UART,<n>` port number is the HC-12 TX UART.
4. Station write-only: run while the OpenRB sweep monitor is open and watch which
   `SerialN_rx` rises (Serial2 rx is GPS NMEA, not HC-12).
5. HC-12 link / ping-pong: only meaningful when both sides are powered and one
   responds (`pong_rx>0` → `HC12_LINK_OK`).

Observed 2026-06-03: station `stability` and `read-only` runs opened
`/dev/cu.usbserial-02442CA5` cleanly with `serial_error_count=0` and
`total_bytes=0` → `USB_SERIAL_STABLE_NO_RF_BYTES` (USB bridge healthy; no RF bytes
because the counterpart was not transmitting in that test). This is the expected
"station/off" interpretation, not a firmware fault.

DTR/RTS handling (Mac temporary station, 2026-06-03): earlier `write-only` /
`ping-pong` runs failed with `OSError Errno 6 Device not configured` →
`STATION_USB_UNSTABLE`. A manual PySerial test only worked with hardware flow
control off and DTR/RTS forced low (`rtscts=False`, `dsrdtr=False`,
`setDTR(False)`, `setRTS(False)`). The station USB-Serial is therefore not
necessarily unstable — pyserial's default DTR/RTS assertion on open resets some
adapters. All station tools now open through `tools/station_serial.safe_open_serial`
with `--dtr low --rts low --write-timeout-s 1` by default, and print
`dtr_mode/rts_mode/rtscts/dsrdtr`. After the fix, a real `write-only` run on
`/dev/cu.usbserial-02442CA5` reached `tx_count>0`, `serial_error_count=0`,
verdict `STATION_TX_OK_NO_RX` (TX healthy, no RF back) — not unstable. Treat
`DEVICE_NOT_CONFIGURED` during write as DTR/RTS / adapter-reset behavior, not a
code or firmware fault.

## Breadboard Development Status (2026-05-30)

A breadboard rig (no rover chassis, no motors) is used for safe indoor
navigation-stack development with GPS, IMU, and HC-12 connected.

- IMU is now readable. `firmware/imu_probe` reports `bus_state=RELEASED_HIGH`,
  `i2c_scan_count=1`, `i2c_addr=0x69`, `whoami=0x6F`, `imu_present=true`. `0x6F`
  is not a standard InvenSense/ST ID; it is treated as an MPU register-map
  compatible clone/variant for **signal validation only**. IMU yaw/heading is
  NOT trusted yet (needs calibration + drift checks). The rover-mounted IMU may
  be different/faulty and must be validated separately.
- GPS indoors/under a roof is weak or unavailable. `gps_uart_probe` keeps
  printing raw status (`gps_chars`, `last_rmc_status`, `last_gga_fix_quality`,
  `gps_block_reason`, etc.) even with no fix; expect `gps_block_reason=NO_BYTES`
  or `NO_NMEA_FIX`/`LOW_SATS` indoors. Path-planning and path-following dry-run
  fall back to mock/compile-time coordinates indoors.
- HC-12 link validation uses `firmware/hc12_link_probe` (firmware) plus
  `tools/hc12_link_probe.py` (station). Both are motor-free PING/PONG tools.
- UART coexistence: OpenRB-150 has three hardware UARTs — `Serial1` (D26/D27),
  `Serial2` (D28/D29, GPS), `Serial3` (D14/D13). GPS and HC-12 both default to
  `Serial2` in firmware, so they cannot run together on the current rover
  wiring; the firmware disables HC-12 in every GPS-on-`Serial2` build. For
  GPS + HC-12 together on the breadboard, wire HC-12 to `Serial1` or `Serial3`
  and select it (`HC12_PROBE_SERIAL_PORT`, `PATH_FOLLOWING_HC12_SERIAL_PORT`).
- Station path-planning preview (`tools/path_planning_preview.py`) is allowed
  indoors with manual coordinates. Firmware path-following dry-run
  (`PATH_FOLLOWING_DRYRUN=1`) computes distance/bearing/heading/steering and runs
  the HC-12 waypoint protocol with motors disabled.
- Physical path following remains blocked. The four motion gates
  (`PHYSICAL_PATH_FOLLOWING_ENABLE`, `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT`,
  `GROUND_CRAWL_TEST_MODE`, `AUTO_MOTION_ARMED`) and the
  `PATH_FOLLOWING_MODE_CHANNEL_STABLE` acknowledgement all default to `0`, so no
  build moves motors. The RC/PPM Manual/Auto channel blocker (CH5 did not hold
  HIGH) and the untrusted heading source must be resolved first.

## Confirmed working

- OpenRB-150 USB debug: working
- RC receiver PPM input: working
- RC manual mode: working
- RC manual sign convention: working with `MANUAL_FORWARD_SIGN=-1`,
  `MANUAL_TURN_SIGN=1`, `MOTOR_OUTPUT_SWAP_LR=0`, and
  `DRIVE_CALIBRATION_ENABLE=0` for the current uncalibrated baseline. This is
  only an RC axis sign fix; physical A/B output mapping remains
  `A=(L+R)/2`, `B=(R-L)/2`.
- Failsafe STOP behavior: working
- GPS UART receive: working on `Serial2` at `9600` with current central
  connector wiring
- GPS module baudrate: confirmed `9600`
- GPS NMEA output: working; `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, and
  `$GPGLL` observed
- GPS FIX: confirmed on the current `Serial2/9600` probe path when the
  rover/GPS is moved farther outdoors with clearer sky view. Latest
  `gps_uart_probe` logs reached `gps_probe_state=STABLE_FIX`,
  `current_valid_fix=true`, RMC `A`, GGA fix quality `2`, `sats=9`,
  `hdop=3.56`, and `valid_fix_seconds_consecutive=58..60`.
- Main-controller GPS readiness is also recovered outdoors. In the
  single-waypoint experiment with `AUTO_MOTION_ARMED=0`, USBDBG showed
  `gps_location_valid=true`, `gps_location_fresh=true`, `gps_solution_valid=true`,
  `gps_dryrun_ready=true`, `gps_motion_ready=true`, `gps_ready=true`,
  `gps_block_reason=OK`, RMC `A`, GGA fix quality `2`, `gps_sats≈9..11`, and
  `gps_hdop≈1.46`.
- Integrated GPS `Serial2` diagnostic build: confirmed `gps_chars` increase,
  `gps_fix=true`, valid latitude/longitude, valid satellites/HDOP, HC-12
  disabled, and motors neutral
- Unified fixed-wiring RC + GPS autonomy dry-run build: compiles with
  `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1`; GPS uses `Serial2`, HC-12 is
  disabled, RC MANUAL can drive, and AUTO forces neutral while computing
  readiness only
- Unified dry-run validation: MANUAL mode works with
  `control_source=RC_MANUAL` and stick-controlled `left_cmd` / `right_cmd`;
  AUTO mode prints `autonomy_dryrun=true`, GPS fields, target
  distance/bearing, keeps `left_cmd=0` / `right_cmd=0`, and does not move motors
- Final unified dry-run USBDBG identification:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`; this confirms the running build is the unified
  MANUAL RC + AUTO GPS dry-run build, not default HC-12 mode and not GPS-only
  diagnostic mode
- Final unified dry-run AUTO observation: `mode=AUTO_READY`, `auto_sw=true`,
  `control_source=STOP`, `left_cmd=0.000`, and `right_cmd=0.000`
- Final unified dry-run MANUAL observation: `mode=MANUAL`, `auto_sw=false`,
  `control_source=RC_MANUAL`, and RC stick input changes manual command and
  left/right command fields
- Latest outdoor Manual/Auto recovery: the previous stuck-looking RC issue was
  caused by the station/controller being off. After restoring the
  controller/link, AUTO produced `mode=AUTO_READY`, `auto_sw=true`,
  `mode_us≈2001..2002`, and `control_source=STOP`; MANUAL produced
  `mode=MANUAL`, `auto_sw=false`, `mode_us≈1000..1001`, and
  `control_source=RC_MANUAL`. Manual stick input changed `steer_us` /
  `throttle_us`, and at least one MANUAL line produced nonzero
  `left_cmd` / `final_left_cmd`.
- Latest main-controller outdoor gate recovery: AUTO entry is verified again
  with `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001..2002`,
  `timeout_source=auto_entry`, and `timeout_ok=true`. `AUTO_MOTION_ARMED=0` and
  `auto_motor_inhibit=true` kept `final_left_cmd=0.000` and
  `final_right_cmd=0.000`.
- Latest no-motion AUTO waypoint dry-run is successful. With compile-time
  target `35.5705010,129.1872696`, MANUAL showed nearby
  `target_distance_m≈8.4..15.2` and `distance_allowed=true`; AUTO entry showed
  `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001`, `timeout_ok=true`,
  `safety_ready=true`, `candidate_left_cmd=0.100`, and
  `candidate_right_cmd=0.100`. Because `AUTO_MOTION_ARMED=0`,
  `auto_motor_inhibit=true` kept `final_left_cmd=0.000` and
  `final_right_cmd=0.000`.
- Latest armed AUTO output reached, but no visible motion (motor deadband
  suspected). With `AUTO_MOTION_ARMED=1` and motion-grade GPS, USBDBG showed
  `mode=AUTO_RUNNING`, `control_source=AUTO`, `auto_motor_inhibit=false`,
  `gps_motion_ready=true`, `gps_block_reason=OK`, RMC `A`, GGA quality `2`,
  `gps_sats≈7..9`, `gps_hdop≈1.0..1.4`, `distance_allowed=true`,
  `safety_ready=true`, `candidate_left_cmd=0.100`, `candidate_right_cmd=0.100`,
  and for the first time `final_left_cmd=0.100` / `final_right_cmd=0.100`.
  **No visible rover movement occurred.** Returning to MANUAL drove final
  commands to `0.000`. This is firmware-side armed output success but
  **not** a physical ground crawl. `0.100` maps to only ≈1530 µs (30 µs above
  the 1500 µs neutral), almost certainly below the motor/ESC/friction deadband.
  The next motion test must use the guarded ground crawl build
  (`GROUND_CRAWL_TEST_MODE=1`); the AUTO command must not be raised without the
  crawl clamp + latch stop. Floor driving remains **not** approved.
- Latest guarded ground crawl 0.08 safety validation: with
  `GROUND_CRAWL_TEST_MODE=1`, `AUTO_MOTION_ARMED=1`, and
  `GROUND_CRAWL_MAX_CMD=0.08`, a good GPS window reached `AUTO_RUNNING` with
  `gps_motion_ready=true`, `gps_sats=5`, `gps_hdop≈1.34`, and
  `gps_block_reason=OK`. USBDBG showed `ground_crawl_ready=true`,
  `ground_crawl_block_reason=OK`, `candidate_left_cmd=0.100`,
  `candidate_right_cmd=0.100`, and clamped final commands
  `final_left_cmd=0.080` / `final_right_cmd=0.080`. The latch stop then
  asserted `ground_crawl_latched_stop=true` and forced zero output. Later
  target distance dropped to `≈3.9..4.4` m, below
  `GROUND_CRAWL_MIN_TARGET_DISTANCE_M=5.0`, and the harness correctly blocked
  as `DISTANCE_OUT_OF_RANGE`. Intermittent GPS degradation also blocked motion
  as `GPS_NOT_MOTION_READY` or `LATCHED_STOP`. This validates the guarded crawl
  safety harness, but it is still **not** full autonomous driving.
- Historical guarded crawl 0.12 cap-only observation: `GROUND_CRAWL_MAX_CMD=0.120`
  allowed the harness to cap up to 0.120, but the candidate command was still
  `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100`, so final commands
  remained `final_left_cmd=0.100` / `final_right_cmd=0.100`. The latch still
  worked. Firmware now separates candidate speed from final clamp with
  `SINGLE_WP_CRAWL_BASE_CMD` (default `0.100`).
- First successful guarded AUTO crawl after the manual/drive mapping fix:
  MANUAL RC is working with `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`, and
  `old_angle_remap_active=false`; physical output mapping remains A = throttle,
  B = turn, `A=(logical_left+logical_right)/2`,
  `B=(logical_right-logical_left)/2`. With `SINGLE_WP_CRAWL_BASE_CMD=0.220` and
  `GROUND_CRAWL_MAX_CMD=0.220`, the rover briefly moved forward in
  `AUTO_RUNNING`. USBDBG showed `gps_motion_ready=true`,
  `gps_block_reason=OK`, `gps_sats≈9`, `gps_hdop≈1.0..1.2`,
  `target_distance_m≈9.6`, `distance_allowed=true`,
  `ground_crawl_ready=true`, `ground_crawl_block_reason=OK`,
  `left_cmd=0.220`, `right_cmd=0.220`, `final_left_cmd=0.220`,
  `final_right_cmd=0.220`, `physical_a_cmd=0.220`, and `physical_b_cmd=0.000`.
  The latch then stopped output at roughly `ground_crawl_elapsed_ms=510` with
  `ground_crawl_latched_stop=true`. This confirms short guarded forward motion,
  not full autonomous waypoint or coverage driving.
- Repeated 1000 ms guarded AUTO crawl: tested with `GROUND_CRAWL_TEST_MODE=1`,
  `GROUND_CRAWL_MAX_CMD=0.220`, `GROUND_CRAWL_MAX_AUTO_MS=1000`,
  `SINGLE_WP_CRAWL_BASE_CMD=0.220`, `AUTO_MOTION_ARMED=1`,
  `MANUAL_FORWARD_SIGN=-1`, and `MANUAL_TURN_SIGN=1`. The user toggled
  AUTO/MANUAL about `3..4` times, and `AUTO_RUNNING` was observed multiple
  times. Valid AUTO windows showed `gps_block_reason=OK`,
  `gps_motion_ready=true`, `distance_allowed=true`, `ground_crawl_ready=true`,
  `ground_crawl_block_reason=OK`, `left_cmd=0.220`, `right_cmd=0.220`,
  `final_left_cmd=0.220`, `final_right_cmd=0.220`, `physical_a_cmd=0.220`,
  and `physical_b_cmd=0.000`. GPS quality was acceptable with `gps_sats≈8..10`,
  `gps_hdop≈1.0..1.65`, and `last_gga_fix_quality=2`. The latch stopped output
  after roughly `1000` ms; one attempt was shorter because the user returned to
  MANUAL early. `target_distance_m` varied around `16.8..18.0`, which is
  expected because this guarded crawl only drives straight with
  `physical_b_cmd=0.000` and has no steering correction yet.
- Single-waypoint steering dry-run diagnostics are now available behind
  `SINGLE_WP_STEERING_DRYRUN=1`. The mode estimates course-over-ground only
  after at least `SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M` of GPS displacement
  (default `2.0` m) and prints desired forward/turn, logical wheel, and physical
  A/B commands. It does not drive motors by itself. If movement is too small,
  USBDBG reports `heading_ready=false` and `steering_block_reason=NO_HEADING`.
- The course displacement threshold is now compile-time configurable via
  `-DCOURSE_MIN_DISPLACEMENT_M=<meters>`. A USB-tethered diagnostic build can use
  `-DCOURSE_MIN_DISPLACEMENT_M=1.0` so heading can be estimated over the shorter
  movement available while tethered (recent logs reach ~1.7 m). USBDBG now also
  prints `course_min_displacement_source` (the configured macro) alongside the
  active `course_min_displacement_m`, and the startup banner prints
  `SINGLE_WP_COURSE_MIN_DISPLACEMENT_M` / `SINGLE_WP_COURSE_MIN_DISPLACEMENT_SOURCE`.
  This only changes when course-over-ground is estimated; it does NOT weaken
  actual GPS motion safety thresholds — `gps_motion_min_sats`,
  `gps_motion_max_hdop`, and `gps_motion_stale_ms` are unchanged — and it does
  not enable motor execution.
- Motor pulse calibration mode is now available for GPS-independent motor
  deadband checks. Compile with `MOTOR_PULSE_TEST_MODE=1`; HC-12 is disabled,
  GPS readiness/target distance are not used, RC MANUAL is preserved, and AUTO
  emits one neutral-stick pulse for `MOTOR_PULSE_MS` before latching stop until
  MANUAL.
- In `MOTOR_PULSE_TEST_MODE=1`, GPS is intentionally not initialized or read.
  USBDBG fields such as `gps_chars=0`, `last_rmc_status=NA`,
  `last_gga_fix_quality=NA`, and `gps_block_reason=NO_LOCATION` are expected in
  that mode and must not be interpreted as GPS hardware failure. Use
  `gps_uart_probe` or the no-motion single-waypoint main-controller build
  (`FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`,
  `AUTO_MOTION_ARMED=0`) for GPS validation.
- Current drivetrain calibration state: `MOTOR_PULSE_CMD=0.180` produced valid
  software output but no visible motion; `MOTOR_PULSE_CMD=0.220` produced visible
  motion. The 0.22 log showed symmetric software output (`left_cmd=0.220`,
  `right_cmd=0.220`, `motor_pulse_ready=true`,
  `motor_pulse_block_reason=OK`), but the rover appeared to rotate rather than
  drive straight. Manual RC forward motion tends to drift/curve left, and
  backward motion tends to drift/curve right. This suggests right-side drive may
  be stronger than left-side drive, or the left side may have higher friction or
  deadband. Differential motor pulse support is now available with
  `MOTOR_PULSE_LEFT_CMD` / `MOTOR_PULSE_RIGHT_CMD`, and a shared drive
  calibration layer is available behind `DRIVE_CALIBRATION_ENABLE=1` for both
  MANUAL and AUTO paths.
- Latest differential pulse observations: left-only `+0.22` rotates the left
  wheel forward; right-only `+0.22` rotates the right wheel forward and curves
  left as expected; both `+0.22/+0.22` rotate both wheels forward but curve/rotate
  right; both `-0.22/-0.22` rotate backward but curve left while reversing. Code
  inspection confirms motor pulse output bypasses RC stick angle remapping, but
  later `+0.25` direct-pulse tests showed the physical PWM inputs behave like
  steer/throttle rather than direct left/right wheel outputs. Firmware now routes
  motor pulse AUTO through `applyMotorPulseDirectWheelCommand(...)`, which logs
  `logical_*`, `calibrated_*`, `output_*`, and `output_*_pin_cmd` stages. Next
  step is to re-run direct left-only/right-only pulse validation before any
  scale compensation.
- Latest pin-path status: `logical_left_cmd` and `logical_right_cmd` are correct.
  `firmware/physical_output_pin_probe` confirmed physical output channel A is
  throttle and physical output channel B is turn. The integrated controller now
  converts logical wheel commands with `A=(left+right)/2` and
  `B=(right-left)/2`. `output_left_pin_cmd` / `output_right_pin_cmd` remain
  compatibility aliases for physical A/B pin commands, not wheel-side commands.
- Final unified dry-run GPS observation: `gps_chars` increases continuously,
  open-sky antenna placement produced `gps_fix=true`, and
  `target_distance_m` / `target_bearing_deg` were computed
- Latest outdoor single-waypoint safety state: GPS was usable at several
  points (`gps_fix=true`, `gps_ready=true` when HDOP was good), but the
  compile-time target `35.570675,129.186769` was stale while runtime GPS was
  around `35.5716,129.1875`. `target_distance_m≈100..131` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false`,
  `safety_ready=false`, candidate commands stayed zero, and
  `AUTO_MOTION_ARMED=0` / `auto_motor_inhibit=true` kept AUTO final commands at
  zero.
- Latest outdoor nearby dry-run: target override worked with
  `SINGLE_WP_TARGET_LAT=35.5707680` and
  `SINGLE_WP_TARGET_LON=129.1867906`; runtime printed
  `target_lat=35.570768`, `target_lon=129.186791`. Outdoor GPS quality was good
  in many lines (`gps_ready=true`, `gps_sats=7..8`, `gps_hdop≈0.95..1.98`), and
  `target_distance_m` dropped below `30.0` m, so `distance_allowed=true` was
  observed. The run was still blocked because mode stayed mostly MANUAL,
  `auto_sw=false`, `timeout_ok=false`, `safety_ready=false`, and candidate
  commands stayed zero.
- Latest outdoor dry-run after timeout fix: timeout diagnostics printed
  `timeout_source=auto_entry`, `auto_entry_ms=NA`, `auto_elapsed_ms=NA`,
  `timeout_limit_ms=15000`, and `timeout_ok=true`, so the MANUAL-wait timeout
  issue appears improved. Target override was confirmed with
  `target_lat=35.570834`, `target_lon=129.186958`, and `target_ready=true`.
  RC remained in MANUAL (`mode_us≈1000..1001`, `control_source=RC_MANUAL`).
  GPS UART was alive (`gps_chars` increased), but no fix was acquired:
  `gps_fix=false`, `gps_lat=NA`, `gps_lon=NA`, `gps_sats=0`,
  `gps_hdop=99.99`, and `gps_age_ms=NA`. Therefore
  `target_distance_m=NA`, `distance_allowed=false`, `safety_ready=false`, and
  candidate/final commands stayed zero.
- GPS sky-fix validation: previous `gps_sats=0` and `gps_hdop=99.99` was poor
  indoor/window-side reception, not UART or firmware failure; moving the
  external antenna farther outside into open sky produced fix
- Purple module: fixed wiring is believed to be SDA on OpenRB D11 / PA08 and
  SCL on OpenRB D12 / PA09; OpenRB-150 variant files confirm D11/D12 are the
  board's default `Wire` pins, but IMU I2C presence remains unverified
- Fixed Wiring Plan: GPS remains on the central connector / `Serial2`; HC-12
  remains physically mounted as-is until its current wiring is audited

## GPS Antenna Frame Vs Rover Body Frame

Observed sensor-frame issue:

- During sky-view and single-waypoint candidate dry-run tests, the external GPS
  antenna was placed far outside while the rover body remained indoors.
- Therefore `gps_lat` / `gps_lon` represented the antenna location, not the
  rover body location.

Interpretation:

- This setup is valid for GPS reception, UART, satellite fix, and
  distance/bearing computation validation.
- This setup is not valid for floor navigation or rover body localization.
- An IMU cannot fully correct a detached GPS antenna into rover body position.
- The IMU may help later with heading and rotation sensing, but it does not
  replace a rover-mounted GPS position source.

Requirement for real navigation:

- Mount the GPS antenna rigidly on the rover, or use a fixed, measured antenna
  offset from the rover body frame.
- Do not approve `AUTO_MOTION_ARMED=1` floor testing until this is resolved.

## Confirmed RC debug state

Latest default firmware restore check:

```text
fixed_wiring_gps_serial2_diag=false
hc12_enabled=true
gps_chars=0
mode=AUTO_READY
auto_sw=true
mode_us≈2000
station_deadman=false
control_source=STOP
```

Interpretation:

- Default `openrb_robot_controller` was restored successfully.
- `gps_chars=0` is expected in the default build under current fixed wiring
  because default firmware still reads GPS from `Serial3` while GPS is fixed on
  `Serial2`.
- Manual driving appears stopped because the RC mode switch is currently in
  AUTO. To validate manual control, switch RC mode out of AUTO and verify
  `control_source=RC_MANUAL`.
- `FIXED_WIRING_GPS_SERIAL2_DIAG` is for GPS testing only; manual driving does
  not work in that diagnostic build by design.

Historical manual-state example:

Observed from USBDBG:

- `mode=MANUAL`
- `rc_ok=true`
- `auto_sw=false`
- `control_source=RC_MANUAL`
- `steer_norm=0.000`
- `throttle_norm=0.000`
- `left_cmd=0.000`
- `right_cmd=0.000`

## Latest GPS Probe State

Observed from `firmware/gps_uart_probe/gps_uart_probe.ino`:

```text
selected_port=Serial2 baud=9600
chars_1s roughly 350-520
raw_preview contains $GPRMC, $GPVTG, $GPGGA, $GPGSV, $GPGLL
tinygps_chars increases
fix=true
lat around 35.57107
lon around 129.1860
sats around 5
hdop around 1.61-1.62
```

Interpretation:

- GPS UART communication is working.
- GPS satellite fix is working on the `Serial2` probe path.
- GPS should stay on the current central connector.
- Next action is auditing current HC-12 wiring, not rewiring either module.

## Latest Integrated GPS Diagnostic State

Observed from `openrb_robot_controller` with
`FIXED_WIRING_GPS_SERIAL2_DIAG=1`:

```text
fixed_wiring_gps_serial2_diag=true
hc12_enabled=false
gps_chars increased continuously
gps_fix=true
gps_lat/gps_lon appeared
gps_sats valid
gps_hdop valid
```

Safety observation:

- Motors remained disarmed/neutral.
- Manual driving does not work in this build by design.

Interpretation:

- The integrated controller can read GPS from the fixed `Serial2` wiring in the
  diagnostic build.
- Outdoor/open-sky placement is required for reliable first fix. Indoor or
  window-side tests can show increasing `gps_chars` while `gps_fix=false`.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet.
- Rain did not prevent the observed fix once the antenna had open-sky exposure,
  but electronics and antenna connectors must be protected from water.

## Historical GPS Debug State

Observed from USBDBG:

- `gps_fix=true`
- `gps_lat≈35.573188`
- `gps_lon≈129.239825`
- `gps_sats=8`
- `gps_hdop≈1.14`

## Current GPS Wiring

| GPS pin | OpenRB-150 pin |
|---|---|
| VCC / UCC | +5V |
| GND | GND |
| TX | central OpenRB connector; confirmed as `Serial2` receive path |
| RX | not connected |
| PPS | not connected |

Current status:

- Central OpenRB connector is confirmed as `Serial2`.
- HC-12 appears mounted under or behind the OpenRB board and cannot be moved
  right now.
- Whether HC-12 shares GPS `Serial2` is not yet proven.
- Historical `Serial3` D13/D14 wiring notes in older docs must be treated as a
  different wiring setup until revalidated.
- Purple module uses fixed D11/D12 I2C-style wiring and should not be treated as
  UART.
- IMU SCL is believed to be connected to OpenRB D12 / PA09.
- IMU SDA is believed to be connected to OpenRB D11 / PA08.
- OpenRB-150 variant files confirm:
  - Arduino D11 = SDA = PA08
  - Arduino D12 = SCL = PA09
  - `PIN_WIRE_SDA = 11`
  - `PIN_WIRE_SCL = 12`
  - `Wire` is constructed using `PIN_WIRE_SDA` and `PIN_WIRE_SCL`
- Do not ask to move the IMU wires to hardware SDA/SCL.
- The default `Wire.begin()` scanner is the correct primary scanner for the
  current D11/D12 wiring.
- D11/D12 and swapped D12/D11 assignments can be tested with compile-time pin
  override in the bit-bang scanner without moving wires, but that scanner is now
  secondary.
- All-address detection is scanner/bus failure, not evidence of many devices.
- Hardened bit-bang tests with both SDA=D11/SCL=D12 and SDA=D12/SCL=D11
  repeatedly reported `released_sda=LOW`, `released_scl=LOW`, `SDA stuck low`,
  `SCL stuck low`, `raw_found_count=0`, `valid_found_count=0`, and
  `stable_valid_address=NA`.
- D11/D12 stuck-low should be treated as an electrical or bus issue, such as
  IMU power, GND, pullups, or a stuck device, not as a pin mapping issue.
- Robust default `Wire` scanner result:
  - scanner runs and prints repeated scan passes
  - `pre_scan_sda=LOW`
  - `pre_scan_scl=LOW`
  - `BUS_STUCK_LOW_BEFORE_SCAN`
  - `found_count=0`
  - `stable_valid_address=NA`
- The scanner is not hanging; it is correctly refusing to scan while the bus is
  stuck low before address probing.
- IMU presence and exact device identity remain unverified.
- A dedicated IMU signal probe is available at `firmware/imu_probe`. It is the
  richer successor to the plain scanners: it scans, labels candidate families
  (`0x68`/`0x69` MPU/MPU9250/ICM, `0x0C`/`0x1C`/`0x1E` magnetometer,
  `0x28`/`0x29` BNO055, else `UNKNOWN_I2C_DEVICE`), and for a `0x68`/`0x69`
  device reads `WHO_AM_I` plus raw accel/gyro/temp. It initializes only USB
  Serial and `Wire` — no motors, GPS, HC-12, RC, or autonomy — and keeps the
  same bus-stuck-low guard, so on the current wiring it is expected to report
  `bus_state=BUS_STUCK_LOW_BEFORE_SCAN` until the I2C electrical issue (power,
  GND, pull-ups, wiring) is resolved. This probe validates IMU *signal* only;
  IMU heading/yaw is not trusted until calibration and drift checks are done.
- Continue the GPS+RC workflow without IMU support for now. Physical path
  following stays blocked until both the RC/PPM mode channel holds reliably and
  a heading source is validated; the IMU probe does not lift that blocker.

## Firmware mapping

- Integrated controller HC-12 serial: `Serial2`
- Integrated controller GPS serial: `Serial3`
- GPS probe confirmed current physical GPS path: `Serial2` at `9600`
- Possible UART allocation conflict: GPS is confirmed on `Serial2`; HC-12 may
  or may not share that path and must be audited before assumptions are made
- Previous Option A and Option B rewiring plans are superseded by the Fixed
  Wiring Plan
- USB debug baud: `115200`

## Fixed Wiring Decision Table

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

## Pending

- BLOCKER (2026-05-30): physical path following is blocked until the AUTO/MANUAL
  switch channel is stable. A standalone PPM hold test showed receiver CH5 did
  not hold HIGH: `total_ch5_samples=68`, `ch5_high_auto_like=4`,
  `ch5_low_manual_like=64`, `RESULT=CH5_AUTO_DID_NOT_HOLD`. Raw frames were
  mostly manual-like (`CH5≈1000`) with brief AUTO blips (`CH5≈2001`), plus some
  misaligned frames (`CH1=2617`, `CH3=841`) that point to occasional PPM
  channel-slip. When AUTO is raised, the firmware briefly enters `AUTO_READY`
  then `FAILSAFE` as `ppm_age_ms` grows (correct failsafe behavior).
  Use `firmware/ppm_channel_map_probe` plus `tools/analyze_ppm_log.py` to find a
  stable 2-position switch channel, then rebuild with
  `-DMODE_CHANNEL_INDEX=<0-based index>` (default `4` = CH5). The mode channel is
  now compile-time selectable and USBDBG prints `mode_channel_index`,
  `mode_channel_label`, `raw_mode_channel_us`, and `raw_ch1_us`..`raw_ch8_us`.
  Path planning preview is allowed; physical path execution is not.
- Unified RC + GPS dry-run validation is complete. This is the first mode where
  MANUAL and GPS dry-run coexist in one firmware. AUTO is still
  computation-only and real motion is not enabled yet.
- Single-waypoint candidate dry-run with `AUTO_MOTION_ARMED=0` confirms
  candidate-command safety. The earliest nearby retest was blocked by target
  override plumbing: the compile command attempted
  `SINGLE_WP_TARGET_LAT=35.5716800` and `SINGLE_WP_TARGET_LON=129.1866516`,
  while runtime USBDBG still printed `target_lat=35.571120` and
  `target_lon=129.186050`. Later builds fixed and verified target override.
- Earliest nearby candidate retest status: safe failed validation. GPS reached ready
  state on at least one line (`gps_hdop=1.19`, `gps_ready=true`), but the old
  placeholder target kept `target_distance_m` around `40` to `60` m,
  `distance_allowed=false`, `safety_ready=false`, candidate commands at zero,
  and final outputs at zero.
- Runtime `target_lat` / `target_lon` are the source of truth before
  interpreting `distance_allowed` or approving any bench test.
- Firmware source now supports `SINGLE_WP_TARGET_LAT` /
  `SINGLE_WP_TARGET_LON`; USBDBG target override was verified with
  `SINGLE_WP_TARGET_LAT=35.5710210` and
  `SINGLE_WP_TARGET_LON=129.1864016`. Runtime printed
  `target_override_enabled=true`, `target_source=compile_time`,
  `target_lat_macro=35.5710210`, `target_lon_macro=129.1864016`,
  `target_lat=35.571021`, and `target_lon=129.186402`.
- Nearby candidate command remains incomplete. GPS fix was true, but current
  GPS was around `gps_lat≈35.56752..35.56756`, `gps_lon≈129.18688`, so
  `target_distance_m≈380..392` exceeded `max_target_distance_m=30.0`.
  Therefore `distance_allowed=false`, `safety_ready=false`, candidate commands
  stayed zero, and final outputs stayed zero.
- Next-day GPS retest remained safely blocked. The antenna was placed outside
  again and runtime GPS moved to approximately `gps_lat=35.571310`,
  `gps_lon=129.188630`, while firmware still used the previous compile-time
  target `target_lat=35.567560`, `target_lon=129.186792`.
  `target_override_enabled=true` and `target_source=compile_time`, so target
  override itself is working, but `target_distance_m≈448.9` exceeded
  `max_target_distance_m=30.0`. `distance_allowed=false` and
  `safety_ready=false` were expected.
- In the next-day retest, `gps_fix=true` appeared but `gps_ready=false` remained
  because GPS freshness/quality was not stable: `gps_age_ms` was very large in
  many lines, `gps_sats` fluctuated, and `gps_hdop` was often `99.99` and only
  occasionally around `4.7`.
- `AUTO_MOTION_ARMED=0` and `auto_motor_inhibit=true` kept final outputs at
  zero.
- Latest nearby attempt remained safely blocked. Target override was verified
  with `SINGLE_WP_TARGET_LAT=35.5713100` and
  `SINGLE_WP_TARGET_LON=129.1885416`; runtime printed
  `target_lat=35.571310`, `target_lon=129.188542`. GPS UART was alive and GPS
  fix was eventually acquired in MANUAL at `gps_lat=35.571384`,
  `gps_lon=129.187514`, with `gps_sats=4` and `gps_hdop=3.39..4.12`.
  However, `target_distance_m=93.3` exceeded `max_target_distance_m=30.0`, so
  `distance_allowed=false`, `safety_ready=false`, candidate commands stayed
  zero, and final outputs stayed zero. `gps_age_ms` was initially fresh but
  later grew stale.
- Latest window/outside-antenna attempt remained safely blocked. Target
  override was verified with `SINGLE_WP_TARGET_LAT=35.5713840` and
  `SINGLE_WP_TARGET_LON=129.1874256`; runtime printed
  `target_lat=35.571384`, `target_lon=129.187426`. GPS fix appeared around
  `gps_lat≈35.571284`, `gps_lon≈129.188456`, but reception was unstable:
  `gps_sats` often became `0`, `gps_hdop` often became `99.99`, and
  `gps_age_ms` grew very large. `target_distance_m≈93.9` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false`,
  `gps_ready=false`, `safety_ready=false`, candidate commands stayed zero, and
  final outputs stayed zero.
- Latest outdoor Manual/Auto recovery is complete. The station/controller must
  be powered on for meaningful RC mode tests; controller-off can make the RC
  stream appear stuck or failsafe-like. With the link restored, MANUAL and AUTO
  switch positions were confirmed using `mode_us≈1000..1001` and
  `mode_us≈2001..2002`.
- Latest outdoor candidate dry-run remained safely blocked by stale target:
  runtime GPS was around `35.5716,129.1875`, while the target remained
  `35.570675,129.186769`. `target_distance_m≈100..131` exceeded
  `max_target_distance_m=30.0`, so `distance_allowed=false` and
  `safety_ready=false` were expected.
- Latest outdoor nearby dry-run made partial progress. With target
  `35.570768,129.186791`, GPS was repeatedly ready outdoors and
  `target_distance_m` decreased through `27.1`, `25.4`, `23.5`, `21.1`,
  `18.8`, and `18.5` m. `distance_allowed=true` was observed once the target
  was within `max_target_distance_m=30.0`.
- The same nearby dry-run is still not a successful AUTO candidate validation:
  `mode` stayed mostly `MANUAL`, `auto_sw=false`, `timeout_ok=false`,
  `safety_ready=false`, and `candidate_left_cmd=0.000` /
  `candidate_right_cmd=0.000`. Safety must be verified in `AUTO_READY`, not
  only in MANUAL.
- Single-waypoint timeout semantics have been updated in firmware. The timeout
  now starts on AUTO entry, resets when leaving AUTO, and no longer consumes
  time while waiting in MANUAL for GPS/target setup. USBDBG reports
  `timeout_source=auto_entry`, `auto_entry_ms`, `auto_elapsed_ms`,
  `timeout_limit_ms`, and `timeout_ok`.
- Latest post-timeout-fix dry-run was blocked by GPS no-fix, not timeout:
  `gps_chars` increased continuously, but `gps_fix=false`, `gps_sats=0`, and
  `gps_hdop=99.99`. Reacquire stable GPS fix before attempting AUTO_READY
  candidate validation.
- Latest GPS-only `Serial2` probe confirms the current blocker is still GPS
  satellite fix stability, not UART, RC, target override, or timeout. The probe
  was built with `GPS_PROBE_MODE=2` and `GPS_PROBE_BAUD=9600`; NMEA characters
  streamed continuously, but most lines showed RMC status `V`, GGA fix quality
  `0`, `sats=0`, and `hdop=99.99`. Short bursts reached RMC `A`, valid
  lat/lon, `sats=4..5`, and `hdop≈1.77..2.48`, then fell back to no-fix.
  Treat this as an intermittent GPS acquisition failure. TinyGPS++ cached
  coordinates after fallback to RMC `V` are not a stable current rover position.
- GPS fix recovery was confirmed after moving the rover/GPS farther outdoors.
  The same `Serial2/9600` probe then transitioned through
  `INTERMITTENT_FIX` and reached `STABLE_FIX` with
  `valid_fix_seconds_consecutive=58..60`, RMC `A`, GGA quality `2`, `sats=9`,
  `hdop=3.56`, `age_ms≈85..89`, and lat/lon around
  `35.57029,129.187078`. This attributes the previous no-fix primarily to
  placement/sky view, not firmware or UART.
- Main-controller GPS/AUTO gate recovery was confirmed in a prior run, but that
  nearby waypoint candidate dry-run was incomplete. The compile-time target
  `35.5702838,129.1869899` was stale while current GPS was around
  `35.57050,129.18736`, so `target_distance_m≈41` exceeded
  `max_target_distance_m=30.0`. `distance_allowed=false`,
  `safety_ready=false`, and candidate commands stayed zero.
- That stale-target blocker has been resolved for no-motion dry-run by
  recompiling with target `35.5705010,129.1872696`; candidate command
  generation has been observed while final motor outputs remained inhibited.
- Some AUTO dry-run lines may show motion-level `gps_ready=false` /
  `gps_block_reason=BAD_HDOP` while `safety_ready=true`. This is valid only in
  `AUTO_MOTION_ARMED=0` when the active dry-run gate is ready. It is not
  acceptable for any future armed motion.
- GPS readiness diagnostics have been hardened. USBDBG now separates
  `gps_location_valid` from `gps_ready`, prints `gps_age_ok`, `gps_sats_ok`,
  `gps_hdop_ok`, readiness constants, and `gps_block_reason`, and only prints
  operational `gps_lat` / `gps_lon` when `gps_ready=true`. Cached coordinates
  are available only as debug fields: `gps_cached_lat`, `gps_cached_lon`, and
  `gps_cached_age_ms`.
- Single-waypoint target distance and bearing are now computed only when
  `gps_ready=true`; stale cached coordinates no longer produce operational
  target distance/bearing values. The experiment also prints `gps_coord_sane`
  and blocks safety when a ready coordinate is implausibly far from the
  compile-time target.
- GPS readiness is now tiered for single-waypoint dry-run work:
  `gps_solution_valid` checks valid/fresh location plus NMEA fix status when
  available, `gps_dryrun_ready` allows no-motion candidate calculations with
  `GPS_DRYRUN_MIN_SATS=4` and `GPS_DRYRUN_MAX_HDOP=6.0`, and
  `gps_motion_ready` keeps stricter motion gating with
  `GPS_MOTION_MIN_SATS=5` and `GPS_MOTION_MAX_HDOP=2.5`. `gps_ready` remains
  the stricter motion-level field.
- In `AUTO_MOTION_ARMED=0`, target distance/bearing and candidate commands may
  be computed from `gps_dryrun_ready`; final outputs remain inhibited. In any
  future `AUTO_MOTION_ARMED=1` build, `gps_motion_ready` is required.
- A brief PPM/failsafe-like glitch was observed with `mode=FAILSAFE`,
  `rc_ok=false`, `steer_us≈495`, `throttle_us≈2504`, and
  `control_source=STOP`. This is the correct safe response.
- Window/outside-thrown antenna placement is not equivalent to rover body
  localization.
- Target override success must be interpreted separately from
  `distance_allowed` / `safety_ready`.
- Next required validation after repeated forward crawl:
  - station-side path planning preview only, with no motor execution
  - compute a fresh nearby target from the current outdoor GPS position before
    every run
  - verify RC manual override, STOP, failsafe, motion-grade GPS, near-field
    target, neutral sticks, and latch-stop before any armed variant
  - analyze GPS delta and target-distance change after each crawl
  - use single-waypoint steering dry-run before longer ground motion
  - validate heading/course estimation before physical waypoint following
  - do not enable full waypoint following or coverage driving yet
  - do not raise AUTO command outside the guarded crawl harness; use
    `-DSINGLE_WP_CRAWL_BASE_CMD` for candidate speed and
    `-DGROUND_CRAWL_MAX_CMD` for the final clamp, in small steps, under the
    crawl latch stop
  - IMU is optional for the current GPS+RC single-waypoint preparation stage;
    do not block candidate dry-run work on IMU availability
- Use the integrated GPS `Serial2` diagnostic firmware mode only for GPS USB
  debug with motors neutral and HC-12 ignored.
- Audit current HC-12 wiring from code, board inspection, and non-motion
  diagnostics.
- If safe, run receive-only station telemetry testing.
- Keep station-side path planning dry-run only.
- Future fixed-wiring architecture should support GPS `Serial2` plus RC mode
  switch behavior:
  - Auto OFF: RC manual drive
  - Auto ON: onboard GPS mission/autonomy after separate safety design
  - HC-12 unused in that mode until hardware can be revised or proven
    independent from GPS `Serial2`
- Do not add real waypoint following or weaken STOP, heartbeat, failsafe,
  manual override, RC safety, or wheel-off-ground motor-test rules.
- Station-side HC-12-USB device is not confirmed.
- `/dev/ttyUSB*` is not visible yet on the station/development side.
- Need to confirm whether station HC-12-USB is installed and connected to MPC.
