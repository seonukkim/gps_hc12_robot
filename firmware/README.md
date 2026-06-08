# Firmware Notes

- Confirm actual OpenRB-150 UART pin mapping before flashing any sketch.
- Confirm logic-level compatibility for HC-12 and GPS UART wiring.
- Treat all motor / ESC tests as wheel-off-ground only.
- Rover firmware owns low-level safety, RC override, and link-loss stopping behavior.

## Integrated Rover Controller

The active rover sketch is:

```text
firmware/openrb_robot_controller/openrb_robot_controller.ino
```

It prints this USB startup marker when the expected firmware is running:

```text
Firmware: openrb_robot_controller station-manual rc-arcade-manual-fwdneg 2026-05-30
```

## Current No-Motion Bring-Up Baseline

Validated status as of 2026-06-06:

- GPS: long-antenna GPS on the center 5-pin connector, `Serial2 @ 9600`.
  Outdoor `gps_uart_probe` reached `gps_probe_state=STABLE_FIX`,
  `current_valid_fix=true`, `gps_block_reason=OK`, `sats=6`, `hdop≈3.72`, and
  valid lat/lon. Indoor integrated logs can still show `NO_LOCATION`; that is
  expected indoors.
- IMU: BMI160 at I2C `0x68`, `chip_id=0xD1`. The normal-mode BMI160 probe and
  integrated controller both show PMU normal and plausible accel/gyro data.
- HC-12: integrated no-motion build selects `Serial3` and prints
  `hc12_enabled=true`, `hc12_port=Serial3`, `hc12_pf_port=Serial3`, but the RF
  link is not proven (`hc12_rx_count=0`, `hc12_tx_count=0`,
  `hc12_parse_ok=false`). Status is `HC12_DEFERRED_RF_LINK`.
- Motors: no-motion integrated validation must keep
  `PHYSICAL_PATH_FOLLOWING_ENABLE=0` and `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0`.
  Expected USBDBG: `physical_block_reason=COMPILE_GATE_OFF`,
  `physical_output_active=false`, `final_left_cmd=0.000`,
  `final_right_cmd=0.000`.
- RC: an indoor run can show `rc_ok=false` / `FAILSAFE` if the transmitter is
  off. Recheck MANUAL/AUTO outdoors with the transmitter powered on.

## Firmware Modes

| Mode | Compile flag | Purpose | GPS | HC-12 | Motors |
|---|---|---|---|---|---|
| Default `openrb_robot_controller` | none | HC-12/manual legacy mode | default firmware reads `Serial3`; current fixed GPS wiring is not available here | enabled | normal safety-gated behavior |
| GPS-only diagnostic | `FIXED_WIRING_GPS_SERIAL2_DIAG=1` | fixed GPS `Serial2` debug over USB | `Serial2` at `9600` | disabled/ignored | forced neutral |
| MANUAL RC + AUTO GPS dry-run | `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1` | RC manual driving plus AUTO GPS distance/bearing computation | `Serial2` at `9600` | disabled/ignored | MANUAL can drive; AUTO forced neutral |
| Single-waypoint experiment | `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1` | guarded one-target candidate-command experiment | `Serial2` at `9600` | disabled/ignored | MANUAL can drive; AUTO is neutral unless `AUTO_MOTION_ARMED=1` AND `GROUND_CRAWL_TEST_MODE=1` |
| Guarded ground crawl | `...SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=1 -DGROUND_CRAWL_TEST_MODE=1` | only path to armed motion; clamps to ±`GROUND_CRAWL_MAX_CMD` and latches stop after `GROUND_CRAWL_MAX_AUTO_MS` | `Serial2` at `9600` | disabled/ignored | MANUAL can drive; AUTO crawls clamped/latched, else neutral |
| Motor pulse calibration | `MOTOR_PULSE_TEST_MODE=1` | GPS-independent motor deadband calibration | not used | disabled/ignored | MANUAL can drive; AUTO emits one neutral-stick pulse for `MOTOR_PULSE_MS`, then latches stop until MANUAL |
| Physical output pin probe | `firmware/physical_output_pin_probe` | final PWM output pin truth-table probe | not used | not used | writes directly to physical output pin A/B after a startup delay, then neutral forever |
| Path-following dry-run | `PATH_FOLLOWING_DRYRUN=1` | breadboard waypoint distance/bearing/heading/steering diagnostics with HC-12 waypoint protocol | `Serial2` at `9600` (or mock position indoors) | motor-free waypoint protocol on `Serial3`/`Serial1` | MANUAL can drive; AUTO is neutral unless ALL four physical gates are set (default off) |

Exact macOS Arduino CLI path used in this repo:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
```

Default build compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Recommended current MANUAL RC validation build:

- `MANUAL_FORWARD_SIGN=-1`
- `MANUAL_TURN_SIGN=1`
- `MOTOR_OUTPUT_SWAP_LR=0`
- `DRIVE_CALIBRATION_ENABLE=0`

This fixes only the RC throttle-axis sign for the current controller. It does
not change motor pulse behavior, GPS behavior, or the physical A/B mapping
(`A=(L+R)/2`, `B=(R-L)/2`).

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && mkdir -p outputs/logs && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-manual-final-sign --build-property 'compiler.cpp.extra_flags=-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0' firmware/openrb_robot_controller && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-manual-final-sign firmware/openrb_robot_controller && sleep 2 && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/manual_final_sign_$(date +%Y%m%d_%H%M%S).log
```

Manual validation checklist:

- stick up = forward
- stick down = backward
- stick right = right turn
- stick left = left turn

Fixed-wiring GPS Serial2 diagnostic compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_DIAG=1' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

In this diagnostic build, GPS uses `Serial2` at `9600`, HC-12 commands are
disabled/ignored to avoid a `Serial2` conflict, and motor outputs are forced
neutral while USB debug reports GPS and RC status.

Latest GPS sky-fix validation:

- `fixed_wiring_gps_serial2_diag=true`
- `hc12_enabled=false`
- `gps_chars` increased continuously
- `gps_fix=true` after moving the external GPS antenna farther outside into
  open sky
- `gps_lat`, `gps_lon`, `gps_sats`, and `gps_hdop` became valid
- motors remained disarmed/neutral

Interpretation checklist:

- `gps_chars` increasing means the GPS UART path is OK.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet.
- Move the antenna outside/open sky before suspecting firmware.
- Protect electronics and antenna connectors from rain even if the GPS can fix
  during rainy open-sky testing.

Unified fixed-wiring RC + GPS autonomy dry-run compile/upload/monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

In this dry-run build, GPS uses `Serial2` at `9600`, HC-12 is disabled/ignored,
RC MANUAL mode keeps current manual driving behavior, and RC AUTO mode forces
neutral motor outputs while printing GPS, placeholder target, distance, bearing,
and readiness fields. This is not real waypoint following.

Expected dry-run USB debug additions:

```text
autonomy_dryrun=true target_lat=35.571120 target_lon=129.186050 target_distance_m=... target_bearing_deg=... gps_ready=... target_ready=... autonomy_ready=...
```

The dry-run distance/bearing helpers are Arduino-side only. Validate them from
USB debug output: with `gps_fix=true`, `target_distance_m` should be finite,
`target_bearing_deg` should remain in `0..360`, and AUTO mode must still keep
`left_cmd=0` and `right_cmd=0`.

Completed unified dry-run validation:

- Running build identified by USBDBG:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`.
- MANUAL mode: `mode=MANUAL`, `auto_sw=false`,
  `control_source=RC_MANUAL`, and RC stick input changes manual command and
  left/right command fields.
- AUTO mode: `mode=AUTO_READY`, `auto_sw=true`,
  `control_source=STOP`, `left_cmd=0.000`, and `right_cmd=0.000`.
- GPS: `gps_chars` increases continuously, `gps_fix=true` appears with
  open-sky antenna placement, and target distance/bearing fields are computed.
- This build still has no autonomous motor output.

Single-waypoint experiment compile/upload/monitor with motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Compile with the nearby target override and motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_TARGET_LAT=35.5716800 -DSINGLE_WP_TARGET_LON=129.1866516' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Successful no-motion AUTO dry-run command pattern:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-success-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_TARGET_LAT=35.5705010 -DSINGLE_WP_TARGET_LON=129.1872696' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-success-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected single-waypoint USB debug additions:

```text
gps_location_valid=... gps_location_fresh=... gps_solution_valid=... gps_dryrun_ready=... gps_motion_ready=... gps_age_ok=... gps_sats_ok=... gps_hdop_ok=... gps_ready=... gps_block_reason=... gps_dryrun_block_reason=... gps_motion_block_reason=... gps_dryrun_stale_ms=2000 gps_dryrun_min_sats=4 gps_dryrun_max_hdop=6.0 gps_motion_stale_ms=2000 gps_motion_min_sats=5 gps_motion_max_hdop=2.5 gps_lat=... gps_lon=... gps_cached_lat=... gps_cached_lon=... gps_cached_age_ms=... last_rmc_status=... last_gga_fix_quality=...
single_waypoint_experiment=true target_override_enabled=... target_source=... target_lat_macro=... target_lon_macro=... auto_motion_armed=false single_wp_crawl_base_cmd=... auto_motor_inhibit=true active_gps_ready=... dryrun_ready=... motion_ready=... safety_ready_source=... gps_coord_sane=... target_ready=... timeout_source=auto_entry auto_entry_ms=... auto_elapsed_ms=... timeout_limit_ms=15000 timeout_ok=... max_target_distance_m=30.0 max_coord_sanity_distance_m=1000.0 arrival_radius_m=2.5 distance_allowed=... safety_ready=... arrived=... target_lat=... target_lon=... target_distance_m=... target_bearing_deg=... candidate_left_cmd=... candidate_right_cmd=... final_left_cmd=0.000 final_right_cmd=0.000 ground_crawl_test_mode=false ground_crawl_max_cmd=0.080 ground_crawl_max_auto_ms=1200 ground_crawl_elapsed_ms=... ground_crawl_latched_stop=false ground_crawl_neutral_ok=... ground_crawl_ready=false ground_crawl_block_reason=MODE_OFF ground_crawl_min_target_distance_m=5.0 ground_crawl_max_target_distance_m=20.0 unclamped_final_left_cmd=... unclamped_final_right_cmd=...
```

### Guarded ground crawl build (armed-motion harness)

Armed AUTO motion is permitted ONLY through the guarded ground crawl harness.
On 2026-05-29 the armed build reached `final_left_cmd=0.100` /
`final_right_cmd=0.100` with all gates passing but produced no visible motion
(motor/ESC/friction deadband: `0.100` ≈ 1530 µs, 30 µs above neutral). Do NOT
raise the AUTO command ungated. Any armed build without `GROUND_CRAWL_TEST_MODE=1`
now holds final commands at zero.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-ground-crawl --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=1 -DGROUND_CRAWL_TEST_MODE=1 -DGROUND_CRAWL_MAX_CMD=0.08 -DGROUND_CRAWL_MAX_AUTO_MS=1200 -DSINGLE_WP_TARGET_LAT=35.5706361 -DSINGLE_WP_TARGET_LON=129.1870540' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-ground-crawl firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Ground crawl USB debug fields (added to the single-waypoint block):

```text
ground_crawl_test_mode=true ground_crawl_max_cmd=0.080 ground_crawl_max_auto_ms=1200 ground_crawl_elapsed_ms=... ground_crawl_latched_stop=... ground_crawl_neutral_ok=... ground_crawl_ready=... ground_crawl_block_reason=... ground_crawl_min_target_distance_m=5.0 ground_crawl_max_target_distance_m=20.0 unclamped_final_left_cmd=... unclamped_final_right_cmd=...
```

Ground crawl safety gates and stop conditions:

- `SINGLE_WP_CRAWL_BASE_CMD` controls the straight-line candidate command before
  final ground-crawl clamping. Default is `0.100`, preserving the previous
  candidate behavior.
- Final command is clamped to ±`GROUND_CRAWL_MAX_CMD` (default `0.08`).
- Latch hard-stop after `GROUND_CRAWL_MAX_AUTO_MS` (default `1200` ms) of
  continuous AUTO: forces `final_left_cmd=0.000`, `final_right_cmd=0.000`,
  `ground_crawl_latched_stop=true`. The latch clears ONLY on a return to MANUAL.
- Motion requires `ground_crawl_neutral_ok=true` (RC sticks centered),
  `gps_motion_ready=true`, `safety_ready=true`, and the target distance within
  `[5.0, 20.0]` m. Any failed gate forces zero output and reports
  `ground_crawl_block_reason` (e.g. `MODE_OFF`, `LATCHED_STOP`, `RC_NOT_NEUTRAL`,
  `GPS_NOT_MOTION_READY`, `SAFETY_NOT_READY`, `DISTANCE_OUT_OF_RANGE`, `OK`).
- `unclamped_final_left_cmd` / `unclamped_final_right_cmd` show the raw candidate
  (e.g. `0.100`) for deadband diagnosis.
- The 0.08 cap is intentionally below the observed deadband. Raise it only via
  `-DGROUND_CRAWL_MAX_CMD` in small steps, under latch protection,
  wheels-off-ground or open-area-with-kill-switch only. Ground crawl is NOT full
  autonomous driving; floor driving remains not approved.

Latest 0.08 guarded crawl result:

- `GROUND_CRAWL_TEST_MODE=1` and `AUTO_MOTION_ARMED=1` were active.
- During a good GPS window, USBDBG reached `AUTO_RUNNING` with
  `gps_motion_ready=true`, `gps_sats=5`, `gps_hdop≈1.34`, and
  `gps_block_reason=OK`.
- `ground_crawl_ready=true` and `ground_crawl_block_reason=OK` were observed.
- `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100` were clamped to
  `final_left_cmd=0.080` / `final_right_cmd=0.080`.
- After the duration limit, `ground_crawl_latched_stop=true` forced final
  commands to zero.
- When the target later became too close (`target_distance_m≈3.9..4.4`), the
  harness blocked as `DISTANCE_OUT_OF_RANGE` because the crawl minimum is
  `5.0` m.
- Intermittent GPS degradation also blocked as `GPS_NOT_MOTION_READY` or
  `LATCHED_STOP`.

For any future guarded crawl run, reacquire current GPS and compute a fresh
target inside the crawl window. The historical 0.08/0.12 tests established that
`SINGLE_WP_CRAWL_BASE_CMD` controls candidate speed while
`GROUND_CRAWL_MAX_CMD` is only the final clamp. Keep the latch and all safety
gates enabled.

Latest `GROUND_CRAWL_MAX_CMD=0.120` cap-only result:

- Guarded crawl reached `AUTO_RUNNING` with `ground_crawl_ready=true` and
  `ground_crawl_block_reason=OK`.
- `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100` remained at the
  default candidate speed.
- `final_left_cmd=0.100` / `final_right_cmd=0.100` confirmed that a higher cap
  alone does not raise the candidate command.
- The next 0.12 attempt should compile with both
  `-DSINGLE_WP_CRAWL_BASE_CMD=0.12` and `-DGROUND_CRAWL_MAX_CMD=0.12`.

First successful guarded AUTO crawl after the manual/drive mapping fix:

- Manual RC baseline was corrected with `MANUAL_FORWARD_SIGN=-1`,
  `MANUAL_TURN_SIGN=1`, and `old_angle_remap_active=false`.
- Physical output mapping remains A = throttle, B = turn:
  `A=(logical_left+logical_right)/2`, `B=(logical_right-logical_left)/2`.
- Guarded crawl build used `GROUND_CRAWL_TEST_MODE=1`,
  `AUTO_MOTION_ARMED=1`, `SINGLE_WP_CRAWL_BASE_CMD=0.220`, and
  `GROUND_CRAWL_MAX_CMD=0.220`.
- A successful `AUTO_RUNNING` window showed `gps_motion_ready=true`,
  `gps_block_reason=OK`, `gps_sats≈9`, `gps_hdop≈1.0..1.2`,
  `target_distance_m≈9.6`, `distance_allowed=true`,
  `ground_crawl_ready=true`, and `ground_crawl_block_reason=OK`.
- Straight forward output was confirmed in USBDBG:
  `left_cmd=0.220`, `right_cmd=0.220`, `final_left_cmd=0.220`,
  `final_right_cmd=0.220`, `physical_a_cmd=0.220`, `physical_b_cmd=0.000`.
- The rover moved briefly forward.
- The safety latch worked: at roughly `ground_crawl_elapsed_ms=510`,
  `ground_crawl_latched_stop=true` and final outputs returned to zero.

This is the first successful short guarded forward crawl. It is not full
waypoint following and not coverage driving.

Repeated 1000 ms guarded AUTO crawl result:

- Build flags included `GROUND_CRAWL_TEST_MODE=1`,
  `GROUND_CRAWL_MAX_CMD=0.220`, `GROUND_CRAWL_MAX_AUTO_MS=1000`,
  `SINGLE_WP_CRAWL_BASE_CMD=0.220`, `AUTO_MOTION_ARMED=1`,
  `MANUAL_FORWARD_SIGN=-1`, and `MANUAL_TURN_SIGN=1`.
- The user toggled AUTO/MANUAL about `3..4` times.
- `AUTO_RUNNING` was observed multiple times.
- In valid AUTO windows, USBDBG showed `gps_block_reason=OK`,
  `gps_motion_ready=true`, `distance_allowed=true`,
  `ground_crawl_ready=true`, and `ground_crawl_block_reason=OK`.
- GPS quality was acceptable: `gps_sats≈8..10`, `gps_hdop≈1.0..1.65`, and
  `last_gga_fix_quality=2`.
- Straight output repeated as expected:
  `left_cmd=0.220`, `right_cmd=0.220`, `final_left_cmd=0.220`,
  `final_right_cmd=0.220`, `physical_a_cmd=0.220`, `physical_b_cmd=0.000`.
- The latch stopped output after roughly `1000` ms:
  `ground_crawl_latched_stop=true` and final outputs returned to zero.
- One AUTO attempt was shorter because the user returned to MANUAL early.
- `target_distance_m` varied around `16.8..18.0` instead of monotonically
  decreasing. This is expected because this mode only drives straight with
  `physical_b_cmd=0.000`; it does not steer toward the target yet.

This proves repeated short guarded autonomous forward actuation. It is still not
path planning execution. The next stage is station-side path planning preview
only with no motor execution, then single-waypoint steering dry-run, then
heading/course estimation before any physical waypoint following.

### Single-Waypoint Steering Dry-Run

`SINGLE_WP_STEERING_DRYRUN=1` adds diagnostics only. It does not drive motors by
itself and does not change `GROUND_CRAWL_TEST_MODE` or `AUTO_MOTION_ARMED`
safety behavior.

Purpose:

- GPS position provides bearing to target, but not rover heading by itself.
- The firmware estimates course-over-ground only after enough GPS displacement.
- The minimum displacement is `SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M`, which
  defaults to `2.0` m and is now compile-time configurable via
  `-DCOURSE_MIN_DISPLACEMENT_M=<meters>`.
- If displacement is below that threshold, USBDBG reports `heading_ready=false`
  and `steering_block_reason=NO_HEADING`.
- USBDBG prints both the active value (`course_min_displacement_m`) and the
  configured macro source (`course_min_displacement_source`). The startup banner
  prints `SINGLE_WP_COURSE_MIN_DISPLACEMENT_M` and
  `SINGLE_WP_COURSE_MIN_DISPLACEMENT_SOURCE`.
- Do not use `target_bearing_deg` alone as a motor steering command.

Lowering the displacement threshold only relaxes when the steering dry-run is
willing to *estimate* course-over-ground. It does NOT weaken actual GPS motion
safety thresholds: `gps_motion_min_sats`, `gps_motion_max_hdop`, and
`gps_motion_stale_ms` are unchanged, and it does not enable motor execution.

Compile a no-motion steering diagnostic build (default 2.0 m threshold):

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-steering-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_STEERING_DRYRUN=1 -DSINGLE_WP_TARGET_LAT=35.570932 -DSINGLE_WP_TARGET_LON=129.187338' firmware/openrb_robot_controller
```

To exercise heading estimation over shorter, USB-tethered displacement, add
`-DCOURSE_MIN_DISPLACEMENT_M=1.0`:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-steering-dryrun-course-1m --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_STEERING_DRYRUN=1 -DCOURSE_MIN_DISPLACEMENT_M=1.0 -DSINGLE_WP_TARGET_LAT=35.570932 -DSINGLE_WP_TARGET_LON=129.187338' firmware/openrb_robot_controller
```

Expected USBDBG fields:

- `single_wp_steering_dryrun`
- `current_gps_lat`, `current_gps_lon`
- `steering_target_lat`, `steering_target_lon`
- `target_distance_m`, `target_bearing_deg`
- `heading_ready`, `heading_source`
- `course_min_displacement_m`, `course_min_displacement_source`
- `course_displacement_m`
- `estimated_course_deg`, `bearing_error_deg`
- `desired_forward_cmd`, `desired_turn_cmd`
- `desired_logical_left_cmd`, `desired_logical_right_cmd`
- `desired_physical_a_cmd`, `desired_physical_b_cmd`
- `steering_block_reason`

### Motor pulse calibration build

Use this mode only for GPS-independent motor deadband calibration. It does not
use GPS readiness, waypoint target distance, or HC-12. RC MANUAL mode still
drives normally. AUTO emits one pulse only when `rc_ok=true` and
steering/throttle are neutral, then latches stop until the operator returns to
MANUAL. USBDBG runs at `100` ms in this mode so the short pulse window is
observable.

Current calibration result:

- `MOTOR_PULSE_CMD=0.180` produced valid software output but no visible physical
  motion.
- `MOTOR_PULSE_CMD=0.220` produced visible physical motion.
- The 0.22 log showed symmetric software output:
  `left_cmd=0.220`, `right_cmd=0.220`, `motor_pulse_ready=true`,
  `motor_pulse_block_reason=OK`.
- Physical motion looked more like rotation than straight forward motion.
- Manual RC forward tends to drift/curve left; backward tends to drift/curve
  right.
- Differential pulse observations:
  - left-only `+0.22`: left wheel rotates forward;
  - right-only `+0.22`: right wheel rotates forward and the rover curves left as
    expected;
  - both `+0.22/+0.22`: both wheels rotate forward but the rover curves/rotates
    right;
  - both `-0.22/-0.22`: both wheels rotate backward but the rover curves left
    while reversing.
- A later `+0.25` direct-pulse retest showed a direct-output-path problem:
  left-only `+0.25/0.00` also moved the right wheel backward, and right-only
  `0.00/+0.25` also moved the left wheel backward. Treat that as a direct wheel
  command validation failure until the staged USBDBG fields prove otherwise.
- Motor pulse output bypasses RC stick angle remapping. In pulse mode, RC
  steering/throttle are used only for the neutral precondition.
- After the 2026-05-30 differential retest, `MOTOR_PULSE_LEFT_CMD` and
  `MOTOR_PULSE_RIGHT_CMD` must be treated as direct logical wheel commands, not
  steering/throttle inputs. Verify this with the staged USBDBG fields below
  before using any calibration result.
- In `MOTOR_PULSE_TEST_MODE` AUTO pulse, firmware now calls
  `applyMotorPulseDirectWheelCommand(...)`. That applies optional drive
  calibration once, optional `MOTOR_OUTPUT_SWAP_LR` once, then converts the
  final direct wheel command to the current physical PWM inputs. The current
  motor controller inputs behave like steer/throttle, so a left-only wheel
  command is written as a combined steer/throttle pair internally.

Differential pulse and shared drive calibration support is now available:

- `MOTOR_PULSE_LEFT_CMD` and `MOTOR_PULSE_RIGHT_CMD` override the left/right
  pulse independently. If omitted, both default to `MOTOR_PULSE_CMD` for
  backward compatibility.
- `logical_left_cmd` / `logical_right_cmd` are direct pre-swap wheel commands.
- `calibrated_left_cmd` / `calibrated_right_cmd` are after the optional drive
  calibration layer.
- `output_left_cmd` / `output_right_cmd` are the final commands sent to the
  physical left/right motor outputs.
- `output_left_pin_cmd` / `output_right_pin_cmd` are the actual PWM channel
  commands. They are compatibility names for physical A/B pins, not physical
  left/right wheel outputs.
- Physical pin A is throttle / forward-backward. Physical pin B is turn /
  steering. The probe-confirmed wheel model is `left = A - B`,
  `right = A + B`, so the integrated controller converts logical wheel commands
  with `A = (left + right) / 2` and `B = (right - left) / 2`.
- `MOTOR_OUTPUT_SWAP_LR=1` swaps logical left/right wheel commands before the
  physical A/B conversion. The default is `0`; do not enable it unless a direct
  left/right pulse proves the logical wheel sides are reversed.
- `DRIVE_CALIBRATION_ENABLE=1` enables the shared calibration layer used by RC
  MANUAL, station manual, single-waypoint AUTO, and motor pulse output.
- Identity defaults preserve current behavior:
  - `LEFT_MOTOR_SIGN=1`, `RIGHT_MOTOR_SIGN=1`
  - `LEFT_MOTOR_SCALE=1.0`, `RIGHT_MOTOR_SCALE=1.0`
  - `LEFT_MOTOR_MIN_CMD=0.0`, `RIGHT_MOTOR_MIN_CMD=0.0`
- Minimum command compensation applies only when the raw command is nonzero.
  A raw `0.0` always remains `0.0`.

Do not tune drivetrain asymmetry in GPS path planning.

### Physical output pin probe

Use this before any further motor scale, sign, or minimum-command tuning. Recent
logs show `MOTOR_PULSE_LEFT_CMD` and `MOTOR_PULSE_RIGHT_CMD` reach
`logical_left_cmd` and `logical_right_cmd` correctly. The probe confirmed the
physical pin roles:

- physical pin A (`ESC_LEFT_PIN`, OpenRB D4): throttle / forward-backward
- physical pin B (`ESC_RIGHT_PIN`, OpenRB D5): turn / steering
- B positive means right wheel forward and left wheel backward

The standalone probe bypasses RC, GPS, HC-12, station commands, manual mixing,
waypoint logic, logical wheel conversion, and drive calibration. It attaches the
same Servo PWM outputs as the integrated controller:

- physical pin A: OpenRB D4, same output as `ESC_LEFT_PIN`
- physical pin B: OpenRB D5, same output as `ESC_RIGHT_PIN`
- neutral: `1500 us`
- command range: ±`300 us`

Compile-time options:

- `PHYSICAL_PIN_A_CMD`, default `0.0`
- `PHYSICAL_PIN_B_CMD`, default `0.0`
- `PHYSICAL_PIN_PROBE_MS`, default `500`
- `PHYSICAL_PIN_PROBE_START_DELAY_MS`, default `3000`

Expected USB fields include:

```text
physical_output_pin_probe=true phase=WAIT/PULSE/STOP elapsed_ms=... physical_pin_a_cmd=... physical_pin_b_cmd=... physical_pin_a=4 physical_pin_b=5 written_pin_a_cmd=... written_pin_b_cmd=...
```

Truth-table rule:

- Pin A alone creates straight forward/reverse motion, so pin A is throttle.
- Pin B alone creates in-place turn/spin, so pin B is steering/turn.
- The integrated wheel-to-pin conversion is now
  `throttle = (left + right) / 2` and `turn = (right - left) / 2`.
- Single-wheel logical commands are split across throttle and turn. For example,
  logical left-only `+0.50/0.00` becomes physical `A=+0.25`, `B=-0.25`. This
  can still be near deadband, so validate the mapping first with both-wheel
  forward/reverse tests.

Important GPS interpretation:

- `MOTOR_PULSE_TEST_MODE=1` intentionally skips `GPS_SERIAL.begin(...)` and the
  GPS read loop.
- Therefore `gps_chars=0`, `last_rmc_status=NA`, `last_gga_fix_quality=NA`, and
  `gps_block_reason=NO_LOCATION` are expected in this build.
- Those fields must not be used to judge GPS health. Validate GPS either with
  `firmware/gps_uart_probe` or with the no-motion main-controller GPS build
  below.

Recommended no-motion main-controller GPS validation:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-validation-nomotion --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-validation-nomotion firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected GPS fields in that build include increasing `gps_chars`,
`last_rmc_status`, `last_gga_fix_quality`, `gps_lat`, `gps_lon`, `gps_sats`,
`gps_hdop`, and readiness tiers. Motor outputs remain inhibited in AUTO because
`AUTO_MOTION_ARMED=0`.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-motor-pulse-018 --build-property 'compiler.cpp.extra_flags=-DMOTOR_PULSE_TEST_MODE=1 -DMOTOR_PULSE_CMD=0.18 -DMOTOR_PULSE_MS=300' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-motor-pulse-018 firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected USB debug fields:

```text
logical_left_cmd=... logical_right_cmd=... raw_left_cmd=... raw_right_cmd=... calibrated_left_cmd=... calibrated_right_cmd=... output_left_cmd=... output_right_cmd=... output_left_pin_cmd=... output_right_pin_cmd=... physical_a_cmd=... physical_b_cmd=... physical_a_role=throttle physical_b_role=turn wheel_to_physical_mapping=diff_to_throttle_turn final_left_cmd=... final_right_cmd=... motor_output_swap_lr=... mixer_bypassed_for_motor_pulse=... drive_calibration_enable=... left_motor_sign=... right_motor_sign=... left_motor_scale=... right_motor_scale=... left_motor_min_cmd=... right_motor_min_cmd=...
motor_pulse_test_mode=true motor_pulse_cmd=... motor_pulse_left_cmd=... motor_pulse_right_cmd=... motor_pulse_ms=... motor_pulse_elapsed_ms=... motor_pulse_latched_stop=... motor_pulse_ready=... motor_pulse_block_reason=...
```

`left_cmd` / `right_cmd` and `final_left_cmd` / `final_right_cmd` are the same
final intended wheel output values after calibration and optional output swap.
Use `logical_left_cmd` / `logical_right_cmd` or `raw_left_cmd` /
`raw_right_cmd` to inspect the pre-calibration direct wheel command. Use
`physical_a_cmd` / `physical_b_cmd` or the compatibility aliases
`output_left_pin_cmd` / `output_right_pin_cmd` to inspect the actual PWM channel
commands being written.

Expected block reasons include `MODE_OFF`, `RC_INVALID`, `RC_NOT_NEUTRAL`,
`LATCHED_STOP`, and `OK`.

Target override rule:

- Runtime USBDBG `target_lat` and `target_lon` are the source of truth.
- Verify those fields before interpreting `target_distance_m`,
  `distance_allowed`, `safety_ready`, or candidate command values.
- With override macros provided, USBDBG should print
  `target_override_enabled=true`, `target_source=compile_time`,
  `target_lat_macro=35.5716800`, `target_lon_macro=129.1866516`,
  `target_lat=35.571680`, and `target_lon=129.186652`.
- Without override macros, USBDBG should print `target_override_enabled=false`
  and `target_source=fallback`.
- The previous nearby target run was safe because `AUTO_MOTION_ARMED=0` kept
  final outputs at zero, but it was not a successful nearby candidate-command
  test because runtime target fields still showed the old placeholder.
- Latest check: target override plumbing is verified with
  `SINGLE_WP_TARGET_LAT=35.5710210` and
  `SINGLE_WP_TARGET_LON=129.1864016`; USBDBG printed
  `target_override_enabled=true`, `target_source=compile_time`, matching macro
  strings, and runtime `target_lat=35.571021`, `target_lon=129.186402`.
- That run was still blocked because current GPS was about `380` to `392` m
  away, greater than `max_target_distance_m=30.0`, so `distance_allowed=false`
  and `safety_ready=false` were expected. Recompute a nearby target from the
  current GPS position before the next inhibited run.
- Next-day retest: target override still worked, but GPS moved to approximately
  `35.571310,129.188630` while the previous target remained
  `35.567560,129.186792`, making `target_distance_m≈448.9`.
  `distance_allowed=false` and `safety_ready=false` were expected.
- `gps_location_valid=true` alone is not enough; it can be a stale TinyGPS
  cached location. Use motion-level `gps_ready=true` for armed motion, and use
  `gps_dryrun_ready=true` / `active_gps_ready=true` only for inhibited
  no-motion candidate diagnostics.
- Latest nearby attempt: target override and GPS fix worked, but the actual fix
  was `gps_lat=35.571384`, `gps_lon=129.187514` and the target was
  `35.571310,129.188542`, leaving `target_distance_m=93.3`. Recompute the
  target from the actual USBDBG GPS fix before the next inhibited run.
- Window/outside-antenna retest: target override still worked, but GPS quality
  was unstable and the antenna position was not rover body position. The run had
  `target_distance_m≈93.9`, `gps_ready=false`, `distance_allowed=false`, and
  `safety_ready=false`. Do the next inhibited run fully outdoors with rover and
  GPS fixed together, and run promptly because `timeout_ok` can expire.
- Outdoor Manual/Auto recovery: the previous RC issue was caused by the
  station/controller being off. After restoring the controller/link, AUTO was
  verified with `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001..2002`, and
  `control_source=STOP`; MANUAL was verified with `mode=MANUAL`,
  `auto_sw=false`, `mode_us≈1000..1001`, and `control_source=RC_MANUAL`.
  Outdoor GPS was usable, but the compile-time target was stale
  (`35.570675,129.186769` while runtime GPS was around `35.5716,129.1875`), so
  `target_distance_m≈100..131`, `distance_allowed=false`, and
  `safety_ready=false` were expected. Recompute the target from the current
  runtime GPS fix before the next inhibited run.
- Outdoor nearby dry-run progress: target override worked with
  `SINGLE_WP_TARGET_LAT=35.5707680` and
  `SINGLE_WP_TARGET_LON=129.1867906`; runtime printed
  `target_lat=35.570768`, `target_lon=129.186791`. Outdoor GPS was repeatedly
  ready and `target_distance_m` dropped below `30.0` m, so
  `distance_allowed=true` was observed. This was still not a successful AUTO
  candidate dry-run because mode stayed mostly MANUAL, `timeout_ok=false`,
  `safety_ready=false`, and candidate commands remained zero.
- Timeout semantics update: the single-waypoint experiment now starts the
  timeout window on AUTO entry, resets it when leaving AUTO, and does not
  consume timeout while waiting in MANUAL for outdoor GPS. In MANUAL, USBDBG
  should print `timeout_source=auto_entry`, `auto_entry_ms=NA`, and
  `auto_elapsed_ms=NA`. After switching to AUTO, `auto_entry_ms` and
  `auto_elapsed_ms` should become numeric and `timeout_ok=true` until
  `auto_elapsed_ms` exceeds `timeout_limit_ms`.
- Latest post-timeout-fix attempt: timeout fields and target override were
  confirmed, but GPS had no valid fix. `gps_chars` increased, proving serial
  input was alive, but USBDBG printed `gps_location_valid=false`, `gps_lat=NA`,
  `gps_lon=NA`, `gps_sats=0`, `gps_hdop=99.99`, and `gps_age_ms=NA`.
  Reacquire stable outdoor GPS fix before AUTO candidate validation.
- Latest GPS-only `Serial2` probe: `firmware/gps_uart_probe` was compiled with
  `GPS_PROBE_MODE=2` and `GPS_PROBE_BAUD=9600`. It received continuous NMEA
  characters, confirming UART/baud are alive, but GPS fix was intermittent.
  Most lines showed RMC `V`, GGA fix quality `0`, `sats=0`, and `hdop=99.99`.
  Short bursts reached RMC `A`, valid lat/lon, `sats=4..5`, and
  `hdop≈1.77..2.48`, then returned to no-fix. Do not proceed to AUTO dry-run,
  bench test, or floor driving until stable fix is observed.
- Follow-up placement recovery: moving the rover/GPS farther outdoors recovered
  stable fix in `gps_uart_probe`. The latest lines showed
  `gps_probe_state=STABLE_FIX`, `current_valid_fix=true`, RMC `A`, GGA quality
  `2`, `sats=9`, `hdop=3.56`, `age_ms≈85..89`, lat/lon around
  `35.57029,129.187078`, and `valid_fix_seconds_consecutive=58..60`. Treat
  this as proof that the GPS module and UART work when placement is good. It is
  not approval for floor driving.
- Main-controller outdoor validation recovered GPS and AUTO gates:
  `gps_dryrun_ready=true`, `gps_motion_ready=true`, `gps_ready=true`,
  `gps_block_reason=OK`, RMC `A`, GGA quality `2`, `gps_sats≈9..11`,
  `gps_hdop≈1.46`, `mode=AUTO_READY`, `auto_sw=true`, and `timeout_ok=true`.
  `AUTO_MOTION_ARMED=0` kept final outputs at zero. The run was still blocked
  because the compile-time target was stale and `target_distance_m≈41` was
  greater than `max_target_distance_m=30.0`; recompute a target within roughly
  `5..15` m before the next inhibited dry-run.
- No-motion AUTO waypoint dry-run is now validated with target
  `35.5705010,129.1872696`: `target_distance_m≈8.4..15.2`,
  `distance_allowed=true`, `safety_ready=true`,
  `candidate_left_cmd=0.100`, `candidate_right_cmd=0.100`,
  `auto_motor_inhibit=true`, and final outputs still `0.000`.
  Wheel-off-ground bench testing is next; floor driving remains blocked.
- GPS readiness update: readiness is tiered. `gps_solution_valid` checks
  valid/fresh location plus NMEA fix status when available. `gps_dryrun_ready`
  allows no-motion candidate calculation with `GPS_DRYRUN_MIN_SATS=4` and
  `GPS_DRYRUN_MAX_HDOP=6.0`. `gps_motion_ready` is stricter with
  `GPS_MOTION_MIN_SATS=5` and `GPS_MOTION_MAX_HDOP=2.5`; `gps_ready` remains
  motion-level. Stale cached TinyGPS coordinates are printed only as
  `gps_cached_lat`, `gps_cached_lon`, and `gps_cached_age_ms`; they must not be
  used for waypoint decisions. NMEA status diagnostics include
  `last_rmc_status` for `GPRMC` / `GNRMC` and `last_gga_fix_quality` for
  `GPGGA` / `GNGGA`.
- With `AUTO_MOTION_ARMED=0`, single-waypoint distance/bearing and candidate
  commands may be computed from `gps_dryrun_ready`, but final motor outputs
  stay zero. With `AUTO_MOTION_ARMED=1`, `gps_motion_ready` is required.

Safety gates:

- GPS solution valid.
- For no-motion dry-run: age no more than `GPS_DRYRUN_STALE_MS` (`2000` ms),
  satellites at least `GPS_DRYRUN_MIN_SATS` (`4`), and HDOP no more than
  `GPS_DRYRUN_MAX_HDOP` (`6.0`).
- For any future armed motion: age no more than `GPS_MOTION_STALE_MS`
  (`2000` ms), satellites at least `GPS_MOTION_MIN_SATS` (`5`), and HDOP no
  more than `GPS_MOTION_MAX_HDOP` (`2.5`).
- GPS coordinate sanity within `SINGLE_WAYPOINT_MAX_COORD_SANITY_DISTANCE_M`
  (`1000.0` m) of the compile-time target.
- Target available.
- RC input valid.
- RC AUTO switch on.
- Target distance above `SINGLE_WAYPOINT_ARRIVAL_RADIUS_M` (`2.5` m).
- Target distance no more than `SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M` (`30` m).
- AUTO state age no more than `SINGLE_WAYPOINT_AUTO_TIMEOUT_MS` (`15000` ms).

`AUTO_MOTION_ARMED=1` alone no longer produces motion: armed motion is gated to
zero unless `GROUND_CRAWL_TEST_MODE=1` is also set, in which case the guarded
ground crawl harness (clamp + latch + neutral/GPS/near-field gates) applies. The
armed build is reserved for an explicit wheel-off-ground / open-area bench test.
The inhibited build (`AUTO_MOTION_ARMED=0`) computes candidate commands but
forces final left/right outputs to zero. This mode does not load `mission.json`,
does not run multi-waypoint missions, and does not implement coverage/lawnmower
driving.
Candidate commands are straight low-speed placeholders; target bearing is
printed for inspection, but heading control is not implemented yet.

When uploading a compile-time variant, upload the matching build directory.

On Linux/WSL station hosts, the upload port may instead look like
`/dev/ttyACM0`. Confirm the actual port before upload.

Full manual-control bring-up notes are in
[`docs/manual_control.md`](../docs/manual_control.md).

## RC Channel Probe

Use this standalone sketch to identify which receiver PPM channel changes when
each physical RC stick or switch is moved:

```text
firmware/rc_channel_probe/rc_channel_probe.ino
```

It uses the same PPM input pin and decoding style as
`openrb_robot_controller`:

- PPM input: OpenRB `D6`
- channels printed: `ch1_us` through `ch8_us`
- frame sync: pulse width greater than `4000 us`
- interrupt edge: `FALLING` (matches the verified `firmware/rc_mix_test`
  decoder)

The probe does not attach Servo or motor outputs. It prints every `0.5` seconds:

- current channel pulse widths
- min/max observed for each channel since boot
- `changed_channels` when any channel changes by more than `100 us`

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Probe procedure:

1. Keep motors disconnected or wheels off ground.
2. Move one stick or one switch at a time.
3. Record which `chN_us` value changes and the observed min/max range.
4. The AUTO candidate is the channel that reaches around `2000 us`.
5. Do not change `MODE_CHANNEL_INDEX` in `openrb_robot_controller` until this
   raw channel probe identifies the intended physical switch.

## PPM Channel Map Probe

Use this newer standalone sketch to decide which receiver channel is a *stable
2-position* Manual/Auto switch, and to catch channel-slip / misaligned frames.
It exists because a recent PPM hold test showed receiver CH5 did not hold HIGH:
only `4` AUTO-like samples out of `68` (`ch5_high_auto_like=4`,
`ch5_low_manual_like=64`, `RESULT=CH5_AUTO_DID_NOT_HOLD`). Until a channel holds
HIGH reliably, physical path execution stays blocked.

```text
firmware/ppm_channel_map_probe/ppm_channel_map_probe.ino
```

Safety: this sketch does not initialize GPS, HC-12, motors, or autonomous
control. It only reads PPM on OpenRB `D6` and prints diagnostics at `115200`.

Unlike `rc_channel_probe`, it does not print every raw frame (that hides switch
transitions). Instead it prints:

- `PPMEVT` lines immediately when any channel changes `LOW`/`MID`/`HIGH` state.
  Each line carries `timestamp_ms`, `frame_counter`, `frame_age_ms`,
  `frame_valid`, `invalid_reason`, `channel_count`, `CH1_us`..`CH8_us`,
  per-channel `CHn_state`, `changed_channels`, and
  `possible_mode_channel_candidates`.
- `PPMSUM` lines every `1` second with `frames`, `invalid_frames`, and per
  channel `min`/`max` plus `low`/`mid`/`high` counts.

State thresholds: `LOW < 1300us`, `MID 1300..1700us`, `HIGH > 1700us`; pulses
outside `800..2200us` are flagged invalid (a common channel-slip symptom, e.g.
the observed `CH1=2617`, `CH3=841`).

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-ppm-channel-map-probe firmware/ppm_channel_map_probe
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-ppm-channel-map-probe firmware/ppm_channel_map_probe
```

Monitor and log to a file:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200 | tee outputs/logs/ppm_channel_map_$(date +%Y%m%d_%H%M%S).log
```

Analyze the captured log:

```bash
uv run python tools/analyze_ppm_log.py outputs/logs/ppm_channel_map_*.log
```

The analyzer reports per-channel min/max, how often each channel was
LOW/MID/HIGH, which channels changed, the longest HIGH hold (per-sample run and
fully-HIGH 1 s windows), and candidate AUTO/MANUAL channels. A candidate must
reach both LOW and HIGH *and* hold HIGH; the previous CH5 behavior fails this.

Procedure:

1. Keep motors disconnected or wheels off ground.
2. Flip only the suspected Manual/Auto switch, slowly, several times.
3. Watch `PPMEVT` for a channel that toggles cleanly LOW<->HIGH and a `PPMSUM`
   window where that channel is fully HIGH while held.
4. If no channel holds HIGH, the transmitter switch may be momentary or
   misassigned, or the PPM decoder is channel-slipping. Do not enable path
   following.
5. Only once a stable channel is identified, rebuild `openrb_robot_controller`
   with `-DMODE_CHANNEL_INDEX=<0-based index>` (see below).

## Selecting the mode channel in the main controller

`openrb_robot_controller` reads the Manual/Auto switch from a compile-time
0-based PPM index, `MODE_CHANNEL_INDEX`, default `4` (receiver CH5). The default
is unchanged; override it only after the probe proves a different channel is a
stable 2-position switch:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-mode-channel --build-property 'compiler.cpp.extra_flags=-DMODE_CHANNEL_INDEX=5 -DAUTO_MOTION_ARMED=0' firmware/openrb_robot_controller
```

`MODE_CHANNEL_INDEX` is a 0-based PPM index, so `5` selects receiver CH6. The
active value is printed at startup (`mode_channel_index=`) and in every USBDBG
line as `mode_channel_index=` / `mode_channel_label=CHn`. USBDBG now also prints
`raw_mode_channel_us` and `raw_ch1_us`..`raw_ch8_us` so the mode channel and any
slip are visible without a separate probe. This selection does not weaken
failsafe, GPS motion thresholds, or any motion gate.

## GPS UART Probe

Use this standalone sketch when validating GPS UART wiring and baudrate:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
```

It does not attach motor outputs. Full procedure and per-variant compile/upload
commands are in [`docs/gps_bringup.md`](../docs/gps_bringup.md).

GPS is on the OpenRB-150 **center 5-pin connector = `Serial2` at `9600`** (the
known-good wiring), NOT the side `Serial3` `D13`/`D14` pins. The probe now
**defaults to `GPS_PROBE_MODE=2` (Serial2)**; `-DGPS_PROBE_MODE=3` selects the
Serial3 side pins (where the GPS is not wired). Standard run + check:

```bash
scripts/run_gps_probe_serial2_9600.sh        # compile -DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600, upload, monitor+tee
scripts/check_gps_probe_log.sh               # verdict: WRONG_UART / NO_BYTES / BYTES_NO_FIX / FIX_OK
```

A `selected_port=Serial3` / `chars_1s=0` / `NO_BYTES` result is a wrong-UART
assumption, not a dead GPS module — re-run on Serial2.

The probe now reports GPS stability, not only UART bytes. Each one-second line
includes `last_rmc_status`, `last_gga_fix_quality`, `sats`, `hdop`, `lat`,
`lon`, `age_ms`, `chars_1s`, `total_chars`,
`valid_fix_seconds_consecutive`, `no_fix_seconds_consecutive`,
`valid_fix_seconds_total`, `no_fix_seconds_total`, and `gps_probe_state`.

Stable-fix criteria for this probe:

- RMC status `A` or GGA fix quality `>=1`
- TinyGPS++ latitude/longitude valid
- location age `<=2000 ms`
- satellites `>=4`
- HDOP `<=5.0`
- the valid current fix lasts for at least `30` consecutive seconds

Probe states:

- `NO_FIX`: no current valid fix, even if NMEA bytes are arriving.
- `INTERMITTENT_FIX`: a current valid fix exists, but for less than `30`
  consecutive seconds.
- `STABLE_FIX`: the current valid fix has lasted at least `30` consecutive
  seconds.

If the probe prints `warning="TinyGPS cached fix is not stable current fix"`,
RMC has returned to `V` while TinyGPS++ still has cached coordinates. Treat
those coordinates as debug-only and do not use them for target distance or
autonomy decisions.

Latest placement lesson: indoor, near-building, or partially covered positions
can produce persistent `NO_FIX`. Move the rover/GPS farther outdoors and wait
for `gps_probe_state=STABLE_FIX` or
`valid_fix_seconds_consecutive >= 30` before returning to main-controller
`AUTO_MOTION_ARMED=0` dry-run validation.

### GPS All-Pin Activity Probe

Use this when GPS UART probes show no bytes and the user cannot use a multimeter
or does not want to keep unplugging individual wires:

```bash
scripts/run_gps_all_pin_activity_probe.sh
scripts/check_gps_all_pin_activity_log.sh
```

The sketch is `firmware/gps_all_pin_activity_probe`. It samples OpenRB `D0`
through `D29` using only `pinMode(pin, INPUT)` and `digitalRead(pin)`. It never
writes `Serial1`, `Serial2`, or `Serial3`, and it does not use motors, RC,
HC-12, I2C, GPS parsing, or autonomy.

The output only locates electrical edge activity. It does **not** prove NMEA
parsing. If no pin reports `possible_uart_activity=true`, GPS TX is not reaching
OpenRB or the GPS is not transmitting. Do not swap unknown brown/red wires; those
may be `VCC`/`GND`, not UART `RX`/`TX`.

## Breadboard Navigation Development Stack

This is the safe indoor/breadboard workflow for building and validating the GPS
+ IMU + HC-12 navigation stack without driving motors and without the rover
chassis. Motors must not be connected on the breadboard. Default builds never
move motors.

### OpenRB-150 UART map and the GPS/HC-12 coexistence rule

The OpenRB-150 (SAMD21) exposes three independent hardware UARTs plus USB:

| Port | Pins | SERCOM | Current use |
|---|---|---|---|
| `Serial` (USB) | USB | — | USB debug at `115200` |
| `Serial1` | D26 TX / D27 RX | SERCOM2 | free |
| `Serial2` | D28 TX / D29 RX | SERCOM4 | **GPS** (central 4-pin connector, confirmed) |
| `Serial3` | D14 TX / D13 RX | SERCOM5 | HC-12 in integrated no-motion dry-run |
| `Wire` I2C | D11 SDA / D12 SCL | SERCOM0 | **IMU** |

Coexistence rule: GPS is confirmed on `Serial2`. The legacy default
`openrb_robot_controller` build still assigns HC-12 to `Serial2`, so that legacy
mode cannot validate GPS + HC-12 together. The current integrated no-motion
dry-run selects HC-12 on `Serial3`, so GPS (`Serial2`), BMI160 (`Wire`), and
HC-12 (`Serial3`) do not have a software UART conflict. RF is still unproven:
`hc12_rx_count=0`, `hc12_tx_count=0`, and `hc12_parse_ok=false` should be logged
as `HC12_DEFERRED_RF_LINK`, not as a GPS/BMI160 blocker.

### HC-12 Link Probe (`firmware/hc12_link_probe`)

Standalone, motor-free HC-12 link validator. It sends `PING` frames (auto and/or
on USB command), prints received frames, answers `PONG`, and reports TX/RX
counters and a link summary. It initializes only USB Serial and one HC-12 UART —
no motors, GPS, IMU, RC, or autonomy — and never emits motor-driving frames.

Current integrated status: HC-12 is configured on `Serial3` in the no-motion
controller, but RF is deferred/unproven. Treat the current state as
`HC12_DEFERRED_RF_LINK` until actual RX/TX or parse evidence is logged. Do not
let RF absence block GPS/BMI160 integrated no-motion validation.

Status levels:

- `STATION_PORT_FOUND`: CP2104/USB-Serial adapter detected, e.g.
  `/dev/cu.usbserial-02442CA5`.
- `STATION_SERIAL_OPEN_OK`: station tool opens the adapter with DTR/RTS low,
  `rtscts=False`, `dsrdtr=False`.
- `STATION_TX_OK`: station write succeeds; this alone does not prove RF.
- `OPENRB_HC12_PORT_CONFIGURED`: integrated USBDBG shows
  `hc12_enabled=true`, `hc12_port=Serial3`, `hc12_pf_port=Serial3`.
- `RF_RX_SEEN`: OpenRB or station RX counters increase.
- `HC12_PARSE_OK`: `hc12_parse_ok=true` or valid PING/PONG command fields.

Compile-time options:

- `HC12_PROBE_SERIAL_PORT` (default `2`): `1`=Serial1, `2`=Serial2, `3`=Serial3.
  On the breadboard with GPS on `Serial2`, use `3` (or `1`) and wire HC-12 there
  so GPS NMEA bytes do not interfere.
- `HC12_PROBE_BAUD` (default `9600`), `HC12_PROBE_AUTO_PING` (default `1`),
  `HC12_PROBE_PING_INTERVAL_MS` (default `1000`).

USB fields: `hc12_link_probe_alive`, `port`, `hc12_tx_count`, `hc12_rx_count`,
`ping_tx_count`, `pong_rx_count`, `hc12_parse_ok`, `hc12_parse_error`,
`hc12_last_rx_age_ms`, `hc12_last_cmd`, `last_rx_payload`, `link_status`
(`NO_RX_YET` / `LINK_OK` / `LINK_STALE`). USB input sends a `PING` only.

Compile / upload / monitor (macOS + OpenRB-150), HC-12 on Serial3 for GPS coexistence:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-hc12-link-probe --build-property 'compiler.cpp.extra_flags=-DHC12_PROBE_SERIAL_PORT=3' firmware/hc12_link_probe
arduino-cli upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-hc12-link-probe firmware/hc12_link_probe
arduino-cli monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Station side (separate HC-12-USB bridge), `tools/hc12_link_probe.py` sends PING
frames and logs responses (never motor commands):

```bash
uv run python tools/hc12_link_probe.py --port /dev/cu.usbserial-XXXX --baud 9600 --ping-interval-ms 1000 --log-dir outputs/logs
```

### Station path-planning preview (`tools/path_planning_preview.py`)

Indoor-friendly, preview-only. It builds a start→goal waypoint path from
manually supplied coordinates and writes `waypoints.csv`, `summary.md`, and an
optional `preview.png`. No HC-12, no rover commands, no firmware upload.

```bash
uv run python tools/path_planning_preview.py --start-lat 35.570932 --start-lon 129.187338 --goal-lat 35.571120 --goal-lon 129.186050 --spacing-m 2.0 --out-dir outputs/path_preview
```

### Path-Following Dry-Run (`PATH_FOLLOWING_DRYRUN=1`)

A self-contained controller mode that computes — but does not execute — waypoint
following. It is the firmware-side companion to the station path preview. GPS
uses `Serial2`; if no fix is available indoors it falls back to a compile-time
mock position (`PATH_FOLLOWING_ALLOW_MOCK_POSITION=1`). A motor-free HC-12
waypoint protocol runs on `Serial3` (or `Serial1`).

It computes and prints: `position_source` (gps/mock), `current_lat/lon`,
`active_target_source` (compile_time/hc12), `wp_index`/`wp_count`,
`target_distance_m`, `target_bearing_deg`, `arrived`, `heading_ready`,
`heading_source`, `course_displacement_m`, `estimated_course_deg`,
`bearing_error_deg`, `desired_forward_cmd`, `desired_turn_cmd`,
`desired_logical_left_cmd`, `desired_logical_right_cmd`, `desired_physical_a_cmd`,
`desired_physical_b_cmd`, and `path_following_block_reason`. Heading needs real
GPS displacement: a static or mock position prints `heading_ready=false` and
`path_following_block_reason=NO_HEADING`.

HC-12 waypoint protocol (dry-run only; frames are `@TYPE,SEQ,PAYLOAD*CS`):
`PING`→`PONG`, `SET_TARGET lat,lon` (updates the dry-run target,
`active_target_source=hc12`), `STATUS`→`STAT`, `ESTOP` (blocks the gated output),
`CLEAR` (clears target/estop). Reports `hc12_rx_count`, `hc12_tx_count`,
`hc12_last_rx_age_ms`, `hc12_last_cmd`, `hc12_parse_ok`, `hc12_parse_error`.

Physical motor output is impossible unless EVERY gate passes. The four compile
gates all default to `0`:

- `PHYSICAL_PATH_FOLLOWING_ENABLE=1`
- `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1`
- `GROUND_CRAWL_TEST_MODE=1`
- `AUTO_MOTION_ARMED=1`

plus the operator acknowledgement `PATH_FOLLOWING_MODE_CHANNEL_STABLE=1` (default
`0` — see the PPM mode-channel blocker) and the runtime gates `gps_motion_ready`,
`heading_ready`, RC valid + AUTO switch, RC neutral, target distance within
`[PATH_FOLLOWING_MIN_TARGET_DISTANCE_M, PATH_FOLLOWING_MAX_TARGET_DISTANCE_M]`
(`3.0`..`20.0` m), not arrived, no HC-12 ESTOP, and a fresh HC-12 target if one
is used. The reason any gate is blocking is printed as `physical_block_reason`
(default `COMPILE_GATE_OFF`). Hard caps: `PATH_FOLLOWING_MAX_FORWARD_CMD` (0.18),
`PATH_FOLLOWING_MAX_TURN_CMD` (0.04), `PATH_FOLLOWING_MAX_AUTO_MS` (500 ms) with a
latch-stop that clears only on MANUAL.

Compile / upload / monitor the **dry-run** build (no motor output; HC-12 on Serial3):

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-path-dryrun --build-property 'compiler.cpp.extra_flags=-DPATH_FOLLOWING_DRYRUN=1 -DPHYSICAL_PATH_FOLLOWING_ENABLE=0 -DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0' firmware/openrb_robot_controller
arduino-cli upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-path-dryrun firmware/openrb_robot_controller
arduino-cli monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Override the compile-time path or mock position, e.g.
`-DPATH_FOLLOWING_WP1_LAT=... -DPATH_FOLLOWING_WP1_LON=...` (WP1..WP3,
`PATH_FOLLOWING_WP_COUNT`) and `-DPATH_FOLLOWING_MOCK_CURRENT_LAT=... -DPATH_FOLLOWING_MOCK_CURRENT_LON=...`.

#### IMU diagnostic integration in the dry-run (`IMU_ENABLE=1`)

The controller can read the IMU (read-only, on the Wire bus) alongside the
path-following dry-run. It is fully gated by `IMU_ENABLE` (default `0`): with
`IMU_ENABLE=0` there is no Wire include, no IMU code, and the default controller
is unchanged. With `IMU_ENABLE=1`, the controller first checks for BMI160 at
`0x68`/`0x69` by reading `CHIP_ID` register `0x00` (`0xD1`). If BMI160 is found,
it sets accel and gyro normal mode through CMD register `0x7E`, then reads BMI160
gyro registers `0x0C..0x11` and accel registers `0x12..0x17` as little-endian
signed values. The legacy MPU/ICM path is preserved only when BMI160 is not
detected.

With BMI160 active, USBDBG prints fields such as `imu_type=BMI160`,
`imu_i2c_addr=0x68`, `imu_chip_id=0xD1`, `imu_pmu_normal`,
`imu_accel_pmu`, `imu_gyro_pmu`, `imu_data_plausible`, `imu_accel_mag_g`,
`imu_gyro_mag_dps`, `imu_heading_ready`, and `imu_heading_block_reason`. The
latest integrated no-motion run reports `imu_present=true`, `imu_pmu_normal=true`,
`imu_data_plausible=true`, and `imu_heading_ready=true`.

When GPS course and IMU yaw are both available it prints a seeded delta
comparison: `gps_course_deg`, `heading_source`, `heading_agreement_diag`,
`heading_agreement_error_deg` (the change in GPS course minus the change in IMU
yaw since the first moment both were ready).

`HEADING_SOURCE_MODE` selects the heading intent: `0`=GPS_COURSE (default),
`1`=IMU_RELATIVE, `2`=GPS_COURSE_WITH_IMU_DIAG, `3`=MOCK_HEADING, `4`=FUSION_DIAG.
**Only GPS-course modes (0/2) are approved to drive motors**; the others report
`physical_block_reason=HEADING_SOURCE_NOT_APPROVED_FOR_MOTION`. IMU yaw is never
used for motor output. The IMU heading reality check applies: gyro yaw is relative
and drifts and must be compared against GPS course-over-ground outdoors before it
is trusted.

Integrated GPS + IMU + HC-12 dry-run (no motor output; HC-12 on Serial3):

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && mkdir -p outputs/logs && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-integrated --build-property 'compiler.cpp.extra_flags=-DPATH_FOLLOWING_DRYRUN=1 -DIMU_ENABLE=1 -DIMU_HEADING_DRYRUN=1 -DIMU_YAW_DIAG=1 -DHEADING_SOURCE_MODE=2 -DPHYSICAL_PATH_FOLLOWING_ENABLE=0 -DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 -DPATH_FOLLOWING_HC12_SERIAL_PORT=3' firmware/openrb_robot_controller && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-integrated firmware/openrb_robot_controller && sleep 2 && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/integrated_dryrun_$(date +%Y%m%d_%H%M%S).log
```

Physical path following is **not approved**. Do not set the four motion gates or
`PATH_FOLLOWING_MODE_CHANNEL_STABLE=1` until the RC/PPM Manual/Auto channel holds
HIGH reliably (current blocker) and a heading source is validated (GPS course, or
IMU yaw only after the outdoor GPS-vs-IMU agreement check). Even then, motion is
wheel-off-ground / open-area-with-kill-switch only. See
[`docs/outdoor_validation_checklist.md`](../docs/outdoor_validation_checklist.md).

Integrated no-motion PASS criteria:

- GPS outdoors: `gps_block_reason=OK`, `gps_sats>=4`, reasonable `gps_hdop`, and
  `position_source=gps`. Indoor `NO_LOCATION` is expected and is not a
  regression.
- BMI160: `imu_type=BMI160`, `imu_i2c_addr=0x68`, `imu_chip_id=0xD1`,
  `imu_pmu_normal=true`, `imu_data_plausible=true`, plausible accel/gyro
  magnitudes.
- HC-12: `hc12_enabled=true`, `hc12_port=Serial3`, `hc12_pf_port=Serial3`.
  RF is not proven until RX/TX counters and parse fields change from zero/false.
- Safety: `physical_block_reason=COMPILE_GATE_OFF`,
  `physical_output_active=false`, `final_left_cmd=0.000`,
  `final_right_cmd=0.000`.

## UART Port Sweep Probe (`firmware/uart_port_sweep_probe`)

Use this to find which OpenRB-150 hardware UART a fixed module (e.g. the HC-12)
is actually wired to, without removing the module and without a loopback or AT
command. Symptom it solves: `hc12_link_probe` on `Serial3` shows `tx_count`
rising but the station receives 0 bytes (and the OpenRB `rx_count` stays 0),
i.e. the HC-12 is probably not on the assumed UART.

OpenRB-150 UART pin map (from core 0.2.1 `variant.cpp` / `variant.h`):

| UART | SERCOM | TX | RX | MCU pins | Board header |
|---|---|---|---|---|---|
| `Serial1` | SERCOM2 | D26 | D27 | PA12 / PA13 | SD-SPI header (MOSI / SCK) |
| `Serial2` | SERCOM4 | D28 | D29 | PA14 / PA15 | SD-SPI header (SS / MISO) — **GPS** |
| `Serial3` | SERCOM5 | D14 | D13 | PB22 / PB23 | dedicated "Serial3 / exp uart" header |

The probe drives all three UARTs with `@UART,1,TX_TEST` / `@UART,2,TX_TEST` /
`@UART,3,TX_TEST` (once per `UART_SWEEP_TX_PERIOD_MS`, default 1 s) and reads all
three, printing per-port `SerialN_tx` / `SerialN_rx` / `SerialN_rx_preview` plus a
real-time `uart_rx_event` / `RX_FIRST_DETECTED` line. Identification:

- TX side: the UART whose `@UART,<n>,TX_TEST` frame reaches the station (read with
  `tools/serial_raw_read.py`) is where the HC-12 **TX** is wired.
- RX side: the UART whose `rx_count` rises when the station sends is where the
  HC-12 **RX** is wired. `Serial2_rx` showing `$GP`/`$GN` NMEA confirms GPS on
  `Serial2` (expected, not the HC-12).

Safety: USB Serial + the three UARTs only. No motors/ESC/Servo, no GPS parsing,
no RC/PPM, no I2C, no autonomy. It does not change motor/path-following code and
enables no motion. `UART_SWEEP_TX_MASK` (default `0x07`) selects which ports to
drive (`bit0`=Serial1, `bit1`=Serial2, `bit2`=Serial3); `UART_SWEEP_BAUD` default
`9600`.

```bash
# OpenRB: compile + upload + monitor (USB) the sweep probe
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe && sleep 2 && arduino-cli monitor -p "$PORT" --config baudrate=115200

# Station (separate terminal, HC-12-USB bridge): read raw bytes for 15 s and report which @UART,<n> arrived
uv run python tools/serial_raw_read.py --baud 9600 --duration-s 15
```

If the station prints `detected_uart_ports=[2]`, the HC-12 TX is on `Serial2`
(then it shares the GPS UART — see the coexistence rule); `[1]` means `Serial1`,
`[3]` means `Serial3`. `0 bytes` means the HC-12 is on none of the swept UARTs at
this baud (try `-DUART_SWEEP_BAUD=...`) or the station bridge baud is wrong.

## HC-12 operational diagnosis without unplugging the module

The HC-12 is fixed on the board and cannot be removed right now, so the field
diagnosis must work in place — no unplugging, no direct loopback, no AT mode.
Wiring has already been physically verified; treat `total_bytes=0` as a *state*
to interpret, not as automatic proof of a code/firmware bug. Authoritative
mapping: HC-12 is expected on **Serial3** (D14 TX / D13 RX), GPS stays on
**Serial2**.

Two station tools drive the loop:

- `tools/hc12_operational_diagnose.py` — opens the station USB-Serial bridge
  (auto-detects `/dev/cu.usbserial*` / `cu.wchusbserial*` / `cu.SLAB_USBtoUART*`,
  and excludes `/dev/cu.usbmodem*` which is the OpenRB). Modes: `read-only`,
  `write-only`, `ping-pong`, `stability`. It handles `SerialException`,
  `OSError Errno 6 Device not configured`, and a port disappearing/reappearing
  (`--reconnect`), and never crashes with a traceback — it prints structured
  status (`serial_error_count`, `reconnect_count`, `active_port`, `last_error`,
  `total_bytes`, `detected_uart_ports`, `verdict`).
- `tools/hc12_diagnose_report.py` — parses the most recent logs and writes a
  Markdown verdict + next action to `outputs/reports/hc12_diagnosis_<ts>.md`.

`scripts/hc12_field_check.sh` prints the exact ordered terminal commands.

### Distinguish these five states

1. **Station OFF / counterpart off** — if the opposite HC-12 side is not powered
   or its firmware is not transmitting, `NO_RX` / `total_bytes=0` is **expected**
   and does **not** prove a firmware failure. Record it with `--station-off`
   (verdict `TEST_INVALID_STATION_OFF`); the link test is invalid until both
   sides are powered and one side is transmitting.
2. **Station USB-Serial stability** — `stability` mode opens the bridge and polls
   `in_waiting` without sending. Open/close OK and rising `stability_alive_ticks`
   with `serial_error_count=0` means the USB bridge is alive; `in_waiting=0` /
   `total_bytes=0` just means no bytes, not a failure
   (verdict `USB_SERIAL_STABLE_NO_RF_BYTES`).
3. **OpenRB UART sweep** — must run **concurrently** with a station `read-only`
   test. The probe (`firmware/uart_port_sweep_probe`) sends
   `@UART,1/2/3,TX_TEST`; whichever port number the station detects is the UART
   the HC-12 TX is on (`UART_SWEEP_RECEIVED_ON_SERIAL3` etc.).
4. **Station write-only** — must run while the OpenRB uart-sweep monitor is open,
   so you can watch which `SerialN_rx` counter rises (the HC-12 RX port). Serial2
   rx is normally GPS NMEA, not the HC-12.
5. **HC-12 link probe / ping-pong** — only meaningful when **both** sides are
   powered and one side responds. `pong_rx>0` → `HC12_LINK_OK`.

### Mac temporary station: DTR/RTS must be forced low

On the Mac temporary station, some USB-Serial bridges report
`OSError Errno 6 Device not configured` (verdict `STATION_USB_UNSTABLE`) on
**write-only** / **ping-pong** because pyserial asserts DTR/RTS on open and the
adapter resets. A manual PySerial test only succeeded with hardware flow control
off and DTR/RTS forced low (`rtscts=False`, `dsrdtr=False`, `setDTR(False)`,
`setRTS(False)`). So:

- All station tools now open via `tools/station_serial.safe_open_serial`, which
  reproduces that known-good sequence and bounds writes with `write_timeout`.
  Defaults are `--dtr low --rts low --write-timeout-s 1`.
- `DEVICE_NOT_CONFIGURED` during a write is most likely DTR/RTS toggling or the
  USB-Serial adapter reset behavior — **not** a code/firmware fault. The start
  and status summaries print `dtr_mode`, `rts_mode`, `rtscts`, `dsrdtr`.
- A clean write (frames sent, `serial_error_count=0`) with no reply is now
  `STATION_TX_OK_NO_RX` (TX path healthy, no RF back), never
  `STATION_USB_UNSTABLE`.
- The **manual-equivalent write test** is `--mode write-only` with the default
  `--dtr low --rts low`.

### Commands

```bash
# Terminal 1 (OpenRB): uart sweep, capture USB to a report-parseable log
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe && sleep 2 && mkdir -p outputs/logs && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/uart_sweep_openrb_$(date +%Y%m%d_%H%M%S).log

# Terminal 2 (station): DTR/RTS forced low (the default) — stability, read-only, write-only, ping-pong
uv run python tools/hc12_operational_diagnose.py --mode stability  --duration-s 20 --reconnect --dtr low --rts low
uv run python tools/hc12_operational_diagnose.py --mode read-only  --duration-s 60 --reconnect --raw-log --dtr low --rts low
uv run python tools/hc12_operational_diagnose.py --mode write-only --duration-s 60 --reconnect --dtr low --rts low
uv run python tools/hc12_operational_diagnose.py --mode ping-pong  --duration-s 60 --reconnect --dtr low --rts low

# Generate the Markdown report (add --station-off if the counterpart was off)
uv run python tools/hc12_diagnose_report.py
```

## BMI160 IMU Probes

The rover IMU is currently identified as a Bosch BMI160 at I2C address `0x68`.
Use the BMI160 probes for current hardware health checks.

Read-only identity probe:

```text
firmware/imu_bmi160_probe/imu_bmi160_probe.ino
```

Normal-mode diagnostic probe:

```text
firmware/imu_bmi160_normal_probe/imu_bmi160_normal_probe.ino
```

The normal probe writes only the BMI160 diagnostic PMU commands needed to wake
the sensor:

- CMD `0x7E = 0x11` accel normal mode
- CMD `0x7E = 0x15` gyro normal mode

It does not initialize motors, GPS, HC-12, RC, or autonomy. Expected PASS:

- `i2c_addr=0x68`
- `chip_id=0xD1`
- `accel_pmu=NORMAL`
- `gyro_pmu=NORMAL`
- `accel_mag_g` roughly `0.5..1.5` while stationary
- low stationary `gyro_mag_dps`
- `bmi160_normal_probe_pass=true`
- `bmi160_normal_probe_block_reason=OK`

Compile/check:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-bmi160-normal-probe firmware/imu_bmi160_normal_probe
scripts/check_bmi160_normal_probe_log.sh outputs/logs/imu_bmi160_normal_probe_*.log
```

One-shot run helper:

```bash
scripts/run_bmi160_normal_probe.sh
```

## Legacy MPU/ICM IMU Probe (not BMI160 health check)

Use this standalone sketch to check whether an IMU is electrically present and
*readable* on the OpenRB default `Wire` I2C bus. It is the richer successor to
the plain `i2c_scanner_test`: it scans, labels candidate device families, and —
for `0x68`/`0x69` MPU/ICM-class parts — reads `WHO_AM_I` and raw accel/gyro/temp.

```text
firmware/imu_probe/imu_probe.ino
```

This legacy probe assumes an MPU/ICM register map (`WHO_AM_I` at `0x75` and
MPU-style data registers). A BMI160 can respond at `0x68` while returning
misleading values through those addresses; therefore `firmware/imu_probe` must
not be used to judge the current BMI160 module.

This is **signal validation only**. It does not produce trusted heading/yaw.
IMU heading/yaw is not trusted until calibration and drift checks are done, so
this probe must not be wired into autonomy yet.

Safety: this sketch initializes ONLY USB Serial and `Wire`. It does **not**
initialize or drive motors/ESC/Servo outputs, GPS, HC-12, RC/PPM input, or any
rover autonomy logic. It uses the same fixed IMU wiring as the scanners:

- SDA = OpenRB `D11` = PA08 (`PIN_WIRE_SDA = 11`)
- SCL = OpenRB `D12` = PA09 (`PIN_WIRE_SCL = 12`)

It keeps the proven bus-released-high guard from `i2c_scanner_test`: if D11/D12
do not read HIGH with pull-ups, it prints `bus_state=BUS_STUCK_LOW_BEFORE_SCAN`
and refuses to scan, because the bus has historically read stuck LOW (treat that
as an electrical issue: IMU power, GND, pull-ups, or wiring — not pin mapping).

Each pass (every 1 s) prints USB fields at `115200`:

- `imu_probe_alive=true`, `scan_pass`, `sample_ms`
- `pre_scan_sda` / `pre_scan_scl`, `bus_state`
- `i2c_scan_count`
- per found address: `i2c_addr=0x..` and `imu_candidate=...`
- `imu_present` / `imu_mpu_class_present`

Candidate labels by address (address alone does not identify a device):

- `0x68` / `0x69`: `MPU6050_MPU9250_ICM_CANDIDATE`
- `0x0C` / `0x1C` / `0x1E`: `MAGNETOMETER_CANDIDATE`
- `0x28` / `0x29`: `BNO055_CANDIDATE`
- otherwise: `UNKNOWN_I2C_DEVICE`

For a `0x68`/`0x69` device the line also includes `whoami=0x..` with a
`whoami_label` (e.g. `MPU6050_or_MPU9150`, `MPU6500`, `MPU9250`, `ICM20689`;
unknown values print `MPU_ICM_FAMILY_UNKNOWN_WHOAMI`). With raw reads enabled
(default) it wakes the device (writes `PWR_MGMT_1=0x00`, gated to `0x68`/`0x69`
only) and burst-reads registers `0x3B..0x48`:

```text
i2c_addr=0x68 imu_candidate=MPU6050_MPU9250_ICM_CANDIDATE whoami=0x68 whoami_label=MPU6050_or_MPU9150 wake_attempted=true wake_ok=true accel_raw_x=... accel_raw_y=... accel_raw_z=... gyro_raw_x=... gyro_raw_y=... gyro_raw_z=... temp_raw=... temp_c_mpu6050_approx=...
```

`temp_raw` is the trustworthy raw temperature; `temp_c_mpu6050_approx` uses the
MPU6050 constant and is only approximate on MPU9250/ICM parts. Non-`0x68`
candidates (magnetometer/BNO055) report address + candidate hint only; they need
their own driver and are not register-read here.

Historical breadboard MPU/ICM-style result: an earlier module/probe path detected
`i2c_addr=0x69`, `imu_present=true`, with `whoami=0x6F`
(`whoami_label=MPU_CLASS_CLONE_OR_VARIANT_0x6F_signal_only`). `0x6F` is not a
standard InvenSense/ST device ID, but the part ACKs at `0x69` and answers the
`0x75` WHO_AM_I register, so it is MPU register-map compatible and the probe wakes
it and reads raw accel/gyro/temp. Treat this as **signal validation only**: do
not assume a specific datasheet, scale factor, or trusted yaw/heading from it.
For the current rover BMI160, use the BMI160 probes and integrated BMI160 fields
above instead.

**Strict pass criteria** (`imu_present=true` alone is NOT a pass): a pass requires
`imu_mpu_class_present=true` with a `0x68`/`0x69` address and a readable WHO_AM_I.
The scan now prints an explicit `imu_probe_pass` / `imu_probe_block_reason`
(`OK` when an MPU/ICM-class address is found, else `IMU_NOT_MPU_CLASS_DETECTED` /
`NO_I2C_DEVICES` / `BUS_STUCK_LOW`). A lone `i2c_addr=0x03` reading
`UNKNOWN_I2C_DEVICE` is a bus artifact / non-IMU device — `imu_mpu_class_present`
will be `false` and the verdict `IMU_NOT_MPU_CLASS_DETECTED`. Check a captured log
with `scripts/check_imu_probe_log.sh <log>`. (Indoor status as of 2026-06-03 is
BLOCKED for exactly this reason; see `docs/current_hardware_status.md`.)

Beyond the scan, the default build now locks onto the first MPU-class device and
runs continuous diagnostics: it prints `accel_raw_*`, `gyro_raw_*`, `temp_raw`,
`accel_mag_g`, `gyro_mag_dps`, `stationary_detected`/`motion_detected`, a
stationary gyro-bias calibration (`imu_calibrated`, `gyro_bias_x/y/z`,
`gyro_cal_x/y/z`), and `sample_ms`. With `IMU_YAW_DIAG=1` it integrates a chosen
gyro axis into `imu_relative_yaw_deg` (reset to 0 at start), printing
`imu_yaw_axis`/`imu_yaw_sign`.

Heading reality check (printed as a warning by the probe): gyro-integrated yaw is
**relative and drifts**; it is NOT an absolute compass heading. An absolute
heading needs a GPS course-over-ground seed, a magnetometer, or another reference,
plus calibration and a drift/agreement check. Use this probe only to (1) confirm
raw signal, (2) estimate gyro bias, (3) find the yaw axis/sign, (4) watch
relative-yaw drift while stationary — so it can be compared to GPS
course-over-ground outdoors later.

Compile-time options:

- `IMU_PROBE_SCAN_ONLY` (default `0`): generic I2C scanner + labels only, no
  register access — the simplest, fully read-only mode.
- `IMU_PROBE_RAW_ENABLE` (default `1`): set `0` to stay read-only (`WHO_AM_I`
  only, no wake/raw, no continuous diagnostics).
- `IMU_CALIBRATION_SECONDS` (default `5`): stationary gyro-bias window. Hold the
  IMU still during this window after boot.
- `IMU_YAW_DIAG` (default `0`): set `1` to integrate relative yaw.
- `IMU_YAW_AXIS` (default `2` = Z) and `IMU_YAW_SIGN` (default `1`): select the
  gyro axis/sign used for yaw integration.
- `IMU_GYRO_LSB_PER_DPS` (default `131.0`) / `IMU_ACCEL_LSB_PER_G` (default
  `16384.0`): MPU default scales; adjust if the device differs.

Compile (macOS + OpenRB-150):

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe firmware/imu_probe
```

Upload:

```bash
arduino-cli upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe firmware/imu_probe
```

Monitor:

```bash
arduino-cli monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

One-shot compile + upload + monitor with auto-detected port and logging:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && mkdir -p outputs/logs && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe firmware/imu_probe && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe firmware/imu_probe && sleep 2 && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/imu_probe_$(date +%Y%m%d_%H%M%S).log
```

Generic-scanner-only build (no register access):

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe-scanonly --build-property 'compiler.cpp.extra_flags=-DIMU_PROBE_SCAN_ONLY=1' firmware/imu_probe
```

Scan + `WHO_AM_I` only, no wake / no raw reads:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-probe-rawoff --build-property 'compiler.cpp.extra_flags=-DIMU_PROBE_RAW_ENABLE=0' firmware/imu_probe
```

Relative-yaw diagnostic build (configurable axis/sign) — compile + upload + monitor:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && mkdir -p outputs/logs && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-yaw --build-property 'compiler.cpp.extra_flags=-DIMU_YAW_DIAG=1 -DIMU_YAW_AXIS=2 -DIMU_YAW_SIGN=1 -DIMU_CALIBRATION_SECONDS=5' firmware/imu_probe && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-imu-yaw firmware/imu_probe && sleep 2 && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/imu_yaw_$(date +%Y%m%d_%H%M%S).log
```

Yaw-axis/sign procedure: hold still ~5 s (calibration) → rotate the board +90°
clockwise (yaw). Pick `IMU_YAW_AXIS` as the gyro axis that responds most to yaw,
and set `IMU_YAW_SIGN` so a clockwise turn increases `imu_relative_yaw_deg`. Then
confirm drift while stationary is small over ~30–60 s before trusting any delta.

Physical movements to try while monitoring (validate raw signal, not heading):

1. Hold stationary ~5 s — raw accel/gyro should be roughly steady; one accel
   axis near gravity (~16384 at ±2 g) and gyro near zero.
2. Tilt forward / back — accel X/Z (or pitch axis) should change smoothly.
3. Tilt left / right — accel Y/Z (or roll axis) should change smoothly.
4. Rotate yaw slowly — the corresponding gyro axis should spike while turning,
   then return toward zero when you stop.
5. Hold stationary again — values should settle back near the step-1 baseline.

Interpretation: changing raw values that track motion and settle when still mean
the IMU signal is alive. Do **not** treat any of this as a usable yaw/heading
source. Physical path following stays blocked until both the RC/PPM mode channel
holds reliably (see the PPM probe blocker) and a heading source is validated
(calibration + drift). If `bus_state=BUS_STUCK_LOW_BEFORE_SCAN` persists or no
`0x68`/`0x69` address appears, the IMU is not yet electrically usable; continue
the GPS+RC workflow without IMU.

## I2C Scanner Test

Use this standalone sketch to verify whether a device responds on the OpenRB
default hardware `Wire` I2C bus:

```text
firmware/i2c_scanner_test/i2c_scanner_test.ino
```

OpenRB-150 variant files confirm the current fixed IMU wiring matches the
board's default `Wire` pins:

- Arduino D11 = SDA = PA08
- Arduino D12 = SCL = PA09
- `PIN_WIRE_SDA = 11`
- `PIN_WIRE_SCL = 12`

It uses `Wire`, USB Serial at `115200`, scans addresses `0x03` through `0x77`,
and does not attach motors or Servo outputs. The scanner prints startup lines
before `Wire.begin()`, reads D11/D12 pullup states before and after
`Wire.begin()`, and prints `scan_pass`, `found_count`, found addresses, and
`stable_valid_address` every 2 seconds. Addresses such as `0x68`, `0x69`, or
`0x76` are common for some IMU/sensor modules, but address alone is not a
device identification.

A valid IMU result requires one stable address. If all addresses or more than 8
addresses are found, treat the pass as invalid (`INVALID_SCAN_TOO_MANY_ADDRESSES`)
and do not treat it as success. If D11/D12 read LOW with pullups enabled, treat
that as an electrical or bus issue such as power, GND, pullups, or a stuck bus;
do not treat it as a pin mapping issue.

Latest observed result:

- The robust default `Wire` scanner runs and prints repeated scan passes.
- Every pass shows `pre_scan_sda=LOW` and `pre_scan_scl=LOW`.
- Every pass prints `BUS_STUCK_LOW_BEFORE_SCAN`.
- Every pass reports `found_count=0` and `stable_valid_address=NA`.
- The scanner is not hanging; it is correctly refusing to scan because the bus
  is stuck low before address probing.
- IMU remains unverified and must not be used for autonomy yet.
- Continue the GPS+RC safety-gated workflow without IMU for now.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-scanner firmware/i2c_scanner_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-scanner firmware/i2c_scanner_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

If the monitor is blank, press the OpenRB reset button while the monitor is
open. The scanner should print again every 2 seconds even if the first startup
messages were missed.

## D11/D12 Bit-Bang I2C Scanner

Use this standalone sketch only as a secondary diagnostic. The current fixed IMU
wiring is on OpenRB default `Wire` pins, so `firmware/i2c_scanner_test` is the
primary scanner.

```text
firmware/i2c_d11_d12_bitbang_scanner/i2c_d11_d12_bitbang_scanner.ino
```

Current fixed IMU wiring:

- SDA: OpenRB D11 / PA08 / SDA(SC2)
- SCL: OpenRB D12 / PA09 / SCL(SC2)

This sketch implements open-drain style I2C in software:

- release line: `pinMode(pin, INPUT_PULLUP)`
- drive low: `pinMode(pin, OUTPUT); digitalWrite(pin, LOW)`
- never drive HIGH directly

It prints the released SDA/SCL state, reports `SDA stuck low` or `SCL stuck
low` if either line remains low, attempts bus recovery before skipping stuck
passes, scans addresses `0x03` through `0x77` every 2 seconds, and prints
`scan_pass`, `bus_stuck_low`, raw/valid found counts, found addresses, and
`stable_valid_address`.

If a pass reports many addresses or every address, treat it as scanner/bus
failure (`INVALID_SCAN_ACK_STUCK_LOW`), not as many devices. The IMU remains
unverified until a stable single address is detected in at least three
consecutive valid scan passes.

Latest observed result:

- The original bit-bang scanner produced impossible all-address detection; that
  was invalid and must not be treated as success.
- The hardened scanner was tested with SDA=D11/SCL=D12 and with swapped
  SDA=D12/SCL=D11.
- Both variants repeatedly reported `released_sda=LOW`, `released_scl=LOW`,
  `SDA stuck low`, `SCL stuck low`, `raw_found_count=0`, `valid_found_count=0`,
  and `stable_valid_address=NA`.
- IMU presence remains unverified.
- Because D11/D12 are OpenRB default `Wire` pins, the next diagnostic is the
  robust default `Wire` scanner in `firmware/i2c_scanner_test`, not a custom
  SERCOM2 scanner.
- If the robust default `Wire` scanner also fails, continue GPS+RC workflow
  without IMU support.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Compile with explicit pin override:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang --build-property 'compiler.cpp.extra_flags=-DI2C_BITBANG_SDA_PIN=11 -DI2C_BITBANG_SCL_PIN=12' firmware/i2c_d11_d12_bitbang_scanner
```

Compile with swapped D12/D11 assignment, without moving wires:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d12-d11-bitbang --build-property 'compiler.cpp.extra_flags=-DI2C_BITBANG_SDA_PIN=12 -DI2C_BITBANG_SCL_PIN=11' firmware/i2c_d11_d12_bitbang_scanner
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d11-d12-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Upload the swapped D12/D11 build:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-i2c-d12-d11-bitbang firmware/i2c_d11_d12_bitbang_scanner
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

## Serial3 Pin Verification

These standalone sketches are retained as historical/safe UART pin tools:

```text
firmware/pin_finder_test/pin_finder_test.ino
firmware/serial3_loopback_test/serial3_loopback_test.ino
```

Under the Fixed Wiring Plan, do not move GPS or HC-12. They do not attach motor
outputs.
