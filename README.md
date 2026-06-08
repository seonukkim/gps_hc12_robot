# Ship Hull Coverage Robot

Pre-ROS2 Python prototype for an Industrial Engineering project: a ship exterior
cleaning and painting robot intended to operate on the outer hull of a ship. The
current planning assumption is a locally planar surface. The adhesion concept is
magnetic wheels, but magnetic adhesion and full cleaning/painting operation have
not yet been validated.

The repository currently focuses on safe rover bring-up, GPS/log handling,
protocol utilities, and mock planar coverage planning. ROS2 Jazzy integration is
planned, but the current ROS2 packages are skeletons only.

## Safety Defaults

Station-side defaults are intentionally conservative:

- Default USB serial device: `/dev/ttyACM0`.
- Default baudrate: `9600`.
- Serial tools expose `--port` so the device can be changed explicitly.
- Station loops default to heartbeat and `STOP` only.
- Station startup must not send live motor-driving `AUTO` commands.
- Rover motor testing is wheel-off-ground only.
- RC manual override and rover-side failsafe logic remain authoritative.

## Industrial Engineering Motivation

Ship hull exterior work is labor-intensive, repetitive, and difficult to keep
consistent over large surfaces. This project frames that problem as an
Industrial Engineering workflow:

- reduce manual burden in hull cleaning and painting tasks;
- define repeatable coverage regions from operator-selected points;
- generate systematic lawnmower-style paths for surface coverage;
- automate the workflow from manual setup to planned execution;
- validate safety behavior before expanding toward autonomous field operation.

## System Concept

The intended operation concept is:

1. Drive the rover manually on or near the target hull surface.
2. Record operator-selected A/B reference points.
3. Define a rectangular planar work region from those references.
4. Generate a lawnmower-style coverage path with configurable lane spacing.
5. Preview or export the mock mission offline.
6. Later, send mission commands through the station, HC-12 radio, and ROS2 stack.

Current code supports the Python-side protocol and mock planning pieces. It does
not yet implement completed autonomous ROS2 execution or confirmed end-to-end
station HC-12 operation.

## Physical Path Planning (Integrated CLI)

A→B serpentine coverage, calibration, and guarded motion are consolidated into one
package, `tools/physical_path_planning/`, behind a single entrypoint with five modes:

```bash
uv run python -m tools.physical_path_planning.cli <mode> [options]
# launcher wrapper (adds the firmware flash for run/execute-plan):
scripts/run_physical_path_planner.sh <mode> [options]
```

- `preview` — build + render the A→B plan (no serial, no motion; works with no
  calibration).
- `calibrate-turn` — measure a 90° turn via the Stage20 probe with IMU yaw.
- `run` / `execute-plan` — drive the guarded continuous-motion controller.
- `diagnose` — read-only telemetry summary (live port or `--from-log`).

`start` (A) and `goal` (B) are **opposite corners of a rectangle's diagonal**, not a
straight line; `--workspace-width-m` is the short side and must be shorter than the
diagonal. Use `--path-shape direct_line` for an actual straight A→B.

Safety posture is unchanged: `run`/`execute-plan` flash the STAGE20 *guarded-crawl*
firmware behind the same 4-flag compile gate as
`scripts/run_stage20_physical_ab_probe.sh` (that gate, not the CLI, is the real
motor-output safety), and every summary carries `ready_for_full_path_following=false`.
This is guarded, bounded motion, not full autonomous path following — station-side
preview is always allowed, physical execution stays subject to the field
preconditions noted throughout this README. `--print-plan` / `--print-cmd` /
`--from-log` give fully no-hardware paths.

This package supersedes the former `stage30`–`stage36` modules and their scripts.
See [docs/README_physical_path_planning.md](docs/README_physical_path_planning.md)
(usage), [docs/physical_path_planning_architecture.md](docs/physical_path_planning_architecture.md)
(module map + control law), and [docs/field_test_manual.md](docs/field_test_manual.md)
(operator procedure).

## Hardware Overview

- Target surface: outer hull of a ship, currently approximated as planar.
- Adhesion concept: magnetic wheels, pending design and validation.
- Rover controller: OpenRB-150.
- Manual control: RC receiver with PPM input; RC manual mode has been verified.
- GPS: long-antenna GPS on the central OpenRB connector, confirmed as
  `Serial2 @ 9600`; outdoor GPS probe reached `STABLE_FIX` with lat/lon present.
- IMU: BMI160 on OpenRB `Wire` (`D11` SDA / `D12` SCL), integrated at I2C `0x68`
  with `chip_id=0xD1` and plausible accel/gyro data.
- Radio link: HC-12 UART is the intended station-to-rover link. Integrated
  no-motion firmware selects `Serial3`, and the Mac CP2104 station adapter has
  been detected, but RF RX/TX is not yet proven (`HC12_DEFERRED_RF_LINK`).
- Actuation: ESC/motor outputs are managed by rover firmware. Bench motor tests
  must remain wheel-off-ground.
- Station/development OS: Ubuntu 24.04. WSL2 Ubuntu 24.04 and Jetson are target
  station environments.

See [docs/current_hardware_status.md](docs/current_hardware_status.md),
[docs/wiring.md](docs/wiring.md), [docs/outdoor_no_motion_validation.md](docs/outdoor_no_motion_validation.md),
and [firmware/README.md](firmware/README.md).

## GPS Antenna Frame Vs Rover Body Frame

GPS coordinates are the antenna coordinates. In recent sky-view tests, the
external GPS antenna was placed far outside while the rover body remained
indoors. That is valid for GPS UART, satellite-fix, and candidate-command
dry-run validation, but it is not valid for floor navigation because the
reported `gps_lat` / `gps_lon` is not the rover body position.

The IMU cannot fully correct a detached GPS antenna into rover body position.
The IMU may help later with heading and rotation sensing, but it does not
replace a rover-mounted position source. Real outdoor navigation requires the
GPS antenna to be rigidly mounted on the rover, or its offset from the rover
body frame must be fixed, measured, and modeled.

## Next Required Validation

- Current no-motion baseline: integrated logs must keep
  `physical_block_reason=COMPILE_GATE_OFF`, `physical_output_active=false`,
  `final_left_cmd=0.000`, and `final_right_cmd=0.000`. Do not enable physical
  path-following gates for sensor/communication validation.
- Re-run the integrated baseline outdoors with the long-antenna GPS, BMI160, RC
  transmitter on, and HC-12 on `Serial3`.
- Hand-carry the rover to validate GPS course / BMI160 heading diagnostics.
- Recheck RC/manual outdoors with transmitter on; current indoor FAILSAFE can be
  caused by the transmitter being off.
- Run HC-12 station/RF checks separately. Current state is
  `HC12_DEFERRED_RF_LINK`, not a GPS or BMI160 failure.
- BLOCKER (2026-05-30): physical path following is blocked until the AUTO/MANUAL
  switch channel is stable. A PPM hold test showed receiver CH5 did not hold
  HIGH: only `4` AUTO-like samples out of `68`
  (`ch5_high_auto_like=4`, `ch5_low_manual_like=64`,
  `RESULT=CH5_AUTO_DID_NOT_HOLD`). When AUTO is raised, the main firmware briefly
  enters `AUTO_READY` then `FAILSAFE` as `ppm_age_ms` grows. Use
  `firmware/ppm_channel_map_probe` (analyzed with `tools/analyze_ppm_log.py`) to
  determine which receiver channel actually corresponds to a stable 2-position
  switch, then set `-DMODE_CHANNEL_INDEX=<0-based index>` in the main controller.
  Station-side path planning preview is allowed; physical path execution is not.
- Confirm the station/controller is powered on and linked; controller-off can
  make RC appear stuck or failsafe-like.
- Verify Manual/Auto before autonomy dry-run: MANUAL should show
  `mode_us≈1000` and `control_source=RC_MANUAL`; AUTO should show
  `mode_us≈2000`, `mode=AUTO_READY`, and `control_source=STOP`.
- Recompute the single-waypoint target from the current GPS position before
  every guarded crawl attempt; do not reuse old target coordinates after moving
  the rover.
- For guarded crawl tests, use a fresh target inside the crawl window
  (`5..20` m), typically around `10..12` m away.
- The single-waypoint timeout now starts on AUTO entry, so MANUAL GPS waiting
  should not consume the AUTO candidate timeout.
- Confirm `gps_age_ms`, `gps_hdop`, and `gps_sats`, not only `gps_fix=true`,
  before interpreting GPS readiness.
- Re-test GPS candidate fields with the antenna mounted on the rover and placed
  in open sky.
- 2026-05-30 result: guarded crawl 0.08 reached `AUTO_RUNNING`, clamped
  `candidate_left_cmd=0.100` / `candidate_right_cmd=0.100` to
  `final_left_cmd=0.080` / `final_right_cmd=0.080`, latched stop after the
  duration limit, and blocked too-close target distance and degraded GPS. This
  validates the safety harness, not full autonomous driving.
- Historical 0.08/0.12 crawl tests established the safe clamp/latch workflow:
  candidate speed is set by `SINGLE_WP_CRAWL_BASE_CMD`, final clamp by
  `GROUND_CRAWL_MAX_CMD`, and ungated AUTO output must not be raised.
- First successful guarded AUTO crawl is now recorded after the manual/drive
  mapping fix. With `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`,
  `old_angle_remap_active=false`, `SINGLE_WP_CRAWL_BASE_CMD=0.220`, and
  `GROUND_CRAWL_MAX_CMD=0.220`, the rover briefly moved forward under guarded
  AUTO. USBDBG showed `gps_motion_ready=true`, `gps_block_reason=OK`,
  `target_distance_m≈9.6`, `ground_crawl_ready=true`,
  `ground_crawl_block_reason=OK`, `final_left_cmd=0.220`,
  `final_right_cmd=0.220`, `physical_a_cmd=0.220`, and
  `physical_b_cmd=0.000`.
- Repeated 1000 ms guarded AUTO crawl is now recorded. The user toggled
  AUTO/MANUAL about `3..4` times; `AUTO_RUNNING` was observed multiple times
  with `GROUND_CRAWL_TEST_MODE=1`, `GROUND_CRAWL_MAX_CMD=0.220`,
  `GROUND_CRAWL_MAX_AUTO_MS=1000`, `SINGLE_WP_CRAWL_BASE_CMD=0.220`,
  `AUTO_MOTION_ARMED=1`, `MANUAL_FORWARD_SIGN=-1`, and
  `MANUAL_TURN_SIGN=1`. Each valid AUTO window remained straight forward
  (`physical_a_cmd=0.220`, `physical_b_cmd=0.000`) and latched back to zero
  after roughly `1000` ms. One AUTO attempt was shorter because the user
  returned to MANUAL early.
- `target_distance_m` varied around `16.8..18.0` instead of monotonically
  decreasing. This is expected for the current straight-crawl test because no
  steering/course correction is active yet.
- Next stage: station-side path planning preview only with no motor execution,
  then single-waypoint steering dry-run, then heading/course estimation before
  any physical waypoint following. Do not approve full floor waypoint driving or
  coverage driving yet.
- `SINGLE_WP_STEERING_DRYRUN=1` adds steering diagnostics only. It estimates
  course-over-ground from GPS displacement after at least
  `SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M` of movement (default `2.0` m),
  then prints desired forward/turn, logical wheel, and physical A/B commands.
  If the rover has not moved enough, USBDBG reports `heading_ready=false` and
  `steering_block_reason=NO_HEADING`. Do not use target bearing alone as a
  motor steering command.
- The course displacement threshold is compile-time configurable with
  `-DCOURSE_MIN_DISPLACEMENT_M=<meters>`. A USB-tethered diagnostic build can use
  `-DCOURSE_MIN_DISPLACEMENT_M=1.0` to estimate heading over shorter movement.
  USBDBG prints the active value as `course_min_displacement_m` and the
  configured macro as `course_min_displacement_source`. This does not weaken the
  actual GPS motion safety thresholds (`gps_motion_min_sats`,
  `gps_motion_max_hdop`, `gps_motion_stale_ms` are unchanged) and does not enable
  motor execution.
- The Manual/Auto switch channel is now compile-time selectable in the main
  controller via `-DMODE_CHANNEL_INDEX=<0-based index>` (default `4` = receiver
  CH5, unchanged). USBDBG and the startup banner print `mode_channel_index` /
  `mode_channel_label`, plus `raw_mode_channel_us` and `raw_ch1_us`..`raw_ch8_us`
  so the switch channel and any channel-slip are visible. Only change this after
  `ppm_channel_map_probe` proves another channel is a stable 2-position switch.
  Selecting the channel does not weaken failsafe, GPS thresholds, or motion gates.

## Legacy HC-12 References

Legacy HC-12 scripts and notes from `~/Desktop/project-lab/hc12` have been
audited under [references/legacy_hc12](references/legacy_hc12). They are
reference material only, not production station or rover code.

The useful legacy patterns are mostly `9600` baud PC `readline()` loops,
Arduino/Nano `SoftwareSerial` bridges, RP2040 UART bridge notes, and old
OpenRB/Mega-style `Serial3` transmit experiments. Known problems include
hardcoded `COM4` or `/dev/cu.usbserial-*` ports, blocking loops, inconsistent
variable names, old unverified UART assumptions, and examples that directly
drive motors without this rover's STOP/failsafe model.

Do not copy those examples into active firmware or station tools blindly. Use
them only to inform new receive-only HC-12 diagnostics after the current fixed
wiring and safety constraints are rechecked.

## Firmware Modes

The OpenRB firmware modes are intentionally separated. Do not infer GPS,
HC-12, or motor behavior from the wrong mode.

| Mode | Sketch / build | Upload target | Intended use | GPS behavior | HC-12 behavior | Motor behavior |
|---|---|---|---|---|---|---|
| Default rover controller | `firmware/openrb_robot_controller` | OpenRB-150 | RC manual and HC-12 protocol baseline | Default firmware reads GPS from `Serial3`; under current fixed wiring, GPS `Serial2` is not available here and `gps_chars=0` is expected | enabled | normal safety-gated rover behavior |
| Fixed-wiring GPS Serial2 diagnostic | `firmware/openrb_robot_controller` with `FIXED_WIRING_GPS_SERIAL2_DIAG=1` | OpenRB-150 | Integrated GPS-on-`Serial2` USB debug | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | forced neutral; manual driving does not work by design |
| Fixed-wiring RC + GPS autonomy dry-run | `firmware/openrb_robot_controller` with `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1` | OpenRB-150 | RC manual plus GPS readiness/distance/bearing dry-run | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | RC MANUAL drives normally; AUTO forces neutral and computes readiness only |
| Single-waypoint experiment | `firmware/openrb_robot_controller` with `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1` | OpenRB-150 | Guarded one-target candidate-command experiment | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | RC MANUAL drives normally; AUTO computes candidate commands; armed motion gated to zero unless the ground crawl flag is also set |
| Guarded ground crawl | `firmware/openrb_robot_controller` with `...SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=1 -DGROUND_CRAWL_TEST_MODE=1` | OpenRB-150 | Only path to armed motion; safety-bounded crawl for deadband calibration | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | RC MANUAL drives normally; AUTO output clamped to ±`GROUND_CRAWL_MAX_CMD` and hard-latched to stop after `GROUND_CRAWL_MAX_AUTO_MS`, else neutral |
| Motor pulse calibration | `firmware/openrb_robot_controller` with `MOTOR_PULSE_TEST_MODE=1` | OpenRB-150 | GPS-independent motor deadband calibration | not used | disabled/ignored | RC MANUAL drives normally; AUTO emits one neutral-stick pulse for `MOTOR_PULSE_MS`, then latches stop until MANUAL |
| Physical output pin probe | `firmware/physical_output_pin_probe` | OpenRB-150 | Truth-table probe for the two final PWM output pins | not used | not used | writes one timed pulse directly to physical output pin A/B after a startup delay, then neutral forever |
| RC channel probe | `firmware/rc_channel_probe` | OpenRB-150 | Identify which raw PPM channel changes for each RC stick/switch | not used | not used | no motor outputs |
| PPM channel map probe | `firmware/ppm_channel_map_probe` | OpenRB-150 | Find a stable 2-position AUTO/MANUAL channel; detect channel-slip and momentary switches | not used | not used | no motor outputs; event + 1 s summary lines |
| Standalone GPS probe | `firmware/gps_uart_probe` | OpenRB-150 | GPS UART/baud validation | selectable; current fixed GPS path is `Serial2` at `9600` | not used | no motor outputs |
| Serial3 loopback test | `firmware/serial3_loopback_test` | OpenRB-150 | Historical UART pin test | not a GPS test | not used | no motor outputs |
| Pin finder test | `firmware/pin_finder_test` | OpenRB-150 | Historical physical pin finder | not a GPS test | not used | no motor outputs |

### RC Channel Probe

Use this before changing RC mode-channel mapping. It uses OpenRB `D6` and the
same PPM frame decoding style as the rover controller, but it does not attach
Servo or motor outputs.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Move each stick and switch one at a time, then record `changed_channels` and
the `chN_min` / `chN_max` range. The AUTO candidate is the channel that reaches
around `2000 us`; do not rely on physical panel labels alone.

### PPM Channel Map Probe

Use this when a channel reaches AUTO but will not hold, which is the current
blocker: a PPM hold test showed receiver CH5 did not hold HIGH (`4` AUTO-like
samples out of `68`). It prints `PPMEVT` lines on each LOW/MID/HIGH transition
and `PPMSUM` lines every second, so switch behavior and channel-slip are visible
without scrolling raw frames. See `firmware/README.md` for the full field list.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-ppm-channel-map-probe firmware/ppm_channel_map_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-ppm-channel-map-probe firmware/ppm_channel_map_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200 | tee outputs/logs/ppm_channel_map_$(date +%Y%m%d_%H%M%S).log
```

Analyze the captured log and read the candidate verdict:

```bash
uv run python tools/analyze_ppm_log.py outputs/logs/ppm_channel_map_*.log
```

A usable mode channel must reach both LOW and HIGH and hold HIGH. Only then set
`-DMODE_CHANNEL_INDEX=<0-based index>` in `openrb_robot_controller` (default `4`
= CH5). Physical path execution stays blocked until this holds.

### Default Rover Controller

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected under current fixed wiring:

- `fixed_wiring_gps_serial2_diag=false`
- `hc12_enabled=true`
- `gps_chars=0` because default firmware still reads GPS from `Serial3`
- Manual driving requires RC mode switch out of AUTO; if USBDBG shows
  `mode=AUTO_READY`, `auto_sw=true`, and `control_source=STOP`, switch RC mode
  back to manual before validating `control_source=RC_MANUAL`

### Fixed-Wiring GPS Serial2 Diagnostic

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_DIAG=1' firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected:

- GPS uses fixed `Serial2` wiring at `9600`
- HC-12 is disabled/ignored
- motors are forced neutral
- manual driving does not work in this diagnostic build by design

### Fixed-Wiring RC + GPS Autonomy Dry-Run

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1' firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected:

- GPS uses fixed `Serial2` wiring at `9600`
- HC-12 is disabled/ignored
- RC MANUAL mode preserves current manual driving behavior and reports
  `control_source=RC_MANUAL`
- RC AUTO switch position does not move the rover; it forces `left_cmd=0` and
  `right_cmd=0`, prints `autonomy_dryrun=true`, target placeholder fields,
  distance/bearing to target, and `gps_ready`, `target_ready`,
  `autonomy_ready`
- placeholder target is dry-run only:
  `35.571120,129.186050`
- onboard geodesy computes placeholder target distance and initial bearing over
  USB debug only
- no real waypoint following is implemented
- Arduino-side distance/bearing helpers are validated manually from USBDBG:
  `target_distance_m` finite with `gps_fix=true`, `target_bearing_deg` in
  `0..360`, and AUTO still `left_cmd=0` / `right_cmd=0`

Validated:

- USBDBG identified the running build as the unified dry-run build:
  `fixed_wiring_gps_serial2_diag=false`, `hc12_enabled=false`, and
  `autonomy_dryrun=true`.
- This is the first firmware mode where RC MANUAL driving and fixed-wiring GPS
  dry-run coexist in one build.
- MANUAL mode was tested with `control_source=RC_MANUAL`, and stick input
  changed manual command and final command fields.
- AUTO mode was tested with `autonomy_dryrun=true`, GPS fields,
  target distance/bearing fields, `control_source=STOP`, and `left_cmd=0` /
  `right_cmd=0`.
- With the antenna outside/open sky, `gps_fix=true` was observed.
- Earlier `gps_sats=0` / `gps_hdop=99.99` was antenna placement, not UART
  failure.
- AUTO is still computation-only. Real motion is not enabled yet, and HC-12 is
  disabled in this fixed-wiring GPS mode.

### Single-Waypoint Experiment

This mode prepares the next autonomy milestone without enabling coverage
driving. It uses one placeholder target and adds safety gates around candidate
commands. `AUTO_MOTION_ARMED=0` is the default-safe experiment: it prints
candidate commands but forces final motor output to zero.

Compile with motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0' firmware/openrb_robot_controller
```

Compile with the nearby target override and motor output inhibited:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_TARGET_LAT=35.5716800 -DSINGLE_WP_TARGET_LON=129.1866516' firmware/openrb_robot_controller
```

Successful no-motion AUTO dry-run command pattern, using the latest validated
nearby target:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-success-inhibit --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_TARGET_LAT=35.5705010 -DSINGLE_WP_TARGET_LON=129.1872696' firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-success-inhibit firmware/openrb_robot_controller
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Upload the nearby override build using the matching build directory:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-nearby-inhibit firmware/openrb_robot_controller
```

Upload the inhibited build:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-single-waypoint-inhibit firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected USBDBG additions:

```text
single_waypoint_experiment=true target_override_enabled=... target_source=... target_lat_macro=... target_lon_macro=... auto_motion_armed=false auto_motor_inhibit=true active_gps_ready=... dryrun_ready=... motion_ready=... safety_ready_source=... target_ready=... safety_ready=... max_target_distance_m=30.0 arrival_radius_m=2.5 distance_allowed=... arrived=... target_lat=... target_lon=... target_distance_m=... target_bearing_deg=... candidate_left_cmd=... candidate_right_cmd=... final_left_cmd=0.000 final_right_cmd=0.000
```

Runtime `target_lat` and `target_lon` are the source of truth. A nearby target
override build should print `target_override_enabled=true`,
`target_source=compile_time`, `target_lat_macro=35.5716800`,
`target_lon_macro=129.1866516`, `target_lat=35.571680`, and
`target_lon=129.186652`. Without override macros, USBDBG should print
`target_override_enabled=false` and `target_source=fallback`. Do not interpret
`distance_allowed` or approve bench testing until the runtime target fields
match the intended target.

Latest check: target override plumbing is verified. A build with
`SINGLE_WP_TARGET_LAT=35.5710210` and `SINGLE_WP_TARGET_LON=129.1864016`
correctly printed `target_source=compile_time`, matching macro strings, and
runtime `target_lat=35.571021`, `target_lon=129.186402`. It was still blocked
because the current GPS position was about `380` to `392` m away, exceeding
`max_target_distance_m=30.0`. Recompute the target from the current GPS position
before the next `AUTO_MOTION_ARMED=0` run.

Next-day retest: target override still worked, but the GPS antenna was placed
at a new location (`gps_lat≈35.571310`, `gps_lon≈129.188630`) while firmware
still used the previous target (`35.567560,129.186792`). The target was about
`448.9` m away, so `distance_allowed=false` and `safety_ready=false` were
expected. Recalculate nearby targets whenever the antenna location changes.

Latest nearby attempt: target override and GPS fix worked, but the target was
computed from a stale or assumed position. The actual runtime fix was
`gps_lat=35.571384`, `gps_lon=129.187514`, while the target was
`35.571310,129.188542`, producing `target_distance_m=93.3`. Recompute the next
target from the actual USBDBG GPS fix before rerunning `AUTO_MOTION_ARMED=0`.

Window/outside-antenna retest: target override still worked, but tossing or
placing only the antenna outside is not equivalent to rover localization. GPS
fix appeared around `35.571284,129.188456`, but `gps_sats`, `gps_hdop`, and
`gps_age_ms` were unstable and `target_distance_m≈93.9`, so
`gps_ready=false`, `distance_allowed=false`, and `safety_ready=false` were
expected. Go fully outdoors with rover and GPS fixed together before the next
candidate dry-run.

Latest outdoor recovery: the previous RC issue was caused by the
station/controller being off. After restoring the controller/link, MANUAL was
verified with `mode_us≈1000..1001` and `control_source=RC_MANUAL`; AUTO was
verified with `mode_us≈2001..2002`, `mode=AUTO_READY`, and
`control_source=STOP`. GPS was usable outdoors, but the compile-time target was
stale: target `35.570675,129.186769` versus runtime GPS around
`35.5716,129.1875`, giving `target_distance_m≈100..131`. This is a successful
RC recovery and safety-blocked dry-run, not a nearby candidate success.
Recompute the target from the current runtime GPS fix and rerun with
`AUTO_MOTION_ARMED=0`.

Latest outdoor nearby dry-run: target override worked with
`35.570768,129.186791`, GPS was repeatedly ready outdoors, and
`target_distance_m` dropped below `30.0` m, so `distance_allowed=true` was
observed. The run was still blocked because it stayed mostly in MANUAL,
`timeout_ok=false`, `safety_ready=false`, and candidate commands remained zero.
This is partial progress, not a successful AUTO candidate dry-run.

Timeout semantics update: the single-waypoint experiment now reports
`timeout_source=auto_entry`, `auto_entry_ms`, `auto_elapsed_ms`,
`timeout_limit_ms`, and `timeout_ok`. MANUAL should show `auto_entry_ms=NA` and
`auto_elapsed_ms=NA`; after switching to AUTO, those fields should become
numeric and `timeout_ok=true` until the AUTO timeout limit is exceeded.

Latest post-timeout-fix outdoor attempt: timeout fields and target override were
confirmed, but GPS did not acquire a fix. `gps_chars` increased continuously,
while `gps_location_valid=false`, `gps_lat=NA`, `gps_lon=NA`, `gps_sats=0`,
and `gps_hdop=99.99`. This is a GPS no-fix blocked validation, not an AUTO
candidate success. Reacquire stable outdoor GPS fix in MANUAL before attempting
AUTO_READY validation.

GPS readiness update: `gps_location_valid=true` only means TinyGPS has a cached
location. GPS readiness is now tiered: `gps_dryrun_ready=true` may be used for
no-motion candidate calculation with HDOP up to `6.0`, while `gps_motion_ready`
and `gps_ready` remain stricter motion-level gates. HDOP around `5` is
acceptable only for no-motion dry-run diagnostics, not floor driving. Cached
coordinates, if any, are printed only as `gps_cached_lat`, `gps_cached_lon`,
and `gps_cached_age_ms`.

Safety gates include GPS readiness, coordinate sanity, RC validity, AUTO switch
state, target validity, target distance range, arrival radius, and AUTO timeout.
`AUTO_MOTION_ARMED=1` alone no longer produces motion; armed motion is gated to
zero unless `GROUND_CRAWL_TEST_MODE=1` is also set, which applies the guarded
ground crawl harness (command clamp to ±`GROUND_CRAWL_MAX_CMD`, hard latch stop
after `GROUND_CRAWL_MAX_AUTO_MS`, neutral-RC/motion-GPS/near-field-target gates).
The armed build is reserved for a wheel-off-ground / open-area bench test after
explicit approval. This mode does not load `mission.json` and does not implement
multi-waypoint or lawnmower/coverage driving.

### Standalone GPS Probe

Compile for confirmed fixed GPS wiring:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 firmware/gps_uart_probe
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Breadboard Navigation Development Stack

Safe indoor workflow for building the GPS + IMU + HC-12 navigation stack with no
motors and no rover chassis. Full details and exact commands are in
[`firmware/README.md`](firmware/README.md) ("Breadboard Navigation Development
Stack"). Summary:

- IMU diagnostics: use `firmware/imu_bmi160_normal_probe` for the current BMI160
  (`0x68`, `chip_id=0xD1`). The legacy `firmware/imu_probe` is MPU/ICM-only and
  is not a BMI160 health check. Gyro yaw remains relative and diagnostic-only.
- Integrated GPS + IMU + HC-12 dry-run: `firmware/openrb_robot_controller` with
  `-DPATH_FOLLOWING_DRYRUN=1 -DIMU_ENABLE=1 -DIMU_HEADING_DRYRUN=1` (HC-12 on
  Serial3) prints path-following + IMU yaw + a GPS-course-vs-IMU-yaw comparison,
  with motors disabled. `IMU_ENABLE=0` keeps the default build unchanged.
- HC-12 link check: `firmware/hc12_link_probe` (firmware) + `tools/hc12_link_probe.py`
  (station). Motor-free PING/PONG; never sends motor commands.
- GPS check: `firmware/gps_uart_probe` (keeps printing `gps_block_reason` and raw
  status even with no indoor fix).
- Path-planning preview: `tools/path_planning_preview.py` (preview-only CSV +
  markdown + optional PNG from manual coordinates).
- Side-mounted cleaning-tool preview: `tools/side_tool_path_preview.py`
  now defaults to a small, tool-centered serpentine CLI. A is the top-left
  tool/paint-tank center start and B is the bottom-right tool/paint-tank center
  end. The normal inputs are only A, B, `--step-spacing-m`, robot dimensions,
  tool dimensions/offset, and `--tool-side`. The tool center path is generated
  first; the rover chassis path is derived afterward. Sweep tracks are active by
  default, while connectors and rotations are tool-inactive by default. It emits
  only `move_forward`, `move_backward`, `rotate_left`, and `rotate_right`
  preview primitives and produces no rover motor commands. The simple preview
  writes `tool_path.csv`, `primitive_sequence.csv`, `summary.md`,
  `preview_route_sequence.md`, `preview_tool_path_primary.png`,
  `preview_chassis_derived_from_tool.png`, `preview_primitive_sequence.png`, and
  `preview_tool_coverage_only.png`. Legacy/debug planner options are available
  only with `--advanced`; `tools/preview_side_tool_path.py` is a compatibility
  alias that opts into advanced mode.
- Tool-centered A/B side-tool preview is documented in
  [docs/side_tool_path_planning.md](docs/side_tool_path_planning.md). The reset
  readiness gate is intentionally narrow: the tool path must start at A, end at
  B, remain continuous, have one connector between adjacent tracks, derive the
  chassis path from tool-space geometry, emit only the four preview primitives
  (`move_forward`, `move_backward`, `rotate_left`, `rotate_right`), and keep
  `motor_command_generated=False`. Boundary, swept-volume, and contamination
  checks remain optional diagnostic layers rather than default route-shaping
  constraints for `tool_serpentine_ab`.
- Side-tool tracking simulation: `tools/simulate_side_tool_tracking.py` consumes
  `side_tool_path.csv` and writes offline `virtual_*` tracking diagnostics only.
  See [docs/feedback_tracking_design.md](docs/feedback_tracking_design.md).
- Side-tool waypoint preview: `tools/preview_side_tool_waypoints.py` exports
  the same preview geometry as offline target waypoints with bearings, segment
  labels, expected headings, reverse-direction flags, and
  `motor_command_generated=False`. It is not a rover command file.
- Firmware path-following dry-run: `-DPATH_FOLLOWING_DRYRUN=1` computes
  distance/bearing/heading/steering and runs the HC-12 waypoint protocol with
  motors disabled.

UART rule: OpenRB-150 has three hardware UARTs (`Serial1` D26/D27, `Serial2`
D28/D29 = GPS, `Serial3` D14/D13). The current integrated no-motion build uses
GPS on `Serial2` and HC-12 on `Serial3`, so there is no software UART conflict.
The HC-12 RF link is still deferred/unproven. Physical path following stays
blocked (all four motion gates and `PATH_FOLLOWING_MODE_CHANNEL_STABLE` default
to 0) until RC/PPM, heading, GPS, and station safety are validated.

### Serial3 Loopback Test

Historical UART pin test. Under the Fixed Wiring Plan, do not move GPS or
HC-12 unless there is an explicit hardware bench-test reason.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 --build-property 'compiler.cpp.extra_flags=-DSERIAL3_LOOPBACK_BAUD=9600' firmware/serial3_loopback_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 firmware/serial3_loopback_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Pin Finder Test

Historical physical pin finder. Under the Fixed Wiring Plan, do not move GPS or
HC-12 unless there is an explicit hardware bench-test reason.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Do Not Repeat

- Do not expect GPS in the default controller build under current fixed wiring;
  `gps_chars=0` is expected there because default firmware still reads GPS from
  `Serial3`.
- Do not expect manual driving in `FIXED_WIRING_GPS_SERIAL2_DIAG`; motors are
  neutral and HC-12 is disabled by design.
- Do not connect both OpenRB USB and station USB-serial during OpenRB upload if
  `arduino-cli` selects the wrong upload port.
- If upload fails because it selected `/dev/cu.usbserial-02444963`, unplug the
  station USB-serial and upload with only OpenRB connected.

### Next Architecture

For current fixed wiring, the future architecture should be a GPS `Serial2` +
RC-controlled onboard mode:

- Auto OFF: RC manual drive.
- Auto ON: onboard GPS mission/autonomy, after explicit safety design and tests.
- HC-12 is not used in this mode until hardware can be revised or proven
  independent from GPS `Serial2`.
- Station-side path planning remains dry-run until autonomy is explicitly
  implemented and safety-gated.

## Troubleshooting

### GPS In Default Rover Controller

Under current fixed wiring, do not treat default-build `gps_chars=0` as GPS
failure. The default rover controller still reads GPS from `Serial3`, while the
actual fixed GPS wiring is on `Serial2`. This build is kept as the HC-12/manual
baseline.

### GPS Serial2 Diagnostic Sky Test

The `FIXED_WIRING_GPS_SERIAL2_DIAG` build has now been sky-tested:

- `fixed_wiring_gps_serial2_diag=true`
- `hc12_enabled=false`
- `gps_chars` increased continuously
- `gps_fix=true`
- `gps_lat` / `gps_lon`, `gps_sats`, and `gps_hdop` became valid
- motors remained disarmed/neutral

This confirms GPS works through the fixed `Serial2` wiring inside integrated
firmware when HC-12 is disabled for the diagnostic mode. The successful fix
occurred after moving the external GPS antenna farther outside into clearer sky
view.

### GPS Data But No Fix

If `gps_chars>0` but `gps_ready=false`, GPS UART data is arriving but usable
position quality is not sufficient yet. Indoor, window-side, or temporary
outdoor tests may receive NMEA bytes without acquiring a valid fresh fix. For
first fix, place the GPS antenna outdoors with open sky view and wait before
changing firmware.

Latest GPS-only `Serial2/9600` probe result: GPS UART receive is alive, but
satellite fix is intermittent. Most lines showed RMC `V`, GGA fix quality `0`,
`sats=0`, and `hdop=99.99`; a few short bursts reached RMC `A`, valid lat/lon,
`sats=4..5`, and `hdop≈1.77..2.48`, then returned to no-fix. Treat this as a
GPS acquisition blocker, not a target override, RC, timeout, UART, or baudrate
problem. Do not proceed to AUTO dry-run, bench test, or floor driving until a
stable GPS fix is observed.

The standalone GPS probe now prints `gps_probe_state=NO_FIX`,
`INTERMITTENT_FIX`, or `STABLE_FIX`. Treat `STABLE_FIX` as the required probe
result before returning to AUTO dry-run validation.

Latest recovery: after moving the rover/GPS farther outdoors with clearer sky
view, the `Serial2/9600` probe reached `gps_probe_state=STABLE_FIX` with
`valid_fix_seconds_consecutive=58..60`, RMC `A`, GGA quality `2`, `sats=9`,
`hdop=3.56`, and lat/lon around `35.57029,129.187078`. This confirms GPS
module and UART operation; placement/sky view was the main no-fix cause. This
does not approve floor driving.

Main-controller outdoor validation then recovered both GPS and AUTO gates:
`gps_dryrun_ready=true`, `gps_motion_ready=true`, `gps_ready=true`,
`gps_block_reason=OK`, RMC `A`, GGA quality `2`, `gps_sats≈9..11`,
`gps_hdop≈1.46`, `mode=AUTO_READY`, `auto_sw=true`, and `timeout_ok=true`.
The run was still blocked safely because the compile-time target was stale:
`target_distance_m≈41` exceeded `max_target_distance_m=30.0`, so
`distance_allowed=false`, `safety_ready=false`, and candidate commands stayed
zero. The next autonomy step is main-controller `AUTO_MOTION_ARMED=0` dry-run
only after recomputing a target within roughly `5..15` m of the current outdoor
GPS position.

Latest no-motion AUTO waypoint dry-run succeeded with target
`35.5705010,129.1872696`: `target_distance_m≈8.4..15.2`,
`distance_allowed=true`, `mode=AUTO_READY`, `timeout_ok=true`,
`safety_ready=true`, `candidate_left_cmd=0.100`, and
`candidate_right_cmd=0.100`. Because `AUTO_MOTION_ARMED=0`,
`auto_motor_inhibit=true` kept `final_left_cmd=0.000` and
`final_right_cmd=0.000`. This validates candidate command generation only; it
does not approve floor driving. The next step is wheel-off-ground bench
testing.

On 2026-05-29 the armed build (`AUTO_MOTION_ARMED=1`) then reached firmware-side
final output for the first time: `mode=AUTO_RUNNING`, `auto_motor_inhibit=false`,
motion-grade GPS, all gates passing, and `final_left_cmd=0.100` /
`final_right_cmd=0.100`. No visible rover movement occurred (motor/ESC/friction
deadband: `0.100` ≈ 1530 µs, only 30 µs above the 1500 µs neutral). Returning to
MANUAL drove final commands to `0.000`. This is firmware-side armed-output
success, NOT a physical ground crawl, and floor driving is NOT approved. The AUTO
command must not be raised ungated; armed motion is now permitted ONLY through the
guarded ground crawl build (`GROUND_CRAWL_TEST_MODE=1`), which clamps to
±`GROUND_CRAWL_MAX_CMD` (default `0.08`) and hard-latches a stop after
`GROUND_CRAWL_MAX_AUTO_MS` (default `1200` ms, clears only on MANUAL). Step the
cap up only via `-DGROUND_CRAWL_MAX_CMD`, under latch protection.

Latest guarded crawl 0.08 validation succeeded as a safety-harness test:
`GROUND_CRAWL_TEST_MODE=1` and `AUTO_MOTION_ARMED=1` were active, a good GPS
window reached `AUTO_RUNNING`, `ground_crawl_ready=true`,
`ground_crawl_block_reason=OK`, candidate commands `0.100` / `0.100` were
clamped to final commands `0.080` / `0.080`, and the duration latch later forced
zero output. The harness also blocked a too-close `3.9..4.4` m target as
`DISTANCE_OUT_OF_RANGE` and blocked degraded GPS as `GPS_NOT_MOTION_READY`.
For any future guarded crawl, reacquire current GPS and compute a fresh target.
The historical 0.08/0.12 tests established that raising only
`GROUND_CRAWL_MAX_CMD` changes the cap, not the `candidate_left_cmd` /
`candidate_right_cmd` generated by the waypoint candidate logic. This remains
not full autonomous driving.

The latest `GROUND_CRAWL_MAX_CMD=0.120` cap-only run confirmed that distinction:
the guarded crawl system reached `AUTO_RUNNING`, but the candidate command
remained `0.100`, so final commands also remained `0.100`. Use
`SINGLE_WP_CRAWL_BASE_CMD` to change candidate speed and keep
`GROUND_CRAWL_MAX_CMD` as the final safety clamp.

First successful guarded AUTO crawl after the manual/drive mapping fix:
`MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`,
`old_angle_remap_active=false`, physical A/B mapping
`A=(logical_left+logical_right)/2`, `B=(logical_right-logical_left)/2`,
`SINGLE_WP_CRAWL_BASE_CMD=0.220`, and `GROUND_CRAWL_MAX_CMD=0.220`.
During `AUTO_RUNNING`, GPS and crawl gates passed (`gps_motion_ready=true`,
`gps_block_reason=OK`, `gps_sats≈9`, `gps_hdop≈1.0..1.2`,
`target_distance_m≈9.6`, `distance_allowed=true`, `ground_crawl_ready=true`,
`ground_crawl_block_reason=OK`). The command was straight throttle:
`left_cmd=0.220`, `right_cmd=0.220`, `final_left_cmd=0.220`,
`final_right_cmd=0.220`, `physical_a_cmd=0.220`, `physical_b_cmd=0.000`, and
the rover briefly moved forward. The duration latch then worked
(`ground_crawl_elapsed_ms≈510`, `ground_crawl_latched_stop=true`) and final
outputs returned to zero. This confirms short guarded forward motion only, not
full waypoint following or coverage driving.

Repeated 1000 ms guarded AUTO crawl result: the same fixed manual/drive mapping
was used with `GROUND_CRAWL_TEST_MODE=1`, `GROUND_CRAWL_MAX_CMD=0.220`,
`GROUND_CRAWL_MAX_AUTO_MS=1000`, `SINGLE_WP_CRAWL_BASE_CMD=0.220`,
`AUTO_MOTION_ARMED=1`, `MANUAL_FORWARD_SIGN=-1`, and `MANUAL_TURN_SIGN=1`.
The user toggled AUTO/MANUAL about `3..4` times and `AUTO_RUNNING` was observed
multiple times. In valid AUTO windows, GPS and crawl gates passed:
`gps_block_reason=OK`, `gps_motion_ready=true`, `distance_allowed=true`,
`ground_crawl_ready=true`, `ground_crawl_block_reason=OK`, `gps_sats≈8..10`,
`gps_hdop≈1.0..1.65`, and `last_gga_fix_quality=2`. Straight output was
repeated: `left_cmd=0.220`, `right_cmd=0.220`, `final_left_cmd=0.220`,
`final_right_cmd=0.220`, `physical_a_cmd=0.220`, `physical_b_cmd=0.000`. The
latch stopped output after roughly `1000` ms; one attempt ended earlier because
the user returned to MANUAL. `target_distance_m` varied around `16.8..18.0`
rather than decreasing monotonically, which is expected because this crawl only
drives straight and has no steering correction yet. This proves repeated short
guarded autonomous forward actuation, not path planning execution.

Single-waypoint steering dry-run diagnostics are available but do not enable
motion by themselves:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-steering-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1 -DAUTO_MOTION_ARMED=0 -DSINGLE_WP_STEERING_DRYRUN=1 -DSINGLE_WP_TARGET_LAT=35.570932 -DSINGLE_WP_TARGET_LON=129.187338' firmware/openrb_robot_controller
```

This build prints `heading_ready`, `heading_source`, `estimated_course_deg`,
`bearing_error_deg`, `desired_forward_cmd`, `desired_turn_cmd`,
`desired_logical_left_cmd`, `desired_logical_right_cmd`,
`desired_physical_a_cmd`, `desired_physical_b_cmd`, and
`steering_block_reason`. GPS-only position gives bearing to target, but not
rover heading; steering diagnostics require course-over-ground from movement.

GPS-independent motor pulse calibration is now available with
`MOTOR_PULSE_TEST_MODE=1` for drivetrain deadband checks when GPS is
intermittent. This mode disables HC-12, does not use GPS readiness or waypoint
distance, preserves RC MANUAL driving, and emits one AUTO-only pulse only after
`rc_ok=true` and centered steering/throttle. After `MOTOR_PULSE_MS`, it latches
zero output until the operator returns to MANUAL.

Current drivetrain calibration state: `MOTOR_PULSE_CMD=0.180` produced valid
software output but no visible motion; `MOTOR_PULSE_CMD=0.220` produced visible
motion. The 0.22 log showed symmetric software output (`left_cmd=0.220`,
`right_cmd=0.220`, `motor_pulse_ready=true`,
`motor_pulse_block_reason=OK`), but the rover appeared to rotate rather than
drive straight. Manual RC forward motion tends to drift/curve left, and backward
motion tends to drift/curve right. Do not proceed to GPS path planning yet; the
next step is differential left/right motor pulse calibration. The firmware now
supports `MOTOR_PULSE_LEFT_CMD` / `MOTOR_PULSE_RIGHT_CMD` plus a shared
`DRIVE_CALIBRATION_ENABLE` layer with sign, scale, and minimum-command
compensation applied consistently to MANUAL, station manual, AUTO, and motor
pulse outputs. Defaults are identity/off, so normal builds are unchanged.

Earlier differential pulse observations were confounded by the physical output
pin roles. Code inspection confirmed motor pulse output bypasses RC stick angle
remapping, and the physical pin probe has now confirmed A is throttle and B is
turn. Validate the corrected logical-wheel mapping with both-wheel
forward/reverse tests before any side compensation or path planning.

Latest pin-path status: `firmware/physical_output_pin_probe` confirmed physical
pin A is throttle and physical pin B is turn. `MOTOR_PULSE_LEFT_CMD` and
`MOTOR_PULSE_RIGHT_CMD` remain logical wheel commands. The integrated controller
now converts logical wheels to physical pins with `A=(L+R)/2` and
`B=(R-L)/2`. The fields `output_left_pin_cmd` and `output_right_pin_cmd` are
compatibility aliases for physical A/B pin commands, not physical left/right
wheel commands. Single-wheel logical tests are halved at the physical pin level,
so use both-wheel forward/reverse first when validating the mapping.

Manual RC now uses the same logical-wheel output pipeline as AUTO and motor
pulse. The manual path computes `manual_forward_cmd` from throttle,
`manual_turn_cmd` from steering, then mixes `left=forward+turn` and
`right=forward-turn` before the common physical A/B conversion. The old
upper-right-is-forward symptom was a manual mixing bug, not GPS/path planning.
The working sign convention for this rover/controller is
`MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`, `MOTOR_OUTPUT_SWAP_LR=0`, and
`DRIVE_CALIBRATION_ENABLE=0`. This is only an RC axis sign fix; preserve the
physical A/B mapping `A=(L+R)/2`, `B=(R-L)/2`. USBDBG should show
`old_angle_remap_active=false`.

Recommended manual RC validation build:

```bash
cd ~/Desktop/project-lab/gps_hc12_robot && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && mkdir -p outputs/logs && arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-manual-final-sign --build-property 'compiler.cpp.extra_flags=-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0' firmware/openrb_robot_controller && arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-manual-final-sign firmware/openrb_robot_controller && sleep 2 && PORT=$(arduino-cli board list | awk '/OpenRB-150/ {print $1; exit}') && arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee outputs/logs/manual_final_sign_$(date +%Y%m%d_%H%M%S).log
```

Manual validation checklist: stick up = forward, stick down = backward, stick
right = right turn, stick left = left turn.

Do not use motor pulse logs to validate GPS. In `MOTOR_PULSE_TEST_MODE=1`, the
main controller intentionally skips GPS initialization and GPS byte processing,
so `gps_chars=0`, `last_rmc_status=NA`, `last_gga_fix_quality=NA`, and
`gps_block_reason=NO_LOCATION` are expected and do not contradict a successful
`firmware/gps_uart_probe` result. Keep the test flow separate:

1. GPS UART validation: `firmware/gps_uart_probe` on `Serial2/9600`.
2. Main-controller GPS validation: `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
   with `AUTO_MOTION_ARMED=0` and no `MOTOR_PULSE_TEST_MODE`.
3. Motor deadband validation: `MOTOR_PULSE_TEST_MODE=1`, ignoring GPS fields.

If `gps_chars=0`, debug wiring, selected UART, baudrate, power, or GPS output
configuration first.

GPS sky-fix checklist:

- `gps_chars` increasing means the selected UART and baudrate are working.
- RMC `V` / GGA quality `0` / `sats=0` / `hdop=99.99` means no usable current
  fix even when NMEA bytes are arriving.
- RMC `A` for one second is not enough; require stable fix across repeated
  lines.
- `gps_location_valid=true` can be stale cached TinyGPS data; it is not enough
  for target-distance computation.
- `gps_ready=true` is the stricter motion-level gate. In the no-motion
  single-waypoint dry-run, `gps_dryrun_ready=true` / `active_gps_ready=true`
  may be enough to compute diagnostic distance, bearing, and candidate commands
  while final motor outputs stay zero.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet, not a
  UART or firmware failure by themselves.
- Move the antenna outside or into open sky before suspecting code.
- Rain did not prevent fix during the observed test once the antenna had clear
  sky exposure, but electronics, USB adapters, and antenna connectors must be
  protected from water.

## Software Architecture

Current implementation is Python-based and pre-ROS2:

- `gps_coverage_core.protocol`: serial frame encoding/decoding and checksums.
- `gps_coverage_core.geo`: WGS84/local coordinate conversion helpers.
- `gps_coverage_core.planner`: simple planar A/B lawnmower path generation.
- `gps_coverage_core.nmea`: supported NMEA parsing helpers.
- `gps_coverage_core.telemetry`: GPS telemetry model.
- `tools/`: station-side utilities for safe serial loops, GPS logging, log
  analysis, NMEA replay, path previews, and mock mission generation.
- `firmware/`: OpenRB sketches for integrated rover control and focused bring-up
  tests.
- `ros2_ws/src/`: ROS2 Jazzy package skeletons for future bridge, mission,
  planner, and waypoint follower nodes.

Core protocol and planning modules are intentionally independent from ROS2 so
they can be used by plain Python tools now and reused during the ROS2 migration.

Planned ROS2 migration:

- `hc12_bridge_node`: bridge HC-12 serial frames into ROS2 topics/services.
- `station_mission_node`: manage operator workflow and mission state.
- `coverage_planner_node`: expose coverage path generation through ROS2.
- `waypoint_follower_node`: consume planned paths and produce rover commands.

These ROS2 nodes are not complete runtime behavior yet.

## Verified Progress

Verified or implemented so far:

- RC manual control on the rover.
- GPS module communication and GPS FIX.
- Failsafe STOP behavior.
- Safe station defaults that send heartbeat and `STOP`, not live `AUTO`.
- Mock lawnmower path planning and preview generation.
- Unit tests for protocol, geodesy, and planner behavior.

Pending:

- Station-side HC-12 USB device confirmation and end-to-end link test.
- Reconciliation of GPS telemetry payload schema across firmware and Python
  tools.
- ROS2 node integration beyond skeleton packages.
- Magnetic wheel adhesion validation.
- Full autonomous field test on the target surface.

## Directory Structure

```text
gps_coverage_core/        ROS-independent Python protocol, geo, telemetry, NMEA, planner
tools/                    Station utilities, log analyzers, mock mission generators
firmware/                 OpenRB/Arduino sketches and bring-up tests
ros2_ws/src/              ROS2 Jazzy package skeletons, not completed runtime nodes
tests/                    Python tests for protocol, geo, and planner modules
scripts/                  Environment, requirements, and ROS2 workspace helper scripts
docs/                     Architecture, protocol, safety, wiring, reports, figures
docs/project_notes/       Repository audit and cleanup notes
docs/figures/             Shared generated/raw/external figure library
docs/reports/interim/     Interim report workspace
docs/reports/final/       Final report workspace
data/                     Local logs and generated mock mission outputs
outputs/                  Local generated outputs
```

## Setup

Use `uv` for dependency and task execution:

```bash
uv sync --extra dev --extra web
```

Useful checks:

```bash
uv run pytest -q
uv run python tools/verify_env.py --port /dev/ttyACM0
./scripts/export_requirements.sh
```

## Running Current Components

Analyze GPS USB debug logs:

```bash
uv run python tools/analyze_gps_log.py data/gps_logs/*.log
```

Analyze safety USB debug logs:

```bash
uv run python tools/analyze_safety_log.py data/safety_logs/*.log
```

Generate a mock station-side coverage mission without HC-12 or ROS2:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir data/mock_runs/example
```

### Path Planning Preview Dry-run

Generate a start/goal station-side path preview from the latest field-test area.
This writes only local preview artifacts and does **not** open serial, send
HC-12 frames, generate rover motor commands, or upload firmware:

```bash
uv run python tools/path_planning_preview.py \
  --start-lat 35.571083 \
  --start-lon 129.187290 \
  --goal-lat 35.570932 \
  --goal-lon 129.187338 \
  --spacing-m 2.0 \
  --out-dir outputs/path_preview/latest_field_test
```

Outputs:

```text
outputs/path_preview/latest_field_test/waypoints.csv
outputs/path_preview/latest_field_test/summary.md
outputs/path_preview/latest_field_test/preview.png
```

This is preview-only, not autonomous execution. Physical path following still
requires single-waypoint steering dry-run plus heading/course estimation before
any rover follows these waypoints.

Generate a station-side coverage mission dry-run from GPS corner points. Point
A is the start corner, Point B is the opposite/end corner, and
`lane_spacing_m` is the sweep interval. The default `corner-rectangle` mode
does not use `sweep_width_m`. This writes JSON, CSV, and PNG files under
`outputs/missions/` and sends no rover commands:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571250,129.186300 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_corner_rectangle_smoke
```

Outputs:

```text
outputs/missions/codex_corner_rectangle_smoke/mission.json
outputs/missions/codex_corner_rectangle_smoke/mission.csv
outputs/missions/codex_corner_rectangle_smoke/preview.png
```

Inspect the generated files:

```bash
uv run python -m json.tool outputs/missions/codex_corner_rectangle_smoke/mission.json
head -n 12 outputs/missions/codex_corner_rectangle_smoke/mission.csv
tail -n 3 outputs/missions/codex_corner_rectangle_smoke/mission.csv
ls -lh outputs/missions/codex_corner_rectangle_smoke/preview.png
```

See [docs/station_path_planning.md](docs/station_path_planning.md). Path
generation remains dry-run only and must not be sent to the rover yet. The
tested mission output is not yet executed by the rover. Repeated guarded
forward crawl is complete, but steering and heading/course estimation are not
validated, so path planning remains preview-only.

Edge/remainder policy: if the rectangle extent is not exactly divisible by
`lane_spacing_m`, a small remaining margin at the edge is acceptable. Do not add
an extra lane outside the boundary just to remove that margin.

The previous A/B baseline plus sweep-width interpretation is retained only
behind `--planner-mode baseline-width --sweep-width-m ...` for comparison.

Generate a standalone path preview figure:

```bash
uv run python tools/path_preview.py \
  --lat-a 35.123456 --lon-a 129.123456 \
  --lat-b 35.123456 --lon-b 129.124556 \
  --spacing 5.0 \
  --output docs/figures/generated/path_preview.png
```

Generate report-ready mock mission artifacts in the shared figure area:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir docs/figures/generated/mock_mission_example
```

When the station HC-12 USB device is connected, run only safe heartbeat/receive
testing first:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0
```

The safe controller loop also defaults to heartbeat and periodic `STOP`:

```bash
uv run python tools/station_controller.py --port /dev/ttyACM0
```

Manual keyboard station testing exists, but it sends manual command frames and
must be treated as wheel-off-ground motor testing only:

```bash
uv run python tools/station_keyboard_manual.py --port /dev/ttyACM0 --max-speed 0.25
```

The keyboard tool starts in heartbeat-plus-`STOP` mode. Press `e` to arm station
manual control, press space to enable the deadman, then use `WASD` or arrow keys
for short manual pulses. Press `x` for local E-stop/disarm and `q` to exit; exit
sends repeated `STOP` frames.

See [docs/manual_control.md](docs/manual_control.md) for the current rover
firmware upload steps, RC direction mapping, USB debug checks, and station
manual-control procedure.

## Figure Gallery

Shared generated figures belong in
[docs/figures/generated/](docs/figures/generated/). This directory may be empty
until the figure-generation commands above are run.

Figure source classes are documented in [docs/figures/README.md](docs/figures/README.md):

- `generated/`: reproducible project-generated figures from scripts, logs, mock
  missions, diagrams, or processed data.
- `raw/`: original captures such as photos, screenshots, serial captures, or
  unedited exported plots.
- `external/`: third-party or reference figures with source and license notes.
- `thumbnails/`: small convenience previews derived from another source.

Report-specific figures can also be placed under:

- [docs/reports/interim/figures/generated/](docs/reports/interim/figures/generated/)
- [docs/reports/final/figures/generated/](docs/reports/final/figures/generated/)

## Report Material Map

- [docs/reports/interim/](docs/reports/interim/): interim report workspace,
  including generated/raw/external figure folders and tables.
- [docs/reports/final/](docs/reports/final/): final report workspace for
  verified claims and final evidence.
- [docs/project_notes/repo_audit.md](docs/project_notes/repo_audit.md):
  current repository audit and risk notes.
- [docs/project_notes/directory_structure_summary.md](docs/project_notes/directory_structure_summary.md):
  documentation and report folder map.
- [docs/project_notes/cleanup_summary.md](docs/project_notes/cleanup_summary.md):
  cleanup status and remaining artifact-policy decisions.

Final report claims should stay tied to verified evidence. Do not describe HC-12
station operation, ROS2 autonomy, magnetic adhesion, or full cleaning/painting
operation as complete until those items are implemented and tested.

## Limitations And Next Steps

Current limitations:

- Planner assumes a locally planar rectangular work region.
- No hull curvature handling, obstacle handling, edge exclusion zones, coating
  process constraints, or localization uncertainty margins yet.
- Magnetic wheel adhesion is a concept, not a validated subsystem.
- Station HC-12 USB and end-to-end radio communication remain pending.
- ROS2 packages are skeletons only.
- GPS payload schema needs alignment between firmware and Python telemetry
  parsing before relying on all station-side GPS tools.

Next steps:

- Confirm station-side HC-12 USB attachment and safe heartbeat/telemetry link.
- Align and document one GPS telemetry payload schema.
- Keep expanding mock mission evidence without claiming autonomous field
  completion.
- Implement ROS2 bridge, mission, planner, and follower nodes around the
  existing ROS-independent core modules.
- Validate magnetic wheel adhesion and safety behavior before any live hull test.
