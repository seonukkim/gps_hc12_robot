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

## Hardware Overview

- Target surface: outer hull of a ship, currently approximated as planar.
- Adhesion concept: magnetic wheels, pending design and validation.
- Rover controller: OpenRB-150.
- Manual control: RC receiver with PPM input; RC manual mode has been verified.
- GPS: fixed on the central OpenRB connector, confirmed as `Serial2` at
  `9600` baud; GPS FIX has been verified.
- Radio link: HC-12 UART is the intended station-to-rover link. Station-side
  HC-12 USB confirmation and current rover-side wiring audit are still pending.
- Actuation: ESC/motor outputs are managed by rover firmware. Bench motor tests
  must remain wheel-off-ground.
- Station/development OS: Ubuntu 24.04. WSL2 Ubuntu 24.04 and Jetson are target
  station environments.

See [docs/current_hardware_status.md](docs/current_hardware_status.md),
[docs/wiring.md](docs/wiring.md), and [firmware/README.md](firmware/README.md).

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

## Next Required Validation Before Motion

- Confirm the station/controller is powered on and linked; controller-off can
  make RC appear stuck or failsafe-like.
- Verify Manual/Auto before autonomy dry-run: MANUAL should show
  `mode_us≈1000` and `control_source=RC_MANUAL`; AUTO should show
  `mode_us≈2000`, `mode=AUTO_READY`, and `control_source=STOP`.
- Recompute the single-waypoint target from the current GPS position before
  every guarded crawl attempt. The latest 0.08 run ended with the target too
  close (`target_distance_m≈3.9..4.4`), so those coordinates must not be reused.
- For the next guarded crawl attempt, use a fresh target roughly `10..12` m away
  so it remains inside the crawl window (`5..20` m).
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
- If 0.08 showed no visible physical movement, retry only through the same
  guarded crawl harness with `GROUND_CRAWL_MAX_CMD=0.12`, latch protection, and
  a fresh target. Do not raise ungated AUTO output.
- Do not approve full floor waypoint driving yet.

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
| RC channel probe | `firmware/rc_channel_probe` | OpenRB-150 | Identify which raw PPM channel changes for each RC stick/switch | not used | not used | no motor outputs |
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
  changed `manual_steer_cmd`, `manual_throttle_cmd`, `left_cmd`, and
  `right_cmd`.
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
Before another crawl, reacquire current GPS and compute a fresh `10..12` m
target. If 0.08 did not visibly move, the next cap is `0.12` under the same
latch protection. This is still not full autonomous driving.

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
tested mission output is not yet executed by the rover. The unified onboard
RC + GPS dry-run is complete, so the next autonomy step is single-waypoint
controlled motion preparation, not full coverage/lawnmower driving.

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
