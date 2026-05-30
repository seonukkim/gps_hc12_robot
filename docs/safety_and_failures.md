# Safety And Failure Handling

Safety is owned by the rover firmware and reinforced by conservative station
defaults. Station software may request motion, but it must not be treated as the
final safety authority.

## Non-Negotiable Rules

- Do not remove manual control.
- STOP overrides everything.
- Heartbeat timeout or stale command timeout must stop the rover.
- GPS fix loss must block future autonomous motion.
- Station startup must be dry-run or heartbeat plus STOP.
- Path generation must not move the rover.
- Rover motor tests are wheel-off-ground only.
- Do not change pin mappings, serial ports, or hardware assumptions without
  updating docs.

## STOP Sources

STOP can come from:

- station `CMD,STOP,0,0`
- station keyboard manual `x`
- station keyboard manual exit
- station safe controller periodic STOP
- rover stale station manual timeout
- rover link timeout during AUTO
- invalid RC / rover failsafe
- future station UI E-stop

Expected STOP behavior:

- motor outputs go neutral
- station manual command is cleared
- station auto command is cleared
- rover does not resume motion automatically after reconnection

## Current Rover Safety Gates

The integrated firmware evaluates control source in this order:

1. Station E-stop -> STOP.
2. Fresh station manual frame with `deadman=1` -> station manual.
3. Valid RC with mode-channel manual -> RC manual.
4. Valid RC with mode-channel auto-ready plus explicit station AUTO -> AUTO.
5. Otherwise -> STOP, `AUTO_READY`, `DISARMED`, or `FAILSAFE`.

This means a HIGH mode channel alone must not drive the rover. The mode channel
is the compile-time 0-based PPM index `MODE_CHANNEL_INDEX` (default `4` = CH5).

## Mode Channel Must Be Stable Before Path Following

Physical path following is blocked until the AUTO/MANUAL switch channel is
stable. A PPM hold test showed receiver CH5 did not hold HIGH:
`total_ch5_samples=68`, `ch5_high_auto_like=4`, `ch5_low_manual_like=64`,
`RESULT=CH5_AUTO_DID_NOT_HOLD`. When AUTO is raised, the firmware briefly enters
`AUTO_READY` then `FAILSAFE` because `ppm_age_ms` grows; this failsafe is correct
and must not be weakened to compensate for an unstable switch.

Rules:

- Use `firmware/ppm_channel_map_probe` (read-only; no GPS/HC-12/motors) with
  `tools/analyze_ppm_log.py` to identify a stable 2-position switch channel that
  reaches both LOW and HIGH and holds HIGH.
- Only then rebuild with `-DMODE_CHANNEL_INDEX=<0-based index>`; the default
  (CH5) is unchanged until proven.
- Path planning preview is allowed; physical path execution is not allowed until
  the mode channel holds and is verified in USBDBG (`mode_channel_index`,
  `raw_mode_channel_us`, `raw_ch1_us`..`raw_ch8_us`).
- Do not raise AUTO output, weaken failsafe, or relax `ppm_age` limits to work
  around the unstable switch.

## Guarded Ground Crawl (Armed-Motion Harness)

Armed AUTO motion is permitted ONLY through the guarded ground crawl harness.
On 2026-05-29 the armed build reached `final_left_cmd=0.100` /
`final_right_cmd=0.100` with all gates passing but produced no visible motion
(motor/ESC/friction deadband). The AUTO command must NOT be raised ungated.

Rules:

- Any armed build (`AUTO_MOTION_ARMED=1`) without `GROUND_CRAWL_TEST_MODE=1`
  holds final commands at zero. The crawl harness is the only path to motion.
- `SINGLE_WP_CRAWL_BASE_CMD` controls the candidate crawl command before final
  clamping. Its default is `0.100` to preserve current behavior.
- Final autonomous command is clamped to ±`GROUND_CRAWL_MAX_CMD` (default
  `0.08`).
- After `GROUND_CRAWL_MAX_AUTO_MS` (default `1200` ms) of continuous AUTO, the
  harness latches a hard stop (`final_left_cmd=0.000`, `final_right_cmd=0.000`,
  `ground_crawl_latched_stop=true`). The latch clears ONLY on a return to MANUAL.
- Crawl will not move unless RC sticks are neutral (`ground_crawl_neutral_ok`),
  GPS is motion-grade (`gps_motion_ready`), the single-waypoint safety gate
  passes (`safety_ready`), and the target distance is in the near-field window
  (`GROUND_CRAWL_MIN_TARGET_DISTANCE_M=5.0` to
  `GROUND_CRAWL_MAX_TARGET_DISTANCE_M=20.0`). Any failed gate forces zero output;
  the reason is printed as `ground_crawl_block_reason`.
- Raise candidate speed only via `-DSINGLE_WP_CRAWL_BASE_CMD` and keep
  `-DGROUND_CRAWL_MAX_CMD` as the final safety clamp. Increasing only
  `GROUND_CRAWL_MAX_CMD` raises the cap but does not raise
  `candidate_left_cmd` / `candidate_right_cmd`.
- Ground crawl is NOT full autonomous driving. Floor driving remains not
  approved.

Latest validation:

- The 0.08 guarded crawl test reached `AUTO_RUNNING` during a good GPS window
  with `ground_crawl_ready=true` and `ground_crawl_block_reason=OK`.
- Raw candidate commands `0.100` / `0.100` were clamped to
  `final_left_cmd=0.080` / `final_right_cmd=0.080`.
- The duration latch asserted `ground_crawl_latched_stop=true` and forced final
  commands to zero.
- When target distance later fell to roughly `3.9..4.4` m, the harness blocked
  further output as `DISTANCE_OUT_OF_RANGE` because the crawl minimum is `5.0`
  m.
- GPS degradation also blocked output as `GPS_NOT_MOTION_READY` or
  `LATCHED_STOP`.
- Historical 0.08/0.12 tests established that `SINGLE_WP_CRAWL_BASE_CMD`
  controls candidate speed and `GROUND_CRAWL_MAX_CMD` is only the final clamp.
  Reacquire current GPS and compute a fresh target before any future crawl; do
  not reuse stale or too-close targets.

First successful guarded forward crawl:

- After the manual/drive mapping fix, MANUAL RC uses
  `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`, and
  `old_angle_remap_active=false`.
- Physical output mapping remains A = throttle, B = turn,
  `A=(logical_left+logical_right)/2`, `B=(logical_right-logical_left)/2`.
- With `SINGLE_WP_CRAWL_BASE_CMD=0.220` and `GROUND_CRAWL_MAX_CMD=0.220`, the
  rover briefly moved forward under guarded `AUTO_RUNNING`.
- The successful line had `gps_motion_ready=true`, `gps_block_reason=OK`,
  `target_distance_m≈9.6`, `distance_allowed=true`,
  `ground_crawl_ready=true`, `ground_crawl_block_reason=OK`,
  `physical_a_cmd=0.220`, and `physical_b_cmd=0.000`.
- The latch worked at roughly `ground_crawl_elapsed_ms=510`:
  `ground_crawl_latched_stop=true` and final outputs returned to zero.
- This approves only short guarded crawl validation. It does not approve full
  waypoint following, coverage driving, or ungated AUTO output.

Repeated 1000 ms guarded forward crawl:

- Tested with `GROUND_CRAWL_TEST_MODE=1`, `GROUND_CRAWL_MAX_CMD=0.220`,
  `GROUND_CRAWL_MAX_AUTO_MS=1000`, `SINGLE_WP_CRAWL_BASE_CMD=0.220`,
  `AUTO_MOTION_ARMED=1`, `MANUAL_FORWARD_SIGN=-1`, and `MANUAL_TURN_SIGN=1`.
- AUTO/MANUAL was toggled about `3..4` times.
- `AUTO_RUNNING` was observed multiple times with `gps_block_reason=OK`,
  `gps_motion_ready=true`, `distance_allowed=true`, `ground_crawl_ready=true`,
  and `ground_crawl_block_reason=OK`.
- Straight output repeated: `left_cmd=0.220`, `right_cmd=0.220`,
  `final_left_cmd=0.220`, `final_right_cmd=0.220`,
  `physical_a_cmd=0.220`, `physical_b_cmd=0.000`.
- The latch stopped output after roughly `1000` ms. One attempt was shorter
  because the user returned to MANUAL early.
- `target_distance_m` varied around `16.8..18.0` instead of monotonically
  decreasing. This is expected because the current crawl is straight only and
  does not steer toward the waypoint.
- This proves repeated short guarded autonomous forward actuation, not path
  planning execution.

## Single-Waypoint Steering Dry-Run Safety

`SINGLE_WP_STEERING_DRYRUN=1` is diagnostic only:

- It does not drive motors by itself.
- It does not relax `AUTO_MOTION_ARMED` or `GROUND_CRAWL_TEST_MODE`.
- It must not use `target_bearing_deg` alone as a steering command.
- GPS position gives target bearing, but not rover heading.
- Heading is considered ready only when course-over-ground can be estimated from
  at least `2.0` m of GPS displacement.
- If movement is too small, USBDBG must show `heading_ready=false` and
  `steering_block_reason=NO_HEADING`.

Expected diagnostic outputs include `estimated_course_deg`,
`bearing_error_deg`, `desired_forward_cmd`, `desired_turn_cmd`,
`desired_logical_left_cmd`, `desired_logical_right_cmd`,
`desired_physical_a_cmd`, and `desired_physical_b_cmd`. These are desired
values for review, not motor execution approval.

## GPS-Independent Motor Pulse Calibration

`MOTOR_PULSE_TEST_MODE=1` is a separate deadband-calibration mode for cases
where GPS-gated crawl tests are too slow or noisy.

Rules:

- HC-12 is disabled/ignored.
- GPS readiness and waypoint target distance are not used.
- RC MANUAL behavior is preserved.
- AUTO emits a pulse only when `rc_ok=true` and steering/throttle are neutral.
- The pulse command is `MOTOR_PULSE_CMD` (default `0.15`) for
  `MOTOR_PULSE_MS` (default `300` ms).
- After the pulse duration, the firmware latches stop:
  `final_left_cmd=0.000`, `final_right_cmd=0.000`,
  `motor_pulse_latched_stop=true`.
- The latch clears only after returning to MANUAL.
- USBDBG must be checked for `motor_pulse_ready=true` and
  `motor_pulse_block_reason=OK` before interpreting physical movement.
- GPS-looking USBDBG fields in this mode are not meaningful. `gps_chars=0`,
  `last_rmc_status=NA`, `last_gga_fix_quality=NA`, and
  `gps_block_reason=NO_LOCATION` are expected because GPS is not initialized or
  read.
- This mode is for motor deadband calibration only. It is not path following,
  not GPS autonomy, and not coverage driving.

Latest calibration state:

- `MOTOR_PULSE_CMD=0.180` produced valid software output but no visible motion.
- `MOTOR_PULSE_CMD=0.220` produced visible motion with symmetric software
  output (`left_cmd=0.220`, `right_cmd=0.220`,
  `motor_pulse_ready=true`, `motor_pulse_block_reason=OK`).
- The physical motion looked more like rotation than straight forward motion.
- Manual RC forward motion tends to drift/curve left; backward motion tends to
  drift/curve right.
- Differential pulse checks show basic polarity is likely correct:
  left-only `+0.22` rotates the left wheel forward, and right-only `+0.22`
  rotates the right wheel forward.
- Symmetric pulse checks still curve: both `+0.22/+0.22` curve/rotate right,
  and both `-0.22/-0.22` curve left while reversing.
- Code inspection confirms motor pulse output bypasses RC stick angle remapping;
  RC sticks are only checked for neutral before the direct pulse command.
- Later `+0.25` direct-pulse checks showed the current PWM inputs behave like a
  steer/throttle pair: left-only command also moved the right wheel backward,
  and right-only command behaved like forward throttle. Firmware now converts
  `MOTOR_PULSE_LEFT_CMD` / `MOTOR_PULSE_RIGHT_CMD` from direct wheel commands to
  those PWM inputs only at the final pin-write stage.
- Latest evidence shows the logical wheel commands themselves are correct.
  `firmware/physical_output_pin_probe` confirmed physical output A is throttle
  and physical output B is turn, so the integrated controller now maps
  logical wheels with `A=(left+right)/2` and `B=(right-left)/2`.

Safety decision:

- Do not proceed to GPS path planning or waypoint-driving tests until low-level
  corrected direct wheel pulse behavior is validated, then drivetrain asymmetry
  is characterized.
- The next motor tests should be both-wheel forward/reverse mapping validation
  before single-wheel or side compensation tests.
- Any future correction must live in a shared drive calibration layer used by
  both MANUAL and AUTO, not in path planning.
- The shared drive calibration layer is implemented behind
  `DRIVE_CALIBRATION_ENABLE=1`. Default identity/off values preserve normal
  behavior. Minimum command compensation applies only when the raw command is
  nonzero; zero remains zero.

## Station Defaults

Allowed by default:

- heartbeat
- STOP
- receive telemetry
- logs
- path generation dry-run

Not allowed by default:

- AUTO on startup
- movement immediately after plan generation
- movement without explicit operator approval

## GPS Fix Loss

Current status:

- rover firmware emits GPS validity in USB debug and GPS frames
- autonomous GPS-dependent behavior is not implemented yet

Required future behavior:

- no autonomous waypoint execution without valid GPS fix
- stale GPS age should be treated as invalid
- station UI should show GPS fix state before allowing mission approval
- rover should reject future autonomous command modes that require GPS when GPS
  is invalid or stale

## IMU / Heading Loss

BMI160 heading support is not implemented yet. Before heading is used for
control:

- document sensor wiring and bus
- document heading frame convention
- log heading quality
- define stale heading timeout
- keep manual and STOP independent from heading

## HC-12 Link Failures

Expected behavior:

- heartbeat loss must stop autonomous execution
- stale station manual frames must stop station manual motion
- reconnection must not auto-resume AUTO
- station logs should preserve raw TX/RX frames and parse errors

Relevant constants:

- rover station timeout: `STATION_TIMEOUT_MS = 500`
- station heartbeat default: `DEFAULT_HEARTBEAT_HZ = 5.0`

## Manual Direction Failures

Manual axis mapping has already caused repeated mistakes. Before changing manual
direction code:

1. Read `docs/manual_control.md`.
2. Capture USBDBG while moving the stick straight up/down/left/right.
3. Check `manual_forward_cmd`, `manual_turn_cmd`,
   `manual_logical_left_cmd`, and `manual_logical_right_cmd`.
4. Only then judge physical wheel motion.
5. If command values are correct but a wheel spins backward, fix motor/ESC
   direction separately from RC axis mapping.
6. Current confirmed RC sign convention is `MANUAL_FORWARD_SIGN=-1`,
   `MANUAL_TURN_SIGN=1`, `MOTOR_OUTPUT_SWAP_LR=0`, and
   `DRIVE_CALIBRATION_ENABLE=0`. This is an RC axis sign convention, not a
   physical A/B mapping change.

Manual validation checklist:

- stick up = forward
- stick down = backward
- stick right = right turn
- stick left = left turn

## Field Test Minimum Checklist

Before any ground-contact test:

1. Confirm wheels-off-ground direction and STOP behavior.
2. Confirm neutral output at startup.
3. Confirm station starts in heartbeat plus STOP.
4. Confirm current GPS fix state.
5. Confirm logs are being written.
6. Confirm manual control can recover the rover.
7. Confirm no path generation step sends motion.

## Failure Log Requirement

Every field or bench test should record:

- date/time
- firmware marker
- station command used
- serial port
- GPS fix state
- RC mode state
- observed failure or success
- STOP result
- files/logs produced

Use `docs/field_test_log.md` as the project-level index.

## Path-Following Dry-Run And Guarded Execution

`PATH_FOLLOWING_DRYRUN=1` adds waypoint distance/bearing/heading/steering
diagnostics and a motor-free HC-12 waypoint protocol. By itself it does not drive
motors.

Physical path-following output is impossible unless ALL of these hold. The four
compile gates and the mode-channel acknowledgement all default to `0`:

- compile: `PHYSICAL_PATH_FOLLOWING_ENABLE=1`,
  `PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1`, `GROUND_CRAWL_TEST_MODE=1`,
  `AUTO_MOTION_ARMED=1`, and `PATH_FOLLOWING_MODE_CHANNEL_STABLE=1`.
- runtime: `gps_motion_ready`, `heading_ready`, RC valid + AUTO switch on, RC
  sticks neutral, target distance within `[3.0, 20.0]` m, not arrived, no HC-12
  ESTOP, a fresh HC-12 target if one is used, and no active latch-stop.

Hard caps: forward `<= PATH_FOLLOWING_MAX_FORWARD_CMD` (0.18), turn
`<= PATH_FOLLOWING_MAX_TURN_CMD` (0.04), and a latch-stop after
`PATH_FOLLOWING_MAX_AUTO_MS` (500 ms) that clears only on MANUAL. The current
block reason is printed as `physical_block_reason` (default `COMPILE_GATE_OFF`).

This does not weaken STOP, failsafe, manual override, heartbeat, or the
wheel-off-ground rule. HC-12 path-following commands are never connected to motor
execution unless every gate above is satisfied. Physical path following is not
approved while the RC/PPM mode channel and heading source remain unvalidated.
