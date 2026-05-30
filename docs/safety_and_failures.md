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
3. Valid RC with CH5 manual mode -> RC manual.
4. Valid RC with CH5 auto-ready mode plus explicit station AUTO -> AUTO.
5. Otherwise -> STOP, `AUTO_READY`, `DISARMED`, or `FAILSAFE`.

This means CH5 high alone must not drive the rover.

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
- Before any 0.12 retry, reacquire current GPS and compute a fresh target
  `10..12` m away. Compile the retry with both
  `SINGLE_WP_CRAWL_BASE_CMD=0.12` and `GROUND_CRAWL_MAX_CMD=0.12`. Do not reuse
  the too-close target.

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
3. Check `manual_steer_cmd` and `manual_throttle_cmd`.
4. Only then judge physical wheel motion.
5. If command values are correct but a wheel spins backward, fix motor/ESC
   direction separately from RC axis mapping.

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
