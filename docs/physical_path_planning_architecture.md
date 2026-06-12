# Physical Path Planning Architecture

`tools/physical_path_planning/` is the shared implementation behind the unified
field-facing CLI:

```bash
bash scripts/run_physical_path_planner.sh <mode> [options]
```

The package owns rectangle coverage geometry, calibration resolution, telemetry
parsing, guarded pulse execution, controller summaries, and read-only diagnostics.
Numbered development scripts remain only as legacy compatibility wrappers.

## Module Map

Imports flow one way:

```text
geometry  calibration  telemetry  safety  checks
                         \        /
                          executor
                          /      \
                 controller    alignment    tuning
                          \      |      /
                              cli
```

| Module | Responsibility |
|---|---|
| `geometry.py` | Goal resolution, diagonal-rectangle construction, serpentine/direct segment builders, projection metrics, B-axis correction, and GPS degradation policy. |
| `calibration.py` | Normalizes forward/backward/turn calibration and connector fallback behavior. |
| `telemetry.py` | USB debug row parsing and typed accessors for GPS, IMU, RC, and guarded pulse fields. |
| `safety.py` | Guarded motion predicates for ACK/STOP, final-zero checks, RC validity, output-active checks, and neutral preflight. |
| `checks.py` | Enforces `ready_for_full_path_following=false` on every summary. |
| `executor.py` | One guarded pulse transaction: ARM, command, completion wait, STOP, and row collection; plus bounded live-drive setpoints used for probes and straight chunks. |
| `controller.py` | Supervised segment loop that issues guarded pulses/live chunks along the planned coverage path. |
| `alignment.py` | Initial heading alignment: GPS displacement probe for absolute heading, IMU-feedback turn-to-heading, and the `gps_probe`/`user_confirmed`/`skip` strategies. |
| `tuning.py` | Interactive motion calibration candidate adjustment, approved-calibration persistence, and calibration backup/reset. |
| `cli.py` | User-facing modes: `diagnose`, `manual-rc`, `guarded-pulse-ready`, `calibrate-turn`, `tune-motion`, `reset-motion-calibration`, `calibration-check`, `preview`, `inspect-plan`, `align-heading`, `execute-plan`, `run`, and `auto-relative-run`. |

## Coverage Planner

The default preview/run geometry is `coverage_lawnmower`: a local-ENU
ㄹ/lawnmower sweep made from straight lane segments and drivable corner
transitions. For `relative_enu`, start A is local `(0,0)` and goal B
is `(goal_east_m, goal_north_m)`; the planner uses workspace width and step
spacing to sweep the area instead of following a direct diagonal. The explicit
`diagonal_rectangle_serpentine` shape remains available for the older A-B
diagonal frame, but it is not the default coverage mode.

### Connector style: turn_step_turn (default)

Each lane-to-lane transition is decomposed into three sub-segments the rover
can physically execute:

```text
connector_turn (pivot ~90)  ->  step_lane (straight, step_spacing_m)  ->  connector_turn (pivot ~90 back)
```

- After a FORWARD lane the step-over is driven forward; after a BACKWARD
  (reverse-driven) lane it is driven in reverse, so every full lane starts with
  the body already aligned with the lane axis. The ground pattern is
  `forward -> pivot -> short forward -> pivot -> reverse -> pivot -> short
  reverse -> pivot -> forward ...`.
- `connector_turn` segments are zero-length and carry a signed `turn_angle_deg`
  (+left / -right) computed from the actual body-heading geometry, so fields
  that step to the right of travel get right turns.
- Every segment carries `body_heading_deg`: the heading the BODY must hold,
  which is travel heading +180 on reverse-driven lanes.
- `--connector-style single_turn` restores the legacy single `path_connector`
  segment (one pivot whose sideways translation was never actually driven);
  old plan dirs containing `path_connector` segments still execute.
- `lane_count` keeps meaning full coverage lanes; `step_lane_count` and
  `connector_turn_count` are reported separately, and `connector_count` keeps
  meaning corner transitions (`lane_count - 1`).

## Guarded Pulse Contract

One physical pulse is:

```text
ARM -> command -> wait for completion -> STOP
```

The station records ACK/STOP telemetry and rejects the pulse if final motor
commands are nonzero or output remains active after STOP. Firmware telemetry may
still expose legacy internal field names; the unified CLI maps those into
field-facing terms such as `guarded_pulse_ready` and
`guarded_pulse_heartbeat_seen`.

## Control Law

Straight motion keeps calibrated A and pulse duration fixed:

- forward: `A=+0.30`, `800 ms`
- backward: `A=-0.08`, `300 ms`

Optional steering correction is applied only on the B axis:

```text
heading_component = clamp(k_heading * heading_error_deg, ±max_heading_b)
cte_component     = clamp(k_cte * cross_track_error_m, ±max_cte_b)
b_cmd             = clamp(heading_component + cte_component, ±0.08)
```

The controller does not silently lower calibrated A values or pulse durations.

The `stop_correct_go` control mode reuses this same B-only correction math but
applies it discretely: it drives one bounded calibrated chunk, stops, reads a
stabilized GPS/IMU heartbeat, and applies the heading correction (plus a small
bounded cross-track trim) only while stopped before continuing. Because it
drives calibrated chunks, it requires real forward calibration (and backward for
multi-lane serpentine plans); `calibration.calibration_completeness` gates it and
`run`/`auto-relative-run` abort before any motion with `CALIBRATION_INCOMPLETE`
when a required primitive is still the repeated-pulses fallback.

## Connector Turn Semantics (target_angle_deg)

A `turn_left_90` / `turn_right_90` calibration entry is a PULSE whose real
per-pulse rotation is its `target_angle_deg` -- on this rover often a small
15-45 degree twitch despite the `_90` key name. The executor must never assume
one pulse completes a corner:

- `calibration.connector_primitive` surfaces `target_angle_deg` (falling back
  to the measured `imu_yaw_delta_deg`, then 90).
- With IMU yaw available, a `stop_correct_go` connector (and the lane heading
  correction) turns in **burst -> stop -> measure** cycles: one bounded
  deadman live SET sized from the remaining angle and a turn-rate estimate
  (`target_angle_deg / pulse_ms`), then a full stop and a settled stationary
  yaw read before the next burst. In-motion feedback is not used because the
  firmware's MOTOR_TRACE stream saturates the serial link while motors run
  and starves yaw heartbeats (field data: every continuous-feedback turn ran
  blind to its time cap). Stops on tolerance
  (`--connector-turn-tolerance-deg`, default 10), overshoot (sign flip),
  stall (no measurable progress -- wrong-direction or stalled motors), REJECT,
  MANUAL switch, or `--max-connector-turn-ms` (default 20 s). Bursts are
  capped at the firmware-safe live-drive duration, so the guarded-pulse
  maximum (`COMMAND_EXCEEDS_MAX_MS`, baked in at upload) cannot trigger.
- Without IMU yaw, the open-loop count `ceil(|angle| / target_angle_deg)` of
  guarded pulses is used -- never blind extras -- with each pulse clamped to
  the firmware-safe 1000 ms (and the count scaled accordingly).
- `--turn-calibration-angle-policy assume_90` reproduces the legacy
  one-pulse-per-corner planning; `--turn-angle-deg-override` substitutes a
  measured per-pulse angle without editing the calibration JSON.
- `calibration-check` and `set-motion-calibration` print
  `TURN_CALIBRATION_IS_SMALL_PULSE_NOT_90` when a `turn_*_90` entry holds a
  pulse under 60 degrees.

## GPS Jump Guard (small fields)

`--max-gps-jump-m` (off by default) rejects GPS pose steps between
`stop_correct_go` cycles that exceed the threshold (floored at 3x one chunk's
dead-reckon advance), dead-reckoning that cycle instead and counting it as
`gps_jump_rejected`. On fields whose lane length is close to the GPS noise
floor (~5 sats / HDOP ~2 wanders meters), a single noisy fix can otherwise
"teleport" the pose and instantly complete a lane.

## Heading Reference (mission vs per_lane)

`stop_correct_go` defaults to `--heading-reference mission`: one yaw frame
(`heading = imu_yaw + offset`) is captured while the rover is still aligned
with the first lane and chained across the whole run. A connector under-turn
therefore shows up as next-lane heading error and is corrected by the existing
IMU turn-in-place. Reverse-driven lanes hold `body_heading_deg`
(travel +180). `--heading-reference per_lane` keeps the legacy behavior in
which each lane re-captures its reference yaw -- simple, but it silently
absorbs any connector error, which is what used to round every ㄹ corner into
a wide arc.

## AUTO/MANUAL Mode Switch Behavior

`auto-relative-run` waits for the physical PPM mode switch: flipping to AUTO
starts one execution of the planned relative-ENU path from the current GPS
fix; flipping back to MANUAL stops the rover (STOP/zero handshake still runs)
with `stop_reason=USER_SWITCHED_TO_MANUAL`, returning control to RC manual.
The MANUAL flip is honored between chunks, inside a pulse window (telemetry
rows are scanned), and during correction turns, so the worst-case stop latency
is bounded by one move chunk.

## Calibration Resolution

`resolve_physical_calibration` reads on-disk calibration JSONs when present and
falls back to the known safe repeated-pulse connector values when they are
missing. Turn angle calibration is performed through the unified `calibrate-turn`
mode, which internally uses the legacy guarded pulse calibration wrapper with IMU
yaw comparison.

## Safety Invariant

Full path following is not enabled by this package. Every preview, readiness
check, diagnostic summary, and physical execution summary must include:

```text
ready_for_full_path_following=false
```

`checks.assert_not_ready_for_full_path_following` enforces the literal `False`
value before summaries are written.

## Sensor Separation

- GPS is used for planning, logging, and progress estimates. BAD_HDOP can be a
  warning-only condition depending on policy.
- BMI160 relative yaw is used for turn monitoring and heading diagnostics, not as
  an absolute compass.
- Manual RC recovery is a separate first-class mode and should remain available
  after field firmware changes.
- HC-12 path control remains disabled in this workflow.
