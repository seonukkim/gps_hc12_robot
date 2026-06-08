# Physical Path Planning — Field Test Manual

Operator procedure for the unified path planner. Read
[README_physical_path_planning.md](README_physical_path_planning.md) for the CLI
overview and [physical_path_planning_architecture.md](physical_path_planning_architecture.md)
for what each module does. This manual is the order of operations and what to watch.

> The hardware modes (`calibrate-turn`, `run`/`execute-plan`) execute — flash firmware
> and/or open serial — as soon as they are invoked. The operator is responsible for
> satisfying the preconditions below before invoking them. The firmware 4-flag compile
> gate bounds motor output; this CLI never weakens it. Nothing here is full autonomous
> path following, and no summary ever reports `ready_for_full_path_following=true`.

## 0. Preconditions (do not skip)

Confirm against the root `README.md` and
[current_hardware_status.md](current_hardware_status.md):

- **GPS**: a stable outdoor fix — `gps_probe_state=STABLE_FIX`, RMC `A`, GGA quality
  ≥ 1, `gps_sats` healthy, `gps_hdop` low, and a *fresh* `gps_age_ms`. NMEA bytes
  arriving (`gps_chars>0`) is **not** a fix. For real navigation the GPS antenna must
  be rover-mounted; a detached antenna gives the antenna's position, not the rover's.
- **RC AUTO/MANUAL switch**: the mode channel must reliably hold. The known blocker is
  CH5 not holding HIGH; verify with `firmware/ppm_channel_map_probe` +
  `tools/analyze_ppm_log.py` and set `-DMODE_CHANNEL_INDEX` if a different channel is
  the stable 2-position switch. MANUAL ≈ `mode_us≈1000`, `control_source=RC_MANUAL`;
  AUTO ≈ `mode_us≈2000`, `mode=AUTO_READY`.
- **RC transmitter on** and the **station/controller powered and linked** — an off
  transmitter or controller can look like a stuck/failsafe RC.
- **Wheels**: for any first motion, keep it wheel-off-ground or in a clear open area,
  per the bench-test rule.
- **IMU (BMI160)** healthy at `0x68` (`chip_id=0xD1`). Yaw is relative/diagnostic; it
  supports heading-hold between pulses, not absolute localization.

Determine your port. Default is `/dev/ttyACM0` (Ubuntu station); on the Mac dev
machine it is `/dev/cu.usbmodem*` — pass `--port` explicitly. Telemetry baud is
`115200`.

## 1. Dry-run the plan first (no hardware)

Always preview before driving. This opens no serial and flashes no firmware.

```bash
uv run python -m tools.physical_path_planning.cli preview \
  --start-lat <A_lat> --start-lon <A_lon> \
  --goal-mode absolute --goal-lat <B_lat> --goal-lon <B_lon> \
  --workspace-width-m <short_side_m>
```

Check `outputs/physical_path_planning/preview/preview_summary.json` and
`preview.png`:

- `path_shape=diagonal_rectangle_serpentine`, `lane_count ≥ 1`, and
  `diagonal_length_m > workspace_width_m` (if width ≥ diagonal the build fails — A and
  B are corners of a diagonal, not a straight line).
- `connector_mode_effective`: `angle_calibrated` if you have a turn calibration, else
  `repeated_pulses` with `fallback_to_repeated_pulses=true` (uncalibrated connectors).

Remember: **A→B is the diagonal**, `--workspace-width-m` is the short side. For a
literal straight line use `--path-shape direct_line` (no width needed).

You can also preview a `run` exactly as it would execute, still with no serial:

```bash
uv run python -m tools.physical_path_planning.cli run --print-plan \
  --start-lat <A_lat> --start-lon <A_lon> --goal-lat <B_lat> --goal-lon <B_lon> \
  --workspace-width-m <short_side_m>
```

## 2. Calibrate the 90° connector turn (optional but recommended)

Without a turn-angle calibration the connectors fall back to repeated fixed pulses.
To calibrate, measure a real turn with IMU yaw. Inspect the shell-out first:

```bash
uv run python -m tools.physical_path_planning.cli calibrate-turn --mode turn_left --print-cmd
```

Then run it for real (this compiles the BMI160 yaw flags, uploads, and drives the
guarded turn via `scripts/run_stage20_physical_ab_probe.sh`):

```bash
uv run python -m tools.physical_path_planning.cli calibrate-turn \
  --port <PORT> --mode turn_left --target-angle-deg 90 \
  --save-turn-calibration true
```

Only with `--save-turn-calibration true`, a clean ACK→STOP handshake, and your visual
confirmation does it write
`outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json`.
If IMU yaw is unavailable it exits cleanly with
`imu_heading_block_reason=IMU_YAW_NOT_AVAILABLE` and writes nothing — no traceback.
Repeat for `--mode turn_right`. Once both are saved, re-run the `preview` and confirm
`connector_mode_effective=angle_calibrated`.

## 3. Run the guarded continuous motion

When the preview looks right and the preconditions hold:

```bash
scripts/run_physical_path_planner.sh run \
  --start-lat <A_lat> --start-lon <A_lon> --goal-lat <B_lat> --goal-lon <B_lon> \
  --workspace-width-m <short_side_m>
```

The launcher flashes the guarded-crawl firmware (bounded `MAX_ABS_A/B`, `MAX_MS`, IMU
yaw enabled, all path-following/stage flags 0), re-resolves the upload port, then
drives `controller.run_controller`. Keep the RC transmitter in hand — switching to
MANUAL or moving the sticks off neutral is your override.

Watch the live USBDBG stream and the run outputs under
`outputs/physical_path_planning/run/` (`run_summary.json`, `run_rows.csv`,
`run_serial.log`). The run stops cleanly and records an `abort_reason` on any field
fault:

| abort_reason | Meaning |
|---|---|
| `NONE` | Completed without an abort. |
| `NO_STAGE20_HEARTBEAT` | Firmware not exposing the guarded-crawl role; re-flash / check the port. |
| `RC_NOT_OK` | RC frame invalid at heartbeat; check transmitter/link. |
| `MANUAL_OVERRIDE` | Operator took manual control (default policy: abort). |
| `RC_NOT_NEUTRAL` | Sticks never settled to neutral within `--rc-neutral-wait-s`. |
| `GPS_DEGRADED` | GPS degraded under an `abort` policy (default policy is `continue`, dead-reckoned). |
| `RC_INVALID` | `RC_INVALID` reported during an active pulse — hard abort. |
| `SERIAL_DISCONNECT` | Serial link dropped mid-loop — hard abort. |

Per-pulse rows carry `valid_pulse`, `invalid_reason`, `gps_degraded`,
`cross_track_error_m`, `heading_error_deg`, and the `a_cmd`/`b_cmd` actually issued.
Useful knobs: `--gps-degradation-policy continue|pause|abort`,
`--manual-override-mode abort|continue`, `--rc-neutral-wait-s`, `--start-yaw-deg`.

## 4. Diagnose afterward (read-only)

Summarize what happened from the saved log (no serial), or watch live:

```bash
uv run python -m tools.physical_path_planning.cli diagnose \
  --from-log outputs/physical_path_planning/run/run_serial.log
# live:
uv run python -m tools.physical_path_planning.cli diagnose --port <PORT> --duration-s 5
```

The summary reports row/heartbeat counts, event tallies, the last heartbeat's
GPS/IMU fields, and whether the firmware exposed the STAGE20 guarded-crawl role. It
sends no commands.

## Safety reminders

- This is **guarded, bounded** motion via the STAGE20 guarded-crawl firmware — not
  full autonomous path following. The firmware compile gate is the real motor-output
  safety; never weaken it.
- Recompute A/B from the *current* GPS fix before each attempt; do not reuse old
  coordinates after moving the rover or antenna.
- HC-12 RF is deferred/unproven; the planner uses the USB serial link. Do not infer
  GPS, IMU, or motor readiness from HC-12, or from the wrong firmware mode.
- Never run a real `run`/`calibrate-turn` to "test" software changes — use
  `--print-plan` / `--print-cmd` / `--from-log` instead.
