# Physical Path Planning — Architecture

`tools/physical_path_planning/` is the single home for A→B serpentine coverage:
geometry, calibration, telemetry/safety parsing, guarded pulse execution, the
continuous-motion controller, and the unified CLI. This document is the module map
and the data/safety contracts. For usage see
[README_physical_path_planning.md](README_physical_path_planning.md); for the
operator procedure see [field_test_manual.md](field_test_manual.md).

## What this consolidated (and why)

The project had grown by accreting one-off "stages": `stage30`–`stage36` modules
plus their `run_stage3x.sh`, `check_*`, and `build_*` tools, with copy-pasted
serial/telemetry/safety code and a standalone `physical_calibration_resolver`. The
proven logic for A→B coverage, calibration, and guarded motion was already written
and tested — just scattered and duplicated. This package is an **extraction and
consolidation** of that logic behind one CLI, plus one genuinely new module (the
continuous-motion controller, which wraps existing control math). No new
path-planning algorithm was invented.

The superseded `stage30`–`stage36` modules, their scripts, and their redundant tests
were removed. Their behavior lives on here, covered by the `tests/test_ppp_*.py`
suite. The historical `docs/stage3x_*.md` notes are kept only as a record. `stage20`
(the physical A/B probe) is intentionally retained — `calibrate-turn` shells out to
it.

## Module map and import edges

Imports are strictly one-way (leaves first); nothing imports `controller`/`cli` back:

```
geometry  calibration  telemetry  safety  checks        (leaves)
                         \        /
                          executor                       (imports telemetry, safety)
                             |
                         controller                      (imports checks, executor,
                             |                             geometry, safety, telemetry)
                            cli                           (imports everything)
```

| Module | Responsibility |
|---|---|
| `geometry.py` | Goal-mode resolution (4 modes), A→B-diagonal rectangle + serpentine/direct segment builders, projection metrics, the `compute_b_correction` steering law, `gps_policy_action`, and `FALLBACK_RESOLVED_CALIBRATION`. Geodesy via `gps_coverage_core.geo`. |
| `calibration.py` | `resolve_physical_calibration` → the normalized motion schema (forward/backward/turn_left/turn_right/turn_left_90/turn_right_90/smooth_*), connector-mode selection (`auto` → angle-calibrated → smooth → repeated-pulses fallback), and the `connector_primitive`/`stage35_primitive` accessors. |
| `telemetry.py` | Shared `_parse_bool`/`_optional_float`/`_latest`/`_fmt` and the canonical USBDBG row accessors. Re-exports `parse_usbdbg_rows` from `station_path_package_tracker` (the tracker is not moved). |
| `safety.py` | Guarded-motion predicates: `rc_invalid_abort`, `missing_ack_or_stop_abort`, `output_active_after_stop`, `nonzero_final_cmd`, `rc_neutral_wait`, `preflight_heartbeat`, plus the `STOP_EVENTS` set. |
| `checks.py` | `assert_not_ready_for_full_path_following(summary)` — the load-bearing invariant guard (see below). |
| `executor.py` | The discrete guarded pulse FSM: `send_pulse` issues one ARM → command → await-completion → STOP pulse and returns its telemetry rows. Serial-facing; firmware still owns motor-output safety. |
| `controller.py` | The continuous-motion loop (the one new module). Supervises a sequence of `send_pulse` calls along the planned segments, holds heading from IMU yaw between pulses, dead-reckons through GPS degradation, and aborts cleanly on field faults. |
| `cli.py` | The five-mode argparse entrypoint. Pure no-hardware helpers (`build_calibrate_turn_argv`, `resolve_calibration`, `resolve_plan`, `diagnose_summary`, `load_planner_config`) are unit-testable without serial. |

`scripts/run_physical_path_planner.sh` is a thin launcher: `preview`/`calibrate-turn`/
`diagnose` exec the CLI directly; `run`/`execute-plan` flash the guarded-crawl
firmware first (unless `--print-plan`), then exec the CLI pinned to the
re-enumerated port.

## The steering control law (B axis only)

`geometry.compute_b_correction` is the only steering math, and the controller never
changes it:

```
heading_component = clamp(k_heading * heading_error_deg, ±max_heading_b)   # k_heading=0.006, max=0.08
cte_component     = clamp(k_cte     * cross_track_error_m, ±max_cte_b)      # k_cte=0.25,     max=0.04
b_cmd             = clamp(heading_component + cte_component, ±0.08)
```

The correction is applied to the **B (turn) command only**. The forward **A** command
and the **pulse duration (ms)** come straight from calibration and are *never lowered*
by the controller. Connectors are deliberate turns: their B command is the calibrated
turn value, with zero heading/cross-track components.

## Calibration resolution and the connector fallback

`resolve_physical_calibration` reads on-disk calibration JSONs when present and
normalizes them; when a source is missing it degrades rather than raising. Connector
selection in `auto` mode prefers the angle-calibrated 90° turn, then the smooth-IMU
connector, then the **repeated-pulses fallback** (12 left / 12 right fixed pulses)
with `fallback_to_repeated_pulses=true` surfaced so the operator sees that connectors
are uncalibrated. The fallback motion set (forward `a=0.30`/`800 ms`, backward
`a=−0.08`/`300 ms`, turn_left `b=0.26`/`700 ms`, turn_right `b=−0.08`/`250 ms`) is
mirrored in `configs/calibration_default.json` and kept in lockstep by a parity test.

`calibrate-turn` writes its result to
`outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json`
— exactly the path the resolver reads back as the angle-calibrated connector, so a
successful calibration automatically upgrades subsequent `run`/`preview` connectors.

## The guarded pulse FSM (`executor.send_pulse`)

One pulse is four ordered steps, each waiting for its acknowledging event before the
next:

```
arm command   -> {ARM, REJECT}
pulse command -> {ACK, REJECT}
await pulse   -> {STOP, PULSE_COMPLETE, PULSE_DONE}   (>= pulse_ms + 1s slack)
stop command  -> {STOP}
```

`controller.pulse_block_reason` then classifies the captured window, most
safety-critical first: `RC_INVALID` (aborts the run) → missing ACK/STOP →
`OUTPUT_ACTIVE_AFTER_STOP` → `FINAL_COMMANDS_NONZERO`.

## The continuous-motion loop (`controller.run_controller`)

For each planned segment the loop computes a per-pulse budget, then for each pulse:
waits for a STAGE20-compatible heartbeat; checks `rc_ok`, manual override, and RC
neutrality (waiting up to `rc_neutral_wait_s`); resolves usable lat/lon
(dead-reckoning from the last good fix when GPS is degraded); computes the B
correction; issues `send_pulse`; and records a per-pulse row. It returns
`(rows, raw_lines, abort_reason)` and **never raises** on the expected field faults —
serial disconnect, `RC_INVALID`, GPS abort policy, or a missing heartbeat each set
`abort_reason` and stop the loop so the caller can still write a guarded summary.

Abort reasons: `NO_STAGE20_HEARTBEAT`, `RC_NOT_OK`, `MANUAL_OVERRIDE`,
`RC_NOT_NEUTRAL`, `GPS_DEGRADED`, `RC_INVALID`, `SERIAL_DISCONNECT`.

## The load-bearing invariant

Autonomous full-path following is not sanctioned (calibration is incomplete and the
firmware compile gate is the real safety). So **every** summary the package emits
carries `ready_for_full_path_following=false`, enforced by
`checks.assert_not_ready_for_full_path_following`, which hard-asserts the literal
`False` (not just falsy) and raises `FullPathFollowingNotAllowed` otherwise. Preview,
the controller summary, and the diagnose summary all route through it before being
printed or written.

## Hardware/sensor status separation (do not conflate)

These are independent and must not be inferred from one another (see the root
`README.md` and [current_hardware_status.md](current_hardware_status.md)):

- **GPS** — confirmed on the rover's `Serial2 @ 9600`; reached `STABLE_FIX` outdoors.
  GPS is the antenna position, not the rover body position unless the antenna is
  rover-mounted.
- **IMU (BMI160)** — integrated at I2C `0x68`, `chip_id=0xD1`, healthy. Gyro yaw is
  **relative and diagnostic-only**; the controller uses it for between-pulse
  heading-hold, not as an absolute heading source.
- **HC-12** — `Serial3` selected and the station adapter detected, but the RF link is
  **deferred/unproven** (`HC12_DEFERRED_RF_LINK`). The path planner uses the USB
  serial link, not HC-12 RF.
- **Motor-output safety** — the firmware 4-flag compile gate is authoritative.
  Physical path execution also remains blocked at the field until the RC AUTO/MANUAL
  mode channel reliably holds (see the root `README.md` PPM/CH5 blocker).
