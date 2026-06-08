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
| `cli.py` | User-facing modes: `diagnose`, `manual-rc`, `guarded-pulse-ready`, `calibrate-turn`, `tune-motion`, `reset-motion-calibration`, `preview`, `align-heading`, `execute-plan`, `run`, and `auto-relative-run`. |

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
