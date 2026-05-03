# Progress Summary

Date: 2026-05-03 KST

This summary is intended to keep report claims aligned with current repository
state.

## Implemented

| Area | Status |
| --- | --- |
| Python environment | `uv` project with Python 3.12 requirement |
| Protocol module | Frame encode/decode, checksum, command formatting |
| Planner module | Planar A/B lawnmower waypoint generation |
| Geodesy/NMEA helpers | Coordinate conversion and supported NMEA parsing |
| Station safe tools | Heartbeat/STOP loops, `--port`, `/dev/ttyACM0` default |
| Mock mission tool | JSON, CSV, summary, and preview output |
| Analysis tools | GPS and safety USB debug log summarizers |
| Firmware | OpenRB RC input, STOP, HC-12 parser, GPS Serial3, USB debug |
| Figure generation | System, planning, GPS, and safety figures |
| ROS2 workspace | Package skeletons only |

## Verified

- RC manual control.
- GPS Serial3 communication and GPS FIX.
- Failsafe STOP behavior.
- Mock lawnmower planner behavior through unit tests and preview generation.
- Generated figures are available for report drafting.

## Pending

- Station-side HC-12 USB device confirmation.
- End-to-end HC-12 station-to-rover test.
- GPS telemetry payload schema alignment.
- ROS2 runtime implementation.
- Magnetic wheel adhesion validation.
- Cleaning/painting payload validation.
- Autonomous hull or representative-surface field test.

## Planned

- Use ROS2 Jazzy after the Python pre-ROS2 behavior is stable.
- Keep core protocol and planner modules independent from ROS2.
- Add a ROS2 HC-12 bridge, mission manager, planner interface, and waypoint
  follower.
- Extend coverage planning beyond a planar rectangle.
- Add process constraints for cleaning or painting coverage.

## Current Report-Safe Summary

The project has built a safe pre-ROS2 foundation for a ship exterior cleaning
and painting robot. The current system includes Python protocol and planning
modules, station tools with conservative defaults, OpenRB firmware for RC/GPS
bring-up, and generated figures for reporting. RC manual operation, GPS FIX,
failsafe STOP, and mock A/B lawnmower planning have been verified. HC-12
station-side integration, ROS2 runtime integration, magnetic adhesion, and full
cleaning/painting operation remain pending.

## Evidence Map

| Evidence | Location |
| --- | --- |
| Project overview | `README.md` |
| Repository audit | `docs/project_notes/repo_audit.md` |
| Cleanup summary | `docs/project_notes/cleanup_summary.md` |
| Hardware status | `docs/current_hardware_status.md` |
| GPS Serial3 result | `docs/gps_serial3_test.md` |
| RC channel map | `docs/rc_channel_map.md` |
| Safety checklist | `docs/safety_checklist.md` |
| Protocol documentation | `docs/protocol.md` |
| Planner tests | `tests/test_planner.py` |
| Protocol tests | `tests/test_protocol.py` |
| Generated captions | `docs/figures/generated/figure_captions.md` |

## Open Claim Risks

- The firmware GPS frame payload and Python telemetry parser do not yet use one
  shared schema.
- Station keyboard manual display handling needs review before being described
  as a polished operator workflow.
- Generated mock path figures are planning demonstrations, not field execution
  results.
- ROS2 node files are skeletons and should not be described as integrated
  autonomy.
