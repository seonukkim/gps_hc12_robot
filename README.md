# gps_hc12_robot

WSL2 Ubuntu 24.04 / Jetson ground-station project for an OpenRB-150 rover using HC-12 wireless UART, GPS telemetry, offline planning, and ROS2 Jazzy integration points.

Station-side defaults are intentionally safe:

- Default serial port is `/dev/ttyACM0`.
- Default baudrate is `9600`.
- Default station loops send heartbeat and `STOP` only.
- Station scripts do not send real motor-driving `AUTO` commands by default.
- Rover-side low-level safety, RC override, and link-loss stop logic remain authoritative on OpenRB.

## Setup

Use `uv` as the primary dependency manager.

```bash
uv sync --extra dev --extra web
```

Useful local commands:

```bash
uv run pytest -q
uv run python tools/verify_env.py
./scripts/export_requirements.sh
```

## WSL2 USB Attach

Attach the OpenRB / USB serial device into WSL and confirm it appears as `/dev/ttyACM0`.

1. On Windows, list devices with `usbipd list`.
2. Bind and attach the target device into WSL.
3. In WSL, verify with `ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null`.

Additional notes are in [docs/wsl_usb_guide.md](docs/wsl_usb_guide.md).

## Serial Defaults

All station tools expose `--port` and default to `/dev/ttyACM0`:

- `tools/hc12_terminal.py`
- `tools/station_hc12_test.py`
- `tools/station_controller.py`
- `tools/gps_logger.py`

## Safety

- No `AUTO` mode on station startup.
- Ctrl+C in the controller sends `STOP` five times before exit.
- Motor validation is wheel-off-ground only.
- RC manual override must take priority over station commands.
- Link timeout during rover `AUTO` must stop the motors and require explicit re-arming.

See [docs/safety_checklist.md](docs/safety_checklist.md) and [docs/test_plan.md](docs/test_plan.md).

## Project Layout

- `gps_coverage_core/`: protocol, geodesy, planner, NMEA, telemetry
- `tools/`: station-side utilities
- `firmware/`: OpenRB / Arduino sketches
- `ros2_ws/`: ROS2 Jazzy workspace root
- `docs/`: wiring, protocol, architecture, test plan

## Quick Start

Heartbeat / receive-only testing:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0
```

Safe controller loop:

```bash
uv run python tools/station_controller.py --port /dev/ttyACM0
```

Offline path preview:

```bash
uv run python tools/path_preview.py \
  --lat-a 35.123456 --lon-a 129.123456 \
  --lat-b 35.124000 --lon-b 129.124000 \
  --spacing 5.0
```
