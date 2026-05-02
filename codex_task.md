You are working inside the repository:

~/project-lab/gps_hc12_robot

This is a WSL2 Ubuntu 24.04 ground-station project for an OpenRB-150 rover using HC-12 wireless UART.

Important environment constraints:
- Python environment is managed by uv.
- Do not use plain `pip install` in setup instructions unless it is explicitly marked as fallback.
- Keep pyproject.toml, uv.lock, requirements.txt, and requirements-dev.txt reproducible.
- The current USB serial device is /dev/ttyACM0.
- Default serial baudrate is 9600.
- Station-side code must be safe by default: heartbeat and STOP only.
- Do not send real motor-driving AUTO commands from default scripts.
- Do not assume /dev/ttyUSB0; default to /dev/ttyACM0 but expose --port.

Architecture:
- Ground station:
  - WSL2 Ubuntu 24.04 or Jetson
  - Python + ROS2 Jazzy
  - HC-12 connected via USB serial
  - High-level mission logic, GPS telemetry, logging, path planning
- Rover:
  - OpenRB-150
  - HC-12 UART
  - GPS UART
  - RC receiver PPM
  - ESC/motor output
  - Manual override and failsafe logic
- Safety:
  - OpenRB owns low-level safety.
  - RC manual override must take priority over station AUTO.
  - HC-12 timeout during AUTO must cause STOP.
  - Reconnection must not automatically resume AUTO.

Communication protocol:
@TYPE,SEQ,PAYLOAD*CS\n

Checksum:
XOR over the body string without @ and without *CS.

Examples:
@HB,102,STATION*CS
@CMD,103,AUTO,0.25,-0.03*CS
@CMD,104,STOP,0,0*CS
@GPS,520,35.123456,129.123456,48.2,8,1.4,1*CS
@STAT,521,AUTO_RUNNING,RC_OK,LINK_OK,103*CS
@ACK,103,OK*CS

Required repository layout:
.
├── README.md
├── AGENTS.md
├── .gitignore
├── pyproject.toml
├── uv.lock
├── requirements.txt
├── requirements-dev.txt
├── Makefile
├── docs/
│   ├── architecture.md
│   ├── wiring.md
│   ├── protocol.md
│   ├── test_plan.md
│   ├── rc_channel_map.md
│   ├── wsl_usb_guide.md
│   └── safety_checklist.md
├── gps_coverage_core/
│   ├── __init__.py
│   ├── protocol.py
│   ├── geo.py
│   ├── planner.py
│   ├── nmea.py
│   └── telemetry.py
├── tests/
│   ├── test_protocol.py
│   ├── test_geo.py
│   └── test_planner.py
├── tools/
│   ├── hc12_terminal.py
│   ├── station_hc12_test.py
│   ├── station_controller.py
│   ├── gps_logger.py
│   ├── nmea_replay.py
│   ├── path_preview.py
│   └── verify_env.py
├── firmware/
│   ├── README.md
│   ├── hc12_echo_test/
│   │   └── hc12_echo_test.ino
│   ├── ppm_test/
│   │   └── ppm_test.ino
│   ├── gps_test/
│   │   └── gps_test.ino
│   └── openrb_robot_controller/
│       └── openrb_robot_controller.ino
├── ros2_ws/
│   └── src/
├── scripts/
│   ├── create_ros2_packages.sh
│   ├── build_ros2_ws.sh
│   ├── export_requirements.sh
│   └── publish_private_github.sh
├── data/
│   ├── .gitkeep
│   ├── nmea_logs/
│   │   └── .gitkeep
│   └── hc12_logs/
│       └── .gitkeep
├── outputs/
│   └── .gitkeep
└── .github/
    └── workflows/
        └── python-tests.yml

Implementation requirements:

1. pyproject / uv
- Preserve root-level pyproject.toml.
- Use uv as the primary dependency manager.
- Ensure `uv sync --extra dev --extra web` works.
- Ensure `uv run pytest -q` works.
- Add scripts/export_requirements.sh:
  - uv pip compile pyproject.toml -o requirements.txt
  - uv pip compile pyproject.toml --extra dev --extra web -o requirements-dev.txt
- Do not create a second nested pyproject unless there is a clear reason.

2. Core protocol
- Implement gps_coverage_core/protocol.py:
  - checksum_xor(text: str) -> int
  - encode_frame(msg_type: str, seq: int, *fields: object) -> bytes
  - decode_frame(line: bytes | str) -> tuple[str, int, list[str]]
- Strict validation:
  - must start with @
  - must include *
  - checksum must match
  - SEQ must be int
  - raise ValueError on malformed input
- Tests:
  - valid CMD frame
  - valid GPS frame
  - invalid checksum
  - missing @
  - missing checksum
  - non-integer sequence

3. Geo and planner
- Implement:
  - GeoPoint dataclass
  - LocalPoint dataclass
  - latlon_to_local()
  - local_to_latlon()
  - generate_lawnmower_path(point_a, point_b, spacing_m)
- Tests:
  - roundtrip conversion
  - invalid spacing
  - too-small area
  - positive waypoint count

4. Tools
- All tool scripts should support --port and default to /dev/ttyACM0.
- hc12_terminal.py:
  - Open serial port
  - For HC-12 AT command mode
  - No motor commands
- station_hc12_test.py:
  - Send heartbeat at 2 Hz
  - Decode incoming frames
  - Print GPS/STAT/ACK/ERR
  - Log raw RX/TX to data/hc12_logs
- station_controller.py:
  - StationController class
  - send_heartbeat()
  - send_cmd()
  - poll()
  - Default main loop sends heartbeat and STOP only
  - Ctrl+C sends STOP five times
- gps_logger.py:
  - Log GPS frames to CSV
- nmea_replay.py:
  - Replay NMEA or parsed GPS logs for offline testing
- path_preview.py:
  - Generate lawnmower path from two lat/lon points and spacing
  - Save a matplotlib figure to outputs/
- verify_env.py:
  - Print Python version
  - Print uv version if available
  - Print ROS_DISTRO if available
  - Print serial devices
  - Check imports

5. Firmware
- Create Arduino .ino sketches.
- Add comments that actual OpenRB-150 UART pin mapping must be confirmed.
- hc12_echo_test.ino:
  - USB Serial <-> HC12 serial echo
- ppm_test.ino:
  - Interrupt-based PPM pulse width printout
  - Print CH1~CH8
- gps_test.ino:
  - TinyGPS++ skeleton
  - Read GPS UART and print lat/lon/sat/hdop
- openrb_robot_controller.ino:
  - Use Servo for ESC pulse output
  - PPM input
  - HC-12 frame parser with XOR checksum
  - GPS telemetry skeleton
  - Modes:
    DISARMED, MANUAL, AUTO_READY, AUTO_RUNNING, LINK_LOST, FAILSAFE
  - Constants:
    PPM_PIN = 6
    ESC_LEFT_PIN = 4
    ESC_RIGHT_PIN = 5
    HC12_BAUD = 9600
    GPS_BAUD = 9600
  - RC invalid -> FAILSAFE + motorStop()
  - Station timeout during AUTO -> LINK_LOST + motorStop()
  - Manual mode overrides AUTO
  - STOP immediately motorStop()
  - AUTO accepted only if RC valid and RC auto switch is on
  - Mark all motor tests as wheel-off-ground only

6. ROS2
- scripts/create_ros2_packages.sh:
  - Create ament_python packages if missing:
    - hc12_bridge_node
    - station_mission_node
    - coverage_planner_node
    - waypoint_follower_node
  - Dependencies:
    - rclpy
    - std_msgs
    - sensor_msgs
    - geometry_msgs
    - nav_msgs
- scripts/build_ros2_ws.sh:
  - source /opt/ros/jazzy/setup.bash
  - rosdep install
  - colcon build --symlink-install
- Minimal node skeletons are enough for now.
- Keep core protocol/planner independent from ROS2.

7. Docs
- README.md:
  - uv setup
  - WSL USB attach
  - /dev/ttyACM0 default
  - safety defaults
  - test commands
- docs/wiring.md:
  - HC-12 TX -> OpenRB RX
  - HC-12 RX <- OpenRB TX
  - common GND
  - confirm 5V/3.3V logic compatibility
- docs/protocol.md:
  - frame format
  - checksum
  - examples
- docs/test_plan.md:
  - HC-12 echo
  - PPM
  - GPS
  - ESC wheel-off-ground safety
  - station planner
  - ROS2 bridge
- docs/safety_checklist.md:
  - default STOP
  - RC override
  - link timeout STOP
  - no AUTO on startup
  - wheel-off-ground motor test

8. GitHub
- .gitignore must exclude:
  .venv/
  __pycache__/
  .pytest_cache/
  build/
  install/
  log/
  *.pyc
  .env
  *.secret
  *.key
  data/**/*.csv
  data/**/*.log
  outputs/*
- scripts/publish_private_github.sh:
  - usage: ./scripts/publish_private_github.sh repo-name
  - gh auth status
  - git branch -M main
  - git add .
  - commit if changes exist
  - gh repo create "$REPO_NAME" --private --source=. --remote=origin --push
  - if origin exists, push main

Validation commands to run:
- uv sync --extra dev --extra web
- uv run pytest -q
- uv run python tools/verify_env.py
- ./scripts/export_requirements.sh
- git status

If ROS2 is available:
- ./scripts/create_ros2_packages.sh
- ./scripts/build_ros2_ws.sh

At the end, report:
- Files created
- Validation commands passed
- Validation commands failed
- Next manual hardware steps

Do not use full-access dangerous flags.
Do not access real motors from station-side code.
