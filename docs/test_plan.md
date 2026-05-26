# Test Plan

## Bench Validation

1. HC-12 echo
   - flash `firmware/hc12_echo_test/hc12_echo_test.ino`
   - verify USB Serial <-> HC-12 echo before rover protocol testing
2. PPM input
   - flash `firmware/ppm_test/ppm_test.ino`
   - confirm CH1~CH8 pulse widths print consistently
3. GPS input
   - flash `firmware/gps_test/gps_test.ino`
   - confirm latitude, longitude, satellites, and HDOP output
4. ESC safety
   - wheel-off-ground only
   - verify startup neutral, manual override, and `STOP`
   - keep the RC Manual/Auto switch on `CH5` in `MANUAL`
   - center the steering and throttle joystick before powering the ESCs
   - confirm USB debug shows `left_cmd=0.000` and `right_cmd=0.000` before applying throttle
   - use small throttle pulses only
   - transmitter switch-off or RC link loss must return the rover to stop

## Station Validation

1. Environment
   - `uv sync --extra dev --extra web`
   - `uv run python tools/verify_env.py`
2. Core tests
   - `uv run pytest -q`
3. HC-12 link
   - `uv run python tools/station_hc12_test.py --port /dev/ttyACM0 --heartbeat-hz 5`
4. Safe controller loop
   - `uv run python tools/station_controller.py --port /dev/ttyACM0`
5. Keyboard manual bench test
   - wheel-off-ground only
   - start with RC transmitter off or RC invalid so station-manual takeover is exercised
   - run `uv run python tools/station_keyboard_manual.py --port /dev/ttyACM0 --baud 9600 --max-speed 0.25`
   - confirm heartbeat plus `STOP` on startup before pressing `e`
   - press `e` to arm station manual, then confirm neutral manual frames with `deadman` off
   - confirm manual drive only while fresh manual frames continue arriving with `deadman` enabled
   - confirm `x` disarms and sends `STOP`
   - confirm stale link or released controls return outputs to neutral within the station timeout window
6. Planner preview
   - run `tools/path_preview.py` and inspect saved image

## ROS2

1. Create package skeletons
   - `./scripts/create_ros2_packages.sh`
2. Build workspace
   - `./scripts/build_ros2_ws.sh`
3. Confirm package discovery
   - `source ros2_ws/install/setup.bash`
   - `ros2 pkg list | grep -E 'hc12_bridge_node|station_mission_node|coverage_planner_node|waypoint_follower_node'`
