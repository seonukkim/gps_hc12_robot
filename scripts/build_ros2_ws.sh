#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$ROOT_DIR/ros2_ws"

source /opt/ros/jazzy/setup.bash

cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
