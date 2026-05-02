#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/ros2_ws/src"
mkdir -p "$SRC_DIR"

DEPENDENCIES=(rclpy std_msgs sensor_msgs geometry_msgs nav_msgs)
PACKAGES=(hc12_bridge_node station_mission_node coverage_planner_node waypoint_follower_node)

create_package() {
  local package_name="$1"
  local package_dir="$SRC_DIR/$package_name"

  if [[ -d "$package_dir" ]]; then
    echo "Package already exists: $package_name"
    return
  fi

  mkdir -p "$package_dir/$package_name" "$package_dir/resource"

  cat > "$package_dir/package.xml" <<EOF
<?xml version="1.0"?>
<package format="3">
  <name>$package_name</name>
  <version>0.0.0</version>
  <description>Minimal ROS2 Jazzy package skeleton for gps_hc12_robot.</description>
  <maintainer email="noreply@example.com">Codex</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

  cat > "$package_dir/setup.py" <<EOF
from setuptools import setup

package_name = "$package_name"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="noreply@example.com",
    description="Minimal ROS2 Jazzy node skeleton for gps_hc12_robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            package_name + " = " + package_name + ".node:main",
        ],
    },
)
EOF

  cat > "$package_dir/setup.cfg" <<EOF
[develop]
script_dir=\$base/lib/$package_name

[install]
install_scripts=\$base/lib/$package_name
EOF

  echo "$package_name" > "$package_dir/resource/$package_name"
  echo "__all__ = []" > "$package_dir/$package_name/__init__.py"

  cat > "$package_dir/$package_name/node.py" <<EOF
import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    def __init__(self) -> None:
        super().__init__("$package_name")
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self.get_logger().debug("$package_name alive")


def main() -> None:
    rclpy.init()
    node = SkeletonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF

  echo "Created package: $package_name"
}

printf 'ROS2 dependencies: %s\n' "${DEPENDENCIES[*]}"
for package_name in "${PACKAGES[@]}"; do
  create_package "$package_name"
done
