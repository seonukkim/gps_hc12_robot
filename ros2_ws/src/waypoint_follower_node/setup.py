from setuptools import setup

package_name = "waypoint_follower_node"

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
