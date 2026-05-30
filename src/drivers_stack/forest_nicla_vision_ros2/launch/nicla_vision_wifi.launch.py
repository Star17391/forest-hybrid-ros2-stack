"""Nicla Vision over Wi-Fi TCP (Pi -> Nicla IP:9876).

Nicla must already be on Wi-Fi (FOREST_WIFI_AUTO_CONNECT 1 in wifi_secrets.h, or
run scripts/nicla_wifi_connect.sh once after boot if auto-connect is disabled).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_nicla_vision_ros2")
    return LaunchDescription(
        [
            DeclareLaunchArgument("wifi_host", description="Nicla Vision IP address"),
            DeclareLaunchArgument("wifi_port", default_value="9876"),
            Node(
                package="forest_nicla_vision_ros2",
                executable="nicla_serial_bridge",
                name="nicla_serial_bridge",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "nicla_vision.yaml"),
                    os.path.join(pkg, "config", "nicla_camera_info.yaml"),
                    os.path.join(pkg, "config", "nicla_vision_wifi.yaml"),
                    {
                        "wifi_host": LaunchConfiguration("wifi_host"),
                        "wifi_port": LaunchConfiguration("wifi_port"),
                    },
                ],
            ),
        ]
    )
