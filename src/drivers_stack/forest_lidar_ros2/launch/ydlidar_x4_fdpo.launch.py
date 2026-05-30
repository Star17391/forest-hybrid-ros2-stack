"""YDLidar X4 using fdpo-ros-stack serial protocol (recommended for this hardware)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_lidar_ros2")
    serial_port = LaunchConfiguration("serial_port")
    publish_cloud = LaunchConfiguration("publish_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("publish_cloud", default_value="true"),
            Node(
                package="forest_lidar_ros2",
                executable="fdpo_ydlidar_x4",
                name="fdpo_ydlidar_x4",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "fdpo_ydlidar_x4.yaml"),
                    {"port": serial_port},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0.15",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_link", "--child-frame-id", "laser",
                ],
            ),
            Node(
                package="forest_lidar_ros2",
                executable="laserscan_to_pointcloud2",
                name="laserscan_to_pointcloud2",
                parameters=[{
                    "scan_topic": "/scan",
                    "cloud_topic": "/sensors/lidar/points",
                    "min_range_m": 0.12,
                    "max_range_m": 10.0,
                }],
                condition=IfCondition(publish_cloud),
            ),
        ]
    )
