"""Standalone YDLidar X4 test (same hardware as fdpo-ros-stack sdpo_driver_laser_2d).

Requires: bash scripts/lidar/install_driver.sh (no apt package on Jazzy).

Usage:
  ros2 launch forest_lidar_ros2 ydlidar_x4_test.launch.py
  ros2 launch forest_lidar_ros2 ydlidar_x4_test.launch.py serial_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_lidar_ros2")
    sensors_cpp = get_package_share_directory("forest_sensors_cpp")
    params_file = os.path.join(pkg, "config", "ydlidar_x4_forest.yaml")
    extrinsics_file = os.path.join(sensors_cpp, "config", "forest_lidar_extrinsics.yaml")

    serial_port = LaunchConfiguration("serial_port")
    publish_cloud = LaunchConfiguration("publish_cloud")
    laser_yaw_rad = LaunchConfiguration("laser_yaw_rad")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="USB serial device (fdpo default)",
            ),
            DeclareLaunchArgument(
                "publish_cloud",
                default_value="true",
                description="Also publish /sensors/lidar/points (PointCloud2)",
            ),
            DeclareLaunchArgument(
                "laser_yaw_rad",
                default_value="0.0",
                description="Extra yaw (rad) added to forest_lidar_extrinsics.yaml",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="base_link",
                description="TF parent (use marble_hd2/base_link in sim if needed)",
            ),
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                output="screen",
                parameters=[
                    params_file,
                    {"port": serial_port},
                ],
            ),
            Node(
                package="forest_sensors_cpp",
                executable="static_sensor_tf_node",
                name="static_sensor_tf_node",
                output="screen",
                parameters=[
                    extrinsics_file,
                    {
                        "parent_frame": LaunchConfiguration("parent_frame"),
                        "yaw": LaunchConfiguration("laser_yaw_rad"),
                    },
                ],
            ),
            Node(
                package="forest_sensors_cpp",
                executable="laserscan_to_pointcloud2_node",
                name="laserscan_to_pointcloud2_node",
                output="screen",
                parameters=[extrinsics_file],
                condition=IfCondition(publish_cloud),
            ),
        ]
    )
