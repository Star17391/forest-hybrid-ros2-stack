"""Nicla Vision bringup: serial (default) or Wi-Fi, JPEG, camera_info, TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_nicla_vision_ros2")
    bridge_cfg = os.path.join(pkg, "config", "nicla_vision.yaml")
    info_cfg = os.path.join(pkg, "config", "nicla_camera_info.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("transport", default_value="serial"),
            DeclareLaunchArgument("image_encoding", default_value="jpeg"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("baud_rate", default_value="921600"),
            DeclareLaunchArgument("wifi_host", default_value=""),
            DeclareLaunchArgument("wifi_port", default_value="9876"),
            DeclareLaunchArgument("image_rate_hz", default_value="3.0"),
            DeclareLaunchArgument("imu_rate_hz", default_value="25.0"),
            DeclareLaunchArgument("publish_imu", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("tf_parent", default_value="base_link"),
            Node(
                package="forest_nicla_vision_ros2",
                executable="nicla_serial_bridge",
                name="nicla_serial_bridge",
                output="screen",
                parameters=[
                    bridge_cfg,
                    info_cfg,
                    {
                        "transport": LaunchConfiguration("transport"),
                        "image_encoding": LaunchConfiguration("image_encoding"),
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baud_rate": LaunchConfiguration("baud_rate"),
                        "wifi_host": LaunchConfiguration("wifi_host"),
                        "wifi_port": LaunchConfiguration("wifi_port"),
                        "image_rate_hz": LaunchConfiguration("image_rate_hz"),
                        "imu_rate_hz": LaunchConfiguration("imu_rate_hz"),
                        "publish_imu": LaunchConfiguration("publish_imu"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="nicla_camera_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("tf_parent"),
                    "nicla_camera_optical_frame",
                ],
                condition=IfCondition(LaunchConfiguration("publish_tf")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="nicla_imu_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("tf_parent"),
                    "nicla_imu_link",
                ],
                condition=IfCondition(LaunchConfiguration("publish_tf")),
            ),
        ]
    )
