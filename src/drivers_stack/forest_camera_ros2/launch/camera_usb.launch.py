"""Lança usb_cam: tópicos /camera/image_raw e /camera/camera_info (name=camera)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("forest_camera_ros2"),
        "config",
        "camera_usb.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Sincronizar relógio com simulação",
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera",
                output="screen",
                parameters=[
                    cfg,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
