"""Bringup terrestre: câmara (camera_ros), modo de operação, segmentação (placeholder)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    operation_mode = LaunchConfiguration("operation_mode")

    camera_launch = os.path.join(
        get_package_share_directory("forest_camera_ros2"),
        "launch",
        "camera_pi.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Sincronizar com relógio de simulação",
            ),
            DeclareLaunchArgument(
                "operation_mode",
                default_value="ground",
                description="ground | aerial — em aerial a segmentação não processa imagens",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            Node(
                package="forest_robot_supervisor",
                executable="operation_mode_node",
                name="operation_mode_node",
                output="screen",
                parameters=[
                    {"operation_mode": operation_mode, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="forest_semantic_segmentation",
                executable="semantic_segmentation_node",
                name="semantic_segmentation_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
