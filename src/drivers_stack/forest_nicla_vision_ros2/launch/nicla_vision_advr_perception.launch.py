"""ADVR Nicla camera stream + forest semantic segmentation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    forest_pkg = get_package_share_directory("forest_nicla_vision_ros2")
    onnx_path = LaunchConfiguration("onnx_model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "onnx_model_path",
                description="Path to ONNX model for forest_semantic_segmentation",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(forest_pkg, "launch", "nicla_vision_advr.launch.py")
                ),
            ),
            Node(
                package="forest_semantic_segmentation",
                executable="semantic_segmentation_node",
                name="semantic_segmentation",
                output="screen",
                parameters=[{"onnx_model_path": onnx_path}],
            ),
        ]
    )
