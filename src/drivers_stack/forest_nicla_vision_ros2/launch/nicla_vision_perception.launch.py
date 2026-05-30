"""Nicla camera driver + semantic segmentation (computer-vision layer entry point)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_nicla_vision_ros2")

    return LaunchDescription(
        [
            DeclareLaunchArgument("onnx_model_path", default_value=""),
            DeclareLaunchArgument("model_input_width", default_value="512"),
            DeclareLaunchArgument("model_input_height", default_value="384"),
            DeclareLaunchArgument("image_encoding", default_value="jpeg"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "nicla_vision.launch.py")
                ),
                launch_arguments={
                    "image_encoding": LaunchConfiguration("image_encoding"),
                }.items(),
            ),
            Node(
                package="forest_semantic_segmentation",
                executable="semantic_segmentation_node",
                name="semantic_segmentation_node",
                output="screen",
                parameters=[
                    {
                        "onnx_model_path": LaunchConfiguration("onnx_model_path"),
                        "model_input_width": LaunchConfiguration("model_input_width"),
                        "model_input_height": LaunchConfiguration("model_input_height"),
                    }
                ],
            ),
        ]
    )
