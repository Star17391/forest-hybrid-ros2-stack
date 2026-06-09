"""Operation mode (ground) + semantic_segmentation_node — sem Gazebo."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    semantic_share = get_package_share_directory("forest_semantic_segmentation")
    default_onnx = os.path.join(semantic_share, "models", "forest_semantic.onnx")

    use_sim_time = LaunchConfiguration("use_sim_time")
    onnx_path = LaunchConfiguration("onnx_model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("onnx_model_path", default_value=default_onnx),
            Node(
                package="forest_robot_supervisor",
                executable="operation_mode_node",
                name="operation_mode_node",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"operation_mode": "ground"},
                ],
            ),
            Node(
                package="forest_semantic_segmentation",
                executable="semantic_segmentation_node",
                name="semantic_segmentation_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "onnx_model_path": onnx_path,
                        "model_input_width": 768,
                        "model_input_height": 512,
                        "min_logit_margin": 0.0,
                    }
                ],
            ),
        ]
    )
