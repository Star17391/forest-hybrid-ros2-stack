"""Gazebo (ForestGen) + segmentação semântica + RViz com máscara colorida.

Usage:
  cd forest-hybrid-ros2-stack && source install/setup.bash
  ros2 launch forest_semantic_segmentation sim_semantic.launch.py

Nicla (câmara ADVR já a correr):
  ros2 launch forest_semantic_segmentation semantic_only.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    semantic_share = get_package_share_directory("forest_semantic_segmentation")
    sim_share = get_package_share_directory("forest_sim_bridge")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    onnx_path = LaunchConfiguration("onnx_model_path")
    rviz_cfg = os.path.join(semantic_share, "config", "semantic_perception.rviz")

    default_onnx = os.path.join(semantic_share, "models", "forest_semantic.onnx")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "use_rviz": "true",
            "rviz_config": rviz_cfg,
            "use_pose_bridge": "true",
            "use_sensor_tf_static": "true",
            "use_odom_relay": "false",
            "paused": "true",
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(semantic_share, "launch", "semantic_only.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "onnx_model_path": onnx_path,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value="forest_gentle_trees_rocks.sdf",
                description="World under FORESTGEN_PATH/worlds/",
            ),
            DeclareLaunchArgument(
                "onnx_model_path",
                default_value=default_onnx,
                description="ONNX exportado do forest-semantic-training",
            ),
            sim,
            TimerAction(period=6.0, actions=[perception]),
        ]
    )
