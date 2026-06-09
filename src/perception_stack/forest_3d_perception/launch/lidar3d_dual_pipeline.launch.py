"""
Launch legacy + experimental LiDAR pipelines in parallel for A/B comparison.

Legacy: lidar3d_segmentation_node (unchanged code path).
Experimental: lidar3d_experimental_node (CSF + clustering).

Set use_experimental_pipeline:=true to enable the experimental node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_3d_perception")
    legacy_config = os.path.join(pkg, "config", "forest_3d_segmentation.yaml")
    exp_config = os.path.join(pkg, "config", "lidar3d_experimental.yaml")
    mode_config = os.path.join(pkg, "config", "lidar3d_perception_mode.yaml")

    use_sim = LaunchConfiguration("use_sim_time")
    use_legacy = LaunchConfiguration("use_legacy_pipeline")
    use_experimental = LaunchConfiguration("use_experimental_pipeline")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("legacy_config", default_value=legacy_config),
        DeclareLaunchArgument("experimental_config", default_value=exp_config),
        DeclareLaunchArgument("mode_config", default_value=mode_config),
        DeclareLaunchArgument("use_legacy_pipeline", default_value="true"),
        DeclareLaunchArgument("use_experimental_pipeline", default_value="false"),
        Node(
            package="forest_3d_perception",
            executable="lidar3d_segmentation_node",
            name="lidar3d_segmentation_node",
            output="screen",
            condition=IfCondition(use_legacy),
            parameters=[
                {"use_sim_time": use_sim},
                LaunchConfiguration("legacy_config"),
            ],
        ),
        Node(
            package="forest_3d_perception",
            executable="lidar3d_experimental_node",
            name="lidar3d_experimental_node",
            output="screen",
            condition=IfCondition(use_experimental),
            parameters=[
                {"use_sim_time": use_sim},
                LaunchConfiguration("experimental_config"),
            ],
        ),
    ])
