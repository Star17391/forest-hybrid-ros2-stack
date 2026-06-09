"""Sprint 0: camada de mapa de custo / traversabilidade 2.5D (grid_map)."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_traversability_mapping")
    config = os.path.join(pkg, "config", "traversability_mapping.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="forest_traversability_mapping",
                executable="traversability_mapping_node",
                name="traversability_mapping_node",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
