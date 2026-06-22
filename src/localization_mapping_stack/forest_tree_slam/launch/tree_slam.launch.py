"""Tree-SLAM florestal: tracker + back-end GTSAM + relocalizador TreeLoc.

Ver docs/FOREST_TREE_SLAM_DESIGN.md. Autoridade de map->odom no SOLO.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_tree_slam")
    config = os.path.join(pkg, "config", "tree_slam.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="forest_tree_slam",
                executable="tree_slam_node",
                name="tree_slam_node",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
