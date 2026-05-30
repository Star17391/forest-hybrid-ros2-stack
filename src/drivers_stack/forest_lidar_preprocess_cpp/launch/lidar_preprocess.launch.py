"""Classify /scan into ground vs other (all points kept on labeled cloud)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_lidar_preprocess_cpp")
    params_file = os.path.join(pkg, "config", "forest_lidar_preprocess.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "classification_frame",
                default_value="marble_hd2/base_link",
                description="Frame for ground height (base_link on real robot)",
            ),
            Node(
                package="forest_lidar_preprocess_cpp",
                executable="lidar_scan_classify_node",
                name="lidar_scan_classify_node",
                output="screen",
                parameters=[
                    params_file,
                    {"classification_frame": LaunchConfiguration("classification_frame")},
                ],
            ),
        ]
    )
