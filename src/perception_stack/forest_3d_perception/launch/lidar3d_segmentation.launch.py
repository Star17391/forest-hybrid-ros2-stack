"""Launch lidar3d_segmentation_node with YAML config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_3d_perception")
    default_config = os.path.join(pkg, "config", "forest_3d_segmentation.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="forest_3d_perception",
            executable="lidar3d_segmentation_node",
            name="lidar3d_segmentation_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                LaunchConfiguration("config"),
            ],
        ),
    ])
