"""Navigation MVP — Pure Pursuit + trajectory pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_navigation_ros2")
    defaults = os.path.join(pkg, "config", "navigation_defaults.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    metrics_csv = LaunchConfiguration("metrics_csv_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "metrics_csv_path",
                default_value="/tmp/forest_navigation_metrics.csv",
            ),
            Node(
                package="forest_navigation_ros2",
                executable="navigation_node",
                name="navigation_node",
                output="screen",
                parameters=[defaults, {"use_sim_time": use_sim_time, "metrics_csv_path": metrics_csv}],
            ),
        ]
    )
