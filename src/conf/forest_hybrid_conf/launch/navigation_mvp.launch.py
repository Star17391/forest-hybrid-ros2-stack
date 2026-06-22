"""Missão + navegação: Nav2 (ativo) ou legacy Pure Pursuit/NMPC pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    conf_pkg = get_package_share_directory("forest_hybrid_conf")
    nav_pkg = get_package_share_directory("forest_navigation_ros2")
    nav2_pkg = get_package_share_directory("forest_nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")

    mission_only = os.path.join(conf_pkg, "launch", "mission_layer_only.launch.py")
    navigation_legacy = os.path.join(nav_pkg, "launch", "navigation_sim.launch.py")
    navigation_nav2 = os.path.join(nav2_pkg, "launch", "nav2_stack.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="false",
                description="true = Nav2 stack; false = legacy navigation_node (Pure Pursuit/NMPC)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_only),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_nav2),
                condition=IfCondition(use_nav2),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_legacy),
                condition=UnlessCondition(use_nav2),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
