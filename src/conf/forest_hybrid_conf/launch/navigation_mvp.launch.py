"""Missão + navegação MVP (sem câmara/perceção). Usar com ForestGen em outro terminal."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    conf_pkg = get_package_share_directory("forest_hybrid_conf")
    nav_pkg = get_package_share_directory("forest_navigation_ros2")

    use_sim_time = LaunchConfiguration("use_sim_time")

    mission_only = os.path.join(conf_pkg, "launch", "mission_layer_only.launch.py")
    navigation = os.path.join(nav_pkg, "launch", "navigation_sim.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_only),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
