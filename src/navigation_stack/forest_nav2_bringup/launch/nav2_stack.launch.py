"""Mission manager + Nav2 + mission_nav2_bridge (no legacy navigation_node)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    conf_pkg = get_package_share_directory("forest_hybrid_conf")
    nav2_pkg = get_package_share_directory("forest_nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    mission = os.path.join(conf_pkg, "launch", "mission_layer_only.launch.py")
    nav2 = os.path.join(nav2_pkg, "launch", "nav2_bringup.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(nav2_pkg, "config", "nav2_params.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
        ]
    )
