"""Nav2 stack for forest-hybrid (no AMCL / map_server — SLAM owns map→odom)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("nav2_bringup")
    forest_nav2_share = get_package_share_directory("forest_nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    require_slam = LaunchConfiguration("require_slam")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_localization": "False",
            "slam": "False",
            "use_composition": "False",
            "use_namespace": "false",
            "autostart": "true",
            "map": "",
        }.items(),
    )

    bridge = Node(
        package="forest_nav2_bringup",
        executable="mission_nav2_bridge",
        name="mission_nav2_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "require_slam": require_slam}],
    )

    cmd_vel_relay = Node(
        package="forest_nav2_bringup",
        executable="cmd_vel_contract_relay",
        name="cmd_vel_contract_relay",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "input_topic": "cmd_vel", "output_topic": "/forest_gen/cmd_vel"}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(forest_nav2_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "require_slam", default_value="true",
                description="false = bridge conduz por odom mesmo com SLAM LOST (MVP)",
            ),
            nav2,
            bridge,
            cmd_vel_relay,
        ]
    )
