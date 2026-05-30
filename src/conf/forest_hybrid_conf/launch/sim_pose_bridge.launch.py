"""Fase 3a: Gazebo ground-truth pose para validar navegação (sem EKF).

Argumentos `world`, `paused`, `cleanup_first`, `use_rviz` são reencaminhados para
sim_gazebo.launch.py (compatível com perfis forest YAML).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    sim_bridge = get_package_share_directory("forest_sim_bridge")
    rviz = os.path.join(sim_bridge, "config", "forest_pose_bridge_sim.rviz")

    world = LaunchConfiguration("world")
    paused = LaunchConfiguration("paused")
    cleanup_first = LaunchConfiguration("cleanup_first")
    use_rviz = LaunchConfiguration("use_rviz")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "paused": paused,
            "cleanup_first": cleanup_first,
            "use_rviz": use_rviz,
            "rviz_config": rviz,
            "use_pose_bridge": "true",
            "use_state_estimation": "false",
            "use_legacy_sensors": "true",
            "use_sensor_tf_static": "true",
            "use_odom_relay": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("cleanup_first", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            sim,
        ]
    )
