"""Diag Camada 0: TF + wheel odom + EKF — mesmo sim_gazebo.launch.py que forest up.

Sem navegação. Preferir `forest diag tf-audit` com sessão já activa (forest up).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    sim_bridge = get_package_share_directory("forest_sim_bridge")
    hybrid = get_package_share_directory("forest_hybrid_conf")
    rviz = os.path.join(hybrid, "config", "forest_diag_tf.rviz")

    world = LaunchConfiguration("world")
    paused = LaunchConfiguration("paused")
    ekf_mode = LaunchConfiguration("ekf_mode")
    cleanup_first = LaunchConfiguration("cleanup_first")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "paused": paused,
            "cleanup_first": cleanup_first,
            "use_rviz": "true",
            "rviz_config": rviz,
            "use_pose_bridge": "false",
            "use_state_estimation": "true",
            "use_odom_relay": "true",
            "use_gnss": "false",
            "use_sensor_tf_static": "true",
            "ekf_mode": ekf_mode,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("cleanup_first", default_value="true"),
            DeclareLaunchArgument(
                "ekf_mode",
                default_value="local",
                description="wheel_only | local — mesmo parâmetro que sim_mvp / forest up",
            ),
            sim,
        ]
    )
