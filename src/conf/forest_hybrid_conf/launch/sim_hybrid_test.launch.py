"""Sim MVP: mundo plano + forest_hybrid_robot + FSM ground→aerial.

Uso:
  ros2 launch forest_hybrid_conf sim_hybrid_test.launch.py
  forest up sim-hybrid-test -d
  forest test hybrid-transition --assert
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

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "paused": LaunchConfiguration("paused"),
            "cleanup_first": LaunchConfiguration("cleanup_first"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": rviz,
            "use_pose_bridge": "true",
            "use_state_estimation": "false",
            "use_legacy_sensors": "true",
            "use_sensor_tf_static": "true",
            "use_odom_relay": "true",
            "sim_robot_model": "forest_hybrid_robot",
            "use_hybrid_transition": "true",
            "hybrid_leg_deployed_m": LaunchConfiguration("hybrid_leg_deployed_m"),
            "hybrid_min_leg_extend_sec": LaunchConfiguration("hybrid_min_leg_extend_sec"),
            "hybrid_min_tracks_rotate_sec": LaunchConfiguration(
                "hybrid_min_tracks_rotate_sec"
            ),
            "hybrid_min_aerial_ready_sec": LaunchConfiguration("hybrid_min_aerial_ready_sec"),
            "hybrid_disable_leg_commands": LaunchConfiguration("hybrid_disable_leg_commands"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("cleanup_first", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("hybrid_leg_deployed_m", default_value=""),
            DeclareLaunchArgument("hybrid_min_leg_extend_sec", default_value=""),
            DeclareLaunchArgument("hybrid_min_tracks_rotate_sec", default_value=""),
            DeclareLaunchArgument("hybrid_min_aerial_ready_sec", default_value=""),
            DeclareLaunchArgument("hybrid_disable_leg_commands", default_value="false"),
            sim,
        ]
    )
