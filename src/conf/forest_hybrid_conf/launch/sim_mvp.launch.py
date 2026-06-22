"""MVP completo: Gazebo + bridge + pose + navigation + mission + RViz.

All ROS2 logic comes from forest_sim_bridge (bridge nodes) and the hybrid
navigation/planner packages. ForestGen only provides worlds/models via
FORESTGEN_PATH env var.

Usage:
  ros2 launch forest_hybrid_conf sim_mvp.launch.py
  ros2 launch forest_hybrid_conf sim_mvp.launch.py world:=unified_forest_rocks.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    hybrid_share = get_package_share_directory("forest_hybrid_conf")
    sim_bridge_share = get_package_share_directory("forest_sim_bridge")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    paused = LaunchConfiguration("paused")
    cleanup_first = LaunchConfiguration("cleanup_first")
    use_rviz = LaunchConfiguration("use_rviz")
    ekf_mode = LaunchConfiguration("ekf_mode")
    rviz_delay = LaunchConfiguration("rviz_delay_sec")
    # Default sensors tier (stable). Override: rviz_profile:=full or FOREST_RVIZ_PROFILE=minimal
    _rviz_tier = os.environ.get("FOREST_RVIZ_PROFILE", "sensors")
    _rviz_map = {
        "minimal": "forest_mvp_minimal.rviz",
        "sensors": "forest_mvp_sensors.rviz",
        "full": "forest_mvp_sim.rviz",
    }
    rviz_cfg = os.path.join(
        hybrid_share, "config", _rviz_map.get(_rviz_tier, _rviz_map["sensors"])
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge_share, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "paused": paused,
            "cleanup_first": cleanup_first,
            "use_rviz": use_rviz,
            "rviz_config": rviz_cfg,
            "use_pose_bridge": "false",
            "use_sensor_tf_static": "true",
            "use_state_estimation": "true",
            "use_odom_relay": "true",
            "use_gnss": "false",
            "ekf_mode": ekf_mode,
            "rviz_delay_sec": rviz_delay,
        }.items(),
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hybrid_share, "launch", "navigation_mvp.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    nav_stack_delayed = TimerAction(period=5.0, actions=[nav_stack])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument("cleanup_first", default_value="true"),
            DeclareLaunchArgument(
                "ekf_mode",
                default_value="local",
                description="local (wheel+IMU SE3, default/melhor) | wheel_only (2D degradado) — EKF profile",
            ),
            DeclareLaunchArgument(
                "rviz_delay_sec",
                default_value="8",
                description="Seconds before RViz (Gazebo /clock must exist first)",
            ),
            sim,
            nav_stack_delayed,
        ]
    )
