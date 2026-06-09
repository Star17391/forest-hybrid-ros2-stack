"""MVP + Fase 2 SLAM (slam_toolbox map→odom) + navegação + RViz full.

Usage:
  ros2 launch forest_hybrid_conf sim_mvp_slam.launch.py
  forest up sim-slam-nav -d
  forest up sim-lidar3d-test -d --lidar3d
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _rviz_config_for_mode(lidar_mode: str, use_experimental_lidar3d: str) -> str:
    hybrid_share = get_package_share_directory("forest_hybrid_conf")
    mode = lidar_mode.strip().lower()
    exp = use_experimental_lidar3d.strip().lower() in ("1", "true", "yes")
    if mode in ("3d", "lidar3d", "airy"):
        if exp:
            return os.path.join(hybrid_share, "config", "forest_mvp_sim_lidar3d_experimental.rviz")
        return os.path.join(hybrid_share, "config", "forest_mvp_sim_lidar3d.rviz")
    return os.path.join(hybrid_share, "config", "forest_mvp_sim.rviz")


def _setup(context, *_args, **_kwargs):
    hybrid_share = get_package_share_directory("forest_hybrid_conf")
    sim_bridge_share = get_package_share_directory("forest_sim_bridge")

    lidar_mode = LaunchConfiguration("lidar_mode").perform(context)
    use_experimental = LaunchConfiguration("use_experimental_lidar3d").perform(context)
    rviz_cfg = _rviz_config_for_mode(lidar_mode, use_experimental)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge_share, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "paused": LaunchConfiguration("paused"),
            "cleanup_first": LaunchConfiguration("cleanup_first"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": rviz_cfg,
            "use_pose_bridge": "false",
            "use_sensor_tf_static": "true",
            "use_state_estimation": "true",
            "use_odom_relay": "true",
            "use_gnss": "false",
            "ekf_mode": LaunchConfiguration("ekf_mode"),
            "rviz_delay_sec": LaunchConfiguration("rviz_delay_sec"),
            "use_slam": LaunchConfiguration("use_slam"),
            "publish_map_odom_identity": "true",
            "slam_scan_topic": LaunchConfiguration("slam_scan_topic"),
            "lidar_mode": LaunchConfiguration("lidar_mode"),
            "use_experimental_lidar3d": LaunchConfiguration("use_experimental_lidar3d"),
            "use_legacy_lidar3d": LaunchConfiguration("use_legacy_lidar3d"),
        }.items(),
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hybrid_share, "launch", "navigation_mvp.launch.py")
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )
    return [sim, TimerAction(period=5.0, actions=[nav_stack])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument("cleanup_first", default_value="true"),
            DeclareLaunchArgument("ekf_mode", default_value="local"),
            DeclareLaunchArgument("rviz_delay_sec", default_value="8"),
            DeclareLaunchArgument(
                "slam_scan_topic",
                default_value="/sensors/lidar/scan",
                description="LaserScan para SLAM (só modo 2D)",
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="",
                description="2d | 3d (via forest up --lidar2d / --lidar3d)",
            ),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument(
                "use_experimental_lidar3d",
                default_value="false",
                description="Start lidar3d_experimental_node (CSF pipeline)",
            ),
            DeclareLaunchArgument(
                "use_legacy_lidar3d",
                default_value="true",
                description="Start legacy lidar3d_segmentation_node",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
