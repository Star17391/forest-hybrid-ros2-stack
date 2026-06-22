"""MVP + sim + LiDAR 3D; SLAM 2D (slam_toolbox) está **LEGACY / CONGELADO**.

Só o caminho slam_toolbox (use_slam:=true) é legacy — substituído por Tree-SLAM.
Com use_slam:=false isto é o bringup normal de sim + EKF + LiDAR 3D (perfis
sim-lidar3d-*). Ver docs/LEGACY_PATHS.md.
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
    # Só o caminho SLAM 2D (slam_toolbox, use_slam:=true) é legacy. Com use_slam:=false
    # isto é o bringup normal de sim + EKF + LiDAR 3D (perfis sim-lidar3d-*).
    use_slam = LaunchConfiguration("use_slam").perform(context).strip().lower() in (
        "1", "true", "yes",
    )
    allow_legacy = os.environ.get("FOREST_ALLOW_LEGACY", "").strip().lower() in (
        "1", "true", "yes",
    )
    if use_slam and not allow_legacy:
        raise RuntimeError(
            "use_slam:=true (slam_toolbox) está LEGACY. Substituto: Tree-SLAM. "
            "Ver docs/LEGACY_PATHS.md. Comparação histórica: export FOREST_ALLOW_LEGACY=1"
        )
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
            "publish_map_odom_identity": LaunchConfiguration("publish_map_odom_identity"),
            "slam_scan_topic": LaunchConfiguration("slam_scan_topic"),
            "lidar_mode": LaunchConfiguration("lidar_mode"),
            "use_experimental_lidar3d": LaunchConfiguration("use_experimental_lidar3d"),
        }.items(),
    )

    start_navigation = LaunchConfiguration("start_navigation").perform(context).strip().lower() in (
        "1", "true", "yes",
    )
    if not start_navigation:
        return [sim]

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hybrid_share, "launch", "navigation_mvp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_nav2": LaunchConfiguration("use_nav2"),
        }.items(),
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
                "publish_map_odom_identity",
                default_value="true",
                description=(
                    "true=identity (Fase-1, sem Tree-SLAM); false=silent "
                    "(cede a autoridade map->odom no solo ao forest_tree_slam, Fase 2+)"
                ),
            ),
            DeclareLaunchArgument(
                "use_experimental_lidar3d",
                default_value="true",
                description="Start lidar3d_experimental_node (CSF + stem-band + region growing)",
            ),
            DeclareLaunchArgument(
                "start_navigation",
                default_value="true",
                description=(
                    "false = só sim+EKF+LiDAR (navegação numa layer separada do perfil forest)"
                ),
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="false",
                description="Só com start_navigation:=true — true=Nav2, false=legacy navigation_node",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
