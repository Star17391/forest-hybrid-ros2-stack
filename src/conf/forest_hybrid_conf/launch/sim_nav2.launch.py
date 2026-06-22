"""DEPRECATED — usar perfil forest `sim-nav2` (layers sim + nav2).

Mantido só para compatibilidade manual:
  ros2 launch forest_hybrid_conf sim_nav2.launch.py
"""

from __future__ import annotations

import warnings

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    warnings.warn(
        "sim_nav2.launch.py está deprecated — usa: forest up sim-nav2",
        DeprecationWarning,
        stacklevel=1,
    )
    return LaunchDescription(
        [
            LogInfo(msg="DEPRECATED: preferir forest up sim-nav2 (layers sim + nav2)"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value="forest_gentle_trees_rocks.sdf"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("forest_hybrid_conf"), "launch", "sim_mvp_slam.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "start_navigation": "false",
                    "lidar_mode": "2d",
                    "paused": "false",
                }.items(),
            ),
        ]
    )
