"""MVP completo: Gazebo (ForestGen worlds) + missão + navegação + RViz debug (hybrid stack).

Requisitos:
  - forest_gen_bringup instalado (mundos/modelos em share/forest_gen_bringup)
  - forest_hybrid_conf + forest_navigation_ros2 + forest_planner_ros2

Uso:
  ros2 launch forest_hybrid_conf sim_mvp.launch.py
  # Clica ▶ no Gazebo se paused:=true (default)
  ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \\
    "{command_type: 1, frame_type: 0, command_id: 'goto1', source: 'cli', target_x: 5.0, target_y: 0.0, target_z: 0.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    hybrid_share = get_package_share_directory("forest_hybrid_conf")
    forest_share = get_package_share_directory("forest_gen_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    paused = LaunchConfiguration("paused")
    cleanup_first = LaunchConfiguration("cleanup_first")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = os.path.join(hybrid_share, "config", "forest_mvp_sim.rviz")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(forest_share, "launch", "sim_rviz.launch.py")
        ),
        launch_arguments={
            "world": world,
            "paused": paused,
            "cleanup_first": cleanup_first,
            "use_rviz": use_rviz,
            "rviz_config": rviz_cfg,
            "use_keyboard": "false",
            "use_cmd_vel_relay": "true",
            "linear_scale": "2.5",
            "angular_scale": "1.5",
            "use_pose_bridge": "true",
            "use_sensor_tf_static": "true",
            "use_odom_relay": "false",
        }.items(),
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hybrid_share, "launch", "navigation_mvp.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value="mvp_empty_flat.sdf",
                description="Mundo em share/forest_gen_bringup/worlds/",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="true",
                description="Gazebo pausado no arranque — clica ▶ antes de GOTO",
            ),
            DeclareLaunchArgument(
                "cleanup_first",
                default_value="true",
                description="Mata processos zombies antes do Gazebo (desliga se já limpaste manualmente)",
            ),
            sim,
            nav_stack,
        ]
    )
