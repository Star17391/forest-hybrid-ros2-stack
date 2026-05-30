"""slam_toolbox async mapping — publica TF map→odom (Fase 2).

Jazzy: ``async_slam_toolbox_node`` é LifecycleNode. Sem configure+activate não subscreve
``scan`` nem publica ``/map`` (só serviços lifecycle + /clock).
Espelha ``slam_toolbox/launch/online_async_launch.py`` upstream.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_2d_localization")
    config = os.path.join(pkg, "config", "mapper_params_forest_sim.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": use_lifecycle_manager,
            },
        ],
        remappings=[("scan", scan_topic)],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[forest] slam_toolbox: activating (subscribe scan, publish /map)"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/sensors/lidar/scan",
                description="LaserScan for SLAM",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Configure+activate lifecycle (required on Jazzy)",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="slam_toolbox bond manager (off for sim)",
            ),
            slam,
            configure_event,
            activate_event,
        ]
    )
