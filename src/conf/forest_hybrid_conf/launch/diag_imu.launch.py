"""Teste isolado: Gazebo + bridge IMU + imu_sanitize + RViz (sem EKF, sem navegação)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_bridge = get_package_share_directory("forest_sim_bridge")
    sensors_pkg = get_package_share_directory("forest_sensors_cpp")
    hybrid = get_package_share_directory("forest_hybrid_conf")
    rviz = os.path.join(hybrid, "config", "forest_diag_imu.rviz")
    lidar_preprocess = os.path.join(sensors_pkg, "config", "forest_lidar_preprocess.yaml")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bridge, "launch", "sim_gazebo.launch.py")
        ),
        launch_arguments={
            "world": "mvp_empty_flat.sdf",
            "paused": "false",
            "cleanup_first": "true",
            "use_rviz": "true",
            "rviz_config": rviz,
            "use_pose_bridge": "false",
            "use_state_estimation": "false",
            "use_legacy_sensors": "false",
            "use_sensor_tf_static": "false",
            "use_odom_relay": "false",
        }.items(),
    )

    imu_sanitize = Node(
        package="forest_sensors_cpp",
        executable="imu_sanitize_node",
        name="imu_sanitize_node",
        output="screen",
        parameters=[{"use_sim_time": True}, lidar_preprocess],
    )

    imu_markers = Node(
        package="forest_sim_bridge",
        executable="imu_debug_markers",
        name="imu_debug_markers",
        output="screen",
        parameters=[{"use_sim_time": True, "imu_topic": "/sensors/imu/data_raw"}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            sim,
            imu_sanitize,
            imu_markers,
        ]
    )
