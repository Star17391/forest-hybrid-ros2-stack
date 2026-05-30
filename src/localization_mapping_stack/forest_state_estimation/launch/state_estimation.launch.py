"""Camada 0: EKF (robot_localization) + GNSS opcional + /state/pose_fused."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_state_estimation")
    sensors_pkg = get_package_share_directory("forest_sensors_cpp")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gnss = LaunchConfiguration("use_gnss")
    use_wheel_odom = LaunchConfiguration("use_wheel_odom")
    ekf_wheel_only = os.path.join(pkg, "config", "ekf_wheel_only.yaml")
    ekf_local = os.path.join(pkg, "config", "ekf_local.yaml")
    ekf_config = LaunchConfiguration("ekf_config")
    ekf_gnss = os.path.join(pkg, "config", "ekf.yaml")
    navsat_yaml = os.path.join(pkg, "config", "navsat_transform.yaml")
    frames_yaml = os.path.join(pkg, "config", "frames.yaml")
    lidar_preprocess = os.path.join(sensors_pkg, "config", "forest_lidar_preprocess.yaml")
    default_lidar_extrinsics = os.path.join(sensors_pkg, "config", "forest_lidar_extrinsics.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_gnss", default_value="false"),
            DeclareLaunchArgument("use_wheel_odom", default_value="true"),
            DeclareLaunchArgument("use_lidar_preprocess", default_value="true"),
            DeclareLaunchArgument(
                "lidar_extrinsics_config",
                default_value=default_lidar_extrinsics,
                description="static_sensor_tf YAML (2D laser or airy_lidar)",
            ),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=ekf_wheel_only,
                description="ekf_wheel_only.yaml (sim) or ekf_local.yaml (wheel+IMU)",
            ),
            # Identity map->odom until navsat adjusts it (no GNSS).
            DeclareLaunchArgument("publish_map_odom_identity", default_value="true"),
            DeclareLaunchArgument(
                "publish_lidar_static_tf",
                default_value="true",
                description="false when sim publishes base_link->laser early (LiDAR 3D)",
            ),
            Node(
                package="forest_sensors_cpp",
                executable="imu_sanitize_node",
                name="imu_sanitize_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, lidar_preprocess],
            ),
            Node(
                package="forest_sensors_cpp",
                executable="static_sensor_tf_node",
                name="static_sensor_tf_node",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    ParameterFile(
                        LaunchConfiguration("lidar_extrinsics_config"),
                        allow_substs=True,
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("publish_lidar_static_tf")),
            ),
            Node(
                package="forest_sensors_cpp",
                executable="laserscan_preprocess_node",
                name="laserscan_preprocess_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, lidar_preprocess],
                condition=IfCondition(LaunchConfiguration("use_lidar_preprocess")),
            ),
            Node(
                package="forest_sensors_cpp",
                executable="laserscan_to_pointcloud2_node",
                name="laserscan_to_pointcloud2_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, lidar_preprocess],
                condition=IfCondition(LaunchConfiguration("use_lidar_preprocess")),
            ),
            Node(
                package="forest_sensors_cpp",
                executable="tracked_wheel_odometry_node",
                name="tracked_wheel_odometry_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, lidar_preprocess],
                condition=IfCondition(use_wheel_odom),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_identity",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("publish_map_odom_identity")),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_config],
                remappings=[("odometry/filtered", "/state/odometry")],
                condition=UnlessCondition(use_gnss),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_gnss],
                remappings=[("odometry/filtered", "/state/odometry")],
                condition=IfCondition(use_gnss),
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, navsat_yaml],
                remappings=[
                    ("imu", "/sensors/imu/data"),
                    ("gps/fix", "/sensors/gnss/fix"),
                    ("odometry/filtered", "/state/odometry"),
                    ("odometry/gps", "/odometry/gps"),
                ],
                condition=IfCondition(use_gnss),
            ),
            Node(
                package="forest_state_estimation",
                executable="state_contract_node",
                name="state_contract_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, frames_yaml],
            ),
        ]
    )
