"""Camada estimador de estado SE3 (forest_state_estimation).

Arquitectura (FOREST_TREE_SLAM_DESIGN.md §4–6, LAYER_CONTRACTS.md) — UM EKF + autoridade:
  EKF local  (odom→base):  wheel + IMU — sempre a correr (único robot_localization EKF)
  map_odom_authority_node: autoridade ÚNICA de map→odom — comuta a FONTE por modo:
      GROUND → identidade (ground_mode: identity, Fase-1) ou silencioso (Tree-SLAM, Fase 2+)
      AERIAL → pose do ArduPilot DIRETA (posição + atitude); NÃO há 2.º EKF
  state_contract_node:     publica /state/pose_fused (PoseStamped, frame=map)
  navsat_transform/gnss_cov_adapter: só com use_gnss (dormentes; GNSS volta no Tree-SLAM)

NOTA: o antigo ekf_global (robot_localization) foi removido — re-filtrava o ArduPilot no ar
(double-counting). A pose absoluta no ar vem do EKF3 do ArduPilot (§6). O frame map existe
SEMPRE (a autoridade publica identidade no solo), não só em AERIAL.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_state_estimation")
    sensors_pkg = get_package_share_directory("forest_sensors_cpp")

    use_sim_time     = LaunchConfiguration("use_sim_time")
    use_gnss         = LaunchConfiguration("use_gnss")
    use_wheel_odom   = LaunchConfiguration("use_wheel_odom")

    ekf_wheel_only   = os.path.join(pkg, "config", "ekf_wheel_only.yaml")
    navsat_yaml      = os.path.join(pkg, "config", "navsat_transform.yaml")
    gnss_cov_yaml    = os.path.join(pkg, "config", "gnss_cov_adapter.yaml")
    authority_yaml   = os.path.join(pkg, "config", "map_odom_authority.yaml")
    frames_yaml      = os.path.join(pkg, "config", "frames.yaml")

    lidar_preprocess        = os.path.join(sensors_pkg, "config", "forest_lidar_preprocess.yaml")
    default_lidar_extrinsics = os.path.join(sensors_pkg, "config", "forest_lidar_extrinsics.yaml")

    ekf_config = LaunchConfiguration("ekf_config")

    # publish_map_odom_identity → ground_mode da autoridade:
    #   true  → "identity" (a autoridade publica/mantém map→odom no solo)
    #   false → "silent"   (outro publisher é a autoridade no solo: Tree-SLAM, ou o
    #                       static map→odom precoce do sim_gazebo no modo LiDAR 3D)
    authority_ground_mode = PythonExpression([
        "'identity' if '", LaunchConfiguration("publish_map_odom_identity"),
        "'.lower() in ('1', 'true', 'yes') else 'silent'",
    ])

    return LaunchDescription(
        [
            # ── Argumentos ──────────────────────────────────────────────────
            DeclareLaunchArgument("use_sim_time",   default_value="false"),
            DeclareLaunchArgument("use_gnss",       default_value="false"),
            DeclareLaunchArgument("use_wheel_odom", default_value="true"),
            DeclareLaunchArgument("use_lidar_preprocess", default_value="true"),
            DeclareLaunchArgument(
                "lidar_extrinsics_config",
                default_value=default_lidar_extrinsics,
                description="static_sensor_tf YAML (2D laser ou airy_lidar)"),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=ekf_wheel_only,
                description="ekf_wheel_only.yaml (sim sem IMU) ou ekf_local.yaml (wheel+IMU SE3)"),
            # Controla o ground_mode da autoridade map→odom: true=identity (publica/mantém
            # no solo), false=silent (cede a outro publisher — Tree-SLAM ou static do sim).
            DeclareLaunchArgument("publish_map_odom_identity", default_value="true"),
            DeclareLaunchArgument(
                "publish_lidar_static_tf",
                default_value="true",
                description="false quando a sim publica base_link→laser antecipadamente (LiDAR 3D)"),

            # ── Pré-processamento de sensores ────────────────────────────────
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

            # ── EKF local SE3 (odom→base): wheel + IMU — ÚNICO EKF, sempre a correr ─
            # Publica TF odom→base_link e /state/odometry. A correção map→odom é da
            # autoridade (solo: identidade/Tree-SLAM; ar: ArduPilot direto), não de um 2.º EKF.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_config],
                remappings=[("odometry/filtered", "/state/odometry")],
            ),

            # ── GNSS covariance adapter ──────────────────────────────────────
            # /sensors/gnss/fix → /sensors/gnss/fix_adapted (cov inflada para 5–20 m)
            Node(
                package="forest_state_estimation",
                executable="gnss_cov_adapter_node",
                name="gnss_cov_adapter_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, gnss_cov_yaml],
                condition=IfCondition(use_gnss),
            ),

            # ── navsat_transform (NavSatFix → /odometry/gps) ────────────────
            # Usa /sensors/gnss/fix_adapted (covariância realista).
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, navsat_yaml],
                remappings=[
                    ("imu",              "/sensors/imu/data"),
                    ("gps/fix",          "/sensors/gnss/fix_adapted"),
                    ("odometry/filtered", "/state/odometry"),
                    ("odometry/gps",     "/odometry/gps"),
                ],
                condition=IfCondition(use_gnss),
            ),

            # ── map_odom_authority_node ──────────────────────────────────────
            # Autoridade ÚNICA de map→odom, SEMPRE a correr. Solo: identidade (ground_mode);
            # Ar: pose do ArduPilot direta. Garante que o frame map existe sempre.
            Node(
                package="forest_state_estimation",
                executable="map_odom_authority_node",
                name="map_odom_authority_node",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    authority_yaml,
                    {"ground_mode": authority_ground_mode},
                ],
            ),

            # ── state_contract_node ──────────────────────────────────────────
            # Publica /state/pose_fused (PoseStamped no referencial map) via TF map→base_link.
            Node(
                package="forest_state_estimation",
                executable="state_contract_node",
                name="state_contract_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, frames_yaml],
            ),
        ]
    )
