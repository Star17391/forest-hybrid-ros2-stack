"""Launch Gazebo + ros_gz_bridge + pose/sensor nodes for the forest-hybrid stack.

Worlds and models live in ForestGen (referenced via FORESTGEN_PATH env var or
the world_path argument). All ROS2 logic lives here in forest_sim_bridge.

Usage:
  ros2 launch forest_sim_bridge sim_gazebo.launch.py
  ros2 launch forest_sim_bridge sim_gazebo.launch.py world_path:=/path/to/world.sdf
"""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _lidar_mode_robot_model(lidar_mode: str) -> str | None:
    """Map lidar_mode launch arg to ForestGen model URI name (None = world default)."""
    mode = lidar_mode.strip().lower()
    if mode in ("2d", "lidar2d", "ydlidar"):
        return "forest_tracked_robot_lidar2d"
    if mode in ("3d", "lidar3d", "airy"):
        return "forest_tracked_robot_lidar3d"
    return None


def _prepare_world_with_robot_model(world_path: Path, robot_model: str | None) -> Path:
    if not robot_model:
        return world_path
    text = world_path.read_text(encoding="utf-8")
    pattern = r"(<uri>\s*)model://forest_tracked_robot(\s*</uri>)"
    if not re.search(pattern, text):
        raise RuntimeError(
            f"World {world_path} has no model://forest_tracked_robot include; "
            f"cannot apply sim_robot_model:={robot_model}"
        )
    patched = re.sub(pattern, rf"\1model://{robot_model}\2", text, count=1)
    fd, tmp = tempfile.mkstemp(prefix=f"forest_{world_path.stem}_", suffix=".sdf")
    os.close(fd)
    out = Path(tmp)
    out.write_text(patched, encoding="utf-8")
    return out


def _resolve_world(context) -> Path:
    """Resolve world path: explicit path > FORESTGEN_PATH/worlds/ > fallback."""
    world_path_arg = LaunchConfiguration("world_path").perform(context).strip()
    if world_path_arg and os.path.isfile(world_path_arg):
        return Path(world_path_arg)

    world_name = LaunchConfiguration("world").perform(context)
    forestgen = os.environ.get(
        "FORESTGEN_PATH",
        os.path.expanduser("~/Projetos/Gazebo/ForestGen"),
    )
    candidate = Path(forestgen) / "worlds" / world_name
    if candidate.is_file():
        return candidate

    raise FileNotFoundError(
        f"World not found: tried '{world_path_arg}' and '{candidate}'. "
        f"Set FORESTGEN_PATH or pass world_path:=/absolute/path.sdf"
    )


def _opaque_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory("forest_sim_bridge"))
    world_path = _resolve_world(context)
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context).strip()
    robot_model = _lidar_mode_robot_model(lidar_mode)
    world_path = _prepare_world_with_robot_model(world_path, robot_model)
    use_lidar_3d = robot_model == "forest_tracked_robot_lidar3d"

    forestgen = os.environ.get(
        "FORESTGEN_PATH",
        os.path.expanduser("~/Projetos/Gazebo/ForestGen"),
    )
    models_path = Path(forestgen) / "models"
    prev = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_res = f"{models_path}:{prev}" if prev else str(models_path)

    paused = LaunchConfiguration("paused").perform(context).lower() in ("1", "true", "yes")

    cleanup = ExecuteProcess(
        cmd=["ros2", "run", "forest_sim_bridge", "forest_cleanup"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("cleanup_first")),
    )

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription

    bridge_yaml = (
        share / "config" / "marble_bridges_lidar3d.yaml"
        if use_lidar_3d
        else share / "config" / "marble_bridges.yaml"
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": str(bridge_yaml),
            "bridge_name": "forest_sim_gz_bridge",
        }.items(),
    )

    gz_args = ["gz", "sim", str(world_path)]
    if not paused:
        gz_args = ["gz", "sim", "-r", str(world_path)]
    gz = ExecuteProcess(
        cmd=gz_args,
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": gz_res},
        sigterm_timeout="5",
        sigkill_timeout="3",
        on_exit=Shutdown(reason="gz sim terminated"),
    )

    use_sim = {"use_sim_time": True}

    use_state_est = LaunchConfiguration("use_state_estimation").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_pose_bridge = LaunchConfiguration("use_pose_bridge").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    if use_state_est and use_pose_bridge:
        raise RuntimeError(
            "Configuração inválida: use_state_estimation e use_pose_bridge não podem ser "
            "true em simultâneo — ambos publicam map→base_link (EKF via map=odom + "
            "marble_pose_from_gz). Use pose_bridge só para diagnóstico isolado."
        )

    use_slam = LaunchConfiguration("use_slam").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    pub_map_odom = LaunchConfiguration("publish_map_odom_identity").perform(context).lower()
    if use_slam:
        # Bootstrap: static map→odom em /tf_static até o slam_toolbox publicar em /tf.
        # Sem isto, o frame "map" não existe no arranque → RViz/state_contract falham.
        pub_map_odom_str = "true"
    else:
        pub_map_odom_str = "true" if pub_map_odom in ("1", "true", "yes") else "false"

    use_legacy = LaunchConfiguration("use_legacy_sensors").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    rviz_cfg = LaunchConfiguration("rviz_config").perform(context).strip()
    if not rviz_cfg:
        if use_pose_bridge and not use_state_est:
            rviz_cfg = str(share / "config" / "forest_pose_bridge_sim.rviz")
        else:
            rviz_cfg = str(share / "config" / "marble_sim.rviz")

    # Wayland: Qt dock layout breaks without saved geometry; X11 (xcb) matches classic RViz layout.
    rviz_env = dict(os.environ)
    if rviz_env.get("XDG_SESSION_TYPE") == "wayland" and not rviz_env.get("QT_QPA_PLATFORM"):
        rviz_env["QT_QPA_PLATFORM"] = "xcb"

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[use_sim],
        additional_env=rviz_env,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        sigterm_timeout="3",
        sigkill_timeout="2",
    )

    rviz_maximize = ExecuteProcess(
        cmd=["bash", str(share / "scripts" / "rviz_maximize.sh")],
        output="log",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    pose_bridge = Node(
        package="forest_sim_bridge",
        executable="marble_pose_from_gz",
        name="marble_pose_from_gz",
        output="screen",
        parameters=[
            use_sim,
            {
                "source_topic": "/forest_gen/gz/world_tf",
                "fallback_source_topic": "/forest_gen/gz/world_tf_full",
                "fallback_timeout_sec": 1.0,
                "model_name": "marble_hd2",
                "parent_frame": "map",
                "child_frame": "marble_hd2/base_link",
                "republish_hz": 20.0,
                "stale_tf_max_age_sec": 0.5,
                "seed_x": 0.0,
                "seed_y": 0.0,
                "seed_z": 0.35,
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_pose_bridge")),
    )

    sensors_cpp_share = get_package_share_directory("forest_sensors_cpp")
    preprocess_share = get_package_share_directory("forest_lidar_preprocess_cpp")
    if use_lidar_3d:
        lidar_extrinsics = os.path.join(
            sensors_cpp_share, "config", "forest_lidar_extrinsics_airy.yaml"
        )
        lidar_classify_yaml = os.path.join(
            preprocess_share, "config", "forest_lidar_preprocess_lidar3d.yaml"
        )
    else:
        lidar_extrinsics = os.path.join(sensors_cpp_share, "config", "forest_lidar_extrinsics.yaml")
        lidar_classify_yaml = os.path.join(
            preprocess_share, "config", "forest_lidar_preprocess.yaml"
        )
    lidar_pipeline = os.path.join(sensors_cpp_share, "config", "forest_lidar_preprocess.yaml")

    state_est_share = get_package_share_directory("forest_state_estimation")
    ekf_mode = LaunchConfiguration("ekf_mode").perform(context).strip().lower()
    ekf_wheel_only = os.path.join(state_est_share, "config", "ekf_wheel_only.yaml")
    ekf_local = os.path.join(state_est_share, "config", "ekf_local.yaml")
    if ekf_mode in ("local", "imu", "wheel_imu", "ekf_local"):
        ekf_config_path = ekf_local
    else:
        ekf_config_path = ekf_wheel_only

    # Modo 3D: state_estimation atrasa 6 s — publicar map→odom cedo (RViz / tf_static).
    map_odom_early = None
    if pub_map_odom_str == "true" and use_lidar_3d:
        map_odom_early = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_odom_identity_sim",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[use_sim],
        )
    state_est_map_odom = "false" if map_odom_early is not None else pub_map_odom_str

    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_est_share, "launch", "state_estimation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_gnss": LaunchConfiguration("use_gnss"),
            "use_wheel_odom": LaunchConfiguration("use_odom_relay"),
            "publish_map_odom_identity": state_est_map_odom,
            "ekf_config": ekf_config_path,
            "use_lidar_preprocess": "false" if use_lidar_3d else "true",
            "lidar_extrinsics_config": lidar_extrinsics,
            "publish_lidar_static_tf": "false" if use_lidar_3d else "true",
        }.items(),
    ) if use_state_est else None

    # LiDAR 3D: laser TF no t=1s (classify/RViz); EKF atrasa 6s — evita duplicar em state_est.
    lidar_tf_early = None
    if use_lidar_3d and use_state_est:
        lidar_tf_early = Node(
            package="forest_sensors_cpp",
            executable="static_sensor_tf_node",
            name="static_sensor_tf_node",
            output="screen",
            parameters=[use_sim, lidar_extrinsics],
        )

    sensor_tf_static = Node(
        package="forest_sensors_cpp",
        executable="static_sensor_tf_node",
        name="static_sensor_tf_node",
        output="screen",
        parameters=[use_sim, lidar_extrinsics],
        condition=IfCondition(LaunchConfiguration("use_legacy_sensors")),
    )

    lidar_2d_legacy = not use_lidar_3d
    lidar_scan_preprocess = Node(
        package="forest_sensors_cpp",
        executable="laserscan_preprocess_node",
        name="laserscan_preprocess_node",
        output="screen",
        parameters=[use_sim, lidar_pipeline],
        condition=IfCondition(LaunchConfiguration("use_legacy_sensors")),
    ) if lidar_2d_legacy else None

    lidar_scan_to_cloud = Node(
        package="forest_sensors_cpp",
        executable="laserscan_to_pointcloud2_node",
        name="laserscan_to_pointcloud2_node",
        output="screen",
        parameters=[use_sim, lidar_pipeline],
        condition=IfCondition(LaunchConfiguration("use_legacy_sensors")),
    ) if lidar_2d_legacy else None

    lidar_classify = Node(
        package="forest_lidar_preprocess_cpp",
        executable="lidar_scan_classify_node",
        name="lidar_scan_classify_node",
        output="screen",
        parameters=[use_sim, lidar_classify_yaml],
    )

    # Fase 1: 3D segmentation (ground/trunks/obstacles) — só em modo 3D.
    lidar3d_seg = None
    if use_lidar_3d:
        seg_share = get_package_share_directory("forest_3d_perception")
        seg_yaml = os.environ.get(
            "FOREST_LIDAR3D_SEG_CONFIG",
            os.path.join(seg_share, "config", "forest_3d_segmentation.yaml"),
        )
        seg_params = [use_sim, seg_yaml]
        slice_overlay = os.path.join(seg_share, "config", "forest_3d_segmentation_slice.yaml")
        column_overlay = os.path.join(seg_share, "config", "forest_3d_segmentation_column.yaml")
        use_slice = os.environ.get("FOREST_LIDAR3D_TRUNK_SLICE", "").strip().lower() in (
            "1", "true", "yes"
        ) or "segmentation_slice" in os.path.basename(seg_yaml)
        use_column = os.environ.get("FOREST_LIDAR3D_TRUNK_COLUMN", "").strip().lower() in (
            "1", "true", "yes"
        ) or "segmentation_column" in os.path.basename(seg_yaml)
        if os.path.isfile(slice_overlay) and use_slice:
            seg_params.append(slice_overlay)
        elif os.path.isfile(column_overlay) and use_column:
            seg_params.append(column_overlay)
        lidar3d_seg = Node(
            package="forest_3d_perception",
            executable="lidar3d_segmentation_node",
            name="lidar3d_segmentation_node",
            output="screen",
            parameters=seg_params,
        )

    # Pose-bridge mode: no EKF → TF is map→base only; track odom headers use map (not orphan odom).
    track_odom_parent = "map" if (use_pose_bridge and not use_state_est) else "odom"
    odom_stamp = Node(
        package="forest_sim_bridge",
        executable="gz_track_odometry_stamp",
        name="gz_track_odometry_stamp",
        output="screen",
        parameters=[use_sim, {"parent_frame": track_odom_parent}],
        condition=IfCondition(LaunchConfiguration("use_odom_relay")),
    )

    imu_sanitize_legacy = None
    if use_legacy and not use_state_est:
        imu_sanitize_legacy = Node(
            package="forest_sensors_cpp",
            executable="imu_sanitize_node",
            name="imu_sanitize_node",
            output="screen",
            parameters=[use_sim, lidar_pipeline],
        )

    camera_tf_static = Node(
        package="forest_sim_bridge",
        executable="marble_sensor_tf_static",
        name="marble_sensor_tf_static",
        output="screen",
        parameters=[use_sim, {"parent_frame": "marble_hd2/base_link", "republish_period_sec": 5.0}],
        condition=IfCondition(LaunchConfiguration("use_sensor_tf_static")),
    )

    try:
        rviz_delay = float(LaunchConfiguration("rviz_delay_sec").perform(context))
    except ValueError:
        rviz_delay = 8.0
    rviz_delay = max(rviz_delay, 3.0)

    odom_bootstrap = None
    if use_lidar_3d and use_state_est:
        odom_bootstrap = Node(
            package="forest_sim_bridge",
            executable="gz_odom_base_bootstrap",
            name="gz_odom_base_bootstrap",
            output="screen",
            parameters=[use_sim],
        )

    # RViz must start after Gazebo publishes /clock — early start crashes Ogre (no GL buffers).
    main_actions = [
        bridge,
        gz,
    ]
    if map_odom_early is not None:
        main_actions.append(map_odom_early)
    if lidar_tf_early is not None:
        main_actions.append(lidar_tf_early)
    if odom_bootstrap is not None:
        main_actions.append(odom_bootstrap)
    if state_estimation is not None:
        # LiDAR 3D pesado: dar tempo ao Gazebo/esteiras antes do EKF.
        if use_lidar_3d:
            main_actions.append(TimerAction(period=6.0, actions=[state_estimation]))
        else:
            main_actions.append(state_estimation)
    main_actions.extend(
        [
            camera_tf_static,
            odom_stamp,
            pose_bridge,
            lidar_classify,
        ]
    )
    if lidar3d_seg is not None:
        main_actions.append(lidar3d_seg)
    if state_estimation is None:
        legacy_nodes = [n for n in (sensor_tf_static, lidar_scan_preprocess, lidar_scan_to_cloud) if n]
        if imu_sanitize_legacy is not None:
            legacy_nodes.insert(0, imu_sanitize_legacy)
        main_actions[2:2] = legacy_nodes  # após bridge + gz
    delayed_main = TimerAction(period=1.0, actions=main_actions)

    slam_delayed = None
    if use_slam and use_lidar_3d:
        raise RuntimeError(
            "use_slam com lidar_mode:=3d não suportado (sem /scan 2D). "
            "Use --lidar2d ou perfil sim-lidar3d-test."
        )
    if use_slam:
        if not use_state_est:
            raise RuntimeError("use_slam requer use_state_estimation:=true (EKF odom→base).")
        loc_share = get_package_share_directory("forest_2d_localization")
        scan_topic = LaunchConfiguration("slam_scan_topic").perform(context).strip()
        if not scan_topic:
            scan_topic = "/sensors/lidar/scan"
        slam_inc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, "launch", "slam_toolbox_online_async.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_topic": scan_topic,
            }.items(),
        )
        slam_delayed = TimerAction(period=10.0, actions=[slam_inc])

    delayed_rviz = TimerAction(
        period=rviz_delay,
        actions=[rviz, TimerAction(period=2.0, actions=[rviz_maximize])],
    )

    _final_cleanup_done = {"v": False}

    def _on_shutdown_actions(*_args, **_kwargs):
        if _final_cleanup_done["v"]:
            return []
        _final_cleanup_done["v"] = True
        return [
            ExecuteProcess(
                cmd=["ros2", "run", "forest_sim_bridge", "forest_cleanup", "--term-wait", "0.8"],
                output="screen",
            )
        ]

    on_shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=_on_shutdown_actions)
    )

    launch_sequence = [delayed_main]
    if slam_delayed is not None:
        launch_sequence.append(slam_delayed)
    launch_sequence.append(delayed_rviz)

    if LaunchConfiguration("cleanup_first").perform(context).lower() in ("1", "true", "yes"):
        return [
            cleanup,
            RegisterEventHandler(
                OnProcessExit(target_action=cleanup, on_exit=launch_sequence)
            ),
            on_shutdown_handler,
        ]
    return launch_sequence + [on_shutdown_handler]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="",
                description="2d | 3d — selects ForestGen robot model and sensor pipeline",
            ),
            DeclareLaunchArgument("world", default_value="mvp_empty_flat.sdf"),
            DeclareLaunchArgument("world_path", default_value="",
                                  description="Absolute path to .sdf world (overrides 'world')"),
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument("cleanup_first", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            DeclareLaunchArgument("use_pose_bridge", default_value="true"),
            DeclareLaunchArgument("use_sensor_tf_static", default_value="true"),
            DeclareLaunchArgument(
                "use_state_estimation",
                default_value="false",
                description="Camada 0: preprocess + EKF (robot_localization); desliga sensores legacy",
            ),
            DeclareLaunchArgument(
                "use_legacy_sensors",
                default_value="true",
                description="TF/preprocess/cloud sem EKF (ignorado se use_state_estimation=true)",
            ),
            DeclareLaunchArgument("use_gnss", default_value="false"),
            DeclareLaunchArgument("use_odom_relay", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "rviz_delay_sec",
                default_value="8",
                description="Delay before RViz (wait for Gazebo /clock; avoids Ogre crash)",
            ),
            DeclareLaunchArgument(
                "ekf_mode",
                default_value="wheel_only",
                description="wheel_only | local (wheel+IMU gyro Z) — passed to state_estimation ekf_config",
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="false",
                description="Fase 2: slam_toolbox map→odom (desliga map_odom identity)",
            ),
            DeclareLaunchArgument(
                "publish_map_odom_identity",
                default_value="true",
                description="Static map→odom; false quando use_slam ou SLAM activo",
            ),
            DeclareLaunchArgument(
                "slam_scan_topic",
                default_value="/perception/lidar/scan_ground",
                description="LaserScan input for slam_toolbox",
            ),
            OpaqueFunction(function=_opaque_setup),
        ]
    )
