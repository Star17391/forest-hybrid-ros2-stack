# forest_2d_localization — Fase 2

Integração **slam_toolbox** (`async_slam_toolbox_node`) para publicar **TF `map → odom`**.

| Owner | Transform |
|-------|-----------|
| slam_toolbox | `map → odom` |
| EKF (`forest_state_estimation`) | `odom → marble_hd2/base_link` |
| static | `base_link → laser`, câmaras |

**Não** activar `publish_map_odom_identity` nem `marble_pose_from_gz` em simultâneo.

## Dependência

```bash
sudo apt install ros-jazzy-slam-toolbox
```

## Launch

```bash
ros2 launch forest_2d_localization slam_toolbox_online_async.launch.py
# ou perfil completo:
forest up sim-slam-nav -d
```

## Scan input

Default: `/perception/lidar/scan_ground` (Palacín). Se o mapa não construir, tentar:

```bash
forest up sim-slam-nav -d slam_scan_topic:=/sensors/lidar/scan
```

Contrato: [docs/LOCALIZATION_SLAM_ARCHITECTURE.md](../../../docs/LOCALIZATION_SLAM_ARCHITECTURE.md) § Fase 2.
