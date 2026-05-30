# `forest_state_estimation`

**Camada 0** — estimação de estado multi-sensor para floresta (derrapagem, ruído, GNSS intermitente).

## Pipeline

```text
/scan ──► laserscan_preprocess ──► /sensors/lidar/scan
/sensors/imu/data ────────────────┘ (leveling opcional)

left/right track odom ──► tracked_wheel_odometry ──► /sensors/wheel_odometry
/sensors/imu/data ───────────────────────────────► robot_localization EKF
/sensors/gnss/fix (opcional) ──► navsat_transform ──► /odometry/gps ──► EKF

EKF ──► TF odom→base_link, /state/odometry
TF map→base_link ──► state_contract_node ──► /state/pose_fused
```

## Dependência

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

## Launch

```bash
# Sim (rodas Gazebo + IMU + map=odom identidade):
ros2 launch forest_state_estimation state_estimation.launch.py use_sim_time:=true use_wheel_odom:=true

# Com GNSS (publicar sensor_msgs/NavSatFix em /sensors/gnss/fix):
ros2 launch forest_state_estimation state_estimation.launch.py use_gnss:=true publish_map_odom_identity:=false
```

Ajustar `config/navsat_transform.yaml` (`datum`) ao local de teste.

## GNSS pouco fiável

- O driver GNSS deve publicar **covariância** realista em `NavSatFix.position_covariance`.
- Com HDOP alto / poucos satélites, aumentar covariância → o EKF confia menos no GPS.
- `navsat_transform` publica `map`→`odom`; quando o GPS falha, a deriva fica no ramo odom (rodas+IMU).

## Arquitectura e roadmap

Ver [docs/LOCALIZATION_SLAM_ARCHITECTURE.md](../../../docs/LOCALIZATION_SLAM_ARCHITECTURE.md) — camadas, EKF vs localizer, fases 0–5, métricas.

**Próximo passo:** Fase 0 (baseline métrico EKF) antes de SLAM. Scan matching (MHD) como `odom2` opcional na Fase 2b — ver também [docs/FOREST_SLAM_BIBLIOGRAPHY.md](../../../docs/FOREST_SLAM_BIBLIOGRAPHY.md).
