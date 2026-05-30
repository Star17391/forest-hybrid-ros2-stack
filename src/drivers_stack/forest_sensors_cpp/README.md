# `forest_sensors_cpp`

Camada 0 — utilitários de sensor (C++).

| Nó | Função |
|----|--------|
| `static_sensor_tf_node` | TF `base_link` → `laser` (tilt 25°, Palacín) |
| `laserscan_preprocess_node` | `/scan` → `/sensors/lidar/scan` (range + IMU leveling) |
| `laserscan_to_pointcloud2_node` | scan → `/sensors/lidar/points` |
| `tracked_wheel_odometry_node` | esteiras → `/sensors/wheel_odometry` |

Config: `config/forest_lidar_extrinsics.yaml`, `config/forest_lidar_preprocess.yaml`.

Launch completo (EKF): `ros2 launch forest_state_estimation state_estimation.launch.py`.

## RViz (X4 2D, não é 3D)

| Display | Tópico |
|---------|--------|
| **LiDAR 2D (scan processado)** | `/sensors/lidar/scan` |
| LiDAR bruto Gazebo (debug) | `/scan` |
| LiDAR 2D nuvem (opcional) | `/sensors/lidar/points` |
| Classificação | `/perception/lidar/points_labeled` |

A linha tracejada no RViz costuma ser o **eixo do frame `laser`** (TF), não o feixe. Os hits reais são o display **LaserScan** vermelho.

**`/sensors/lidar/points`** não é o eixo TF — é a mesma leitura 2D convertida em `PointCloud2` (x,y,z no frame `laser`). Opcional no RViz; o contrato principal é `/sensors/lidar/scan`.
