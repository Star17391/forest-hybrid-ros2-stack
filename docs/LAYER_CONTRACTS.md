# Layer contracts — forest-hybrid-ros2-stack

This document freezes ROS 2 **topic names**, **message types**, and **who publishes what**. External projects (simulador Gazebo, drivers, campo) must **implement these interfaces**, not redefine them. When this stack evolves, incompatible changes get a bumped contract version.

**Machine-readable snapshot:** [`src/conf/forest_hybrid_conf/config/layer_contracts.yaml`](../src/conf/forest_hybrid_conf/config/layer_contracts.yaml).

---

## Utilities layer (`operation_mode_node`)

**Role:** Published configuration for *ground vs aerial*: perception can skip GPU work while the coaxial rotor is flying.

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/system/locomotion_mode` | `forest_hybrid_msgs/msg/OperationMode` | QoS transient local (`mode_name`: `ground` / `aerial`) |

---

## Mission manager layer (`mission_manager_node`)

**Role:** Operator/UI-facing mission **FSM**: commands, acknowledgement for certain transitions, **`PoseStamped` goal for the eventual low-level planner**, and **`MissionStatus`** for supervision.

**Subscribes**

| Topic | Type | Notes |
|--------|------|--------|
| `/mission/command` | `forest_hybrid_msgs/msg/MissionCommand` | HOLD, GOTO, PATROL, …; **`CMD_CLEAR_EMERGENCY_LATCH`** after ESTOP; optional **yaw** fields (`use_target_yaw`, `waypoint_yaw`). |
| `/mission/ack` | `forest_hybrid_msgs/msg/MissionAck` | e.g. approve `RETURN_HOME` after prompt |
| `/state/pose_fused` | `geometry_msgs/msg/PoseStamped` | Fused pose; **`header.frame_id` must be `map`** for arrival checks vs `/planning/mission_goal`. |
| `/planning/progress` | `std_msgs/msg/Float32` | 0–1 along path leg (UI / telemetry); **does not** complete waypoints. |
| `/planning/path_blocked` | `std_msgs/msg/Bool` | True when planner cannot proceed. |
| `/planning/goal_reached` | `std_msgs/msg/Bool` | Optional shortcut if parameter `allow_goal_reached_topic_shortcut` is enabled (default off). |

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/mission/status` | `forest_hybrid_msgs/msg/MissionStatus` | FSM state + text |
| `/planning/mission_goal` | `geometry_msgs/msg/PoseStamped` | Published **once** per active leg; **`map`** frame; heading in quaternion (tangent or locked at publish time). |

Mission manager **does not** publish `/system/locomotion_mode`; that stays with the utilities layer unless a future planner takes over hybrid mode transitions.

---

## Camera driver layer

**Role:** Stable RGB calibration and image stream.

**Publishes**

| Topic | Type |
|--------|------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

**Implementations**

- USB: `usb_cam` (`camera_usb.launch.py` in `forest_camera_ros2`)
- CSI: `camera_ros` (`camera_pi.launch.py`)

---

## Segmentation layer (`semantic_segmentation_node`)

**Role:** Per-pixel semantic mask for fusion / traversal heuristics (future).

**Subscribes**

| Topic | Type |
|--------|------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/system/locomotion_mode` | `forest_hybrid_msgs/msg/OperationMode` |

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/perception/semantic_mask` | `sensor_msgs/msg/Image` | `mono8`, class ID per pixel |

**Runtime rule:** If mode is **aerial**, frame processing is skipped (bypass).

---

## Sensor preprocess layer (Camada 0a)

**Subscribes:** `/scan`, `/sensors/imu/data` (optional, for scan leveling)  
**Publishes:** `/sensors/lidar/scan`, `/sensors/lidar/points`, `/sensors/wheel_odometry` (tracked fusion)

Package: `forest_sensors_cpp`. See `config/forest_lidar_preprocess.yaml`.

## State estimation layer (Camada 0b) — SE3

**Nós:** `ekf_local` (odom→base — ÚNICO EKF), `map_odom_authority_node`, `state_contract_node`, e (só `use_gnss`) `gnss_cov_adapter_node`, `navsat_transform`. O `ekf_global` foi removido (a pose no ar vem direta do ArduPilot EKF3, design §6).

**Subscribes:**
| Tópico | De | Notas |
|--------|----|-------|
| `/sensors/wheel_odometry` | drivers | ekf_local |
| `/sensors/imu/data` | drivers | ekf_local (gyro 3D; accel bloqueada) |
| `/sensors/gnss/fix` | driver GNSS | gnss_cov_adapter_node infla cov (só `use_gnss`) |
| `/ardupilot/local_position_odom` | `hybrid_hop_executor` (porta 5760) | fonte aérea da autoridade map→odom |
| `/system/locomotion_mode` | utilities | gatilho autoridade map→odom |
| `/slam/status` | Tree-SLAM | gatilho auxiliar autoridade |

**Publishes:**
| Tópico | Tipo | Notas |
|--------|------|-------|
| `/state/odometry` | `nav_msgs/Odometry` | ekf_local (`frame_id=odom`) |
| `/state/pose_fused` | `geometry_msgs/PoseStamped` | via TF map→base_link |
| `/sensors/gnss/fix_adapted` | `sensor_msgs/NavSatFix` | cov inflada ≥ 25 m² horizontal (só `use_gnss`) |

**TF:**
| TF | Autoridade | Condição |
|----|-----------|----------|
| `odom → base_link` | `ekf_local` (always) | — |
| `map → odom` | `map_odom_authority_node` (pose do ArduPilot) | `MODE_AERIAL` |
| `map → odom` | `map_odom_authority_node` (hold/identidade, `ground_mode=identity`) | `MODE_GROUND` (Fase 1) |
| `map → odom` | Tree-SLAM (`forest_tree_slam`; autoridade com `ground_mode=silent`) | `MODE_GROUND` (Fase 2+) |

**Regra de ouro:** nunca dois publishers de `map→odom` em simultâneo. A autoridade é sempre a mesma; comuta a FONTE por modo (solo: hold/identidade ou cede ao Tree-SLAM; ar: ArduPilot).

Package: `forest_state_estimation`. Configuração: `ekf_local.yaml`, `gnss_cov_adapter.yaml`, `map_odom_authority.yaml`, `navsat_transform.yaml`.

---

## Fusion mapping layer (future)

**Subscribes:** LiDAR, semantic mask, semantic points, fused pose  
**Publishes:** `/mapping/traversability_map`, `/mapping/volumetric_map`

Late-fusion intermediate topic:

| Topic | Type | Notes |
|--------|------|--------|
| `/perception/semantic_points` | `sensor_msgs/msg/PointCloud2` | LiDAR points with semantic class label per point |

For **online mapping in forest without a global map**, see [FUTURE_TILED_MAPS.md](FUTURE_TILED_MAPS.md) (tile-based local maps / stitching — backlog, not implemented in this repo yet).

---

## Planning / decision layer (future)

Consumes maps + pose + **`/planning/mission_goal`**. Publishes local path and the **navigation feedback topics** consumed by mission manager (`/planning/progress`, `/planning/path_blocked`, `/planning/goal_reached`). In **v1** of the contract, `/system/locomotion_mode` remains owned by utilities; later, this layer may also drive hybrid mode.

---

## Control layer (future)

**Subscribes:** local trajectory, fused pose, `/system/locomotion_mode`  
**Publishes:** `/actuators/motor_cmd` (`std_msgs/msg/Float32MultiArray`)

---

## LiDAR perception layer (`forest_3d_perception`)

**Role:** Extração de landmarks de troncos de **um único scan** LiDAR — stateless, sem tracking, sem IDs persistentes. Publica em `base_link`, por-frame.

**Subscribes**

| Topic | Type | Notes |
|--------|------|--------|
| `/sensors/lidar/points` | `sensor_msgs/msg/PointCloud2` | nuvem pré-processada da camada 0a |

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/perception/lidar/tree_landmarks` | `forest_hybrid_msgs/msg/TreeLandmarkArray` | `frame_id=base_link`, por-frame; geometria + incerteza (`base_covariance`, `diameter_stddev`) e **probabilidades semânticas** (`class_scores` [tronco, rocha, obstáculo]). A perceção **não decide** a classe final — emite scores; o `semantic_class` no landmark é só argmax para debug/RViz. A **decisão de classe** (fusão multi-frame, gating) fica no **Tree-SLAM** |
| `/perception/lidar/semantic_points` | `sensor_msgs/msg/PointCloud2` | nuvem com label semântico por ponto (opcional, para fusão) |

---

## SLAM / Localization layer (`forest_tree_slam`)

**Role:** Tree-SLAM por landmarks de troncos; autoridade de `map→odom` **no solo**. Desenho: [`FOREST_TREE_SLAM_DESIGN.md`](./FOREST_TREE_SLAM_DESIGN.md).

**Subscribes**

| Topic | Type | Notes |
|--------|------|--------|
| `/perception/lidar/tree_landmarks` | `forest_hybrid_msgs/msg/TreeLandmarkArray` | `frame_id=base_link`, por-frame; consome `class_scores` (não o argmax `semantic_class` de debug) para associar e fundir classe em `TrackedTreeLandmark.semantic_class` |
| `/state/odometry` | `nav_msgs/msg/Odometry` | do EKF SE3 (`forest_state_estimation`) |

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/slam/tree_map` | `forest_hybrid_msgs/msg/TrackedTreeLandmarkArray` | mapa de landmarks com `uid` imutável, pose@`map`, DBH, cov; `frame_id=map` |
| `/slam/status` | `forest_hybrid_msgs/msg/SlamStatus` | estados `GROUND`/`AERIAL`/`RELOCALIZING`/`LOST`; gatilho de comutação de autoridade TF |
| `/slam/pose_graph` | `visualization_msgs/msg/MarkerArray` | debug / visualização |

**TF: `map → odom` — autoridade comutável**

| Modo | Autoridade | Condição |
|------|-----------|----------|
| Solo (`GROUND`) | `forest_tree_slam` | `/slam/status.mode == GROUND && owns_map_to_odom == true` |
| Ar (`AERIAL`) | `forest_state_estimation` (EKF SE3) | `/slam/status.mode == AERIAL` |

Regra de ouro: **nunca dois publishers de `map→odom` em simultâneo**. A comutação é explícita: o nó que perde autoridade para de publicar o TF antes de o outro começar, usando `/slam/status.owns_map_to_odom` e `/system/locomotion_mode` como sinais. O EKF usa `two_d_mode: false` (SE3 completo) para suportar voo.

---

## Planning / navigation layer (Nav2 + ponte)

**Role:** navegação de solo (global + local). Contrato detalhado: [`PLANNING_LAYER_CONTRACT.md`](./PLANNING_LAYER_CONTRACT.md). Stack = **Nav2**: `planner_server` (plugin **D\* Lite** custom) + `controller_server` (**MPPI**) + costmap layered + `bt_navigator`, ligado ao `mission_manager` por um nó-ponte **`mission_nav2_bridge`**.

**Subscribes**

| Topic / source | Type | Notes |
|--------|------|--------|
| `/planning/mission_goal` | `geometry_msgs/msg/PoseStamped` | da missão; a ponte converte para action `nav2_msgs/NavigateToPose` (`map`) |
| traversability **costmap layer** | plugin `nav2_costmap_2d::Layer` | do mapping; **não** é um tópico solto |
| TF `map→odom→base_link` | — | localização (Nav2 não a produz) |
| `/slam/status` | `forest_hybrid_msgs/msg/SlamStatus` | modos degradados |
| `/state/odometry` | `nav_msgs/msg/Odometry` | controller |

**Publishes**

| Topic | Type | Notes |
|--------|------|--------|
| `/forest_gen/cmd_vel` | `geometry_msgs/msg/Twist` | **único produtor** (Pure Pursuit é **legacy**, desligado) |
| `/planning/progress` · `/planning/path_blocked` · `/planning/goal_reached` | `Float32` · `Bool` · `Bool` | a ponte traduz o feedback/result do Nav2; `path_blocked` dispara o salto híbrido na missão |

**Legacy:** `PurePursuitController` e o pipeline GlobalPlanner/LocalPlanner/TrajectorySampler próprios ficam **legacy** (caminho ativo = Nav2). Fallback de segurança = recovery behaviors do Nav2, não Pure Pursuit. NMPC (acados) = extensão futura.

---

## Versioning

When you rename or repurpose a topic, increment `contract_version` at the top of `layer_contracts.yaml` and update this file in the same commit.
