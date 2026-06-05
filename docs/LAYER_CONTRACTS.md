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

## State estimation layer (Camada 0b)

**Subscribes:** `/sensors/imu/data`, `/sensors/wheel_odometry`, optional `/sensors/gnss/fix`  
**Publishes:** `/state/pose_fused`, `/state/odometry` (EKF via `robot_localization`)

Package: `forest_state_estimation`. GNSS: publish `NavSatFix` with realistic covariance; `navsat_transform` adjusts `map`→`odom` when fixes are good.

**TF:** `map` → `odom` (static identity or navsat), `odom` → `marble_hd2/base_link` (EKF), `base_link` → `laser` (static 25° tilt).

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

## Versioning

When you rename or repurpose a topic, increment `contract_version` at the top of `layer_contracts.yaml` and update this file in the same commit.
