# Layer Contracts

This document freezes the first version of layer responsibilities and ROS 2 contracts.

## Camera Driver Layer

- Inputs: hardware camera stream (USB for now; CSI later).
- Outputs:
  - `/camera/image_raw` (`sensor_msgs/msg/Image`)
  - `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)
- Implementation now:
  - USB: `usb_cam` (`camera_usb.launch.py`)
  - CSI fallback: `camera_ros` (`camera_pi.launch.py`)

## Segmentation Layer

- Inputs:
  - `/camera/image_raw` (`sensor_msgs/msg/Image`)
  - `/system/locomotion_mode` (`forest_hybrid_msgs/msg/OperationMode`)
- Outputs:
  - `/perception/semantic_mask` (`sensor_msgs/msg/Image`, `mono8`)
- Runtime rule:
  - If mode is aerial, skip frame processing.

## State Estimation Layer (future)

- Inputs:
  - `/sensors/imu/data` (`sensor_msgs/msg/Imu`)
  - `/sensors/lidar/points` (`sensor_msgs/msg/PointCloud2`)
- Outputs:
  - `/state/pose_fused` (`geometry_msgs/msg/PoseStamped`)
  - `/state/odometry` (`nav_msgs/msg/Odometry`)

## Fusion Mapping Layer (future)

- Inputs:
  - `/sensors/lidar/points`
  - `/perception/semantic_mask`
  - `/state/pose_fused`
- Outputs:
  - `/mapping/traversability_map`
  - `/mapping/volumetric_map`

## Planning Decision Layer (future)

- Inputs:
  - `/mapping/traversability_map`
  - `/mapping/volumetric_map`
  - `/state/pose_fused`
- Outputs:
  - `/planning/local_trajectory`
  - `/system/locomotion_mode`

## Control Layer (future)

- Inputs:
  - `/planning/local_trajectory`
  - `/state/pose_fused`
  - `/system/locomotion_mode`
- Outputs:
  - `/actuators/motor_cmd`
