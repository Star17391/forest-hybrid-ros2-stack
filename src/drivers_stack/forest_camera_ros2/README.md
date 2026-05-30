# `forest_camera_ros2`

Drivers de câmara para o contrato **`/camera/image_raw`** e **`/camera/camera_info`**.

- `launch/camera_usb.launch.py` — `usb_cam` (baseline desktop / USB)
- `launch/camera_pi.launch.py` — `camera_ros` / libcamera (CSI na Pi)

Integração simulação: remaps a partir do bridge Gazebo → estes tópicos. Ver [docs/LAYER_CONTRACTS.md](../../../docs/LAYER_CONTRACTS.md).
