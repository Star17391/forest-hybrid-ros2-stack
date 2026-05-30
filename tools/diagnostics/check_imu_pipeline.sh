#!/usr/bin/env bash
# Verifica IMU no Gazebo (gz) e no ROS2. Correr com sim em Play.
set -eo pipefail
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[[ -f install/setup.bash ]] && source install/setup.bash

GZ_IMU="/world/unified_world/model/marble_hd2/link/base_link/sensor/imu_sensor/imu"

echo "=== Gazebo (1 amostra) ==="
if timeout 5 gz topic -e -t "$GZ_IMU" -n 1 2>&1 | head -5; then
  echo "(gz IMU OK)"
else
  echo "FALHA: sem dados em $GZ_IMU"
  echo "  → Confirma plugin gz-sim-imu-system no world SDF e Play no sim."
fi

echo ""
echo "=== ROS2 (3 s) ==="
timeout 3 ros2 topic hz /sensors/imu/data_raw -s 2>&1 || true
timeout 3 ros2 topic hz /sensors/imu/data -s 2>&1 || true
