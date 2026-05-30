#!/usr/bin/env bash
# YDLidar X4 via fdpo protocol (ROS 2) — same logic as 5DPO/fdpo-ros-stack sdpo_driver_laser_2d.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
PORT="${1:-/dev/ttyUSB0}"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

if ! python3 -c "import serial" 2>/dev/null; then
  echo "Installing python3-serial..."
  sudo apt install -y python3-serial
fi

echo "fdpo protocol driver (ROS 2) port=$PORT"
echo "Check: ros2 topic hz /scan --qos-profile sensor_data"
exec ros2 launch forest_lidar_ros2 ydlidar_x4_fdpo.launch.py "serial_port:=$PORT"
