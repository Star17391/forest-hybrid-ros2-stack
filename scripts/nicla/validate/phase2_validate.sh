#!/usr/bin/env bash
# Phase 2 — QVGA camera + LSM6DSOX IMU over USB serial.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
STACK_ROOT="$(forest_repo_root)"
cd "$STACK_ROOT"

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source install/setup.bash
fi
set -u

PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"
PROBE=(--port "$PORT" --imu --snap --out /tmp/nicla_snap_qvga.ppm)

echo "== Nicla Vision Phase 2 (QVGA + IMU) =="
echo "Expect firmware STATUS: res=320x240 imu=lsm6dsox"
echo ""

if command -v nicla_device_probe >/dev/null 2>&1; then
  nicla_device_probe "${PROBE[@]}"
else
  ros2 run forest_nicla_vision_ros2 nicla_device_probe "${PROBE[@]}"
fi

echo ""
echo "Snapshot: /tmp/nicla_snap_qvga.ppm  (xdg-open to view)"
echo "ROS stream: ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py"
