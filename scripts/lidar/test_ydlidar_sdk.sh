#!/usr/bin/env bash
# Quick YDLidar X4 validation (forest stack + fdpo-compatible serial settings).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
PORT="${1:-/dev/ttyUSB0}"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi
set -u

if ! ros2 pkg prefix ydlidar_ros2_driver &>/dev/null; then
  echo "ydlidar_ros2_driver not in workspace — installing from source..."
  bash "$ROOT/scripts/lidar/install_driver.sh"
  set +u
  source "$ROOT/install/setup.bash"
  set -u
fi

if [[ ! -e "$PORT" ]]; then
  echo "WARNING: $PORT not found. Pass device: $0 /dev/ttyUSB1"
  ls /dev/ttyUSB* 2>/dev/null || true
fi

echo "Launching YDLidar SDK test (port=$PORT)."
echo "If checksum/timeout errors: bash scripts/lidar/test_fdpo_ros2.sh"
exec ros2 launch forest_lidar_ros2 ydlidar_x4_test.launch.py "serial_port:=$PORT"
