#!/usr/bin/env bash
# Official YDLidar launch + X4.yaml (no forest wrapper) — use to compare with forest launch.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
PORT="${1:-/dev/ttyUSB0}"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

if ! ros2 pkg prefix ydlidar_ros2_driver &>/dev/null; then
  echo "Run: bash scripts/lidar/install_driver.sh"
  exit 1
fi

X4_YAML="$(ros2 pkg prefix ydlidar_ros2_driver)/share/ydlidar_ros2_driver/params/X4.yaml"
echo "Upstream driver node — params: $X4_YAML port=$PORT"
echo "Verify: ros2 topic hz /scan --qos-profile sensor_data"
exec ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  --params-file "$X4_YAML" \
  -p port:="$PORT" \
  -p frame_id:=laser
