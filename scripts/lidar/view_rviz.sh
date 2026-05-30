#!/usr/bin/env bash
# Open RViz with Best Effort QoS for /scan and /sensors/lidar/points (YDLidar drivers).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
RVIZ_CFG="$ROOT/install/forest_lidar_ros2/share/forest_lidar_ros2/config/forest_lidar.rviz"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

if [[ ! -f "$RVIZ_CFG" ]]; then
  RVIZ_CFG="$ROOT/src/drivers_stack/forest_lidar_ros2/config/forest_lidar.rviz"
fi

echo "RViz config: $RVIZ_CFG"
echo "Fixed Frame: laser — run test_lidar_fdpo_ros2.sh in another terminal first."
exec rviz2 -d "$RVIZ_CFG"
