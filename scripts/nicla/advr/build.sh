#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -e
# Avoid stale install/ symlinks when new launch files were added
rm -rf build/forest_nicla_vision_ros2 install/forest_nicla_vision_ros2
colcon build --packages-select forest_nicla_vision_ros2 nicla_vision_ros2 --symlink-install
echo ""
echo "Done. Run: source $ROOT/install/setup.bash"
