#!/usr/bin/env bash
# Phase 3 — camera_info, health, TF contract check (bridge must be running separately).
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

check_topic() {
  local topic=$1
  if ros2 topic list | grep -qx "$topic"; then
    echo "[OK] topic $topic"
    return 0
  fi
  echo "[FAIL] missing topic $topic (is nicla_vision.launch.py running?)"
  return 1
}

echo "== Nicla Vision Phase 3 validation =="
echo "Start in another terminal:"
echo "  ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py"
echo ""

FAIL=0
check_topic "/camera/image_raw" || FAIL=1
check_topic "/camera/camera_info" || FAIL=1
check_topic "/sensors/imu/data" || FAIL=1
check_topic "/sensors/nicla_serial/connected" || FAIL=1

if [[ "$FAIL" -eq 0 ]]; then
  echo ""
  echo "Sample camera_info width:"
  ros2 topic echo /camera/camera_info --once 2>/dev/null | grep -E '^width:|^height:|^k:' | head -5 || true
  echo ""
  echo "Health (2 s timeout):"
  timeout 2 ros2 topic echo /sensors/nicla_serial/connected --once 2>/dev/null || echo "  (no sample — restart bridge)"
fi

echo ""
if [[ "$FAIL" -eq 0 ]]; then
  echo "== Phase 3 topics OK =="
else
  echo "== Phase 3 incomplete — launch the bridge first =="
  exit 1
fi
