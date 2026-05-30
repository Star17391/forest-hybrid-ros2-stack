#!/usr/bin/env bash
# Phase 4 — JPEG over serial + optional Wi-Fi probe.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
STACK_ROOT="$(forest_repo_root)"
cd "$STACK_ROOT"

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash
set -u

PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"
PROBE=(--transport serial --port "$PORT" --jpeg --snap --out /tmp/nicla_snap_jpeg.ppm)

echo "== Nicla Phase 4: JPEG on USB serial =="
echo "If PING fails: re-flash firmware (Wi-Fi is now non-blocking in setup)."
echo "Quick check: printf 'PING\\n' > /dev/ttyACM0; timeout 2 cat /dev/ttyACM0"
echo ""

if command -v nicla_device_probe >/dev/null 2>&1; then
  nicla_device_probe "${PROBE[@]}"
else
  ros2 run forest_nicla_vision_ros2 nicla_device_probe "${PROBE[@]}"
fi

echo ""
echo "ROS (JPEG default):"
echo "  ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py"
echo ""
echo "CV layer (camera + segmentation):"
echo "  ros2 launch forest_nicla_vision_ros2 nicla_vision_perception.launch.py \\"
echo "    onnx_model_path:=/path/to/model.onnx"
echo ""
echo "Wi-Fi (after wifi_secrets.h + Nicla IP):"
echo "  ros2 launch forest_nicla_vision_ros2 nicla_vision_wifi.launch.py wifi_host:=<IP>"
