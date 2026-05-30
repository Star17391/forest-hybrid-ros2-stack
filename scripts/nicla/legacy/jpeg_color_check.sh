#!/usr/bin/env bash
# Compare RGB565 vs JPEG color from Nicla in one Wi-Fi session.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "ERROR: run 'colcon build --packages-select forest_nicla_vision_ros2' first." >&2
  exit 1
fi
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi
set +u
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"
set -e

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not in PATH after sourcing install/setup.bash" >&2
  exit 1
fi

HOST="${1:-192.168.1.235}"
TRANSPORT="${2:-wifi}"
PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"

extra=()
if [[ "${TRANSPORT}" == wifi ]]; then
  extra=(--transport wifi --wifi-host "${HOST}")
else
  extra=(--transport serial --port "${PORT}")
fi

echo "=== RGB565 + JPEG color check (single TCP session) ==="
ros2 run forest_nicla_vision_ros2 nicla_device_probe \
  "${extra[@]}" \
  --color-check \
  --out /tmp/nicla_color_rgb565.ppm \
  --out-jpeg /tmp/nicla_color_jpeg.ppm

echo ""
echo "Done. Open:"
echo "  /tmp/nicla_color_rgb565.ppm"
echo "  /tmp/nicla_color_jpeg.ppm  (+ /tmp/nicla_color_jpeg.jpg if device JPEG)"
echo ""
echo "STATUS must contain enc=rgb888. After firmware fix, re-upload:"
echo "  bash scripts/nicla/legacy/upload_sensor_firmware.sh"
