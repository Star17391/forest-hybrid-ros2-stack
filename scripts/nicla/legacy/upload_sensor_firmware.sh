#!/usr/bin/env bash
# Build and upload forest nicla_sensor_node firmware.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
SKETCH="${ROOT}/src/drivers_stack/forest_nicla_vision_ros2/firmware/nicla_sensor_node"
FQBN="arduino:mbed_nicla:nicla_vision"
PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "ERROR: arduino-cli not installed" >&2
  exit 1
fi

echo "Sketch: ${SKETCH}"
echo "Port:   ${PORT}"
arduino-cli compile -b "${FQBN}" "${SKETCH}"
arduino-cli upload -b "${FQBN}" -p "${PORT}" "${SKETCH}"

echo ""
echo "After upload, verify serial shows INFO jpeg_encoder=rgb888_lazy"
echo "and STATUS contains: enc=rgb888"
echo "  bash scripts/nicla/legacy/serial_ping.sh"
echo "  ros2 run forest_nicla_vision_ros2 nicla_device_probe --transport serial --port ${PORT}"
