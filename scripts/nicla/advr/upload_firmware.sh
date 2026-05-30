#!/usr/bin/env bash
# Build and upload ADVR Nicla firmware (after apply_config).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
SKETCH="$ROOT/third_party/nicla_vision_drivers/arduino/main"
FQBN="arduino:mbed_nicla:nicla_vision"
PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"

if [[ ! -f "$SKETCH/config.h" ]]; then
  echo "ERROR: $SKETCH/config.h missing — run: bash scripts/nicla/advr/apply_config.sh" >&2
  exit 1
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "ERROR: arduino-cli not installed" >&2
  exit 1
fi

echo "Sketch: $SKETCH"
echo "Port:   $PORT"
arduino-cli core update-index >/dev/null 2>&1 || true
arduino-cli core install arduino:mbed_nicla >/dev/null 2>&1 || true
# ADVR firmware uses Arduino_LSM6DSOX (not STM32duino LSM6DSOX from forest legacy sketch)
arduino-cli lib install "Arduino_LSM6DSOX" "JPEGENC" "VL53L1X" >/dev/null
arduino-cli lib list | grep -E 'Arduino_LSM6DSOX|JPEGENC|VL53L1X' || true
arduino-cli compile -b "$FQBN" "$SKETCH"
arduino-cli upload -b "$FQBN" -p "$PORT" "$SKETCH"
echo "Firmware uploaded. Power-cycle or reset; LED off = streaming."
