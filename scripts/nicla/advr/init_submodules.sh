#!/usr/bin/env bash
# Fetch ADVR Nicla repos into submodule paths (works even if submodules are not committed yet).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
cd "$ROOT"

DRIVERS_URL="https://github.com/ADVRHumanoids/nicla_vision_drivers.git"
ROS2_URL="https://github.com/ADVRHumanoids/nicla_vision_ros2.git"
DRIVERS_PATH="third_party/nicla_vision_drivers"
ROS2_PATH="src/external/nicla_vision_ros2"

clone_if_missing() {
  local url="$1"
  local dest="$2"
  if [[ -f "$dest/arduino/main/main.ino" ]] || [[ -f "$dest/package.xml" ]]; then
    echo "OK  $dest (already present)"
    return 0
  fi
  if [[ -d "$dest" ]] && [[ -n "$(ls -A "$dest" 2>/dev/null)" ]]; then
    echo "WARN $dest exists but looks incomplete — remove it and re-run" >&2
    return 1
  fi
  mkdir -p "$(dirname "$dest")"
  echo "Cloning $url -> $dest ..."
  git clone --depth 1 "$url" "$dest"
}

if [[ ! -f .gitmodules ]]; then
  echo "ERROR: .gitmodules missing in repo root" >&2
  exit 1
fi

# Official submodule init (no-op if not registered in git yet)
git submodule sync --recursive 2>/dev/null || true
git submodule update --init --recursive 2>/dev/null || true

clone_if_missing "$DRIVERS_URL" "$DRIVERS_PATH"
clone_if_missing "$ROS2_URL" "$ROS2_PATH"

# Link manual clones into git submodule metadata when possible
if git rev-parse --git-dir >/dev/null 2>&1; then
  git submodule absorbgitdirs 2>/dev/null || true
fi

if [[ ! -f "$DRIVERS_PATH/arduino/main/main.ino" ]]; then
  echo "ERROR: $DRIVERS_PATH/arduino/main/main.ino not found after clone" >&2
  exit 1
fi
if [[ ! -f "$ROS2_PATH/package.xml" ]]; then
  echo "ERROR: $ROS2_PATH/package.xml not found after clone" >&2
  exit 1
fi

echo ""
echo "Submodules ready:"
echo "  $DRIVERS_PATH"
echo "  $ROS2_PATH"
git submodule status 2>/dev/null || true

echo ""
echo "Next:"
echo "  1) Edit config/forest_nicla_advr_config.h"
echo "  2) bash scripts/nicla/advr/apply_config.sh"
echo "  3) bash scripts/nicla/advr/upload_firmware.sh"
echo "  4) bash scripts/nicla/advr/build.sh"
