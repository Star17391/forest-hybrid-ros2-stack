#!/usr/bin/env bash
# YDLidar ROS 2 driver is NOT in ros-jazzy apt — build SDK + driver from source.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
DRIVER_DIR="$ROOT/src/external/ydlidar_ros2_driver"
SDK_DIR="${YDLIDAR_SDK_DIR:-$ROOT/build/_deps/YDLidar-SDK}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  ROS_SETUP="/opt/ros/humble/setup.bash"
fi

echo "==> forest-hybrid: install YDLidar-SDK + ydlidar_ros2_driver (branch humble → Jazzy)"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found. Set ROS_SETUP=/path/to/setup.bash" >&2
  exit 1
fi

# --- 1) System build tools ---
if ! command -v cmake >/dev/null; then
  echo "Installing cmake / build-essential (sudo)..."
  sudo apt update
  sudo apt install -y cmake build-essential git libudev-dev python3-colcon-common-extensions
fi

# --- 2) YDLidar-SDK (required by driver CMake) ---
sdk_installed() {
  [[ -f /usr/local/include/ydlidar_sdk.h ]] \
    || [[ -f /usr/include/ydlidar_sdk.h ]] \
    || ldconfig -p 2>/dev/null | grep -q ydlidar
}

if ! sdk_installed; then
  echo "==> Building YDLidar-SDK..."
  rm -rf "$SDK_DIR"
  git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
  cmake -S "$SDK_DIR" -B "$SDK_DIR/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$SDK_DIR/build" -j"$(nproc)"
  sudo cmake --install "$SDK_DIR/build"
  sudo ldconfig 2>/dev/null || true
  echo "YDLidar-SDK installed to /usr/local"
else
  echo "OK  YDLidar-SDK already present"
fi

# --- 3) ROS 2 driver package ---
if [[ ! -f "$DRIVER_DIR/package.xml" ]]; then
  echo "==> Cloning ydlidar_ros2_driver (humble branch)..."
  mkdir -p "$(dirname "$DRIVER_DIR")"
  git clone --depth 1 -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$DRIVER_DIR"
else
  echo "OK  $DRIVER_DIR"
fi

# --- 4) colcon build ---
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u
cd "$ROOT"
colcon build --packages-select ydlidar_ros2_driver forest_lidar_ros2 --symlink-install

echo ""
echo "Done. Next:"
echo "  source $ROOT/install/setup.bash"
echo "  sudo usermod -aG dialout \$USER   # re-login if first time"
echo "  bash scripts/lidar/test_ydlidar_sdk.sh"
