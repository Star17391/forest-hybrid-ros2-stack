#!/usr/bin/env bash
# Offline: build + CSF smoke test (no Gazebo). CI-friendly.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

colcon build \
  --packages-select forest_3d_perception \
  --cmake-args -DBUILD_TESTING=ON \
  --allow-overriding forest_3d_perception

# O launch usa install/.../lidar3d_experimental_node (symlink para build/).
source install/setup.bash
echo "Binary: $(readlink -f install/forest_3d_perception/lib/forest_3d_perception/lidar3d_experimental_node 2>/dev/null || true)"

BIN="install/forest_3d_perception/lib/forest_3d_perception/csf_ground_segmentation_smoke_test"
if [[ ! -x "${BIN}" ]]; then
  BIN="$(find build/forest_3d_perception -name csf_ground_segmentation_smoke_test -type f | head -1)"
fi
if [[ -z "${BIN}" || ! -x "${BIN}" ]]; then
  echo "ERROR: csf_ground_segmentation_smoke_test not found after build" >&2
  exit 1
fi

echo "=== Running CSF smoke test: ${BIN} ==="
"${BIN}"
echo "=== colcon test (ament) ==="
colcon test --packages-select forest_3d_perception --event-handlers console_direct+
colcon test-result --test-result-base build/forest_3d_perception/test_results
