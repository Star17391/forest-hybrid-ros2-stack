#!/usr/bin/env bash
# Aceitação pipeline LiDAR 3D experimental (CSF + clustering).
# Pré-requisitos: forest up sim-lidar3d-experimental -d (ou csf-only), Gazebo PLAY ~15s
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST_DIAG_ROOT="${ROOT}/tools/diagnostics"
DURATION="${1:-25}"

# shellcheck source=../lib/env.bash
source "$(dirname "${BASH_SOURCE[0]}")/../lib/env.bash"
forest_source_ros || exit 1

echo "=== test-lidar3d-experimental (${DURATION}s) ==="
echo "Requer: forest up sim-lidar3d-experimental -d  (Gazebo PLAY)"

if ! ros2 node list 2>/dev/null | grep -q '/lidar3d_experimental_node'; then
  echo "ERROR: lidar3d_experimental_node não está activo." >&2
  echo "  forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks" >&2
  exit 1
fi

python3 "${FOREST_DIAG_ROOT}/lidar3d_experimental_pipeline_audit.py" --duration "${DURATION}"
