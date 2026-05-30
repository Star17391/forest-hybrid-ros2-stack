#!/usr/bin/env bash
# Aceitação Fase 0 — pipeline LiDAR 3D estável (TF + tópicos).
# Pré-requisitos: forest up sim-lidar3d-test -d --lidar3d, Gazebo PLAY ~15s
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST_DIAG_ROOT="${ROOT}/tools/diagnostics"
DURATION="${1:-45}"

# shellcheck source=../lib/env.bash
source "$(dirname "${BASH_SOURCE[0]}")/../lib/env.bash"
forest_source_ros || exit 1

echo "=== test-lidar3d-stack (${DURATION}s) ==="
echo "Requer: forest up sim-lidar3d-test -d --lidar3d, Gazebo PLAY"
python3 "${FOREST_DIAG_ROOT}/lidar3d_stack_monitor.py" --duration "${DURATION}"
