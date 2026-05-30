#!/usr/bin/env bash
# Verifica 3D: nuvem Gazebo + classificação Palacín em pontos próximos.
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
forest_source_ros || exit 1

DURATION="${1:-30}"
echo "=== test-lidar3d-classify (${DURATION}s) ==="
echo "Requer: forest up sim-lidar3d-test -d  (ou --lidar3d), Gazebo PLAY"
python3 "${FOREST_DIAG_ROOT}/lidar3d_pipeline_audit.py" --duration "${DURATION}"
python3 "${FOREST_DIAG_ROOT}/lidar_classify_audit.py" --duration "${DURATION}"
