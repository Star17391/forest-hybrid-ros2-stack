#!/usr/bin/env bash
# Verifica pipeline 2D: /scan e /sensors/lidar/scan com feixes válidos.
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
forest_source_ros || exit 1

DURATION="${1:-25}"
echo "=== test-lidar2d-pipeline (${DURATION}s) ==="
echo "Requer: forest up <sim-profile> --lidar2d (Gazebo PLAY)"
python3 "${FOREST_DIAG_ROOT}/lidar_pipeline_audit.py" --duration "${DURATION}"
