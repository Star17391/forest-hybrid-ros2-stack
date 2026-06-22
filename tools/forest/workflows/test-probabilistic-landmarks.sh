#!/usr/bin/env bash
# Validação Fase 1 — class_scores + emissão multi-classe (P-A + P-B).
#
# Pré-requisito:
#   forest up sim-lidar3d-experimental -d --world forest_gentle_trees_rocks
#   (Gazebo PLAY; o script auto-conduz se random_move disponível)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST_DIAG_ROOT="${ROOT}/tools/diagnostics"
DURATION="${1:-40}"
WORLD="${FOREST_WORLD:-forest_gentle_trees_rocks}"

# shellcheck source=../lib/env.bash
source "$(dirname "${BASH_SOURCE[0]}")/../lib/env.bash"
forest_source_ros || exit 1

echo "=== test-probabilistic-landmarks (${DURATION}s) — Fase 1 perceção ==="
echo ""
echo "Pré-requisito:"
echo "  forest up sim-lidar3d-experimental -d --world ${WORLD}"
echo ""

if ! ros2 node list 2>/dev/null | grep -q '/lidar3d_experimental_node'; then
  echo "ERROR: lidar3d_experimental_node não está activo." >&2
  exit 1
fi

# Unitários offline (rápidos, sem sim)
echo "--- unit test soft_cluster_scorer ---"
"${ROOT}/build/forest_3d_perception/soft_cluster_scorer_test"
echo "--- colcon test forest_3d_perception ---"
( cd "${ROOT}" && colcon test --packages-select forest_3d_perception --event-handlers console_direct+ )

SDF="${FORESTGEN_PATH:-${ROOT}/../../Gazebo/ForestGen}/worlds/${WORLD}.sdf"
if [[ ! -f "${SDF}" ]]; then
  SDF="${ROOT}/../../Gazebo/ForestGen/worlds/${WORLD}.sdf"
fi

echo "--- diag prob-landmarks (sim) ---"
python3 "${FOREST_DIAG_ROOT}/probabilistic_landmarks_eval.py" \
  --world "${SDF}" \
  --duration "${DURATION}"
