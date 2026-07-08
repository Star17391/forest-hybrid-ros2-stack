#!/usr/bin/env bash
# Valida o pipeline de captura automática de imagens para treino YOLO.
#
# Verifica: nó ativo, imagens a chegar, labels a ser escritas, taxa de captura.
#
# Pré-requisito:
#   forest up sim-vision-capture -d --world forest_realistic_v2_trees_rocks
#   Conduz o robô com uma missão de waypoints planeada (camada de missões) para
#   cobrir o mundo — o labeler captura o que a câmara vir, seja qual for o motor.
#
# Uso:
#   forest test vision-capture [DURATION_S]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST_DIAG_ROOT="${ROOT}/tools/diagnostics"
DURATION="${1:-60}"

source "$(dirname "${BASH_SOURCE[0]}")/../lib/env.bash"
forest_source_ros || exit 1

echo "=== test-vision-capture (${DURATION}s) — captura GT automática ==="
echo ""
echo "Pré-requisito:"
echo "  forest up sim-vision-capture -d --world forest_realistic_v2_trees_rocks"
echo "  + conduzir o robô com uma missão de waypoints planeada (camada de missões)"
echo ""

if ! ros2 node list 2>/dev/null | grep -q '/gz_auto_labeler'; then
  echo "ERROR: gz_auto_labeler não está activo." >&2
  echo "  forest up sim-vision-capture -d --world forest_realistic_v2_trees_rocks" >&2
  exit 1
fi

python3 "${FOREST_DIAG_ROOT}/vision_labels_audit.py" \
  --duration "${DURATION}" \
  --live
