#!/usr/bin/env bash
# Captura um bag CURTO do mundo controlado trunk_range, com o sim já a correr.
#
# Grava: input CRU do LiDAR (para re-processar offline) + saídas REAIS do node
# experimental (para comparar) + TF. Robô parado -> ~6 s chegam; gravar uns
# segundos a mais dá várias realizações de ruído por árvore.
#
# Uso (com 'forest up sim-lidar3d-experimental -d --world trunk_range_dXX' a correr):
#   ./capture_range.sh d4 8       # tag d4, grava 8 segundos
set -euo pipefail

TAG="${1:?uso: capture_range.sh <tag, ex. d4> [segundos=8]}"
SECS="${2:-8}"
OUT_DIR="/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack/src/perception_stack/forest_3d_perception/tools/canopy_isolation/bags"
BAG="${OUT_DIR}/trunk_range_${TAG}"

mkdir -p "${OUT_DIR}"
rm -rf "${BAG}"

TOPICS=(
  /sensors/lidar/points                                  # input cru -> re-processar offline
  /perception/lidar/tree_clusters                        # inliers do fit por árvore (intensity=idx)
  /perception/lidar3d/experimental/trunk_fit_points      # pontos da banda do DBH (todos)
  /perception/lidar3d/experimental/clusters              # clusters do stem-band
  /perception/lidar/tree_landmarks                        # contrato: posição+DBH por landmark
  /perception/lidar/tree_landmark_markers                # markers RViz
  /perception/lidar3d/experimental/debug_stats           # stats por frame
  /tf /tf_static
)

echo "A gravar ${SECS}s -> ${BAG}"
# 'timeout' gere o processo diretamente: SIGINT ao fim de SECS, SIGKILL 3s depois se
# resistir. Robusto mesmo em background (o kill -INT a $! NÃO apanhava o record real).
timeout --signal=SIGINT --kill-after=3 "${SECS}" \
  nice -n 10 ros2 bag record -o "${BAG}" "${TOPICS[@]}" || true
echo "feito: ${BAG}"
ros2 bag info "${BAG}" 2>/dev/null | grep -E "Duration|Count|points|tree_clusters" || true
