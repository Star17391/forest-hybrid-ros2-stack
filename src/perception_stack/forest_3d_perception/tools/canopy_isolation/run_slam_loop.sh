#!/usr/bin/env bash
# Validação end-to-end do Tree-SLAM num mundo MULTI-ÁRVORE com trajetória em LAÇO.
# Sobe sim-tree-slam (RViz ON), espera ficar pronto, grava os tópicos do SLAM e
# conduz o robô num quadrado que volta à origem (revisita = teste de loop closure).
#
# Uso: run_slam_loop.sh [world=forest_flat_trees] [side=8] [v=0.5]
set -uo pipefail
ROOT=/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
F="$ROOT/tools/forest/bin/forest"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-forest_flat_trees}"; SIDE="${2:-8}"; V="${3:-0.5}"
tag="loop_${WORLD}"
BAG="$HERE/bags/slam_${tag}"

echo "== down limpo =="; "$F" down --force >/dev/null 2>&1 || true; sleep 5
echo "== up sim-tree-slam world=$WORLD =="
"$F" up sim-tree-slam -d --world "$WORLD" >/dev/null 2>&1 || true

echo "== à espera do tree_slam + LiDAR =="
ready=0
for i in $(seq 1 60); do
  if timeout 4 ros2 topic echo --once /slam/status >/dev/null 2>&1 \
     && timeout 4 ros2 topic echo --once /sensors/lidar/points >/dev/null 2>&1; then
    ready=1; break
  fi
  sleep 2
done
[ "$ready" -eq 1 ] || { echo "!! TIMEOUT: SLAM/LiDAR não prontos"; exit 1; }
echo "== pronto; estabiliza 4s =="; sleep 4

rm -rf "$BAG"
ros2 bag record -s mcap -o "$BAG" \
  /slam/status /slam/tree_map /slam/pose_graph \
  /perception/lidar/tree_landmarks /state/pose_fused /state/odometry /tf /tf_static \
  >/dev/null 2>&1 &
REC=$!
sleep 2
python3 "$HERE/slam_loop_drive.py" --side "$SIDE" --v "$V" 2>&1 | grep -vE "warning|deprecat" || true
sleep 2
kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null || true
echo "== FIM. Bag: $BAG. Sim de pé (RViz). 'forest down --force' p/ fechar. =="
