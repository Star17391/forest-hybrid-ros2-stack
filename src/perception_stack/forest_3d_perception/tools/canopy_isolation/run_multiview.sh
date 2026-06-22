#!/usr/bin/env bash
# Teste MULTI-VISTA da referência de tronco (Tree-SLAM): lança o perfil sim-tree-slam
# (perceção + EKF + Tree-SLAM + RViz) num mundo de 1 árvore isolada, ESPERA ficar pronto,
# e depois MOVE o robô enquanto grava — para se VER no RViz a referência (pontos
# acumulados por landmark, ns tree_slam_landmark_points) a crescer e o DBH a convergir.
#
# RViz FICA ABERTO (não headless) para observação. O sim fica de pé no fim.
#
# Movimento = ÓRBITA à volta da árvore (raio=distância), mantendo-a SEMPRE AO LADO
# (nunca de costas — senão o LiDAR inclinado perde o tronco). Varre `sweep_deg` graus.
#
# Uso:  ./run_multiview.sh <tid 1..6> <dist> [sweep_deg=180] [segundos=50] [v=0.5]
#   ex: ./run_multiview.sh 1 4 180      # Tree1 a 4m, meia-volta à árvore
set -uo pipefail

ROOT=/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
F="$ROOT/tools/forest/bin/forest"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TID="${1:?uso: run_multiview.sh <tid> <dist> [sweep_deg] [secs] [v]}"
D="${2:?falta a distância}"; SWEEP="${3:-180}"; SECS="${4:-50}"; V="${5:-0.5}"
world="trunk_one_t${TID}_d${D}"; tag="t${TID}_d${D}_mv"

# garante que o mundo existe
python3 "$HERE/make_range_world.py" "$TID" "$D" >/dev/null 2>&1

echo "== down limpo =="; "$F" down --force >/dev/null 2>&1 || true; sleep 4
echo "== up sim-tree-slam (RViz ON) world=$world =="
"$F" up sim-tree-slam -d --world "$world" >/dev/null 2>&1 || true

echo "== à espera do tree_slam_node + LiDAR =="
ready=0
for i in $(seq 1 60); do
  if timeout 4 ros2 topic echo --once /slam/status >/dev/null 2>&1 \
     && timeout 4 ros2 topic echo --once /sensors/lidar/points >/dev/null 2>&1; then
    ready=1; break
  fi
  sleep 2
done
[ "$ready" -eq 1 ] || { echo "!! TIMEOUT: SLAM/LiDAR não ficaram prontos"; exit 1; }
echo "== pronto; estabiliza 4s, depois move + grava ${SECS}s =="
sleep 4

# grava em background, orbita em primeiro plano (a órbita auto-termina ao varrer SWEEP)
bash "$HERE/capture_multiview.sh" "$tag" "$SECS" &
CAP=$!
sleep 2
python3 "$HERE/orbit_tree.py" --tree-x "$D" --tree-y 0 --radius "$D" \
  --sweep-deg "$SWEEP" --step-deg 30 --dwell 4 --v "$V" 2>&1 | grep -vE "warning|deprecat" || true
wait "$CAP" 2>/dev/null || true

echo "== FIM. Bag: $HERE/bags/mv_${tag}. Sim CONTINUA de pé (RViz). 'forest down --force' p/ fechar. =="
