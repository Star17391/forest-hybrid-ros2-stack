#!/usr/bin/env bash
# Orquestra um A/B determinístico do Tree-SLAM por replay de bag.
# Lança o tree_slam_node + o capturador + toca o bag (só entradas), espera o bag
# acabar, e mata tudo. Os processos ROS são SEMPRE detached do shell
# (setsid, </dev/null, stdout→ficheiro) para não pendurar o orquestrador.
#
# Uso: slam_replay.sh <bag_dir> <world> <rate> <out.json> <tag>
set -u
BAG="${1:?bag}"; WORLD="${2:-forest_flat_trees}"; RATE="${3:-4.0}"
OUT="${4:?out.json}"; TAG="${5:-run}"
# 6º arg: overrides de params do nó, separados por ';' (ex: "use_motion_odom_sigma:=false;foo:=1")
EXTRA_PARAMS="${6:-}"
EXTRA_ARGS=()
if [ -n "$EXTRA_PARAMS" ]; then
  IFS=';' read -ra _kv <<< "$EXTRA_PARAMS"
  for p in "${_kv[@]}"; do EXTRA_ARGS+=(-p "$p"); done
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CFG="$ROOT/src/localization_mapping_stack/forest_tree_slam/config/tree_slam.yaml"
TMP="$(dirname "$OUT")"

pkill -9 -f tree_slam_node 2>/dev/null; pkill -9 -f "ros2 bag play" 2>/dev/null
sleep 1
rm -f "$OUT"

# 1) Nó do SLAM (sim_time → segue o /clock do bag).
setsid ros2 run forest_tree_slam tree_slam_node --ros-args \
  --params-file "$CFG" -p use_sim_time:=true "${EXTRA_ARGS[@]}" \
  </dev/null >"$TMP/node_$TAG.log" 2>&1 &
NODE=$!
# readiness: espera o nó aparecer
for i in $(seq 1 40); do
  ros2 node list 2>/dev/null | grep -q tree_slam_node && break
  sleep 0.5
done

# 2) Capturador/avaliador (subscreve, grava JSON a cada msg).
setsid python3 -u "$HERE/slam_capture_eval.py" --world "$WORLD" \
  --out "$OUT" --tag "$TAG" \
  </dev/null >"$TMP/cap_$TAG.log" 2>&1 &
CAP=$!
sleep 2

# 3) Toca o bag (só os tópicos a montante do SLAM). Inclui tree_clusters quando
# existir no bag — necessário para a ingestão multi-vista (DBH + posição Fase 3);
# `ros2 bag play --topics` ignora silenciosamente os que não existem no bag.
setsid ros2 bag play "$BAG" --clock --rate "$RATE" \
  --topics /perception/lidar/tree_landmarks /perception/lidar/tree_clusters \
           /state/odometry /tf /tf_static \
  </dev/null >"$TMP/bag_$TAG.log" 2>&1 &
BAG_PID=$!

# Espera o bag acabar (com guarda-fogo).
SECS=0
while kill -0 "$BAG_PID" 2>/dev/null; do
  sleep 2; SECS=$((SECS+2))
  [ "$SECS" -gt 1000 ] && { echo "guarda-fogo"; break; }
done
sleep 3  # drena últimas publicações

# 4) Mata tudo (capturador grava no finally via SIGTERM).
kill -INT "$CAP" 2>/dev/null; sleep 2
pkill -9 -f tree_slam_node 2>/dev/null
pkill -9 -f "ros2 bag play" 2>/dev/null
pkill -9 -f slam_capture_eval 2>/dev/null
sleep 1
echo "=== [$TAG] terminado após ${SECS}s ==="
[ -f "$OUT" ] && cat "$OUT" || echo "SEM resultado ($OUT)"
