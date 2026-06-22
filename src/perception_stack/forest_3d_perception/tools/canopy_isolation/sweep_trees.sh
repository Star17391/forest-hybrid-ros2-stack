#!/usr/bin/env bash
# Varrimento ÚNICO: para cada espécie x distância, lança o sim controlado
# (1 árvore isolada, terreno plano, robô parado) e grava um bag curto.
#
# HEADLESS (sem RViz) -> leve e seguro para o PC durante o batch. Depois podes
# relançar qualquer (árvore,dist) com GUI:  forest up sim-lidar3d-experimental -d --world trunk_one_t<t>_d<d>
#
# NÃO sobrepõe down+up (colisão mata o bridge): down --force, espera, up, espera pronto, grava, down.
#
# Uso:  ./sweep_trees.sh                # todas as árvores em 4/8/12 m
#       ./sweep_trees.sh "1 3 5" "4 8"  # subconjunto
set -uo pipefail

ROOT=/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
F="$ROOT/tools/forest/bin/forest"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$HERE/out/sweep_$(date +%H%M%S).log"
mkdir -p "$HERE/out"

read -r -a TREES <<< "${1:-1 2 3 4 5 6}"
read -r -a DISTS <<< "${2:-4 8 12}"
SECS="${3:-8}"

LIDAR_TOPIC=/sensors/lidar/points

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

wait_ready() {  # espera o LiDAR (PointCloud2 std) a publicar -> sim+bridge prontos
  local i
  for i in $(seq 1 45); do
    if timeout 4 ros2 topic echo --once "$LIDAR_TOPIC" >/dev/null 2>&1; then return 0; fi
    sleep 2
  done
  return 1
}

log "INÍCIO sweep  trees=[${TREES[*]}]  dists=[${DISTS[*]}]  secs=$SECS"
for t in "${TREES[@]}"; do
  # gera os mundos desta espécie (todas as distâncias de uma vez)
  python3 "$HERE/make_range_world.py" "$t" "${DISTS[@]}" >>"$LOG" 2>&1
done

for t in "${TREES[@]}"; do
  for d in "${DISTS[@]}"; do
    world="trunk_one_t${t}_d${d}"; tag="t${t}_d${d}"
    log "===== $tag (world=$world) ====="
    "$F" down --force >>"$LOG" 2>&1 || true
    sleep 4
    log "  up (headless)…"
    "$F" up sim-lidar3d-experimental --headless --world "$world" >>"$LOG" 2>&1 || true
    if wait_ready; then
      log "  sim pronto; estabiliza 4s e grava ${SECS}s"
      sleep 4
      bash "$HERE/capture_range.sh" "$tag" "$SECS" >>"$LOG" 2>&1 || true
      log "  gravado bag trunk_range_${tag}"
    else
      log "  !! TIMEOUT à espera do LiDAR — $tag sem bag"
    fi
    "$F" down --force >>"$LOG" 2>&1 || true
    sleep 3
  done
done
log "FIM sweep. Bags em $HERE/bags/  | log: $LOG"
