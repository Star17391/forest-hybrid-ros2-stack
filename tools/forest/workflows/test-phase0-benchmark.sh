#!/usr/bin/env bash
# Fase 0: benchmark de pose + latência (stack já a correr).
# Uso:
#   forest up sim-mvp-nav -d          # ou sim-mvp-nav-imu
#   forest test phase0-benchmark --label ekf_wheel_only --duration 90
#
# Com movimento: noutro terminal, durante a recolha:
#   forest test patrol-rect
#   # ou forest teleop (com sim-teleop / painel)
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/diag.bash
source "${FOREST_ROOT}/lib/diag.bash"

LABEL="ekf_wheel_only"
DURATION="90"
RUN_LATENCY="1"
OUTPUT_DIR=""

usage() {
  cat <<EOF
forest test phase0-benchmark — recolha métricas Fase 0 (pose + latência)

Requer: forest up sim-mvp-nav -d (ou sim-mvp-nav-imu) + Gazebo PLAY + robô em movimento.

Options:
  --label NAME       Tag metrics.json (default: ekf_wheel_only)
  --duration SEC     Segundos pose benchmark (default: 90)
  --output-dir PATH  Pasta de saída (default: reports/phase0/TIMESTAMP_LABEL)
  --no-latency       Não correr ekf-latency antes do pose-benchmark
  -h, --help

Exemplo A/B completo:
  forest down
  forest up sim-mvp-nav -d
  forest test phase0-benchmark --label ekf_wheel_only --duration 90
  forest test patrol-rect    # durante os 90s, noutro terminal
  forest down
  forest up sim-mvp-nav-imu -d
  forest test phase0-benchmark --label ekf_local --duration 90
  forest diag phase0-compare \\
    reports/phase0/<run_a>/metrics.json reports/phase0/<run_b>/metrics.json
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --label) LABEL="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --no-latency) RUN_LATENCY="0"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

forest_source_ros || exit 1

if ! ros2 topic list 2>/dev/null | grep -q "/state/pose_fused"; then
  echo "ERROR: /state/pose_fused ausente. Corre: forest up sim-mvp-nav -d" >&2
  exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -q "world_tf_full"; then
  echo "WARNING: /forest_gen/gz/world_tf_full não visível — benchmark pode falhar" >&2
fi

forest_log_section "À espera de ground truth Gazebo (world_tf)…"
waited=0
while (( waited < 20 )); do
  if ros2 topic list 2>/dev/null | grep -q "/forest_gen/gz/world_tf"; then
    if timeout 3 ros2 topic echo /forest_gen/gz/world_tf_full --once >/dev/null 2>&1 \
      || timeout 3 ros2 topic echo /forest_gen/gz/world_tf --once >/dev/null 2>&1; then
      echo "world_tf OK (${waited}s)"
      break
    fi
  fi
  sleep 1
  ((waited++)) || true
done
if (( waited >= 20 )); then
  echo "WARNING: world_tf ainda não recebido — benchmark pode falhar (Gazebo PLAY + modelo visível)" >&2
fi

forest_log_section "Fase 0 benchmark — label=${LABEL} duration=${DURATION}s"
echo "Durante os ${DURATION}s, mantém o robô em movimento (patrol, teleop ou cmd_vel)."

POSE_ARGS=(--label "$LABEL" --duration "$DURATION")
[[ -n "$OUTPUT_DIR" ]] && POSE_ARGS+=(--output-dir "$OUTPUT_DIR")

if [[ "$RUN_LATENCY" == "1" ]]; then
  forest_log_section "ekf-latency (30s)"
  python3 "${FOREST_DIAG_ROOT}/ekf_latency_analyzer.py" --duration 30 || true
fi

forest_log_section "pose-benchmark (${DURATION}s)"
python3 "${FOREST_DIAG_ROOT}/pose_benchmark.py" "${POSE_ARGS[@]}"
rc=$?

if [[ $rc -eq 0 ]]; then
  echo ""
  echo "OK. Ver metrics.json e trajectories.png em reports/phase0/"
fi
exit "$rc"
