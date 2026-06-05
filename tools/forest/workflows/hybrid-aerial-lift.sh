#!/usr/bin/env bash
# Diagnóstico: por que o híbrido não levanta voo em AERIAL_FLY.
#
# Fase 1 — offline (SDF, eixo de empuxo, TWR, frames das hélices)
# Fase 2 — live (requer forest up sim-hybrid-test -d, Gazebo PLAY)
#   → to_aerial, amostra tópicos, verifica subida de z e tópico gz motor_speed
#
# Uso:
#   forest test hybrid-aerial-lift              # offline + live se stack up
#   forest test hybrid-aerial-lift --assert
#   forest test hybrid-aerial-lift --offline-only
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
OFFLINE_ONLY=false
LIVE_SAMPLE=12
TIMEOUT_SEC=90
while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --offline-only) OFFLINE_ONLY=true ;;
    --live-sample) shift; LIVE_SAMPLE="${1:?}" ;;
    --timeout) shift; TIMEOUT_SEC="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-aerial-lift [--assert] [--offline-only] [--live-sample SEC] [--timeout SEC]"
      echo "  offline: SDF/thrust/TWR (sem Gazebo)"
      echo "  live:    requer sim-hybrid-test + PLAY; dispara to_aerial e mede lift"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

HYBRID_WS="${FOREST_HYBRID_WS:-$(cd "${FOREST_ROOT}/../.." && pwd)}"
SDF="${FORESTGEN_PATH}/models/forest_hybrid_robot/model.sdf"

if ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
  forest_log_section "Gazebo truth probe (attach — sim já a correr)"
  TRUTH_ARGS=(--attach --duration 14)
else
  forest_log_section "Gazebo truth probe (headless)"
  TRUTH_ARGS=(--world "${FORESTGEN_PATH}/worlds/mvp_hybrid_flat.sdf" --duration 14)
fi
if ! ros2 run forest_sim_bridge hybrid_gz_truth_probe "${TRUTH_ARGS[@]}"; then
  echo "FAIL: Gazebo truth — empuxo ou z (ver thrust_z_* no log)" >&2
  if [[ "$ASSERT" == "true" ]]; then exit 1; fi
fi

forest_log_section "Offline lift diagnostic"
if ! ros2 run forest_sim_bridge hybrid_aerial_lift_diagnostic --mode offline --sdf "$SDF"; then
  echo "FAIL: offline checks (SDF / thrust / prop frames)" >&2
  if [[ "$ASSERT" == "true" ]]; then exit 1; fi
  exit 0
fi

if [[ "$OFFLINE_ONLY" == "true" ]]; then
  echo "SUCCESS: offline-only lift diagnostic passed"
  exit 0
fi

if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
  echo "SKIP live phase: hybrid_transition_manager not running" >&2
  echo "  Start: forest up sim-hybrid-test -d  (Gazebo PLAY)" >&2
  if [[ "$ASSERT" == "true" ]]; then exit 1; fi
  exit 0
fi

forest_log_section "Trigger to_aerial"
ros2 topic pub --once /forest_gen/hybrid/transition_request std_msgs/msg/String \
  "{data: to_aerial}" >/dev/null 2>&1 || true

echo "=== Wait for AERIAL_FLY (max ${TIMEOUT_SEC}s) ==="
deadline=$((SECONDS + TIMEOUT_SEC))
in_aerial=false
while (( SECONDS < deadline )); do
  line="$(timeout 2 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null || true)"
  state_name="$(echo "$line" | sed -n 's/^state_name: //p' | head -1)"
  if [[ "$state_name" == "AERIAL_FLY" || "$state_name" == "AERIAL_HOVER" ]]; then
    in_aerial=true
    echo "  FSM=$state_name — sampling ${LIVE_SAMPLE}s for lift"
    break
  fi
  if [[ "$state_name" == "FAILED" ]]; then
    echo "FAIL: FSM FAILED before aerial fly" >&2
    exit 1
  fi
  sleep 0.5
done

if [[ "$in_aerial" != "true" ]]; then
  echo "FAIL: timeout waiting for AERIAL_FLY" >&2
  if [[ "$ASSERT" == "true" ]]; then exit 1; fi
  exit 0
fi

forest_log_section "Live lift diagnostic (${LIVE_SAMPLE}s)"
if ros2 run forest_sim_bridge hybrid_aerial_lift_diagnostic \
  --mode live --live-sample-sec "$LIVE_SAMPLE"; then
  echo "SUCCESS: hybrid-aerial-lift (offline + live)"
  exit 0
fi

echo "FAIL: live lift diagnostic — ver mensagens FAIL acima" >&2
echo "  Hipóteses comuns:" >&2
echo "    1) Gazebo sem PLAY ou model.sdf antigo (hélices desligadas das lagartas)" >&2
echo "    2) gz motor_speed sem subscribers (transport / namespace)" >&2
echo "    3) Empuxo nas hélices mas base preso (esteiras no chão após ±90°)" >&2
echo "    4) TWR ~1.0 — aumentar motorConstant ou reduzir massa" >&2
if [[ "$ASSERT" == "true" ]]; then
  exit 1
fi
exit 0
