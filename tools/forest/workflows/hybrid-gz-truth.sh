#!/usr/bin/env bash
# Medição Gazebo pura: poses de hélices, eixo de empuxo, subida de z (sem ROS).
# Uso: forest test hybrid-gz-truth [--assert]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros
ASSERT=false
DURATION=16
while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --duration) shift; DURATION="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-gz-truth [--assert] [--duration SEC]"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

WORLD="${FORESTGEN_PATH}/worlds/mvp_hybrid_flat.sdf"
forest_log_section "Gazebo truth probe (${DURATION}s)"
ARGS=(--world "$WORLD" --duration "$DURATION")
if ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
  ARGS=(--attach --duration "$DURATION")
fi
if ros2 run forest_sim_bridge hybrid_gz_truth_probe "${ARGS[@]}"; then
  echo "SUCCESS: hybrid-gz-truth"
  exit 0
fi
echo "FAIL: hybrid-gz-truth — ver thrust_z_* e model_z_rises" >&2
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
