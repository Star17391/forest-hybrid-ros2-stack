#!/usr/bin/env bash
# Audita fonte da pose RViz (pião = índice dynamic_pose errado).
# Requer: forest up sim-hybrid-test -d, Gazebo PLAY, preferível AERIAL_FLY.
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros
ASSERT=false
SAMPLE=8
while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --sample) shift; SAMPLE="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-pose-audit [--assert] [--sample SEC]"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/marble_pose_from_gz"; then
  echo "ERROR: marble_pose_from_gz em falta — forest up sim-hybrid-test -d" >&2
  exit 1
fi

forest_log_section "Pose source audit (${SAMPLE}s)"
if ros2 run forest_sim_bridge hybrid_pose_source_audit --sample-sec "$SAMPLE"; then
  echo "SUCCESS: hybrid-pose-audit"
  exit 0
fi
echo "FAIL: pose_fused provavelmente ligado a hélice/índice errado" >&2
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
