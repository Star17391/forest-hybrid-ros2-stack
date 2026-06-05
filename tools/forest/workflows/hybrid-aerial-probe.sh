#!/usr/bin/env bash
# Sim headless: multicopter enable + vz suave; falha se roll/pitch > limite.
# Uso: forest test hybrid-aerial-probe [--assert] [--duration SEC]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
DURATION=14
while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --duration) shift; DURATION="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-aerial-probe [--assert] [--duration SEC]"
      echo "  Requer gz sim; não precisa de forest up."
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

WORLD="${FORESTGEN_PATH}/worlds/mvp_hybrid_flat.sdf"
forest_log_section "hybrid_aerial_gz_probe (${DURATION}s)"
if ros2 run forest_sim_bridge hybrid_aerial_gz_probe --world "$WORLD" --duration "$DURATION"; then
  echo "SUCCESS: aerial probe stable"
  exit 0
fi

echo "FAIL: aerial probe (tumble or no pose samples)" >&2
echo "  Tip: confirma GZ_SIM_RESOURCE_PATH e que nenhum outro gz sim está a correr." >&2
if [[ "$ASSERT" == "true" ]]; then
  exit 1
fi
exit 0
