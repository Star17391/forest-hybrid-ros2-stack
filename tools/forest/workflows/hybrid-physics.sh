#!/usr/bin/env bash
# Auditoria offline: massa, TWR, rank Lee, avisos de tuning.
# Uso: forest test hybrid-physics [--assert]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    -h|--help)
      echo "Usage: forest test hybrid-physics [--assert]"
      echo "  Verifica model.sdf (massa ≤7 kg, motores, matriz rank-4)."
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

SDF="${FORESTGEN_PATH}/models/forest_hybrid_robot/model.sdf"
forest_log_section "hybrid_physics_audit"
if ! ros2 run forest_sim_bridge hybrid_physics_audit --sdf "$SDF" -v; then
  echo "FAIL: physics audit" >&2
  exit 1
fi

forest_log_section "pytest test_hybrid_physics"
PYTEST=""
if command -v pytest &>/dev/null; then
  PYTEST=pytest
elif python3 -m pytest --version &>/dev/null 2>&1; then
  PYTEST="python3 -m pytest"
fi
if [[ -n "$PYTEST" ]]; then
  # shellcheck disable=SC2086
  $PYTEST "${HYBRID_WS}/src/sim_bridge/forest_sim_bridge/test/test_hybrid_physics.py" -q
else
  echo "SKIP: pytest not installed"
fi

echo "SUCCESS: hybrid-physics checks passed"
exit 0
