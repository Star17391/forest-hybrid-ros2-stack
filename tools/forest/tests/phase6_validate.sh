#!/usr/bin/env bash
# Phase 6 — legacy wrappers removed; forest CLI is sole entrypoint.
set -euo pipefail

HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
export PATH="${HYBRID_WS}/tools/forest/bin:${PATH}"
export HYBRID_WS

pass=0
fail=0

assert() {
  local name="$1"
  shift
  echo ""
  echo ">>> TEST: $name"
  if "$@"; then
    echo "    PASS: $name"
    pass=$((pass + 1))
  else
    echo "    FAIL: $name" >&2
    fail=$((fail + 1))
    return 1
  fi
}

echo "=== Forest Phase 6 validation ==="

_removed() {
  local path="$1"
  [[ ! -e "$path" ]]
}

for s in \
  "$HYBRID_WS/scripts/run_trajectory_following.sh" \
  "$HYBRID_WS/scripts/run_pose_bridge_mission.sh" \
  "$HYBRID_WS/scripts/run_sensors_validation.sh" \
  "$HYBRID_WS/scripts/test_patrol_waypoints.sh" \
  "$HYBRID_WS/scripts/kill_forest_stack.sh" \
  "$HYBRID_WS/scripts/navigation/run_trajectory_following.sh" \
  "$HYBRID_WS/scripts/navigation/run_pose_bridge_mission.sh" \
  "$HYBRID_WS/scripts/navigation/run_mvp_test.sh" \
  "$HYBRID_WS/scripts/navigation/test_patrol_waypoints.sh" \
  "$HYBRID_WS/scripts/stack/run_sensors_validation.sh" \
  "$HYBRID_WS/scripts/diagnostics/analyze_imu_stream.py" \
  "$HYBRID_WS/tools/forest/compat/run_trajectory_following.sh"; do
  assert "removed: $(basename "$s")" _removed "$s"
done

assert "forest help documents completion refresh" bash -c 'forest help 2>&1 | grep -q sync_completions'

echo ""
echo "=== Results: ${pass} passed, ${fail} failed ==="
(( fail == 0 )) || exit 1
echo "Phase 6 structural validation OK"
