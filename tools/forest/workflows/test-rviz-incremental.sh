#!/usr/bin/env bash
# Incremental RViz bisection — run with stack already up (forest up -d) + Gazebo PLAY.
set -euo pipefail

HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
# shellcheck source=../lib/env.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/lib/env.bash"
forest_source_ros

CONF_DIR="${HYBRID_WS}/install/forest_hybrid_conf/share/forest_hybrid_conf/config"
if [[ ! -d "$CONF_DIR" ]]; then
  CONF_DIR="${HYBRID_WS}/src/conf/forest_hybrid_conf/config"
fi

wait_clock() {
  local n=0
  while (( n < 60 )); do
    if timeout 2 ros2 topic echo /clock --once >/dev/null 2>&1; then
      echo "  /clock OK"
      return 0
    fi
    sleep 1
    ((n++)) || true
  done
  echo "  WARNING: /clock not seen in 60s" >&2
  return 1
}

run_phase() {
  local name="$1"
  local cfg="${2:-}"
  echo ""
  echo "=== Phase: $name ==="
  wait_clock || true
  if [[ -z "$cfg" ]]; then
    echo "  rviz2 (empty — add displays manually)"
    timeout 45 rviz2 || echo "  exit=$?"
  else
    echo "  rviz2 -d $cfg"
    python3 "${HYBRID_WS}/tools/diagnostics/rviz_config_validate.py" "$cfg" || true
    timeout 45 rviz2 -d "$cfg" || echo "  exit=$?"
  fi
  echo "  Check terminal for 'failed to create drawable' or 'process has died'"
}

echo "Prerequisite: forest up sim-mvp-nav-imu -d running, Gazebo PLAY"
echo "Config dir: $CONF_DIR"
run_phase "0-probe" ""
run_phase "1-minimal" "${CONF_DIR}/forest_mvp_minimal.rviz"
run_phase "2-sensors" "${CONF_DIR}/forest_mvp_sensors.rviz"
run_phase "3-full" "${CONF_DIR}/forest_mvp_sim.rviz"
echo ""
echo "Record which phase first shows drawable failures → see docs/reports/RVIZ_STABILITY_ROOT_CAUSE.md"
