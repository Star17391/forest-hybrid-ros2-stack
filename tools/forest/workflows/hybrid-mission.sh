#!/usr/bin/env bash
# Missão híbrida: terrestre → transição → voo → aterragem → terrestre.
# Requer: forest up sim-hybrid-test -d  (PLAY no Gazebo)
#
# Uso: forest test hybrid-mission [--assert] [--timeout SEC]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
TIMEOUT_SEC=300

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --timeout) shift; TIMEOUT_SEC="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-mission [--assert] [--timeout SEC]"
      echo "  Terrestre→(3.5,0) → voo→(3.5,3.5,1m) → terra→(7,3.5)"
      echo "  Arranca hybrid_trajectory_demo se ainda não estiver activo."
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

for n in hybrid_transition_manager marble_pose_from_gz; do
  if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/${n}"; then
    echo "ERROR: nó /${n} em falta." >&2
    echo "  Arranca: forest up sim-hybrid-test -d" >&2
    exit 1
  fi
done

if ! ros2 pkg executables forest_sim_bridge 2>/dev/null | grep -q ' hybrid_trajectory_demo$'; then
  echo "ERROR: executável hybrid_trajectory_demo em falta no install." >&2
  echo "  cd \"\$HYBRID_WS\" && colcon build --packages-select forest_sim_bridge --symlink-install" >&2
  echo "  source \"\$HYBRID_WS/install/setup.bash\" (ou novo terminal com forest)" >&2
  exit 1
fi

if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_trajectory_demo"; then
  forest_log_section "Starting hybrid_trajectory_demo"
  ros2 run forest_sim_bridge hybrid_trajectory_demo --ros-args -p auto_start:=true &
  DEMO_PID=$!
  if ! kill -0 "$DEMO_PID" 2>/dev/null; then
    echo "ERROR: hybrid_trajectory_demo failed to start" >&2
    exit 1
  fi
  for _ in $(seq 1 30); do
    if ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_trajectory_demo"; then
      break
    fi
    sleep 0.5
  done
else
  DEMO_PID=""
  echo "hybrid_trajectory_demo already running"
fi

cleanup() {
  if [[ -n "${DEMO_PID}" ]] && kill -0 "$DEMO_PID" 2>/dev/null; then
    kill "$DEMO_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

forest_log_section "Monitor hybrid trajectory (${TIMEOUT_SEC}s)"
deadline=$((SECONDS + TIMEOUT_SEC))
last_phase=""
ok=false

while (( SECONDS < deadline )); do
  if [[ -n "${DEMO_PID}" ]] && ! kill -0 "$DEMO_PID" 2>/dev/null; then
    echo "ERROR: hybrid_trajectory_demo process exited"
    exit 1
  fi
  phase="$(timeout 2 ros2 topic echo /forest_gen/hybrid/trajectory_phase --once 2>/dev/null \
    | sed -n 's/^data: //p' | head -1 || true)"
  if [[ -n "$phase" && "$phase" != "$last_phase" ]]; then
    echo "[$(date +%H:%M:%S)] phase=$phase"
    last_phase="$phase"
    if [[ "$phase" == "DONE" ]]; then
      ok=true
      break
    fi
    if [[ "$phase" == "FAILED" ]]; then
      echo "FAIL: trajectory demo FAILED"
      exit 1
    fi
  fi
  st="$(timeout 2 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null || true)"
  if [[ -n "$st" ]]; then
    sn="$(echo "$st" | sed -n 's/^state_name: //p' | head -1)"
    z="$(echo "$st" | sed -n 's/^base_z_m: //p' | head -1)"
    ab="$(echo "$st" | sed -n 's/^airborne: //p' | head -1)"
    if [[ -n "$sn" ]]; then
      echo "  FSM=$sn z=$z airborne=$ab"
    fi
    if [[ "$sn" == "FAILED" ]]; then
      echo "FAIL: FSM FAILED"
      exit 1
    fi
  fi
  sleep 1
done

if [[ "$ok" == "true" ]]; then
  echo "SUCCESS: hybrid mission complete"
  exit 0
fi

echo "INCOMPLETE: mission did not reach DONE within ${TIMEOUT_SEC}s (last_phase=${last_phase:-none})"
if [[ "$ASSERT" == "true" ]]; then
  exit 1
fi
exit 0
