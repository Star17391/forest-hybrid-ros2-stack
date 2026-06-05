#!/usr/bin/env bash
# Transição terrestre → aéreo no forest_hybrid_robot (Gazebo).
# Requer: forest up sim-hybrid-test -d  (PLAY no Gazebo se paused)
#
# Uso: forest test hybrid-transition [--assert] [--timeout SEC]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
TIMEOUT_SEC=120
REQUIRED_STATES=(
  GROUND_DRIVE
  TRANSITION_LOCK
  LEGS_EXTENDING
  TRACKS_ROTATING
  AERIAL_READY
  AERIAL_FLY
)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --timeout) shift; TIMEOUT_SEC="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-transition [--assert] [--timeout SEC]"
      echo "  Verifica sequência de estados até AERIAL_FLY/AERIAL_HOVER e airborne."
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
  echo "ERROR: nó /hybrid_transition_manager em falta." >&2
  echo "  Arranca: forest up sim-hybrid-test -d" >&2
  exit 1
fi

declare -A SEEN=()
for s in "${REQUIRED_STATES[@]}"; do SEEN["$s"]=0; done

forest_log_section "Hybrid transition to_aerial"
ros2 topic pub --once /forest_gen/hybrid/transition_request std_msgs/msg/String \
  "{data: to_aerial}"

echo "=== Monitor /forest_gen/hybrid/transition_status (${TIMEOUT_SEC}s) ==="
deadline=$((SECONDS + TIMEOUT_SEC))
final_ok=false

while (( SECONDS < deadline )); do
  line="$(timeout 2 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null || true)"
  if [[ -z "$line" ]]; then
    sleep 0.5
    continue
  fi
  state_name="$(echo "$line" | sed -n 's/^state_name: //p' | head -1)"
  airborne="$(echo "$line" | sed -n 's/^airborne: //p' | head -1)"
  base_z="$(echo "$line" | sed -n 's/^base_z_m: //p' | head -1)"
  left_yaw="$(echo "$line" | sed -n 's/^left_track_yaw_rad: //p' | head -1)"
  right_yaw="$(echo "$line" | sed -n 's/^right_track_yaw_rad: //p' | head -1)"
  leg_ext="$(echo "$line" | sed -n 's/^leg_extension_m: //p' | head -1)"
  if [[ -n "$state_name" ]]; then
    SEEN["$state_name"]=1
    echo "[$(date +%H:%M:%S)] state=$state_name airborne=$airborne z=$base_z legs=$leg_ext L=$left_yaw R=$right_yaw"
    if [[ "$state_name" == "AERIAL_HOVER" ]] || [[ "$state_name" == "AERIAL_FLY" && "$airborne" == "true" ]]; then
      final_ok=true
      break
    fi
    if [[ "$state_name" == "FAILED" ]]; then
      echo "FAIL: FSM entered FAILED"
      exit 1
    fi
  fi
done

echo ""
echo "=== Estados observados ==="
missing=0
for s in "${REQUIRED_STATES[@]}"; do
  if [[ "${SEEN[$s]:-0}" -eq 1 ]]; then
    echo "  OK  $s"
  else
    echo "  --  $s (não visto)"
    missing=$((missing + 1))
  fi
done

if [[ "$final_ok" == "true" && "$missing" -eq 0 ]]; then
  echo "SUCCESS: transição completa e robô no ar"
  exit 0
fi

echo "INCOMPLETE: final_ok=$final_ok missing_states=$missing"
if [[ "$ASSERT" == "true" ]]; then
  exit 1
fi
exit 0
