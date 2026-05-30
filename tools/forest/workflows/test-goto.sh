#!/usr/bin/env bash
# GOTO (5, 0) smoke — requer forest up sim-mvp-nav -d + PLAY no Gazebo.
# Uso: forest test goto [--assert] [--x X] [--y Y] [--timeout SEC]
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

ASSERT=false
TARGET_X=5.0
TARGET_Y=0.0
TIMEOUT_SEC=90

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --x) shift; TARGET_X="${1:?}" ;;
    --y) shift; TARGET_Y="${1:?}" ;;
    --timeout) shift; TIMEOUT_SEC="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test goto [--assert] [--x X] [--y Y] [--timeout SEC]"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

for n in mission_manager_node navigation_node; do
  if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/${n}"; then
    echo "ERROR: nó /${n} em falta." >&2
    echo "  Arranca: forest up sim-mvp-nav -d   (e PLAY no Gazebo)" >&2
    exit 1
  fi
done

forest_log_section "GOTO (${TARGET_X}, ${TARGET_Y})"
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 1, frame_type: 0, command_id: 'forest_test_goto', source: 'forest_test', \
    target_x: ${TARGET_X}, target_y: ${TARGET_Y}, target_z: 0.0}"

echo "=== Aguardar goal_reached (${TIMEOUT_SEC}s) ==="
if timeout "${TIMEOUT_SEC}" ros2 topic echo /planning/goal_reached --once 2>/dev/null; then
  echo "SUCCESS: goal_reached"
  exit 0
fi

echo "TIMEOUT: goal_reached não recebido em ${TIMEOUT_SEC}s"
if [[ "$ASSERT" == "true" ]]; then
  exit 1
fi
exit 0
