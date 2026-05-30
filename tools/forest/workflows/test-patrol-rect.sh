#!/usr/bin/env bash
# PATROL retângulo — requer forest up sim-mvp-nav -d + PLAY no Gazebo.
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

forest_source_ros

for n in mission_manager_node navigation_node; do
  if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/${n}"; then
    echo "ERROR: nó /${n} em falta." >&2
    echo "  Arranca: forest up sim-mvp-nav -d   (e PLAY no Gazebo)" >&2
    exit 1
  fi
done

forest_log_section "PATROL retângulo (2,0)→(5,0)→(5,3)→(2,3)"
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 2, frame_type: 0, command_id: 'patrol_rect', source: 'forest_test', \
    waypoint_x: [2.0, 5.0, 5.0, 2.0], waypoint_y: [0.0, 0.0, 3.0, 3.0], \
    waypoint_z: [0.0, 0.0, 0.0, 0.0], waypoint_yaw: []}"

echo "=== Aguardar route ==="
timeout 8 ros2 topic echo /planning/mission_route --once || true

echo "=== Progresso (60s max) ==="
timeout 60 ros2 topic echo /planning/progress 2>/dev/null | head -20 || true

echo "=== goal_reached ==="
timeout 90 ros2 topic echo /planning/goal_reached --once || true

echo "=== Métricas ==="
tail -n 8 /tmp/forest_navigation_metrics.csv 2>/dev/null || true
