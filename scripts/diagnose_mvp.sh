#!/usr/bin/env bash
# Diagnóstico rápido do MVP (com sim_mvp.launch.py já a correr).
set -eo pipefail

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${FORESTGEN_WS:-$HOME/Projetos/Gazebo/ForestGen}/install/setup.bash"
  # shellcheck disable=SC1091
  source "${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}/install/setup.bash"
  set -u
}

source_ros

echo "=== Nós ==="
ros2 node list 2>/dev/null | grep -E 'navigation|mission|marble_pose|twist|bridge' || true

echo "=== Tópicos chave ==="
ros2 topic list 2>/dev/null | grep -E 'clock|pose_fused|mission_goal|cmd_vel|local_trajectory|debug/markers' || true

echo "=== Publishers cmd_vel ==="
ros2 topic info /forest_gen/cmd_vel -v 2>/dev/null | head -20 || true

echo "=== Pose (1 msg) ==="
timeout 8 ros2 topic echo /state/pose_fused --once 2>/dev/null || echo "SEM POSE (Gazebo em play? marble_pose_from_gz?)"

echo "=== Clock ==="
timeout 3 ros2 topic hz /clock 2>/dev/null || echo "SEM /clock (Gazebo parado — clica Play)"
