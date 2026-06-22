#!/usr/bin/env bash
# Diagnóstico: joints das lagartas + pernas + comandos híbridos (Gazebo + ROS).
# Uso: forest up sim-hybrid-test -d  →  forest diag hybrid-joints
set -euo pipefail

forest_source_ros 2>/dev/null || {
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
}

echo "=== Gazebo: joints no modelo marble_hd2 ==="
if command -v gz >/dev/null 2>&1; then
  gz model -m marble_hd2 -j 2>/dev/null | grep -E "support_leg|track_yaw|Type" || echo "(modelo não encontrado — dá PLAY no Gazebo?)"
  echo ""
  echo "=== Gazebo: joint_state (posições) ==="
  timeout 3 gz topic -e -t /world/unified_world/model/marble_hd2/joint_state -n 1 2>/dev/null \
    | grep -E "name:|position:|xyz" || echo "(sem joint_state)"
  echo ""
  echo "=== Gazebo: subscritores cmd_pos lagartas ==="
  for t in \
    /model/marble_hd2/hybrid/left_track_yaw/cmd_pos \
    /model/marble_hd2/hybrid/right_track_yaw/cmd_pos; do
    echo "--- $t"
    gz topic -i -t "$t" 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' || true
  done
else
  echo "gz não disponível"
fi

echo ""
echo "=== ROS: hybrid_transition_manager + joint_states ==="
if ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
  timeout 2 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null || true
  echo ""
  timeout 2 ros2 topic echo /forest_gen/hybrid/joint_states --once 2>/dev/null || true
else
  echo "Nó /hybrid_transition_manager em falta — corre: forest up sim-hybrid-test -d"
fi

echo ""
echo "=== Teste manual (opcional) ==="
echo "  ros2 topic pub --once /forest_gen/hybrid/left_track_yaw_cmd std_msgs/msg/Float64 \"{data: 1.57}\""
echo "  ros2 topic pub --once /forest_gen/hybrid/right_track_yaw_cmd std_msgs/msg/Float64 \"{data: -1.57}\""
