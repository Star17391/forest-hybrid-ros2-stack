#!/usr/bin/env bash
# Verifica se restam processos Gazebo/ROS do stack Forest (após shutdown).
set -uo pipefail

count=0
while IFS= read -r line; do
  # Ignorar wrappers do Cursor que só referenciam o comando sem o executar
  if [[ "$line" == *"COMMAND_EXIT_CODE"* ]] || [[ "$line" == *"__CURSOR_SANDBOX"* ]]; then
    continue
  fi
  echo "  $line"
  count=$((count + 1))
done < <(
  ps -u "$USER" -o pid=,args= 2>/dev/null \
    | grep -E 'gz sim|ros_gz_bridge|rviz2|forest_sim_bridge|forest_gen_bringup|forest_planner_ros2|forest_navigation_ros2|ros2 launch forest_' \
    | grep -v grep \
    | grep -v cursorsandbox \
    || true
)

if [[ "$count" -eq 0 ]]; then
  echo "OK — nenhum processo Forest/Gazebo/RViz/navegação encontrado."
  exit 0
fi

echo "AVISO — $count processo(s) ainda activo(s) (lista acima)."
echo "       Correr: bash scripts/stack/kill_stack.sh"
exit 1
