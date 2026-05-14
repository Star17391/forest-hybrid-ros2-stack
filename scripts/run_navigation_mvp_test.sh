#!/usr/bin/env bash
# Smoke test GOTO com stack MVP (sim_mvp.launch.py deve estar activo + Play no Gazebo).
set -eo pipefail

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
  FORESTGEN_WS="${FORESTGEN_WS:-$HOME/Projetos/Gazebo/ForestGen}"
  # shellcheck disable=SC1091
  source "$FORESTGEN_WS/install/setup.bash"
  # shellcheck disable=SC1091
  source "$HYBRID_WS/install/setup.bash"
  set -u
}

source_ros

echo "=== Arranque recomendado (outro terminal) ==="
echo "  ros2 launch forest_hybrid_conf sim_mvp.launch.py"
echo "  # Clica Play no Gazebo se paused:=true (default)"

echo "=== Verificar cmd_vel relay ==="
if ! ros2 topic list | grep -q '/forest_gen/cmd_vel'; then
  echo "AVISO: /forest_gen/cmd_vel não visível — arranca sim_mvp.launch.py primeiro."
fi

echo "=== Publicar GOTO (5, 0) em map ==="
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 1, frame_type: 0, command_id: 'mvp_goto', source: 'test_script', target_x: 5.0, target_y: 0.0, target_z: 0.0}"

echo "=== Aguardar mission_goal ==="
timeout 5 ros2 topic echo /planning/mission_goal --once || true

echo "=== cmd_vel (8s) ==="
timeout 8 ros2 topic hz /forest_gen/cmd_vel || true

echo "=== Monitorizar progress (15s) ==="
timeout 15 ros2 topic hz /planning/progress || true

echo "=== Últimas linhas do CSV de métricas ==="
tail -n 5 /tmp/forest_navigation_metrics.csv 2>/dev/null || echo "(sem CSV ainda)"

echo "=== Gerar plots (se matplotlib disponível) ==="
if python3 -c "import matplotlib" 2>/dev/null; then
  ros2 run forest_navigation_ros2 plot_navigation_metrics.py /tmp/forest_navigation_metrics.csv
else
  echo "matplotlib não instalado — pip install matplotlib para plots"
fi

echo "OK — rever RViz (forest_mvp_sim.rviz) e movimento no Gazebo"
