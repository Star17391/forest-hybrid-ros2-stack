#!/usr/bin/env bash
# Grava tópicos para diagnosticar desync pose Gazebo vs RViz (yaw / TF).
# Uso:
#   1) Lança sim_mvp / run_trajectory_following.sh
#   2) Gazebo em Play
#   3) bash scripts/navigation/record_pose_debug_bag.sh
#   4) Conduz o robô ou arranca PATROL ~30–60 s
#   5) Ctrl+C para parar gravação
set -euo pipefail

OUT_DIR="${1:-/tmp/forest_pose_debug}"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$OUT_DIR/pose_debug_${STAMP}"

set +u
source /opt/ros/jazzy/setup.bash
source "${FORESTGEN_WS:-$HOME/Projetos/Gazebo/ForestGen}/install/setup.bash"
source "${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}/install/setup.bash"
set -u

echo "A gravar em: ${BAG}"
echo "Tópicos: pose_fused, track odom L/R, world_tf_full, world_tf, cmd_vel, laser, clock, tf"
echo "Depois: python3 scripts/navigation/analyze_pose_jitter_bag.py <pasta_do_bag>"

exec ros2 bag record -o "$BAG" \
  /clock \
  /tf \
  /tf_static \
  /state/pose_fused \
  /forest_gen/gz/world_tf_full \
  /forest_gen/gz/world_tf \
  /forest_gen/gz/left_track_odometry_raw \
  /forest_gen/gz/right_track_odometry_raw \
  /forest_gen/cmd_vel \
  /forest_gen/planar_laser/scan \
  /planning/local_trajectory \
  /planning/debug/markers
