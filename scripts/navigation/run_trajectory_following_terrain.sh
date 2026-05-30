#!/usr/bin/env bash
# ============================================================================
# run_trajectory_following_terrain.sh
#
# Trajectory following on gentle procedural terrain (no trees/rocks).
# Waypoints are planar (X/Y only); robot height follows Gazebo physics.
#
# Prerequisite (once, or after changing terrain params):
#   cd ~/Projetos/Gazebo/ForestGen
#   source venv_forest/bin/activate
#   python3 scripts/generate_world.py --config scripts/config_terrain_gentle.yaml \
#     --save mvp_terrain_gentle
#
# Usage:
#   bash scripts/navigation/run_trajectory_following_terrain.sh
# ============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/_forest_common.sh"

WORLD_FILE="${FORESTGEN_PATH}/worlds/mvp_terrain_gentle.sdf"
if [[ ! -f "$WORLD_FILE" ]]; then
  echo "ERROR: World not found: $WORLD_FILE"
  echo "Generate it first:"
  echo "  cd \$FORESTGEN_PATH && source venv_forest/bin/activate"
  echo "  python3 scripts/generate_world.py --config scripts/config_terrain_gentle.yaml --save mvp_terrain_gentle"
  exit 1
fi

source_ros

log_section "Cleanup old sessions"
run_cleanup --hybrid

log_section "Launching trajectory following on gentle terrain"
launch_sim forest_hybrid_conf sim_mvp.launch.py \
  world:=mvp_terrain_gentle.sdf \
  paused:=true \
  cleanup_first:=false \
  use_rviz:=true

wait_for_nodes marble_pose_from_gz navigation_node mission_manager_node || true

log_section "Stack ready (terrain)"
echo ""
echo "  Mundo: mvp_terrain_gentle.sdf (relevo suave, sem obstáculos)"
echo "  1. Press PLAY in Gazebo"
echo "  2. Add waypoints (X/Y only) and send PATROL from the panel"
echo "  3. Success = mission COMPLETED (planar XY tolerance)"
echo "  4. RViz waypoints may appear below terrain — expected"
echo "  5. Close panel or Ctrl+C to shutdown"
echo ""

log_section "Opening mission panel"
ros2 run forest_sim_bridge forest_mission_panel || true

log_section "Shutdown"
