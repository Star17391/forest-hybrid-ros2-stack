# Profile: Gazebo + pose bridge + nav (parity with run_pose_bridge_mission.sh)
forest_profile_name="sim-pose-bridge"

forest_profile_up() {
  local panel_only="${1:-false}"
  forest_launch_layer sim forest_hybrid_conf sim_pose_bridge.launch.py \
    world:=mvp_empty_flat.sdf paused:=false cleanup_first:=false use_rviz:=true
  forest_session_register_layer sim "$FOREST_LAST_LAUNCH_PGID" "$FOREST_LAST_LAUNCH_PID" \
    "ros2 launch forest_hybrid_conf sim_pose_bridge.launch.py"
  sleep 8

  local -a nav_args=(use_sim_time:=true)
  if [[ "$panel_only" == "true" ]]; then
    nav_args+=(use_mission_panel:=true)
  fi
  forest_launch_layer nav forest_hybrid_conf navigation_mvp.launch.py "${nav_args[@]}"
  forest_session_register_layer nav "$FOREST_LAST_LAUNCH_PGID" "$FOREST_LAST_LAUNCH_PID" \
    "ros2 launch forest_hybrid_conf navigation_mvp.launch.py"
}

forest_profile_wait_nodes=(
  marble_pose_from_gz
  mission_manager_node
  navigation_node
)
