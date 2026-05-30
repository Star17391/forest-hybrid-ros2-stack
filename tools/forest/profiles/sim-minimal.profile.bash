# Minimal profile for Phase 1 validation (sim only, fast up/down)
forest_profile_name="sim-minimal"

forest_profile_up() {
  forest_launch_layer sim forest_hybrid_conf sim_pose_bridge.launch.py \
    world:=mvp_empty_flat.sdf paused:=false cleanup_first:=false use_rviz:=false
  forest_session_register_layer sim "$FOREST_LAST_LAUNCH_PGID" "$FOREST_LAST_LAUNCH_PID" \
    "ros2 launch forest_hybrid_conf sim_pose_bridge.launch.py (minimal)"
}

forest_profile_wait_nodes=(marble_pose_from_gz)
