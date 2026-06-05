# shellcheck shell=bash
[[ -n "${_FOREST_DIAG_LOADED:-}" ]] && return 0
_FOREST_DIAG_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

FOREST_DIAG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../diagnostics" && pwd)"
export FOREST_DIAG_ROOT

forest_diag_list() {
  cat <<'EOF'
imu          Launch diag_imu (Gazebo + bridge + sanitize)
tf           TF diag: audit se sessão up; senão sim_gazebo (mesmo launch que up)
tf-audit     TF publishers + tf2_echo (requer forest up / sim a correr)
imu-check    Gazebo + ROS IMU topic check (bash)
imu-analyze  Quantitative IMU stream stats (python)
tf-audit     TF publishers + view_frames + tf2_echo
lidar        LiDAR pipeline audit (/scan vs preprocess)
lidar-classify  Fase 1: label distribution on points_labeled
tf-frames       TF tree audit (map/odom/base/laser)
lidar3d-stack   LiDAR 3D: TF chain, hz, gaps (requer sim + PLAY)
lidar3d-tilt    LiDAR 3D: diagnóstico slope/pitch/transform errors
lidar3d-seg     Fase 1: segmentation audit (ground/trunk/obstacle %)
lidar3d-slice   Slice trunk rejections (30s, needs debug_stats + teleop)
lidar3d-exp-tune  Web UI live tuning CSF+clustering (port 8766)
lidar3d-exp-audit Pipeline experimental: funnel + root-cause hints
pose         Compare pose_fused vs Gazebo world_tf (quick)
pose-benchmark  Fase 0: CSV+JSON+PNG vs GT (--label, --duration)
ekf-latency  Fase 0: sensor/EKF rates and latency JSON
phase0-compare  Compare two metrics.json (A/B); pass paths after name
world        Audit ForestGen world/model collisions
hybrid-joints  Lagartas/pernas: joints Gazebo, joint_state, cmd_pos, ROS status
EOF
}

forest_diag_run() {
  local name="${1:-}"
  shift || true
  case "$name" in
    imu)
      forest_source_ros || return 1
      forest_log_section "diag imu — press PLAY in Gazebo"
      exec ros2 launch forest_hybrid_conf diag_imu.launch.py "$@"
      ;;
    tf)
      forest_source_ros || return 1
      if forest_session_active; then
        forest_log_section "diag tf — sessão activa (não relança Gazebo)"
        echo "Stack já vem de 'forest up'. A auditar TF na sessão actual…"
        echo "  (Para sim isolada: forest down && forest diag tf)"
        bash "${FOREST_DIAG_ROOT}/analyze_tf_tree.sh" "$@" || true
        python3 "${FOREST_DIAG_ROOT}/ekf_tf_health.py" --duration 10 "$@" || true
        return 0
      fi
      forest_log_section "diag tf — sim isolada (sim_gazebo.launch.py, PLAY no Gazebo)"
      exec ros2 launch forest_hybrid_conf diag_tf_ekf.launch.py paused:=false "$@"
      ;;
    imu-check)
      forest_source_ros || return 1
      exec bash "${FOREST_DIAG_ROOT}/check_imu_pipeline.sh" "$@"
      ;;
    imu-analyze)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/analyze_imu_stream.py" "$@"
      ;;
    tf-audit)
      forest_source_ros || return 1
      exec bash "${FOREST_DIAG_ROOT}/analyze_tf_tree.sh" "$@"
      ;;
    lidar)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar_pipeline_audit.py" "$@"
      ;;
    lidar-classify)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar_classify_audit.py" "$@"
      ;;
    tf-frames)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/tf_frame_audit.py" "$@"
      ;;
    lidar3d-stack)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_stack_monitor.py" "$@"
      ;;
    lidar3d-tilt)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_tilt_debug.py" "$@"
      ;;
    lidar3d-seg)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_segmentation_audit.py" "$@"
      ;;
    lidar3d-slice)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_slice_debug.py" "$@"
      ;;
    lidar3d-exp-tune)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_experimental_live_tuning.py" "$@"
      ;;
    lidar3d-exp-audit)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/lidar3d_experimental_pipeline_audit.py" "$@"
      ;;
    pose)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/compare_pose_sources.py" "$@"
      ;;
    pose-benchmark)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/pose_benchmark.py" "$@"
      ;;
    ekf-latency)
      forest_source_ros || return 1
      exec python3 "${FOREST_DIAG_ROOT}/ekf_latency_analyzer.py" "$@"
      ;;
    phase0-compare)
      exec python3 "${FOREST_DIAG_ROOT}/compare_phase0_reports.py" "$@"
      ;;
    world)
      exec python3 "${FOREST_DIAG_ROOT}/audit_world_collisions.py" "$@"
      ;;
    hybrid-joints)
      forest_source_ros || return 1
      exec bash "${FOREST_DIAG_ROOT}/hybrid_joints_diag.sh" "$@"
      ;;
    ""|-h|--help|help|list)
      echo "forest diag <name> [args...]"
      echo ""
      forest_diag_list | sed 's/^/  /'
      return 0
      ;;
    *)
      echo "ERROR: unknown diag: $name" >&2
      forest_diag_list | sed 's/^/  /' >&2
      return 2
      ;;
  esac
}
