#!/usr/bin/env bash
# Ablação experimental das pernas (uma variável de cada vez).
#
# Requer Gazebo PLAY após cada forest up.
#
# Uso:
#   forest test hybrid-leg-ablation --case baseline
#   forest test hybrid-leg-ablation --case test1
#   forest test hybrid-leg-ablation --case test2
#   forest test hybrid-leg-ablation --case test3
#   forest test hybrid-leg-ablation --case all
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
# shellcheck source=../lib/launch.bash
source "${FOREST_ROOT}/lib/launch.bash"
# shellcheck source=../lib/session.bash
source "${FOREST_ROOT}/lib/session.bash"

forest_source_ros

CASE="baseline"
DURATION=45
RUN_RUNAWAY=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --case) shift; CASE="${1:?}" ;;
    --duration) shift; DURATION="${1:?}" ;;
    --runaway-probe) RUN_RUNAWAY=true ;;
    -h|--help)
      cat <<'EOF'
Usage: forest test hybrid-leg-ablation [--case NAME] [--duration SEC] [--runaway-probe]

Cases (reinicia o stack entre casos):
  baseline  — FSM por defeito (leg deploy 0.17 m)
  test1     — leg_extension_deployed_m=0.0
  test2     — min dwell: leg 3s, tracks 2s, aerial_ready 2s
  test3     — disable_leg_commands=true (FSM igual, sem cmd às pernas)
  all       — corre baseline, test1, test2, test3 em sequência

Após cada up: PLAY no Gazebo, depois o probe publica to_aerial.

Critério:
  • Se test1 elimina queda → pernas candidatas a causa raiz
  • Se queda persiste → pernas amplificam outro problema

EOF
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

launch_args_for_case() {
  local c="$1"
  local -a args=(
    cleanup_first:=false
    paused:=false
    world:=mvp_empty_flat.sdf
    use_rviz:=true
  )
  case "$c" in
    baseline)
      ;;
    test1)
      args+=(hybrid_leg_deployed_m:=0.0)
      ;;
    test2)
      args+=(
        hybrid_min_leg_extend_sec:=3.0
        hybrid_min_tracks_rotate_sec:=2.0
        hybrid_min_aerial_ready_sec:=2.0
      )
      ;;
    test3)
      args+=(hybrid_disable_leg_commands:=true)
      ;;
    *)
      echo "Unknown case: $c" >&2
      return 1
      ;;
  esac
  printf '%s\n' "${args[@]}"
}

run_one_case() {
  local c="$1"
  forest_log_section "Case: $c — forest down && up"
  forest down 2>/dev/null || true
  sleep 2

  mapfile -t LAUNCH_ARGS < <(launch_args_for_case "$c")
  forest_log_section "Launch sim-hybrid-test (${LAUNCH_ARGS[*]})"
  forest_session_log_dir
  # shellcheck disable=SC2091
  if ! forest_launch_layer sim forest_hybrid_conf sim_hybrid_test.launch.py "${LAUNCH_ARGS[@]}"; then
    echo "FAIL: launch layer" >&2
    return 1
  fi
  forest_session_register_layer sim "$FOREST_LAST_LAUNCH_PGID" "$FOREST_LAST_LAUNCH_PID" \
    "${FOREST_SESSION_LOG_DIR}/sim.log" 2>/dev/null || true

  if ! forest_wait_for_nodes hybrid_transition_manager hybrid_aerial_motor_controller marble_pose_from_gz; then
    echo "FAIL: nós em falta — ver ${FOREST_SESSION_LOG_DIR}/sim.log (gz/bridge crash?)" >&2
    tail -30 "${FOREST_SESSION_LOG_DIR}/sim.log" >&2 || true
    return 1
  fi

  if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/hybrid_transition_manager"; then
    echo "FAIL: hybrid_transition_manager sumiu após wait" >&2
    return 1
  fi

  echo ""
  echo ">>> Gazebo em PLAY (não pausado) — Enter para disparar to_aerial e gravar 45s..."
  read -r _

  local exp_name="$c"
  [[ "$c" == "test1" ]] && exp_name="test1_no_leg_deploy"
  [[ "$c" == "test2" ]] && exp_name="test2_slow_fsm"
  [[ "$c" == "test3" ]] && exp_name="test3_no_leg_cmds"

  forest_log_section "Ablation probe ($exp_name, ${DURATION}s)"
  ros2 run forest_sim_bridge hybrid_leg_ablation_probe \
    --experiment "$exp_name" --duration "$DURATION" || true

  if [[ "$RUN_RUNAWAY" == "true" ]]; then
    forest_log_section "Runaway probe"
    ros2 run forest_sim_bridge hybrid_runaway_probe --sample-sec 20 || true
  fi

  echo ""
  echo "Log: ${FOREST_SESSION_LOG_DIR}/sim.log"
  echo ""
}

if [[ "$CASE" == "all" ]]; then
  for c in baseline test1 test2 test3; do
    run_one_case "$c" || echo "FAIL: case $c" >&2
  done
  echo "DONE: all ablation cases — comparar fall_pre_fly e runaway entre relatórios"
  exit 0
fi

run_one_case "$CASE"
