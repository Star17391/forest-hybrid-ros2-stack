# shellcheck shell=bash
# Bridge legacy scripts/lib/_forest_common.sh API to forest modules.
[[ -n "${_FOREST_COMPAT_LOADED:-}" ]] && return 0
_FOREST_COMPAT_LOADED=1

# shellcheck source=session.bash
source "$(dirname "${BASH_SOURCE[0]}")/session.bash"

SIM_PID=""
SIM_PGID=""

launch_sim() {
  local pkg="$1"
  local launch_file="$2"
  shift 2
  forest_launch_layer "sim" "$pkg" "$launch_file" "$@"
  SIM_PID=$FOREST_LAST_LAUNCH_PID
  SIM_PGID=$FOREST_LAST_LAUNCH_PGID
}

wait_for_nodes() {
  forest_wait_for_nodes "$@"
}

shutdown_all() {
  if forest_session_active; then
    forest_session_down true
  elif [[ -n "${SIM_PGID:-}" ]]; then
    forest_kill_pgid "$SIM_PGID" "sim"
    forest_run_cleanup --hybrid || true
    forest_verify_clean || true
  else
    forest_run_cleanup --hybrid || true
    forest_verify_clean || true
  fi
}

forest_legacy_install_trap() {
  trap shutdown_all INT TERM EXIT
}
