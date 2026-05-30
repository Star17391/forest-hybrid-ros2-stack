# shellcheck shell=bash
[[ -n "${_FOREST_LIFE_LOADED:-}" ]] && return 0
_FOREST_LIFE_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

forest_run_cleanup() {
  local extra=()
  if [[ "${1:-}" == "--hybrid" || "${1:-}" == "hybrid" ]]; then
    extra=(--hybrid)
  fi
  local term_wait="${FOREST_CLEANUP_TERM_WAIT:-2.5}"
  forest_log_section "Cleanup (forest_cleanup ${extra[*]} term-wait=${term_wait}s)"
  forest_source_ros || return 1
  ros2 run forest_sim_bridge forest_cleanup "${extra[@]}" --term-wait "$term_wait"
}

# Retry cleanup + optional kill_stack until verify_clean passes.
forest_ensure_clean() {
  local max="${FOREST_CLEANUP_RETRIES:-4}"
  local n=1
  while (( n <= max )); do
    forest_run_cleanup --hybrid || true
    if forest_verify_clean; then
      return 0
    fi
    echo "  cleanup retry ${n}/${max}..." >&2
    sleep 1
    n=$((n + 1))
  done
  local kill_script="${HYBRID_WS}/scripts/stack/kill_stack.sh"
  if [[ -f "$kill_script" ]]; then
    forest_log_section "Hard kill (kill_stack.sh)"
    bash "$kill_script" && return 0
  fi
  forest_verify_clean
}

forest_verify_clean() {
  local verify_script="$HYBRID_WS/scripts/stack/verify_clean.sh"
  if [[ ! -x "$verify_script" ]]; then
    verify_script="$HYBRID_WS/scripts/stack/verify_clean.sh"
  fi
  forest_log_section "Verifying clean state"
  if bash "$verify_script"; then
    return 0
  fi
  return 1
}

forest_kill_pgid() {
  local pgid="$1"
  local label="${2:-process}"
  if [[ -z "$pgid" || "$pgid" == "0" ]]; then
    return 0
  fi
  if ! kill -0 "-${pgid}" 2>/dev/null; then
    return 0
  fi
  echo "  SIGINT → PGID ${pgid} (${label})"
  kill -INT "-${pgid}" 2>/dev/null || true
  local i
  for i in $(seq 1 40); do
    kill -0 "-${pgid}" 2>/dev/null || return 0
    sleep 0.15
  done
  echo "  SIGTERM → PGID ${pgid}"
  kill -TERM "-${pgid}" 2>/dev/null || true
  sleep 0.5
  if kill -0 "-${pgid}" 2>/dev/null; then
    echo "  SIGKILL → PGID ${pgid}"
    kill -KILL "-${pgid}" 2>/dev/null || true
  fi
}
