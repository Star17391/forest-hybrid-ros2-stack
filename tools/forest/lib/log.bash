# shellcheck shell=bash
[[ -n "${_FOREST_LOG_LOADED:-}" ]] && return 0
_FOREST_LOG_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"

forest_log_section() {
  echo ""
  echo "=== [$(date +%H:%M:%S)] $* ==="
}

forest_session_log_dir() {
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  FOREST_SESSION_LOG_DIR="${FOREST_STATE_DIR}/sessions/${stamp}"
  mkdir -p "$FOREST_SESSION_LOG_DIR"
  export FOREST_SESSION_LOG_DIR
}
