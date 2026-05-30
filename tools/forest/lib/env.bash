# shellcheck shell=bash
# Forest operational environment — paths and ROS workspace sourcing.
[[ -n "${_FOREST_ENV_LOADED:-}" ]] && return 0
_FOREST_ENV_LOADED=1

forest_resolve_root() {
  if [[ -n "${FOREST_ROOT:-}" && -d "${FOREST_ROOT}/tools/forest" ]]; then
    return 0
  fi
  local here
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [[ -d "${here}/lib" && -d "${here}/bin" ]]; then
    FOREST_ROOT="$here"
    export FOREST_ROOT
    return 0
  fi
  echo "ERROR: cannot resolve FOREST_ROOT (tools/forest)" >&2
  return 1
}

forest_resolve_root || return 1

export FORESTGEN_PATH="${FORESTGEN_PATH:-$HOME/Projetos/Gazebo/ForestGen}"
export HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"

forest_init_state_dir() {
  if [[ -n "${FOREST_STATE_DIR:-}" ]]; then
    mkdir -p "$FOREST_STATE_DIR" 2>/dev/null || true
    return 0
  fi
  local candidate="${XDG_RUNTIME_DIR:-/tmp}/forest"
  if mkdir -p "$candidate" 2>/dev/null; then
    FOREST_STATE_DIR="$candidate"
  else
    FOREST_STATE_DIR="/tmp/forest"
    mkdir -p "$FOREST_STATE_DIR"
  fi
  export FOREST_STATE_DIR
}

forest_init_state_dir
export FOREST_SESSION_FILE="${FOREST_SESSION_FILE:-$FOREST_STATE_DIR/session.state.json}"
export FOREST_LOCK_FILE="${FOREST_LOCK_FILE:-$FOREST_STATE_DIR/session.lock}"

forest_source_ros_base() {
  set +u
  # shellcheck disable=SC1091
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found" >&2
    set -u
    return 1
  fi
  set -u
}

forest_source_hybrid_install() {
  set +u
  if [[ ! -f "$HYBRID_WS/install/setup.bash" ]]; then
    echo "ERROR: HYBRID_WS install missing — run colcon build in $HYBRID_WS" >&2
    set -u
    return 1
  fi
  # shellcheck disable=SC1091
  source "$HYBRID_WS/install/setup.bash"
  set -u
}

forest_source_ros() {
  forest_source_ros_base || return 1
  forest_source_hybrid_install || return 1
}
