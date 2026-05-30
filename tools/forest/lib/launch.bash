# shellcheck shell=bash
[[ -n "${_FOREST_LAUNCH_LOADED:-}" ]] && return 0
_FOREST_LAUNCH_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

forest_wait_for_nodes() {
  local timeout="${FOREST_NODE_WAIT_TIMEOUT:-120}"
  local -a nodes=("$@")
  forest_log_section "Waiting for nodes: ${nodes[*]} (timeout ${timeout}s)"
  sleep 3
  local elapsed=3
  while (( elapsed < timeout )); do
    local all=true
    local n
    for n in "${nodes[@]}"; do
      if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/${n}"; then
        all=false
        break
      fi
    done
    if [[ "$all" == "true" ]]; then
      echo "All nodes ready (${elapsed}s)"
      return 0
    fi
    sleep 1
    ((elapsed++)) || true
  done
  echo "WARNING: timeout waiting for nodes" >&2
  echo "  Found: $(ros2 node list 2>/dev/null | tr '\n' ' ')" >&2
  return 1
}

# Launch one ros2 launch in a new session; log to FOREST_SESSION_LOG_DIR/<id>.log
# Sets FOREST_LAST_LAUNCH_PID and FOREST_LAST_LAUNCH_PGID
forest_launch_layer() {
  local layer_id="$1"
  local pkg="$2"
  local launch_file="$3"
  shift 3
  local log_file="${FOREST_SESSION_LOG_DIR:-/tmp/forest}/${layer_id}.log"
  mkdir -p "$(dirname "$log_file")"
  forest_log_section "Launch layer '${layer_id}': ros2 launch ${pkg} ${launch_file} $*"
  # shellcheck disable=SC2091
  setsid ros2 launch "$pkg" "$launch_file" "$@" >"$log_file" 2>&1 &
  FOREST_LAST_LAUNCH_PID=$!
  export FOREST_LAST_LAUNCH_PID
  local waited=0
  FOREST_LAST_LAUNCH_PGID=""
  while (( waited < 30 )); do
    if kill -0 "$FOREST_LAST_LAUNCH_PID" 2>/dev/null; then
      FOREST_LAST_LAUNCH_PGID="$(ps -o pgid= -p "$FOREST_LAST_LAUNCH_PID" 2>/dev/null | tr -d ' ')"
      export FOREST_LAST_LAUNCH_PGID
      echo "  pid=${FOREST_LAST_LAUNCH_PID} pgid=${FOREST_LAST_LAUNCH_PGID} log=${log_file}"
      return 0
    fi
    sleep 1
    ((waited++)) || true
  done
  echo "ERROR: layer ${layer_id} exited within ${waited}s — see ${log_file}" >&2
  tail -n 40 "$log_file" >&2 || true
  return 1
}
