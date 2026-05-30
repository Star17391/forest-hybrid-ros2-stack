# shellcheck shell=bash
# Wait until /clock is publishing (Gazebo sim ready for RViz use_sim_time).

forest_wait_for_clock() {
  local timeout="${1:-60}"
  local n=0
  echo "Waiting for /clock (max ${timeout}s)..."
  while (( n < timeout )); do
    if timeout 2 ros2 topic echo /clock --once >/dev/null 2>&1; then
      echo "  /clock ready (${n}s)"
      return 0
    fi
    sleep 1
    ((n++)) || true
  done
  echo "ERROR: /clock not available after ${timeout}s" >&2
  return 1
}
