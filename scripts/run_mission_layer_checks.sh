#!/usr/bin/env bash
# Run mission-manager smoke tests against the freshly built workspace.
# Usage (from workspace root):
#   source /opt/ros/jazzy/setup.bash
#   ./scripts/run_mission_layer_checks.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WS}"

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "install/setup.bash not found — run colcon build from ${WS}" >&2
  exit 2
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"

if ! ros2 pkg prefix forest_hybrid_msgs &>/dev/null; then
  echo "forest_hybrid_msgs still not visible in AMENT_PREFIX_PATH." >&2
  echo "Re-open this shell or run: source ${WS}/install/setup.bash" >&2
  exit 3
fi

exec python3 "${SCRIPT_DIR}/test_mission_manager_smoke.py"
