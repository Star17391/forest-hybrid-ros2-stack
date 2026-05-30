#!/usr/bin/env bash
# 30s slice rejection monitor — use with forest up + teleop.
#
#   Terminal 1: forest up sim-lidar3d-test -d --world forest_rugged_trees_rocks --trunk-slice
#   Terminal 2: forest test lidar3d-slice-debug
#   Terminal 3: forest teleop   (or forest random_move)

set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HYBRID_WS="$(cd "${FOREST_ROOT}/../.." && pwd)"
DIAG="${HYBRID_WS}/tools/diagnostics"

# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"

DURATION=30
INTERVAL=2

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: forest test lidar3d-slice-debug [--duration 30]"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
done

forest_source_ros || exit 1

if ! ros2 topic list 2>/dev/null | grep -q '/perception/lidar3d/debug_stats'; then
  echo "ERROR: stack not running or lidar3d_segmentation_node missing." >&2
  echo "  Start: forest up sim-lidar3d-test -d --world forest_rugged_trees_rocks --trunk-slice" >&2
  exit 1
fi

exec python3 "${DIAG}/lidar3d_slice_debug.py" --duration "${DURATION}" --interval "${INTERVAL}"
