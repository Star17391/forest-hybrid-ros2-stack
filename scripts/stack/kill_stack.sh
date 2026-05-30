#!/usr/bin/env bash
# Kill all lingering Forest stack processes (sim + navigation + launch).
set -eo pipefail

HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$HYBRID_WS/install/setup.bash"
set -u

echo "=== forest_cleanup --hybrid ==="
ros2 run forest_sim_bridge forest_cleanup --hybrid --term-wait 1.5

echo ""
bash "$HYBRID_WS/scripts/stack/verify_clean.sh"
