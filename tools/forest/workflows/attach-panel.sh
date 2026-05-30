#!/usr/bin/env bash
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/mission.bash
source "${FOREST_ROOT}/lib/mission.bash"

forest_source_ros
forest_open_mission_panel
