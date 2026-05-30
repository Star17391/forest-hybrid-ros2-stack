#!/usr/bin/env bash
set -euo pipefail
FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/teleop.bash
source "${FOREST_ROOT}/lib/teleop.bash"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
forest_source_ros || exit 1
forest_open_teleop_panel
