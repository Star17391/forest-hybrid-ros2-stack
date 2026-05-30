#!/usr/bin/env bash
# Update commands.bash. Reload Tab completion in the CURRENT shell (must be sourced).
set -euo pipefail

if [[ -n "${_FOREST_SYNC_RUNNING:-}" ]]; then
  exit 0
fi
export _FOREST_SYNC_RUNNING=1

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CMD_FILE="${ROOT}/completions/commands.bash"
COMP_FILE="${ROOT}/completions/forest.bash"

_forest_write_commands() {
  cat >"$CMD_FILE" <<'EOF'
# Canonical forest CLI words for bash Tab completion (sync_completions.sh).

FOREST_CLI_TOP_LEVEL="status down cleanup up world profile panel teleop attach test diag logs completion help -h --help"
FOREST_CLI_DIAG_SUBS="imu tf imu-check imu-analyze tf-audit lidar lidar-classify tf-frames lidar3d-stack pose pose-benchmark ekf-latency phase0-compare world"
FOREST_CLI_COMPLETION_SUBS="refresh sync"
FOREST_CLI_ATTACH_SUBS="panel teleop logs"
FOREST_CLI_LOGS_OPTS="-f --follow -n --grep -h --help"
FOREST_CLI_PROFILE_SUBS="list validate"
FOREST_CLI_WORLD_SUBS="list"
FOREST_CLI_UP_OPTS="-d --detach --panel-only --headless --no-rviz --lidar2d --lidar3d --world -w --timeout -h --help"
EOF
}

_forest_reload_completion() {
  complete -r forest 2>/dev/null || true
  unset _FOREST_COMPLETION_LOADED 2>/dev/null || true
  # shellcheck source=forest.bash
  source "$COMP_FILE"
}

_forest_write_commands
echo "[sync] Wrote ${CMD_FILE}"

# Sourced: reload Tab in this interactive shell
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  _forest_reload_completion
  echo "[sync] Tab completion reloaded (--world, world list, forest logs, …)"
  return 0 2>/dev/null || exit 0
fi

# Executed as bash script: subshell cannot update parent Tab — tell user what to run
echo ""
echo "Ficheiros actualizados. Para o Tab nesta shell, corre:"
echo "  source ${COMP_FILE}"
echo ""
echo "Ou:"
echo "  source ${BASH_SOURCE[0]}"
