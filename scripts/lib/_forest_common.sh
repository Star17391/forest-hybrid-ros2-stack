#!/usr/bin/env bash
# Common library for forest test scripts — thin wrapper over tools/forest/lib.
# Source this file; do NOT execute directly.
set -eo pipefail

export HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
_FOREST_LIB="${HYBRID_WS}/tools/forest/lib"

if [[ ! -f "${_FOREST_LIB}/compat_legacy.bash" ]]; then
  echo "ERROR: forest lib missing at ${_FOREST_LIB}" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=../../tools/forest/lib/env.bash
source "${_FOREST_LIB}/env.bash"
# shellcheck source=../../tools/forest/lib/log.bash
source "${_FOREST_LIB}/log.bash"
# shellcheck source=../../tools/forest/lib/lifecycle.bash
source "${_FOREST_LIB}/lifecycle.bash"
# shellcheck source=../../tools/forest/lib/launch.bash
source "${_FOREST_LIB}/launch.bash"
# shellcheck source=../../tools/forest/lib/compat_legacy.bash
source "${_FOREST_LIB}/compat_legacy.bash"

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/forest_tests"
mkdir -p "$LOG_DIR"

# Legacy aliases (same names as before)
source_ros() { forest_source_ros "$@"; }
log_section() { forest_log_section "$@"; }
run_cleanup() {
  if [[ "${1:-}" == "--hybrid" ]]; then
    forest_run_cleanup --hybrid
  else
    forest_run_cleanup
  fi
}

_CLEANUP_DONE=0
_shutdown_all_once() {
  if [[ "${_CLEANUP_DONE}" == 1 ]]; then
    return
  fi
  _CLEANUP_DONE=1
  shutdown_all
}

# Do not install EXIT trap in non-interactive subshells (e.g. phase1 tests) — avoids
# tearing down a forest session when the subshell exits after sourcing this file.
if [[ -z "${FOREST_NO_LEGACY_TRAP:-}" ]]; then
  if [[ -t 1 ]] || [[ "${FOREST_LEGACY_TRAP:-}" == "1" ]]; then
    trap _shutdown_all_once EXIT INT TERM
  fi
fi
