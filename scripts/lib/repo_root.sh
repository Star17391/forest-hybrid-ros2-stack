#!/usr/bin/env bash
# Resolve repository root from any script under scripts/.
forest_repo_root() {
  local d
  d="$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)"
  while [[ "$d" != "/" ]]; do
    if [[ -d "$d/src/conf/forest_hybrid_conf" ]]; then
      echo "$d"
      return 0
    fi
    d="$(dirname "$d")"
  done
  echo "ERROR: forest-hybrid-ros2-stack root not found (no src/conf/forest_hybrid_conf)" >&2
  return 1
}
