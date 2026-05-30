#!/usr/bin/env bash
# Source forest test helpers from scripts/<area>/<script>.sh
# Usage (from scripts/foo/bar/script.sh):
#   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   # shellcheck disable=SC1091
#   source "$SCRIPT_DIR/../../lib/source_common.sh"  # adjust ../ count to reach scripts/lib
forest_source_common() {
  local lib_dir
  lib_dir="$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)"
  # shellcheck disable=SC1091
  source "$lib_dir/_forest_common.sh"
}
