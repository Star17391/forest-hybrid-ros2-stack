#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
ROOT="$(forest_repo_root)"
python3 "$SCRIPT_DIR/apply_config.py"
