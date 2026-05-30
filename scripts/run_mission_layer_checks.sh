#!/usr/bin/env bash
# Wrapper → scripts/mission/run_layer_checks.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/mission/run_layer_checks.sh" "$@"
