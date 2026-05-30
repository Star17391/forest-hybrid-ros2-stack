#!/usr/bin/env bash
# Wrapper → scripts/nicla/advr/apply_config.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/advr/apply_config.sh" "$@"
