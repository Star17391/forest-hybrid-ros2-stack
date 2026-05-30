#!/usr/bin/env bash
# Wrapper → scripts/stack/verify_clean.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/stack/verify_clean.sh" "$@"
