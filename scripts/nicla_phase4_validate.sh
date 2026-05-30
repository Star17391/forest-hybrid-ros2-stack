#!/usr/bin/env bash
# Wrapper → scripts/nicla/validate/phase4_validate.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/validate/phase4_validate.sh" "$@"
