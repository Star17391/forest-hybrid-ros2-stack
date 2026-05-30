#!/usr/bin/env bash
# Wrapper → scripts/nicla/legacy/jpeg_color_check.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/legacy/jpeg_color_check.sh" "$@"
