#!/usr/bin/env bash
# Wrapper → scripts/nicla/legacy/wifi_connect.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/legacy/wifi_connect.sh" "$@"
