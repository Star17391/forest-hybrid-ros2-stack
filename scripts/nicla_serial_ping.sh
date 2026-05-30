#!/usr/bin/env bash
# Wrapper → scripts/nicla/legacy/serial_ping.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/legacy/serial_ping.sh" "$@"
