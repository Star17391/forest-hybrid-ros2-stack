#!/usr/bin/env bash
# Wrapper → scripts/nicla/advr/upload_firmware.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/advr/upload_firmware.sh" "$@"
