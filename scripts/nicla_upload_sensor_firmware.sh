#!/usr/bin/env bash
# Wrapper → scripts/nicla/legacy/upload_sensor_firmware.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nicla/legacy/upload_sensor_firmware.sh" "$@"
