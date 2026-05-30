#!/usr/bin/env bash
# Wrapper → scripts/lidar/test_ydlidar_sdk.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lidar/test_ydlidar_sdk.sh" "$@"
