#!/usr/bin/env bash
# Wrapper → scripts/lidar/view_rviz.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lidar/view_rviz.sh" "$@"
