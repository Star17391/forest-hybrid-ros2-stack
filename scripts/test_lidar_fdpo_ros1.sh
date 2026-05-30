#!/usr/bin/env bash
# Wrapper → scripts/lidar/test_fdpo_ros1.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lidar/test_fdpo_ros1.sh" "$@"
