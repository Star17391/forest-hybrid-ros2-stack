#!/usr/bin/env bash
# Wrapper → scripts/lidar/install_driver.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lidar/install_driver.sh" "$@"
