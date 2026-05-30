#!/usr/bin/env bash
# Wrapper → scripts/navigation/run_trajectory_following_terrain.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/navigation/run_trajectory_following_terrain.sh" "$@"
