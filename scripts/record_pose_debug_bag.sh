#!/usr/bin/env bash
# Wrapper → scripts/navigation/record_pose_debug_bag.sh (paths at repo root kept for compatibility)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/navigation/record_pose_debug_bag.sh" "$@"
