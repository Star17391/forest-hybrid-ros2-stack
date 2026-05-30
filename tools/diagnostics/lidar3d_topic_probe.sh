#!/usr/bin/env bash
# Prova rápida de tópicos LiDAR 3D (stack a correr).
set -euo pipefail

TOPICS=(
  /sensors/lidar/points
  /perception/lidar3d/ground
  /perception/lidar3d/obstacles
  /perception/lidar3d/trunks
  /perception/lidar3d/terrain_mesh
  /perception/lidar3d/debug_stats
)

echo "=== LiDAR 3D topic probe ==="
for t in "${TOPICS[@]}"; do
  echo ""
  echo "--- ${t} ---"
  if ! ros2 topic list 2>/dev/null | grep -qx "${t}"; then
    echo "  MISSING (not advertised)"
    continue
  fi
  ros2 topic hz "${t}" --window 5 2>/dev/null | head -5 || echo "  (no messages in 5s window)"
  if [[ "${t}" == *PointCloud2* ]] || [[ "${t}" == /perception/* ]] || [[ "${t}" == /sensors/* ]]; then
    msg=$(timeout 4 ros2 topic echo "${t}" --once 2>/dev/null || true)
    if [[ -n "${msg}" ]]; then
      frame=$(echo "${msg}" | awk '/frame_id:/{print $2; exit}')
      w=$(echo "${msg}" | awk '/width:/{print $2; exit}')
      h=$(echo "${msg}" | awk '/height:/{print $2; exit}')
      pts=$((w * h))
      echo "  frame_id=${frame:-?} points≈${pts} (w=${w:-0} h=${h:-0})"
    else
      echo "  echo --once: timeout"
    fi
  fi
  if [[ "${t}" == */debug_stats ]]; then
    timeout 4 ros2 topic echo "${t}" --once 2>/dev/null | head -3 || echo "  debug_stats: timeout"
  fi
done
