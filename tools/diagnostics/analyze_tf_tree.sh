#!/usr/bin/env bash
# TF audit: publishers, view_frames PDF, tf2_echo samples.
# Requer sim ou bag com /tf e /tf_static activos.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"  # tools/diagnostics → repo root
# shellcheck disable=SC1091
source "$STACK_ROOT/scripts/lib/_forest_common.sh" 2>/dev/null || true
source_ros 2>/dev/null || source /opt/ros/jazzy/setup.bash

OUT="${1:-/tmp/forest_tf_audit}"
mkdir -p "$OUT"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "=== TF audit -> $OUT ==="

echo "--- ros2 doctor ---" | tee "$OUT/ros2_doctor.txt"
ros2 doctor 2>&1 | tee -a "$OUT/ros2_doctor.txt" || true

echo "--- tf2_frames (YAML) ---" | tee "$OUT/tf2_frames.yaml"
ros2 run tf2_tools view_frames -o "$OUT" 2>&1 | tee -a "$OUT/view_frames.log" || true
if [[ -f "$OUT/frames.pdf" ]]; then
  mv -f "$OUT/frames.pdf" "$OUT/frames_${STAMP}.pdf"
fi

echo "--- TF publishers (nodes) ---"
{
  ros2 node list 2>/dev/null | while read -r n; do
    [[ -z "$n" ]] && continue
    if ros2 node info "$n" 2>/dev/null | grep -qE '/tf|Transform'; then
      echo "  $n"
    fi
  done
} | tee "$OUT/tf_publishers.txt"

FRAMES=(
  "map odom"
  "odom marble_hd2/base_link"
  "marble_hd2/base_link laser"
  "map marble_hd2/base_link"
)

echo "--- tf2_echo samples ---" | tee "$OUT/tf2_echo.txt"
for pair in "${FRAMES[@]}"; do
  read -r parent child <<< "$pair"
  echo "### $parent -> $child" | tee -a "$OUT/tf2_echo.txt"
  timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | head -20 | tee -a "$OUT/tf2_echo.txt" || echo "(timeout/missing)" | tee -a "$OUT/tf2_echo.txt"
done

echo "--- topic hz /tf /tf_static ---" | tee "$OUT/tf_hz.txt"
timeout 4 ros2 topic hz /tf 2>&1 | tee -a "$OUT/tf_hz.txt" || true
timeout 4 ros2 topic hz /tf_static 2>&1 | tee -a "$OUT/tf_hz.txt" || true

echo "Done. Inspect $OUT/frames_${STAMP}.pdf and tf2_echo.txt"
