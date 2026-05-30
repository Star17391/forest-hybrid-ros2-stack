#!/usr/bin/env bash
# Host environment probe for RViz/Ogre stability (run on StarTech workstation).
set -euo pipefail

echo "=== RViz stability probe ==="
echo "date: $(date -Iseconds)"
echo "session: ${XDG_SESSION_TYPE:-unknown}  DISPLAY=${DISPLAY:-unset}  WAYLAND=${WAYLAND_DISPLAY:-unset}"
echo ""

echo "--- GPU ---"
if command -v nvidia-smi &>/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used,memory.free --format=csv,noheader
else
  echo "nvidia-smi not found"
fi
if command -v glxinfo &>/dev/null; then
  glxinfo -B 2>/dev/null | head -20 || true
else
  echo "install mesa-utils for glxinfo: sudo apt install mesa-utils"
fi
echo ""

echo "--- OpenGL (rviz uses Ogre GL) ---"
if command -v eglinfo &>/dev/null; then
  eglinfo 2>/dev/null | head -15 || true
fi
echo ""

echo "--- ROS / RViz ---"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
if [[ -f "${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}/install/setup.bash"
fi
ros2 pkg prefix rviz2 2>/dev/null && dpkg -l | grep -E 'ros-jazzy-rviz|ogre' | head -5 || true
echo ""

echo "--- RViz configs ---"
ROOT="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
for f in \
  "$ROOT/src/conf/forest_hybrid_conf/config/forest_mvp_minimal.rviz" \
  "$ROOT/src/conf/forest_hybrid_conf/config/forest_mvp_sensors.rviz" \
  "$ROOT/src/conf/forest_hybrid_conf/config/forest_mvp_sim.rviz"; do
  if [[ -f "$f" ]]; then
    python3 "$ROOT/tools/diagnostics/rviz_config_validate.py" "$f" || true
    echo ""
  fi
done

echo "=== Done — save this output in docs/reports/ for comparison ==="
