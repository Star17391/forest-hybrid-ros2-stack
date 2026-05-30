#!/usr/bin/env bash
# Maximiza a janela RViz após arranque (X11/wmctrl). Ignora falha em Wayland puro.
set -euo pipefail
sleep 3
if ! command -v wmctrl >/dev/null 2>&1; then
  exit 0
fi
for _ in 1 2 3 4 5 6 7 8 9 10; do
  if wmctrl -l 2>/dev/null | grep -qi 'rviz'; then
    wmctrl -r 'RViz' -b add,maximized_vert,maximized_horz 2>/dev/null \
      || wmctrl -l | grep -i rviz | awk '{print $1}' | head -1 | xargs -r -I{} wmctrl -i -r {} -b add,maximized_vert,maximized_horz 2>/dev/null \
      || true
    exit 0
  fi
  sleep 1
done
exit 0
