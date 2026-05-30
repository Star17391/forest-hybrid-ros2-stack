#!/usr/bin/env bash
# Phase 1 — Nicla Vision: USB detect, serial ping, optional snapshot.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../lib/repo_root.sh"
STACK_ROOT="$(forest_repo_root)"
cd "$STACK_ROOT"

# ROS setup scripts reference unset vars; avoid nounset until after sourcing.
set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source install/setup.bash
fi
set -u

echo "== Nicla Vision Phase 1 validation =="

if lsusb | grep -qi 'nicla vision'; then
  echo "[OK] USB: Nicla Vision detected"
  lsusb | grep -i 'nicla vision' || true
else
  echo "[WARN] USB: Nicla Vision not found in lsusb (is the board plugged in?)"
fi

if lsusb | grep -qi 'nicla vision bootloader'; then
  echo "[WARN] board is in BOOTLOADER mode (upload firmware, then press reset)"
  echo "       After upload you should see: 'Nicla Vision Virtual Comm Port' (2341:045f)"
fi

PORT="${NICLA_SERIAL_PORT:-}"
if [[ -z "$PORT" ]]; then
  PORT="$(python3 - <<'PY' || true
from forest_nicla_vision_ros2.protocol import find_nicla_serial_port
p = find_nicla_serial_port()
print(p or "", end="")
PY
)"
fi
if [[ -z "$PORT" && -e /dev/ttyACM0 ]]; then
  PORT=/dev/ttyACM0
fi

if [[ -n "$PORT" ]]; then
  echo "[OK] serial device: $PORT"
else
  echo "[FAIL] no runtime serial port (expected /dev/ttyACM* after firmware is running)"
  echo "       Set NICLA_SERIAL_PORT=/dev/ttyACM0 or upload firmware/nicla_sensor_node.ino"
  exit 1
fi

if ! groups | grep -q dialout; then
  echo "[WARN] user not in dialout group — you may need: sudo usermod -aG dialout \$USER"
fi

SNAP=0
if [[ "${1:-}" == "--snap" ]]; then
  SNAP=1
fi

ARGS=(--port "$PORT")
if [[ "$SNAP" -eq 1 ]]; then
  ARGS+=(--snap --out /tmp/nicla_snap.ppm)
fi

echo "== nicla_device_probe ${ARGS[*]} =="
if command -v nicla_device_probe >/dev/null 2>&1; then
  nicla_device_probe "${ARGS[@]}"
else
  ros2 run forest_nicla_vision_ros2 nicla_device_probe "${ARGS[@]}"
fi

if [[ "$SNAP" -eq 1 ]]; then
  echo "Snapshot written to /tmp/nicla_snap.ppm"
  if command -v xdg-open >/dev/null 2>&1; then
    echo "Open with: xdg-open /tmp/nicla_snap.ppm"
  fi
fi

echo "== done =="
