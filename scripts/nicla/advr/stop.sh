#!/usr/bin/env bash
# Free TCP port 8002 and stop stuck nicla_receiver after a bad Ctrl+C.
set -euo pipefail

PORT="${1:-8002}"

echo "Stopping nicla_receiver processes..."
pkill -f '/nicla_vision_ros2/nicla_receiver' 2>/dev/null || true
sleep 0.5

if command -v fuser >/dev/null 2>&1; then
  if fuser "${PORT}/tcp" >/dev/null 2>&1; then
    echo "Killing process(es) on port ${PORT}..."
    fuser -k "${PORT}/tcp" 2>/dev/null || true
    sleep 0.5
  fi
elif command -v ss >/dev/null 2>&1; then
  PIDS=$(ss -lptn "sport = :${PORT}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | sort -u || true)
  if [[ -n "${PIDS}" ]]; then
    echo "Killing PID(s) on port ${PORT}: ${PIDS}"
    kill -9 ${PIDS} 2>/dev/null || true
  fi
fi

if ss -lptn 2>/dev/null | grep -q ":${PORT} "; then
  echo "WARNING: port ${PORT} may still be in use. Try: ss -lptn | grep ${PORT}"
  exit 1
fi

echo "Port ${PORT} is free."
