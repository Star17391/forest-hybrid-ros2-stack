#!/usr/bin/env bash
# Quick semantic runtime validation (mask + fusion rates + flicker).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

echo "Run your camera/segmentation/fusion launch in another terminal."
echo "Then this workflow probes semantic runtime stability."

python3 "$ROOT/tools/diagnostics/semantic_runtime_probe.py" &
P1=$!
python3 "$ROOT/tools/diagnostics/semantic_flicker_probe.py" &
P2=$!

trap 'kill $P1 $P2 2>/dev/null || true' EXIT INT TERM
wait

