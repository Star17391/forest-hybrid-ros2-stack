#!/usr/bin/env bash
# Phase 4 — diagnostics in tools/diagnostics + forest diag dispatcher.
set -euo pipefail

HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
export PATH="${HYBRID_WS}/tools/forest/bin:${PATH}"
export HYBRID_WS

pass=0
fail=0

assert() {
  local name="$1"
  shift
  echo ""
  echo ">>> TEST: $name"
  if "$@"; then
    echo "    PASS: $name"
    pass=$((pass + 1))
  else
    echo "    FAIL: $name" >&2
    fail=$((fail + 1))
    return 1
  fi
}

echo "=== Forest Phase 4 validation ==="

assert "tools/diagnostics exists" test -d "$HYBRID_WS/tools/diagnostics"
assert "analyze_imu_stream in tools" test -f "$HYBRID_WS/tools/diagnostics/analyze_imu_stream.py"
assert "forest diag lists imu" bash -c 'forest diag 2>&1 | grep -q imu'
assert "forest diag lists tf-audit" bash -c 'forest diag 2>&1 | grep -q tf-audit'
assert "scripts/diagnostics removed" test ! -f "$HYBRID_WS/scripts/diagnostics/analyze_imu_stream.py"

assert "profile overrides parse" python3 -c '
import os
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / "tools/forest/lib"))
from profile import parse_launch_overrides, merge_launch_overrides, load_profile
o = parse_launch_overrides("use_rviz:=false,paused:=true")
assert o["use_rviz"] is False and o["paused"] is True
os.environ["FOREST_ALLOW_LEGACY"] = "1"
p = load_profile(Path(sys.argv[1]) / "tools/forest/profiles/legacy/sim-mvp-nav.yaml")
' "$HYBRID_WS"

assert "ekf-se3-config validator" test -f "$HYBRID_WS/tools/diagnostics/ekf_se3_config_validate.py"
assert "ekf-se3-config gate passes" python3 "$HYBRID_WS/tools/diagnostics/ekf_se3_config_validate.py" --repo "$HYBRID_WS"

echo ""
echo "=== Results: ${pass} passed, ${fail} failed ==="
(( fail == 0 )) || exit 1
echo "Phase 4 structural validation OK"
