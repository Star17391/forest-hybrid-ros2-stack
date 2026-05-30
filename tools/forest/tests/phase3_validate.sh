#!/usr/bin/env bash
# Phase 3 — workflows, wrappers, launch_defaults consistency.
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

echo "=== Forest Phase 3 validation ==="

assert "launch_defaults → all profiles paused false (PLAY)" python3 -c '
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / "tools/forest/lib"))
from profile import load_profile
root = Path(sys.argv[1]) / "tools/forest/profiles"
for name in ("sim-minimal", "sim-pose-bridge", "sim-mvp-nav", "sim-sensors-only"):
    p = load_profile(root / f"{name}.yaml")
    paused = p["layers"][0]["args"].get("paused")
    assert paused is False, f"{name}: paused={paused} (expected PLAY)"
' "$HYBRID_WS"

assert "legacy run_trajectory_following.sh removed" test ! -f "$HYBRID_WS/scripts/run_trajectory_following.sh"

assert "forest test lists patrol-rect" bash -c '
  forest test 2>&1 | grep -q patrol-rect || forest test 2>&1 | grep -q patrol
'

assert "workflow scripts exist" test -x "$HYBRID_WS/tools/forest/workflows/test-patrol-rect.sh"

echo ""
echo "=== Results: ${pass} passed, ${fail} failed ==="
(( fail == 0 )) || exit 1
echo "Phase 3 structural validation OK"
exit 0
