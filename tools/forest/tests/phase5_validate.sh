#!/usr/bin/env bash
# Phase 5 — CI workflow, headless flags, test-goto workflow.
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

echo "=== Forest Phase 5 validation ==="

assert "CI workflow file" test -f "$HYBRID_WS/.github/workflows/forest-ci.yml"
assert "forest documents --headless" bash -c 'forest help 2>&1 | grep -q headless'
assert "test-goto workflow exists" test -f "$HYBRID_WS/tools/forest/workflows/test-goto.sh"
assert "forest test lists goto" bash -c 'forest test 2>&1 | grep -q goto'

assert "headless sets use_rviz false" python3 -c '
import os
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / "tools/forest/lib"))
from profile import load_profile
os.environ["FOREST_LAUNCH_OVERRIDES"] = "use_rviz:=false"
p = load_profile(Path(sys.argv[1]) / "tools/forest/profiles/sim-mvp-nav.yaml")
args = p["layers"][0]["args"]
assert args.get("use_rviz") is False, args
' "$HYBRID_WS"

echo ""
echo "=== Results: ${pass} passed, ${fail} failed ==="
(( fail == 0 )) || exit 1
echo "Phase 5 structural validation OK"
