#!/usr/bin/env bash
# Fase 1 — validação estrutural (sem Gazebo obrigatório).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FAIL=0

fail() { echo "FAIL: $1" >&2; FAIL=1; }
ok() { echo "OK: $1"; }

echo "=== phase1_validate.sh ==="

[[ -f "${ROOT}/src/drivers_stack/forest_lidar_preprocess_cpp/include/forest_lidar_preprocess_cpp/palacin_ground_line.hpp" ]] \
  || fail "palacin_ground_line.hpp"
ok "palacin header"

python3 "${ROOT}/tools/diagnostics/test_palacin_ground_fit.py" || fail "synthetic palacin tests"
ok "test_palacin_ground_fit.py"

grep -q 'palacin_v1' "${ROOT}/src/drivers_stack/forest_lidar_preprocess_cpp/config/forest_lidar_preprocess.yaml" \
  || fail "yaml palacin_v1"
grep -q 'scan_hazard' "${ROOT}/src/drivers_stack/forest_lidar_preprocess_cpp/config/forest_lidar_preprocess.yaml" \
  || fail "yaml scan_hazard"
ok "config yaml"

grep -q 'kLabelHole' "${ROOT}/src/drivers_stack/forest_lidar_preprocess_cpp/src/lidar_scan_classify_node.cpp" \
  || fail "hole label in node"
grep -q 'fit_ground_line_ransac' "${ROOT}/src/drivers_stack/forest_lidar_preprocess_cpp/src/lidar_scan_classify_node.cpp" \
  || fail "ransac usage"
ok "classify node sources"

[[ -f "${ROOT}/tools/diagnostics/lidar_classify_audit.py" ]] || fail "lidar_classify_audit"
grep -q 'lidar-classify' "${ROOT}/tools/forest/lib/diag.bash" || fail "diag lidar-classify"
ok "diagnostics"

[[ -f "${ROOT}/docs/reports/PHASE1_VERIFICATION.md" ]] || fail "PHASE1_VERIFICATION.md"
ok "phase1 doc"

if [[ "$FAIL" -ne 0 ]]; then
  echo "phase1_validate: FAILED"
  exit 1
fi
echo "phase1_validate: PASSED"
exit 0
