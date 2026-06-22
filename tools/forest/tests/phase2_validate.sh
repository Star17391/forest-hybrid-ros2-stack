#!/usr/bin/env bash
# Fase 2 — validação estrutural (sem Gazebo).
# slam_toolbox / sim-slam-nav estão LEGACY — ver docs/LEGACY_PATHS.md.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST="${ROOT}/tools/forest"
LEGACY="${FOREST}/profiles/legacy"
DIAG="${ROOT}/tools/diagnostics"
fail() { echo "FAIL: $*" >&2; exit 1; }
ok() { echo "OK: $*"; }

echo "=== phase2_validate.sh ==="

[[ -d "${ROOT}/src/localization_mapping_stack/forest_2d_localization" ]] \
  || fail "forest_2d_localization package (legacy, frozen)"
grep -q 'LEGACY' "${ROOT}/src/localization_mapping_stack/forest_2d_localization/README.md" \
  || fail "forest_2d_localization README LEGACY banner"
grep -q 'FOREST_ALLOW_LEGACY' \
  "${ROOT}/src/localization_mapping_stack/forest_2d_localization/launch/slam_toolbox_online_async.launch.py" \
  || fail "slam_toolbox FOREST_ALLOW_LEGACY guard"

[[ -f "${ROOT}/src/conf/forest_hybrid_conf/launch/sim_mvp_slam.launch.py" ]] \
  || fail "sim_mvp_slam.launch.py (legacy)"
grep -q 'FOREST_ALLOW_LEGACY' "${ROOT}/src/conf/forest_hybrid_conf/launch/sim_mvp_slam.launch.py" \
  || fail "sim_mvp_slam FOREST_ALLOW_LEGACY guard"
grep -q 'use_slam' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "sim_gazebo use_slam arg"
grep -q 'LEGACY' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "sim_gazebo use_slam LEGACY block"

[[ -f "${LEGACY}/sim-slam-nav.yaml" ]] || fail "legacy sim-slam-nav profile"
grep -q 'status: legacy' "${LEGACY}/sim-slam-nav.yaml" || fail "sim-slam-nav status legacy"
FOREST_ALLOW_LEGACY=1 python3 "${FOREST}/lib/profile.py" validate "${LEGACY}/sim-slam-nav.yaml" \
  || fail "profile validate legacy sim-slam-nav"

python3 "${DIAG}/ekf_se3_config_validate.py" --repo "${ROOT}" || fail "ekf_se3_config_validate"

[[ -f "${ROOT}/docs/LEGACY_PATHS.md" ]] || fail "LEGACY_PATHS.md"
[[ -f "${ROOT}/docs/reports/PHASE2_VERIFICATION.md" ]] || fail "PHASE2_VERIFICATION.md"
ok "phase2 legacy freeze + EKF config gate"

echo "phase2_validate: PASSED"
