#!/usr/bin/env bash
# Fase 2 — validação estrutural (sem Gazebo).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FOREST="${ROOT}/tools/forest"
fail() { echo "FAIL: $*" >&2; exit 1; }
ok() { echo "OK: $*"; }

echo "=== phase2_validate.sh ==="

[[ -d "${ROOT}/src/localization_mapping_stack/forest_2d_localization" ]] \
  || fail "forest_2d_localization package"
[[ -f "${ROOT}/src/localization_mapping_stack/forest_2d_localization/config/mapper_params_forest_sim.yaml" ]] \
  || fail "mapper_params_forest_sim.yaml"
grep -q 'marble_hd2/base_link' \
  "${ROOT}/src/localization_mapping_stack/forest_2d_localization/config/mapper_params_forest_sim.yaml" \
  || fail "base_frame marble_hd2/base_link"
grep -q 'scan_topic: /sensors/lidar/scan' \
  "${ROOT}/src/localization_mapping_stack/forest_2d_localization/config/mapper_params_forest_sim.yaml" \
  || fail "default scan topic"
grep -q 'LifecycleNode' \
  "${ROOT}/src/localization_mapping_stack/forest_2d_localization/launch/slam_toolbox_online_async.launch.py" \
  || fail "slam_toolbox LifecycleNode"
grep -q 'TRANSITION_ACTIVATE' \
  "${ROOT}/src/localization_mapping_stack/forest_2d_localization/launch/slam_toolbox_online_async.launch.py" \
  || fail "slam_toolbox lifecycle activate"

[[ -f "${ROOT}/src/conf/forest_hybrid_conf/launch/sim_mvp_slam.launch.py" ]] \
  || fail "sim_mvp_slam.launch.py"
grep -q 'use_slam' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "sim_gazebo use_slam"
grep -q 'publish_map_odom_identity' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "publish_map_odom_identity launch arg"

[[ -f "${FOREST}/profiles/sim-slam-nav.yaml" ]] || fail "sim-slam-nav profile"
python3 "${FOREST}/lib/profile.py" validate "${FOREST}/profiles/sim-slam-nav.yaml" \
  || fail "profile validate"
grep -q 'slam_toolbox' "${FOREST}/profiles/sim-slam-nav.yaml" || fail "wait slam_toolbox"
grep -q 'publish_map_odom_identity.*true' \
  "${ROOT}/src/conf/forest_hybrid_conf/launch/sim_mvp_slam.launch.py" \
  || fail "sim_mvp_slam publish_map_odom_identity bootstrap true"

[[ -f "${ROOT}/docs/reports/PHASE2_VERIFICATION.md" ]] || fail "PHASE2_VERIFICATION.md"
ok "phase2 doc"

echo "phase2_validate: PASSED"
