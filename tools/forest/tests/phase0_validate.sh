#!/usr/bin/env bash
# Validação estrutural Fase 0 (sem Gazebo).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DIAG="${ROOT}/tools/diagnostics"
FOREST="${ROOT}/tools/forest"
FAIL=0

fail() { echo "FAIL: $1" >&2; FAIL=1; }
ok() { echo "OK: $1"; }
assert() {
  local name="$1"
  shift
  if "$@"; then ok "$name"; else fail "$name"; fi
}

echo "=== phase0_validate.sh ==="

for f in pose_benchmark.py ekf_latency_analyzer.py pose_metrics_lib.py compare_phase0_reports.py; do
  [[ -f "${DIAG}/${f}" ]] || fail "missing ${f}"
done
ok "diagnostic scripts present"

python3 -m py_compile \
  "${DIAG}/pose_metrics_lib.py" \
  "${DIAG}/pose_benchmark.py" \
  "${DIAG}/ekf_latency_analyzer.py" \
  "${DIAG}/ros_time_util.py" \
  "${DIAG}/gz_world_tf_pick.py" \
  "${DIAG}/compare_phase0_reports.py" \
  || fail "py_compile"
ok "python syntax"

[[ -f "${FOREST}/workflows/test-phase0-benchmark.sh" ]] || fail "test-phase0-benchmark workflow"
[[ -x "${FOREST}/workflows/test-phase0-benchmark.sh" ]] || chmod +x "${FOREST}/workflows/test-phase0-benchmark.sh"
ok "test-phase0-benchmark workflow"
assert "forest test lists phase0-benchmark" bash -c 'forest test -h 2>&1 | grep -q phase0-benchmark'

[[ -f "${FOREST}/profiles/sim-mvp-nav-imu.yaml" ]] || fail "sim-mvp-nav-imu profile"
python3 "${FOREST}/lib/profile.py" validate "${FOREST}/profiles/sim-mvp-nav-imu.yaml" \
  || fail "profile validate imu"
python3 "${FOREST}/lib/profile.py" validate "${FOREST}/profiles/sim-mvp-nav.yaml" \
  || fail "profile validate mvp"
ok "profiles validate"

grep -q 'ekf_mode: wheel_only' "${FOREST}/profiles/sim-mvp-nav.yaml" \
  || fail "sim-mvp-nav must use ekf_mode: wheel_only for Fase 0 baseline"
grep -q 'ekf_mode: local' "${FOREST}/profiles/sim-mvp-nav-imu.yaml" \
  || fail "sim-mvp-nav-imu must use ekf_mode: local for Fase 0 candidate"
ok "profile A/B ekf_mode split"

grep -q 'ekf_mode' "${ROOT}/src/conf/forest_hybrid_conf/launch/sim_mvp.launch.py" \
  || fail "sim_mvp ekf_mode"
grep -q 'ekf_mode' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "sim_gazebo ekf_mode"
grep -q 'ekf_local' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "sim_gazebo ekf_local path"
ok "launch ekf_mode wiring"

grep -q 'pose-benchmark' "${FOREST}/lib/diag.bash" || fail "diag pose-benchmark"
grep -q 'ekf-latency' "${FOREST}/lib/diag.bash" || fail "diag ekf-latency"
grep -q 'lidar3d-stack' "${FOREST}/lib/diag.bash" || fail "diag lidar3d-stack"
[[ -f "${DIAG}/lidar3d_stack_monitor.py" ]] || fail "lidar3d_stack_monitor.py"
grep -q 'publish_lidar_static_tf' "${ROOT}/src/localization_mapping_stack/forest_state_estimation/launch/state_estimation.launch.py" \
  || fail "publish_lidar_static_tf launch arg"
grep -q 'static_sensor_tf_node' "${ROOT}/src/sim_bridge/forest_sim_bridge/launch/sim_gazebo.launch.py" \
  || fail "lidar_tf_early in sim_gazebo"
[[ -f "${FOREST}/workflows/test-lidar3d-w0-w3.sh" ]] || fail "test-lidar3d-w0-w3 workflow"
[[ -f "${DIAG}/lidar3d_world_validation.py" ]] || fail "lidar3d_world_validation.py"
[[ -f "${DIAG}/config/lidar3d_w0_w3_worlds.json" ]] || fail "lidar3d_w0_w3_worlds.json"
grep -q 'publish_debug_stats' "${ROOT}/src/perception_stack/forest_3d_perception/config/forest_3d_segmentation.yaml" \
  || fail "forest_3d_segmentation.yaml missing publish_debug_stats"
grep -q 'ground_connectivity_enable' "${ROOT}/src/perception_stack/forest_3d_perception/config/forest_3d_segmentation.yaml" \
  || fail "ground_connectivity_enable in segmentation yaml"
ok "forest diag + LiDAR 3D Fase 0 + W0-W3 validation"

[[ -f "${ROOT}/docs/reports/PHASE0_VERIFICATION.md" ]] || fail "PHASE0_VERIFICATION.md"
ok "verification doc"

if [[ "$FAIL" -ne 0 ]]; then
  echo "phase0_validate: FAILED"
  exit 1
fi
echo "phase0_validate: PASSED"
exit 0
