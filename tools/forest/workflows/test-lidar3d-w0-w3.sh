#!/usr/bin/env bash
# Validação incremental W0–W3: arranca cada mundo, diagnostica, gera relatório.
#
# Uso (um comando):
#   forest test lidar3d-w0-w3
#   forest test lidar3d-w0-w3 --duration 35 --no-rviz
#   forest test lidar3d-w0-w3 --random-move   # robô explora durante cada mundo
#   forest test lidar3d-w0-w3 --trunk-slice   # pipeline slice+continuity (recomendado)
#   forest test lidar3d-w0-w3 --trunk-column  # pipeline nDSM+column (legado)
#
# Opções:
#   --duration SEC     Segundos de diagnóstico por mundo (default: 40)
#   --startup-timeout  Espera máx. arranque Gazebo/nós (default: 150)
#   --only W0,W3       Só estes mundos
#   --no-rviz          Sem RViz (CI / servidor)
#   --random-move      Arranca forest_random_explore em background por mundo
#   --skip-down        Não faz forest down no início (debug)
#   --keep-last        Não faz forest down no fim
#
# Saída: docs/reports/lidar3d_validation/YYYYMMDD_HHMMSS/
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HYBRID_WS="$(cd "${FOREST_ROOT}/../.." && pwd)"
DIAG="${HYBRID_WS}/tools/diagnostics"
WORLDS_JSON="${DIAG}/config/lidar3d_w0_w3_worlds.json"

# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
# shellcheck source=../lib/wait_for_clock.bash
source "${FOREST_ROOT}/lib/wait_for_clock.bash"
# shellcheck source=../lib/launch.bash
source "${FOREST_ROOT}/lib/launch.bash"

DURATION=40
STARTUP_TIMEOUT=150
ONLY_WORLDS=""
SKIP_INITIAL_DOWN=false
KEEP_LAST=false
USE_RVIZ=true
RANDOM_MOVE=false
TRUNK_COLUMN=false
TRUNK_SLICE=false
PROFILE="sim-lidar3d-test"
RANDOM_MOVE_PID=""

usage() {
  cat <<EOF
forest test lidar3d-w0-w3 — validação automática W0–W3 (LiDAR 3D)

Corre cada mundo em sequência:
  W0 mvp_empty_flat
  W1 forest_gentle_trees_rocks
  W2 forest_rugged
  W3 forest_rugged_trees_rocks

Requer: colcon build forest_3d_perception, Gazebo/GPU disponível.

Options:
  --duration SEC        Diagnóstico por mundo (default: 40)
  --startup-timeout SEC Espera arranque (default: 150)
  --only W0,W2          Subconjunto de mundos
  --no-rviz             Desactiva RViz (default: RViz ligado)
  --random-move         Movimento aleatório contínuo durante cada mundo
  --trunk-slice         Activa trunk_method=slice (fatias+continuidade+cylinder)
  --trunk-column        Activa trunk_method=column (nDSM+column legado)
  --skip-down           Não forest down no início
  --keep-last           Deixa sessão do último mundo activa
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --startup-timeout) STARTUP_TIMEOUT="$2"; shift 2 ;;
    --only) ONLY_WORLDS="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --random-move) RANDOM_MOVE=true; shift ;;
    --trunk-slice) TRUNK_SLICE=true; shift ;;
    --trunk-column) TRUNK_COLUMN=true; shift ;;
    --skip-down) SKIP_INITIAL_DOWN=true; shift ;;
    --keep-last) KEEP_LAST=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "${WORLDS_JSON}" ]]; then
  echo "ERROR: missing ${WORLDS_JSON}" >&2
  exit 2
fi

if ! command -v forest >/dev/null 2>&1; then
  export PATH="${FOREST_ROOT}/bin:${PATH}"
fi

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${HYBRID_WS}/docs/reports/lidar3d_validation/${RUN_STAMP}"
mkdir -p "${RUN_DIR}"

echo "=== lidar3d-w0-w3 validation ==="
echo "Run directory: ${RUN_DIR}"
echo "Duration per world: ${DURATION}s"
echo "RViz: ${USE_RVIZ}"
echo "Random move: ${RANDOM_MOVE}"
echo "Trunk slice: ${TRUNK_SLICE}"
echo "Trunk column: ${TRUNK_COLUMN}"
echo ""

forest_stop_random_move() {
  if [[ -n "${RANDOM_MOVE_PID:-}" ]] && kill -0 "${RANDOM_MOVE_PID}" 2>/dev/null; then
    kill "${RANDOM_MOVE_PID}" 2>/dev/null || true
    wait "${RANDOM_MOVE_PID}" 2>/dev/null || true
  fi
  RANDOM_MOVE_PID=""
}

forest_start_random_move() {
  forest_stop_random_move
  if [[ "${RANDOM_MOVE}" != "true" ]]; then
    return 0
  fi
  forest_source_ros || return 1
  ros2 run forest_sim_bridge forest_random_explore >/tmp/forest_random_explore.log 2>&1 &
  RANDOM_MOVE_PID=$!
  echo "  random_move pid=${RANDOM_MOVE_PID} (log: /tmp/forest_random_explore.log)"
  sleep 1
}

if [[ "${SKIP_INITIAL_DOWN}" != "true" ]]; then
  forest_log_section "Initial cleanup"
  forest down --force 2>/dev/null || forest down 2>/dev/null || true
  sleep 2
fi

run_one_world() {
  local wid="$1"
  local wname="$2"
  local purpose="$3"
  local out_sub="${RUN_DIR}/${wid}_${wname}"

  forest_log_section "${wid}: ${wname}"
  echo "  Purpose: ${purpose}"
  echo ""

  forest down --force 2>/dev/null || forest down 2>/dev/null || true
  sleep 2

  unset FOREST_LAUNCH_OVERRIDES
  unset FOREST_LIDAR3D_TRUNK_COLUMN
  unset FOREST_LIDAR3D_TRUNK_SLICE
  unset FOREST_LIDAR3D_SEG_CONFIG
  local rviz_flag="true"
  [[ "${USE_RVIZ}" == "true" ]] || rviz_flag="false"
  export FOREST_LAUNCH_OVERRIDES="lidar_mode:=3d,use_rviz:=${rviz_flag},paused:=false"
  seg_share="${HYBRID_WS}/install/forest_3d_perception/share/forest_3d_perception"
  if [[ "${TRUNK_SLICE}" == "true" ]]; then
    export FOREST_LIDAR3D_TRUNK_SLICE=1
    if [[ -d "${seg_share}/config" ]]; then
      export FOREST_LIDAR3D_SEG_CONFIG="${seg_share}/config/forest_3d_segmentation.yaml"
    fi
    echo "  trunk pipeline: slice (FOREST_LIDAR3D_TRUNK_SLICE=1)"
  elif [[ "${TRUNK_COLUMN}" == "true" ]]; then
    export FOREST_LIDAR3D_TRUNK_COLUMN=1
    if [[ -d "${seg_share}/config" ]]; then
      export FOREST_LIDAR3D_SEG_CONFIG="${seg_share}/config/forest_3d_segmentation.yaml"
    fi
    echo "  trunk pipeline: column (FOREST_LIDAR3D_TRUNK_COLUMN=1)"
  fi

  if [[ "${USE_RVIZ}" == "true" && -z "${DISPLAY:-}" ]]; then
    echo "ERROR: RViz pedido mas DISPLAY não definido (usa --no-rviz em CI)" >&2
    return 1
  fi

  mkdir -p "${out_sub}"
  local -a up_args=(-d --lidar3d --world "${wname}")
  [[ "${USE_RVIZ}" == "true" ]] || up_args+=(--headless)
  if ! forest up "${PROFILE}" "${up_args[@]}"; then
    echo "ERROR: forest up failed for ${wname}" >&2
    echo "{\"world_id\":\"${wid}\",\"world_name\":\"${wname}\",\"verdict\":\"FAIL\",\"issues\":[\"forest_up_failed\"]}" \
      > "${out_sub}/world_summary.json"
    return 1
  fi

  forest_source_ros || return 1

  if ! forest_wait_for_clock 90; then
    echo "WARNING: /clock slow — continuing" >&2
  fi

  export FOREST_NODE_WAIT_TIMEOUT="${STARTUP_TIMEOUT}"
  if ! forest_wait_for_nodes \
    ekf_filter_node \
    lidar_scan_classify_node \
    lidar3d_segmentation_node; then
    echo "WARNING: nodes timeout — continuing" >&2
  fi

  echo "Waiting for first LiDAR cloud..."
  local waited=0
  while (( waited < 90 )); do
    if timeout 3 ros2 topic echo /sensors/lidar/points --once >/dev/null 2>&1; then
      echo "  LiDAR points OK (${waited}s)"
      break
    fi
    sleep 2
    ((waited += 2)) || true
  done
  if (( waited >= 90 )); then
    echo "ERROR: no /sensors/lidar/points" >&2
    mkdir -p "${out_sub}"
    echo "{\"world_id\":\"${wid}\",\"world_name\":\"${wname}\",\"verdict\":\"FAIL\",\"issues\":[\"no_lidar_points\"]}" \
      > "${out_sub}/world_summary.json"
    forest down 2>/dev/null || true
    return 1
  fi

  sleep 5
  mkdir -p "${out_sub}"

  forest_start_random_move || true

  set +e
  python3 "${DIAG}/lidar3d_world_validation.py" \
    --world-id "${wid}" \
    --world-name "${wname}" \
    --duration "${DURATION}" \
    --out-dir "${out_sub}"
  local vcode=$?
  set -e

  forest_stop_random_move

  if [[ -f "${FOREST_SESSION_FILE:-}" ]]; then
    local log_dir
    log_dir="$(python3 -c "import json; print(json.load(open('${FOREST_SESSION_FILE}'))['log_dir'])" 2>/dev/null || echo '')"
    if [[ -n "${log_dir}" && -d "${log_dir}" ]]; then
      cp -a "${log_dir}" "${out_sub}/session_logs" 2>/dev/null || true
    fi
  fi

  forest down 2>/dev/null || true
  sleep 3

  return "${vcode}"
}

# Parse worlds from JSON
mapfile -t WORLD_LINES < <(python3 - "${WORLDS_JSON}" "${ONLY_WORLDS}" <<'PY'
import json, sys
path, only = sys.argv[1], sys.argv[2].strip()
data = json.load(open(path))
only_set = {x.strip() for x in only.split(",") if x.strip()}
for w in data["worlds"]:
    if only_set and w["id"] not in only_set:
        continue
    print(f"{w['id']}\t{w['world']}\t{w['purpose']}")
PY
)

if [[ ${#WORLD_LINES[@]} -eq 0 ]]; then
  echo "ERROR: no worlds selected" >&2
  exit 2
fi

FAIL=0
for line in "${WORLD_LINES[@]}"; do
  IFS=$'\t' read -r wid wname purpose <<<"${line}"
  if ! run_one_world "${wid}" "${wname}" "${purpose}"; then
    FAIL=1
  fi
done

forest_log_section "Aggregate report"
python3 "${DIAG}/lidar3d_w0_w3_aggregate.py" "${RUN_DIR}" || FAIL=1

forest_stop_random_move
if [[ "${KEEP_LAST}" != "true" ]]; then
  forest down 2>/dev/null || true
fi

echo ""
echo "Done. Results: ${RUN_DIR}"
echo "  Read: ${RUN_DIR}/RUN_REPORT.md"
if [[ "${FAIL}" -ne 0 ]]; then
  echo "Some worlds FAILED — see EXPECTED_OUTPUT.md (troncos W1/W3 são WARN até Fase 1c)."
  exit 1
fi
exit 0
