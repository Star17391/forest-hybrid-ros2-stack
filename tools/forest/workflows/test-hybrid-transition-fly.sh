#!/usr/bin/env bash
# forest test hybrid-transition-fly — transição terrestre→voo com FSM NATIVA + ArduPilot.
#
# A hybrid_transition_manager (FSM) faz a sequência mecânica (lock→pernas→rodar lagartas
# ±90°) via ROS→bridge→gz; ao chegar a AERIAL_READY, este workflow arma o ArduPilot (MAVLink)
# e valida o hover. É o M3 "nativo" (sem --pre-rotate por gz direto).
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
AP_ENV="${REPO_ROOT}/scripts/autopilot/autopilot_env.sh"
HOVER_CHECK="${REPO_ROOT}/scripts/autopilot/sitl_hover_check.py"

ASSERT=false; GUI=false; KEEP=false; RVIZ=false
ALT=5.0; HOVER=12.0; TOL=0.8; TIMEOUT=90
WORLD="${REPO_ROOT}/../../Projetos/Gazebo/ForestGen/worlds/mvp_morph_flight.sdf"
[[ -f "$WORLD" ]] || WORLD="$HOME/Projetos/Gazebo/ForestGen/worlds/mvp_morph_flight.sdf"
PARM="${REPO_ROOT}/scripts/autopilot/params/forest_hybrid_flight.parm"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --gui) GUI=true ;;
    --rviz) RVIZ=true; GUI=true ;;
    --alt) shift; ALT="${1:?}" ;;
    --hover) shift; HOVER="${1:?}" ;;
    --tol) shift; TOL="${1:?}" ;;
    --timeout) shift; TIMEOUT="${1:?}" ;;
    --keep) KEEP=true ;;
    -h|--help)
      echo "Usage: forest test hybrid-transition-fly [--assert] [--gui] [--alt M] [--hover SEC] [--tol M] [--keep]"
      echo "  FSM nativa orquestra ground→rodar lagartas; ArduPilot arma em AERIAL_READY."
      exit 0 ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── Pré-requisitos ───────────────────────────────────────────────────
AP_BIN="${ARDUPILOT_DIR:-$HOME/ardupilot}/build/sitl/bin/arducopter"
[[ -x "$AP_BIN" ]] || { echo "ERROR: ArduPilot SITL em falta. Corre install_ardupilot_sitl.sh" >&2; exit 1; }
[[ -f "$WORLD" ]] || { echo "ERROR: mundo não encontrado: $WORLD" >&2; exit 1; }
forest_source_ros || { echo "ERROR: não consegui sourcing ROS/workspace" >&2; exit 1; }
[[ -f "$AP_ENV" ]] && source "$AP_ENV"
command -v sim_vehicle.py >/dev/null || { echo "ERROR: sim_vehicle.py não no PATH" >&2; exit 1; }
VENV_PY="$HOME/venv-ardupilot/bin/python3"; PYBIN="$VENV_PY"; [[ -x "$PYBIN" ]] || PYBIN="python3"

export GZ_PARTITION="${GZ_PARTITION:-forest_fsm_$$}"
export GZ_SIM_RESOURCE_PATH="${HOME}/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"
BRIDGE_YAML="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/marble_bridges_hybrid.yaml"

LOGDIR="$(mktemp -d "${FOREST_STATE_DIR:-/tmp}/fsmfly.XXXXXX")"
PIDS=()
cleanup() {
  echo "[cleanup] a terminar processos…"
  for p in "${PIDS[@]:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done
  pkill -TERM -x arducopter 2>/dev/null || true
  pkill -TERM -f 'sim[_]vehicle' 2>/dev/null || true
  sleep 1
  pkill -KILL -x arducopter 2>/dev/null || true
  pkill -KILL -f 'sim[_]vehicle' 2>/dev/null || true
  fuser -k 5760/tcp 2>/dev/null || true
  pkill -KILL -f 'gz[ ]sim' 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT

forest_log_section "Hybrid transition FLY (FSM nativa + ArduPilot) — partition $GZ_PARTITION"

# ── 1. Gazebo ────────────────────────────────────────────────────────
GZ_FLAGS="-s -r"; [[ "$GUI" == "true" ]] && GZ_FLAGS="-r"
echo "[1/5] Gazebo: gz sim $GZ_FLAGS (mvp_morph_flight)"
# shellcheck disable=SC2086
gz sim $GZ_FLAGS "$WORLD" > "$LOGDIR/gz.log" 2>&1 &
PIDS+=($!); sleep 8

# ── 2. ros_gz_bridge (FSM precisa de joint_states + comanda lagartas/pernas) ──
echo "[2/5] ros_gz_bridge (hybrid)"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_YAML" \
  > "$LOGDIR/bridge.log" 2>&1 &
PIDS+=($!); sleep 4

# ── 2c. RViz (opcional): pose gz→TF + visualização ───────────────────
if [[ "$RVIZ" == "true" ]]; then
  echo "[rviz] marble_pose_from_gz (gz→/state/pose_fused + map→base_link) + RViz"
  ros2 run forest_sim_bridge marble_pose_from_gz --ros-args -p use_sim_time:=true \
    -p gz_pose_topic:=/world/unified_world/pose/info -p model_name:=marble_hd2 \
    -p parent_frame:=map -p child_frame:=marble_hd2/base_link > "$LOGDIR/pose.log" 2>&1 &
  PIDS+=($!)
  RVIZ_CFG="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/forest_pose_bridge_sim.rviz"
  rviz2 -d "$RVIZ_CFG" > "$LOGDIR/rviz.log" 2>&1 &
  PIDS+=($!); sleep 2
fi

# ── 3. FSM ───────────────────────────────────────────────────────────
echo "[3/5] hybrid_transition_manager (FSM)"
ros2 run forest_sim_bridge hybrid_transition_manager --ros-args -p use_sim_time:=true \
  -p rotate_tracks_for_aerial:=true -p spawn_z_m:=0.2 -p airborne_z_threshold_m:=0.55 \
  > "$LOGDIR/fsm.log" 2>&1 &
PIDS+=($!); sleep 3

# ── 4. ArduPilot SITL ────────────────────────────────────────────────
echo "[4/5] ArduPilot SITL (-w + frame parm)"
FRAME_PARM="${ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params/gazebo-iris.parm"
ADD_PARM="--add-param-file=$FRAME_PARM"; [[ -f "$PARM" ]] && ADD_PARM="$ADD_PARM --add-param-file=$PARM"
# shellcheck disable=SC2086
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w $ADD_PARM --no-mavproxy --no-rebuild -I0 \
  > "$LOGDIR/sitl.log" 2>&1 &
PIDS+=($!); sleep 16

# ── 5. Disparar transição e esperar AERIAL_READY ─────────────────────
echo "[5/5] to_aerial → FSM faz a transição mecânica…"
ros2 topic pub --once /forest_gen/hybrid/transition_request std_msgs/msg/String "{data: to_aerial}" >/dev/null 2>&1

echo "    a aguardar FSM chegar a AERIAL_READY/FLY (${TIMEOUT}s)…"
ready=false; deadline=$((SECONDS + TIMEOUT))
while (( SECONDS < deadline )); do
  st="$(timeout 2 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null | sed -n 's/^state_name: //p')"
  [[ -n "$st" ]] && echo "    FSM: $st"
  case "$st" in
    AERIAL_READY|AERIAL_FLY|AERIAL_HOVER) ready=true; break ;;
    FAILED) echo "ERRO: FSM em FAILED (ver fsm.log)"; break ;;
  esac
done
if [[ "$ready" != "true" ]]; then
  echo "FAIL: FSM não chegou a AERIAL_READY"; echo "logs: $LOGDIR"
  [[ "$ASSERT" == "true" ]] && exit 1; exit 0
fi

echo "    lagartas a ±90° → handoff ao ArduPilot (arm + takeoff)…"
set +e
"$PYBIN" "$HOVER_CHECK" --connect tcp:127.0.0.1:5760 --alt "$ALT" --hover-sec "$HOVER" --tol "$TOL"
RC=$?
set -e

echo ""; echo "logs: $LOGDIR"
if [[ "$RC" -eq 0 ]]; then
  echo "SUCCESS: transição terrestre→voo com FSM nativa + ArduPilot (hover estável)"
  exit 0
fi
echo "FAIL: hover não validado"
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
