#!/usr/bin/env bash
# forest test hybrid-mission-hop — valida a INTEGRAÇÃO do salto na missão de solo.
#
# Cenário: o mission_manager está a executar um GOTO; o caminho de solo "bloqueia"
# (path_blocked disparado manualmente, já que ainda não há deteção real de obstáculos).
# Ao esgotar os replans, o mission_manager autoriza o modo aéreo e dispara um
# HybridHopRequest; o hybrid_hop_executor faz o salto; ao receber HybridHopStatus DONE
# o mission_manager re-planeia a partir da nova posição.
#
# Testa o modo automático (allow_auto_aerial_on_block:=true). Uma só sessão de ArduPilot.
#
# Uso: forest test hybrid-mission-hop [--assert] [--headless] [--keep]
#        [--timeout SEC] [--gx X] [--gy Y] [--alt Z]
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
AP_ENV="${REPO_ROOT}/scripts/autopilot/autopilot_env.sh"
PARM="${REPO_ROOT}/scripts/autopilot/params/forest_hybrid_flight.parm"

ASSERT=false; GUI=true; RVIZ=true; KEEP=false
TIMEOUT=300
GX=3.5; GY=3.5; ALT=3.0
MAX_REPLANS=2
WORLD="${HOME}/Projetos/Gazebo/ForestGen/worlds/mvp_morph_flight.sdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --gui) GUI=true; RVIZ=true ;;
    --headless) GUI=false; RVIZ=false ;;
    --no-rviz) RVIZ=false ;;
    --keep) KEEP=true ;;
    --timeout) shift; TIMEOUT="${1:?}" ;;
    --gx) shift; GX="${1:?}" ;;
    --gy) shift; GY="${1:?}" ;;
    --alt) shift; ALT="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-mission-hop [--assert] [--headless] [--no-rviz] [--keep]"
      echo "  [--timeout SEC] [--gx X] [--gy Y] [--alt Z]"
      echo "  GOTO (GX,GY) → path_blocked → salto automático → re-planeamento."
      exit 0 ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── Pré-requisitos ───────────────────────────────────────────────────────
AP_BIN="${ARDUPILOT_DIR:-$HOME/ardupilot}/build/sitl/bin/arducopter"
[[ -x "$AP_BIN" ]] || { echo "ERROR: ArduPilot SITL em falta." >&2; exit 1; }
[[ -f "$WORLD" ]] || { echo "ERROR: mundo não encontrado: $WORLD" >&2; exit 1; }
forest_source_ros || { echo "ERROR: não consegui sourcing ROS/workspace" >&2; exit 1; }
[[ -f "$AP_ENV" ]] && source "$AP_ENV"
command -v sim_vehicle.py >/dev/null || { echo "ERROR: sim_vehicle.py não no PATH" >&2; exit 1; }

export GZ_PARTITION="${GZ_PARTITION:-forest_mhop_$$}"
export GZ_SIM_RESOURCE_PATH="${HOME}/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"
BRIDGE_YAML="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/marble_bridges_hybrid.yaml"

LOGDIR="$(mktemp -d "${FOREST_STATE_DIR:-/tmp}/hybmhop.XXXXXX")"
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
  [[ "$KEEP" == "false" ]] || echo "[--keep] logs: $LOGDIR"
}
trap cleanup EXIT

forest_log_section "Hybrid mission hop — partition $GZ_PARTITION"
echo "  GOTO (${GX},${GY}) alt ${ALT}m max_replans=${MAX_REPLANS} | GUI=${GUI}"

GZ_FLAGS="-s -r"; [[ "$GUI" == "true" ]] && GZ_FLAGS="-r"
echo "[1/7] Gazebo (mvp_morph_flight)"
gz sim $GZ_FLAGS "$WORLD" > "$LOGDIR/gz.log" 2>&1 &
PIDS+=($!); sleep 8

echo "[2/7] ros_gz_bridge"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_YAML" \
  > "$LOGDIR/bridge.log" 2>&1 &
PIDS+=($!); sleep 4

echo "[3/7] marble_pose_from_gz"
ros2 run forest_sim_bridge marble_pose_from_gz --ros-args -p use_sim_time:=true \
  -p gz_pose_topic:=/world/unified_world/pose/info -p model_name:=marble_hd2 \
  -p parent_frame:=map -p child_frame:=marble_hd2/base_link > "$LOGDIR/pose.log" 2>&1 &
PIDS+=($!)
if [[ "$RVIZ" == "true" ]]; then
  RVIZ_CFG="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/forest_pose_bridge_sim.rviz"
  rviz2 -d "$RVIZ_CFG" > "$LOGDIR/rviz.log" 2>&1 &
  PIDS+=($!)
fi
sleep 2

echo "[4/7] hybrid_transition_manager (FSM)"
ros2 run forest_sim_bridge hybrid_transition_manager --ros-args -p use_sim_time:=true \
  -p rotate_tracks_for_aerial:=true -p spawn_z_m:=0.2 -p airborne_z_threshold_m:=0.55 \
  > "$LOGDIR/fsm.log" 2>&1 &
PIDS+=($!); sleep 3

echo "[5/7] ArduPilot SITL"
FRAME_PARM="${ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params/gazebo-iris.parm"
ADD_PARM="--add-param-file=$FRAME_PARM"
[[ -f "$PARM" ]] && ADD_PARM="$ADD_PARM --add-param-file=$PARM"
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w $ADD_PARM --no-mavproxy --no-rebuild -I0 \
  > "$LOGDIR/sitl.log" 2>&1 &
PIDS+=($!); sleep 16

echo "[6/7] hybrid_hop_executor (C++)"
ros2 run forest_hybrid_flight hybrid_hop_executor_node --ros-args -p use_sim_time:=true \
  > "$LOGDIR/hop.log" 2>&1 &
PIDS+=($!)

echo "[7/7] mission_manager_node (C++, modo aéreo automático)"
ros2 run forest_planner_ros2 mission_manager_node --ros-args -p use_sim_time:=true \
  -p allow_auto_aerial_on_block:=true -p max_replans:="$MAX_REPLANS" \
  -p hop_cruise_alt_m:="$ALT" -p allow_goal_reached_topic_shortcut:=false \
  > "$LOGDIR/mm.log" 2>&1 &
PIDS+=($!)

# ── Esperar o executor ligar ao MAVLink ──────────────────────────────────
echo ""
echo "A aguardar ligação MAVLink do executor…"
for _ in $(seq 1 40); do
  grep -q "MAVLink ligado" "$LOGDIR/hop.log" 2>/dev/null && break
  sleep 1
done
grep -q "MAVLink ligado" "$LOGDIR/hop.log" 2>/dev/null \
  && echo "  MAVLink ligado." || echo "  AVISO: MAVLink pode não estar ligado ainda."

# ── Enviar GOTO e depois bloquear o caminho de solo ──────────────────────
echo "A enviar GOTO (${GX},${GY})…"
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 1, frame_type: 0, command_id: 'test-goto', source: 'forest_test', target_x: ${GX}, target_y: ${GY}, target_z: 0.0}" \
  >/dev/null 2>&1 || true
sleep 3

echo "A bloquear o caminho de solo (path_blocked × $((MAX_REPLANS + 2)))…"
for _ in $(seq 1 $((MAX_REPLANS + 2))); do
  ros2 topic pub --once /planning/path_blocked std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
  sleep 1.5
done

# ── Aguardar o salto integrado concluir (hop DONE) ───────────────────────
echo ""
echo "A aguardar salto integrado + re-planeamento (${TIMEOUT}s)…"
hop_phase() {
  timeout 2 ros2 topic echo /forest_gen/hybrid/hop_status --once 2>/dev/null \
    | sed -n 's/^phase: //p' | head -1 || true
}
ok=false; deadline=$((SECONDS + TIMEOUT))
while (( SECONDS < deadline )); do
  ph="$(hop_phase)"
  [[ -n "$ph" ]] && echo "  [$(date +%H:%M:%S)] hop=$ph"
  case "$ph" in
    DONE)   ok=true; break ;;
    FAILED) echo "FAIL: salto FAILED"; break ;;
  esac
  sleep 2
done

echo ""
echo "=== mission_manager (cola) ==="
grep -E "Salto aéreo|aerial hop|re-planear|path blocked" "$LOGDIR/mm.log" 2>/dev/null | tail -8 || true
echo ""; echo "logs: $LOGDIR"
if [[ "$ok" == "true" ]] && grep -q "re-planear" "$LOGDIR/mm.log" 2>/dev/null; then
  echo "SUCCESS: missão bloqueou → salto automático → re-planeamento"
  exit 0
fi
echo "FAIL: integração não concluída (hop_done=$ok, replan=$(grep -qc 're-planear' "$LOGDIR/mm.log" 2>/dev/null && echo sim || echo nao))"
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
