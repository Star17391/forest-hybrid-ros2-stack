#!/usr/bin/env bash
# forest test hybrid-hop — valida um "salto aéreo" híbrido isolado.
#
# O robô está parado no solo; dispara-se um HybridHopRequest e ele:
#   transforma-se em drone → arma e descola (ArduPilot) → voa até ao ponto de
#   aterragem → pousa controlado → volta ao modo terrestre.
#
# O voo é todo do executor C++ (forest_hybrid_flight/hybrid_hop_executor_node), que
# fala MAVLink direto ao ArduPilot SITL. Uma só sessão de ArduPilot.
#
# Uso: forest test hybrid-hop [--assert] [--headless] [--no-rviz] [--keep]
#        [--timeout SEC] [--lx X] [--ly Y] [--alt Z]
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
AP_ENV="${REPO_ROOT}/scripts/autopilot/autopilot_env.sh"
PARM="${REPO_ROOT}/scripts/autopilot/params/forest_hybrid_flight.parm"

# GUI + RViz activos por omissão (usa --headless para CI sem janela)
ASSERT=false; GUI=true; RVIZ=true; KEEP=false
TIMEOUT=300
LX=3.5; LY=3.5; ALT=3.0
WORLD="${HOME}/Projetos/Gazebo/ForestGen/worlds/mvp_morph_flight.sdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --gui) GUI=true; RVIZ=true ;;
    --headless) GUI=false; RVIZ=false ;;
    --no-rviz) RVIZ=false ;;
    --keep) KEEP=true ;;
    --timeout) shift; TIMEOUT="${1:?}" ;;
    --lx) shift; LX="${1:?}" ;;
    --ly) shift; LY="${1:?}" ;;
    --alt) shift; ALT="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test hybrid-hop [--assert] [--headless] [--no-rviz] [--keep]"
      echo "  [--timeout SEC] [--lx X] [--ly Y] [--alt Z]"
      echo "  Salto: descola onde está → voa até (LX,LY) à altitude ALT → pousa → volta ao solo."
      echo "  Por omissão: Gazebo GUI + RViz abertos. Usa --headless para CI."
      exit 0 ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── Pré-requisitos ───────────────────────────────────────────────────────
AP_BIN="${ARDUPILOT_DIR:-$HOME/ardupilot}/build/sitl/bin/arducopter"
[[ -x "$AP_BIN" ]] || { echo "ERROR: ArduPilot SITL em falta. Corre install_ardupilot_sitl.sh" >&2; exit 1; }
[[ -f "$WORLD" ]] || { echo "ERROR: mundo não encontrado: $WORLD" >&2; exit 1; }
forest_source_ros || { echo "ERROR: não consegui sourcing ROS/workspace" >&2; exit 1; }
[[ -f "$AP_ENV" ]] && source "$AP_ENV"
command -v sim_vehicle.py >/dev/null || { echo "ERROR: sim_vehicle.py não no PATH" >&2; exit 1; }

export GZ_PARTITION="${GZ_PARTITION:-forest_hop_$$}"
export GZ_SIM_RESOURCE_PATH="${HOME}/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"
BRIDGE_YAML="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/marble_bridges_hybrid.yaml"

LOGDIR="$(mktemp -d "${FOREST_STATE_DIR:-/tmp}/hybhop.XXXXXX")"
PIDS=()
cleanup() {
  echo "[cleanup] a terminar processos…"
  for p in "${PIDS[@]:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done
  # ArduPilot: SIGTERM primeiro para flush limpo, depois SIGKILL
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

forest_log_section "Hybrid hop — partition $GZ_PARTITION"
echo "  Aterragem: (${LX},${LY}) altitude ${ALT}m | GUI=${GUI} RViz=${RVIZ}"

# ── 1. Gazebo ────────────────────────────────────────────────────────────
GZ_FLAGS="-s -r"; [[ "$GUI" == "true" ]] && GZ_FLAGS="-r"
echo "[1/6] Gazebo (mvp_morph_flight)"
gz sim $GZ_FLAGS "$WORLD" > "$LOGDIR/gz.log" 2>&1 &
PIDS+=($!); sleep 8

# ── 2. ros_gz_bridge ────────────────────────────────────────────────────
echo "[2/6] ros_gz_bridge"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_YAML" \
  > "$LOGDIR/bridge.log" 2>&1 &
PIDS+=($!); sleep 4

# ── 3. Pose + RViz (opcional) ────────────────────────────────────────────
echo "[3/6] marble_pose_from_gz"
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

# ── 4. FSM ───────────────────────────────────────────────────────────────
echo "[4/6] hybrid_transition_manager (FSM)"
ros2 run forest_sim_bridge hybrid_transition_manager --ros-args -p use_sim_time:=true \
  -p rotate_tracks_for_aerial:=true -p spawn_z_m:=0.2 -p airborne_z_threshold_m:=0.55 \
  > "$LOGDIR/fsm.log" 2>&1 &
PIDS+=($!); sleep 3

# ── 5. ArduPilot SITL ────────────────────────────────────────────────────
echo "[5/6] ArduPilot SITL"
FRAME_PARM="${ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params/gazebo-iris.parm"
ADD_PARM="--add-param-file=$FRAME_PARM"
[[ -f "$PARM" ]] && ADD_PARM="$ADD_PARM --add-param-file=$PARM"
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w $ADD_PARM --no-mavproxy --no-rebuild -I0 \
  > "$LOGDIR/sitl.log" 2>&1 &
PIDS+=($!); sleep 16

# ── 6. Executor do salto (C++) ───────────────────────────────────────────
echo "[6/6] hybrid_hop_executor (C++)"
ros2 run forest_hybrid_flight hybrid_hop_executor_node --ros-args -p use_sim_time:=true \
  > "$LOGDIR/hop.log" 2>&1 &
PIDS+=($!); sleep 4

# ── Disparar o salto (com retry até o MAVLink ligar) ─────────────────────
echo ""
echo "A disparar o salto (aterrar em ${LX},${LY} @ ${ALT}m)…"
hop_phase() {
  timeout 2 ros2 topic echo /forest_gen/hybrid/hop_status --once 2>/dev/null \
    | sed -n 's/^phase: //p' | head -1 || true
}
publish_hop() {
  ros2 topic pub --once /forest_gen/hybrid/hop_request \
    forest_hybrid_msgs/msg/HybridHopRequest \
    "{command_id: 'test-hop', source: 'forest_test', land_x: ${LX}, land_y: ${LY}, cruise_alt_m: ${ALT}}" \
    >/dev/null 2>&1 || true
}

started=false
for _ in $(seq 1 20); do
  publish_hop
  sleep 3
  ph="$(hop_phase)"
  [[ -n "$ph" ]] && echo "  hop_status phase=$ph"
  if [[ -n "$ph" && "$ph" != "IDLE" ]]; then started=true; break; fi
done
if [[ "$started" != "true" ]]; then
  echo "FAIL: o salto não arrancou (MAVLink não ligou?). logs: $LOGDIR"
  [[ "$ASSERT" == "true" ]] && exit 1
  exit 0
fi

# ── Aguardar DONE ────────────────────────────────────────────────────────
echo ""
echo "Salto em curso — a aguardar conclusão (${TIMEOUT}s)…"
ok=false; deadline=$((SECONDS + TIMEOUT))
while (( SECONDS < deadline )); do
  ph="$(hop_phase)"
  if [[ -n "$ph" ]]; then
    echo "  [$(date +%H:%M:%S)] phase=$ph"
    case "$ph" in
      DONE)   ok=true; break ;;
      FAILED) echo "FAIL: salto FAILED"; break ;;
    esac
  fi
  st="$(timeout 1 ros2 topic echo /forest_gen/hybrid/transition_status --once 2>/dev/null \
    | sed -n 's/^state_name: //p' | head -1 || true)"
  [[ -n "$st" ]] && echo "  FSM: $st"
  sleep 2
done

echo ""; echo "logs: $LOGDIR"
if [[ "$ok" == "true" ]]; then
  echo "SUCCESS: salto híbrido completo (descola→voa→pousa→volta ao solo)"
  exit 0
fi
echo "FAIL: salto não concluído"
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
