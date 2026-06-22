#!/usr/bin/env bash
# forest test ekf-se3 — valida o estimador SE(3) com Gazebo + ArduPilot SITL.
#
# Arquitetura: UM EKF (ekf_local, odom→base) + autoridade map→odom comutada por modo
# (solo=identidade, ar=ArduPilot direto). Sem ekf_global. O hybrid_hop_executor publica
# /ardupilot/local_position_odom a partir da porta 5760 (stream fiável).
#
# Stack lançado (pela ordem correcta):
#   1. Gazebo (mvp_morph_flight.sdf)
#   2. ros_gz_bridge  — /clock + sensores + lagartas
#   3. gz_track_odometry_stamp  — /…_raw → /…  (parent=odom; para o EKF local)
#   4. state_estimation SE3  — ekf_local + map_odom_authority_node (+ state_contract)
#   5. hybrid_transition_manager (FSM)  — publica /system/locomotion_mode (transient_local)
#   6. RViz (forest_ekf_se3.rviz)
#   7. ArduPilot SITL  — porta 5760
#   8. hybrid_hop_executor  — controla voo + publica /ardupilot/local_position_odom (5760)
#
# Verificações (monitor em tempo real durante o salto):
#   check-0 (pré-voo) : TF odom→base_link sem NaN
#   check-1 (pré-voo) : /state/odometry sem NaN
#   check-2 (AERIAL)  : map→base resolve + autoridade map→odom dinâmica
#   check-3 (AERIAL)  : z(TF map→base) sobe (~altitude de cruzeiro)
#   check-4           : salto completo (DONE)
#   check-5 (pós-salto): posição final vs alvo (LX, LY) — WARN se fora de tol
#
# Uso: forest test ekf-se3 [--assert] [--headless] [--no-rviz] [--keep]
#        [--timeout SEC] [--lx X] [--ly Y] [--alt M] [--pos-tol M]
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"
REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
AP_ENV="${REPO_ROOT}/scripts/autopilot/autopilot_env.sh"
PARM="${REPO_ROOT}/scripts/autopilot/params/forest_hybrid_flight.parm"

# ── Defaults ──────────────────────────────────────────────────────────────
ASSERT=false; GUI=true; RVIZ=true; KEEP=false
TIMEOUT=300
LX=3.5; LY=3.5; ALT=3.0
POS_TOL=1.2
WORLD="${HOME}/Projetos/Gazebo/ForestGen/worlds/mvp_morph_flight.sdf"

# ── Helpers (QoS + TF parsing) ───────────────────────────────────────────
read_locomotion_mode() {
  timeout 4 ros2 topic echo /system/locomotion_mode --once \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
    | sed -n 's/^mode_name: //p' | head -1 | tr -d '"' || true
}

read_fsm_state() {
  timeout 4 ros2 topic echo /forest_gen/hybrid/transition_status --once \
    --qos-reliability reliable 2>/dev/null \
    | sed -n 's/^state_name: //p' | head -1 || true
}

tf_map_odom_sample() {
  timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -12 || true
}

# Extrai "x y z" da linha "- Translation: [x, y, z]"
tf_extract_xyz() {
  local tf_out="$1"
  local line
  line="$(echo "$tf_out" | grep -m1 'Translation:' || true)"
  [[ -z "$line" ]] && return 1
  echo "$line" | sed -E 's/.*\[([^]]+)\].*/\1/' \
    | tr ',' ' ' | sed -E 's/[^0-9.+-eE ]//g'
}

tf_xyz_magnitude() {
  local xyz="$1"
  local x y z
  read -r x y z <<< "$xyz"
  awk -v x="${x:-0}" -v y="${y:-0}" -v z="${z:-0}" \
    'BEGIN { printf "%.6f", sqrt(x*x+y*y+z*z) }'
}

is_aerial_fsm_state() {
  case "${1:-}" in
    AERIAL_READY|AERIAL_FLY|AERIAL_HOVER) return 0 ;;
    *) return 1 ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert)    ASSERT=true ;;
    --gui)       GUI=true; RVIZ=true ;;
    --headless)  GUI=false; RVIZ=false ;;
    --no-rviz)   RVIZ=false ;;
    --keep)      KEEP=true ;;
    --timeout)   shift; TIMEOUT="${1:?}" ;;
    --lx)        shift; LX="${1:?}" ;;
    --ly)        shift; LY="${1:?}" ;;
    --alt)       shift; ALT="${1:?}" ;;
    --pos-tol)   shift; POS_TOL="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test ekf-se3 [--assert] [--headless] [--no-rviz] [--keep]"
      echo "  [--timeout SEC] [--lx X] [--ly Y] [--alt M] [--pos-tol M]"
      echo ""
      echo "  Valida o estimador SE(3) com Gazebo + ArduPilot SITL."
      echo "  Lança o stack completo (EKF local + EKF global + SITL + salto híbrido)."
      echo "  Verifica TF sem NaN, /state/odometry, autoridade map→odom em AERIAL,"
      echo "  z do EKF, DONE do salto, e posição final vs alvo."
      exit 0 ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── Pré-requisitos ────────────────────────────────────────────────────────
AP_BIN="${ARDUPILOT_DIR:-$HOME/ardupilot}/build/sitl/bin/arducopter"
[[ -x "$AP_BIN" ]] || {
  echo "ERROR: ArduPilot SITL em falta ($AP_BIN)." >&2
  echo "  Instala: bash scripts/autopilot/install_ardupilot_sitl.sh" >&2
  exit 1
}
[[ -f "$WORLD" ]] || { echo "ERROR: mundo não encontrado: $WORLD" >&2; exit 1; }
forest_source_ros || { echo "ERROR: não consegui sourcing ROS/workspace" >&2; exit 1; }
[[ -f "$AP_ENV" ]] && source "$AP_ENV"
command -v sim_vehicle.py >/dev/null || {
  echo "ERROR: sim_vehicle.py não no PATH (source $AP_ENV)" >&2; exit 1
}

export GZ_PARTITION="${GZ_PARTITION:-forest_ekfse3_$$}"
export GZ_SIM_RESOURCE_PATH="${HOME}/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"

BRIDGE_YAML="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/marble_bridges_hybrid.yaml"
RVIZ_CFG="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/config/forest_ekf_se3.rviz"
EKF_CFG="$(ros2 pkg prefix forest_state_estimation 2>/dev/null)/share/forest_state_estimation/config/ekf_local.yaml"
RVIZ_MAX="$(ros2 pkg prefix forest_sim_bridge 2>/dev/null)/share/forest_sim_bridge/scripts/rviz_maximize.sh"

LOGDIR="$(mktemp -d "${FOREST_STATE_DIR:-/tmp}/ekfse3.XXXXXX")"
PIDS=()

cleanup() {
  if [[ "${KEEP:-false}" == "true" ]]; then
    echo "[cleanup] --keep: processos mantidos; logs: $LOGDIR"
    return
  fi
  echo "[cleanup] a terminar processos…"
  for p in "${PIDS[@]:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done
  pkill -TERM -x arducopter 2>/dev/null || true
  pkill -TERM -f 'sim[_]vehicle' 2>/dev/null || true
  sleep 1
  pkill -KILL -x arducopter 2>/dev/null || true
  pkill -KILL -f 'sim[_]vehicle' 2>/dev/null || true
  fuser -k 5760/tcp 2>/dev/null || true
  fuser -k 5763/tcp 2>/dev/null || true
  pkill -KILL -f 'gz[ ]sim' 2>/dev/null || true
  sleep 1
  echo "  logs: $LOGDIR"
}
trap cleanup EXIT

forest_log_section "EKF SE(3) — Gazebo + ArduPilot SITL (partition $GZ_PARTITION)"
echo "  Aterragem: (${LX},${LY}) altitude ${ALT}m | GUI=${GUI} RViz=${RVIZ}"
echo "  pos_tol=${POS_TOL}m | logs: $LOGDIR"

# ── 1. Gazebo ─────────────────────────────────────────────────────────────
GZ_FLAGS="-s -r"; [[ "$GUI" == "true" ]] && GZ_FLAGS="-r"
echo "[1/8] Gazebo (mvp_morph_flight)"
# shellcheck disable=SC2086
gz sim $GZ_FLAGS "$WORLD" > "$LOGDIR/gz.log" 2>&1 &
PIDS+=($!); sleep 8
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
  echo "ERROR: Gazebo morreu no arranque. tail:" >&2
  tail -10 "$LOGDIR/gz.log" >&2; exit 1
fi

# ── 2. ros_gz_bridge ──────────────────────────────────────────────────────
echo "[2/8] ros_gz_bridge (/clock + sensores + lagartas)"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_YAML" \
  > "$LOGDIR/bridge.log" 2>&1 &
PIDS+=($!); sleep 4

# ── 3. gz_track_odometry_stamp ────────────────────────────────────────────
echo "[3/8] gz_track_odometry_stamp (raw → clean odometry, parent=odom)"
ros2 run forest_sim_bridge gz_track_odometry_stamp --ros-args \
  -p use_sim_time:=true -p parent_frame:=odom \
  > "$LOGDIR/odom_stamp.log" 2>&1 &
PIDS+=($!); sleep 2

# ── 4. State estimation SE3 ───────────────────────────────────────────────
echo "[4/8] State estimation SE(3)  (EKF local + EKF global + authority)"
# Arquitetura UM EKF + autoridade: ekf_local (sempre) + map_odom_authority_node (sempre).
# A autoridade publica map→odom identidade no solo (ground_mode) e pose do AP no ar.
ros2 launch forest_state_estimation state_estimation.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  use_wheel_odom:=true \
  use_lidar_preprocess:=false \
  publish_lidar_static_tf:=false \
  ekf_config:="$EKF_CFG" \
  > "$LOGDIR/state_est.log" 2>&1 &
PIDS+=($!); sleep 3

# ── 5. hybrid_transition_manager (FSM) ────────────────────────────────────
echo "[5/8] hybrid_transition_manager (FSM)"
ros2 run forest_sim_bridge hybrid_transition_manager --ros-args -p use_sim_time:=true \
  -p rotate_tracks_for_aerial:=true -p spawn_z_m:=0.2 -p airborne_z_threshold_m:=0.55 \
  > "$LOGDIR/fsm.log" 2>&1 &
PIDS+=($!); sleep 3

# ── 6. RViz ──────────────────────────────────────────────────────────────
if [[ "$RVIZ" == "true" ]]; then
  echo "[6/8] RViz (forest_ekf_se3.rviz — EKF + salto híbrido)"
  if [[ ! -f "$RVIZ_CFG" ]]; then
    echo "  WARN: $RVIZ_CFG em falta — rebuild forest_sim_bridge" >&2
  fi
  rviz2 -d "$RVIZ_CFG" > "$LOGDIR/rviz.log" 2>&1 &
  PIDS+=($!)
  if [[ -x "$RVIZ_MAX" ]]; then
    bash "$RVIZ_MAX" > "$LOGDIR/rviz_maximize.log" 2>&1 &
  fi
else
  echo "[6/8] RViz (skip --no-rviz)"
fi
sleep 2

# ── 7. ArduPilot SITL ─────────────────────────────────────────────────────
echo "[7/8] ArduPilot SITL (porta 5760 + 5763)"
FRAME_PARM="${ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params/gazebo-iris.parm"
ADD_PARM="--add-param-file=$FRAME_PARM"
[[ -f "$PARM" ]] && ADD_PARM="$ADD_PARM --add-param-file=$PARM"
# shellcheck disable=SC2086
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w $ADD_PARM \
  --no-mavproxy --no-rebuild -I0 \
  --out tcp:0.0.0.0:5763 \
  > "$LOGDIR/sitl.log" 2>&1 &
PIDS+=($!)
echo "    a aguardar SITL (TCP 5760+5763; -w faz wipe+reboot)…"
sleep 18

# ── 8. hybrid_hop_executor (publica também /ardupilot/local_position_odom) ──
# O executor (porta 5760, stream sustentado) publica a odometria do AP que a
# autoridade map→odom consome no ar. Substitui o mavlink_state_bridge_node (a porta
# 5763 não sustentava o stream de LOCAL_POSITION_NED → odometria congelada).
echo "[8/8] hybrid_hop_executor (C++) — MAVLink porta 5760 + /ardupilot/local_position_odom"
ros2 run forest_hybrid_flight hybrid_hop_executor_node --ros-args -p use_sim_time:=true \
  > "$LOGDIR/hop.log" 2>&1 &
PIDS+=($!); sleep 4

# ── check-0: TF odom→base_link (pré-voo) ─────────────────────────────────
echo ""
echo "[check-0] TF odom→base_link sem NaN (EKF local, pré-voo)"
ok_tf=false
for _ in $(seq 1 15); do
  tf_out="$(timeout 2 ros2 run tf2_ros tf2_echo odom marble_hd2/base_link 2>/dev/null \
    | head -8 || true)"
  if echo "$tf_out" | grep -q "Translation:"; then
    if echo "$tf_out" | grep -qv "nan"; then
      ok_tf=true
      echo "  ✓ odom→marble_hd2/base_link OK (EKF local activo)"
      break
    else
      echo "  ✗ NaN no TF odom→base_link!"
    fi
  fi
  sleep 2
done
[[ "$ok_tf" == "true" ]] || {
  echo "  FAIL: TF odom→base_link ausente ou NaN"
  echo "  Diagnóstico:"
  echo "    ekf_local log: $(tail -5 "$LOGDIR/state_est.log" | head -5)"
  echo "    odom_stamp log: $(tail -3 "$LOGDIR/odom_stamp.log")"
  [[ "$ASSERT" == "true" ]] && exit 1
}

# ── check-1: /state/odometry sem NaN (pré-voo) ────────────────────────────
echo "[check-1] /state/odometry sem NaN (EKF, pré-voo)"
ok_odom=false
for _ in $(seq 1 8); do
  odom_out="$(timeout 2 ros2 topic echo /state/odometry --once 2>/dev/null \
    | head -30 || true)"
  if [[ -n "$odom_out" ]]; then
    if echo "$odom_out" | grep -qv "nan"; then
      ok_odom=true
      echo "  ✓ /state/odometry OK (sem NaN, frame_id=$(echo "$odom_out" | grep 'frame_id:' | head -1 | awk '{print $2}'))"
      break
    else
      echo "  ✗ NaN em /state/odometry!"
    fi
  fi
  sleep 2
done
[[ "$ok_odom" == "true" ]] || echo "  WARN: /state/odometry ausente antes do salto"

# Baseline map→odom (sem identidade estática — authority só em AERIAL)
pre_map_tf="$(tf_map_odom_sample)"
pre_map_xyz="$(tf_extract_xyz "$pre_map_tf" 2>/dev/null || echo "0 0 0")"
pre_map_mag="$(tf_xyz_magnitude "$pre_map_xyz")"
echo "  baseline map→odom (pré-voo): xyz=(${pre_map_xyz}) |mag|=${pre_map_mag} (esperado ~0 sem static TF)"

# ── Disparar salto ────────────────────────────────────────────────────────
echo ""
echo "A disparar salto (aterrar em ${LX},${LY} @ ${ALT}m)…"
echo "  FSM: hybrid_transition_manager faz GROUND→AERIAL_FLY e publica MODE_AERIAL (latched)"
echo "  Após AERIAL: map_odom_authority_node deve publicar map→odom (≠ só identidade estática)"
publish_hop() {
  ros2 topic pub --once /forest_gen/hybrid/hop_request \
    forest_hybrid_msgs/msg/HybridHopRequest \
    "{command_id: 'ekfse3-test', source: 'forest_test', land_x: ${LX}, land_y: ${LY}, cruise_alt_m: ${ALT}}" \
    >/dev/null 2>&1 || true
}

# Amostradores CONTÍNUOS em background (escrevem para ficheiros). O monitor lê os ficheiros
# instantaneamente — evita a latência de spawnar ros2 CLI por amostra, que perdia fases
# rápidas (CRUISE dura ~4 s). Mortos no cleanup (estão em PIDS).
HOP_F="$LOGDIR/mon_hop.txt"; TFMO_F="$LOGDIR/mon_map_odom.txt"; TFMB_F="$LOGDIR/mon_map_base.txt"
ros2 topic echo /forest_gen/hybrid/hop_status > "$HOP_F" 2>/dev/null & PIDS+=($!)
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true > "$TFMO_F" 2>/dev/null & PIDS+=($!)
ros2 run tf2_ros tf2_echo map marble_hd2/base_link --ros-args -p use_sim_time:=true > "$TFMB_F" 2>/dev/null & PIDS+=($!)

# `|| true` no fim: com set -e + pipefail, um grep sem matches (ficheiro ainda vazio)
# devolveria 1 e a substituição `$(...)` mataria o script. Devolvem sempre 0.
last_phase()  { { grep -a '^phase:'  "$HOP_F" 2>/dev/null | tail -1 | sed 's/^phase: //'  | tr -d ' \r'; } || true; }
last_detail() { { grep -a '^detail:' "$HOP_F" 2>/dev/null | tail -1 | sed 's/^detail: //' | tr -d '"\r'; } || true; }
last_xyz()    { { grep -a 'Translation:' "$1" 2>/dev/null | tail -1 \
                  | sed -E 's/.*\[([^]]+)\].*/\1/' | tr ',' ' '; } || true; }

# ── Monitor em TEMPO REAL desde o disparo (checks 2/3/4) ───────────────────
# Funde disparo e observação: amostra a TF IMEDIATAMENTE e republica o pedido até o
# executor aceitar (só aceita após ligar o MAVLink ~13 s). Nunca perde a janela aérea,
# mesmo com a latência de arranque do `ros2 topic echo`. z máximo rastreado sempre.
echo ""
echo "[check-2+3+4] Monitor em tempo real (map→odom + z + fases) desde o disparo…"
ok_aerial_tf=false; ok_aerial_tf_authority=false; ok_aerial_z=false; ok_hop=false
saw_aerial=false; started=false; aerial_tf_kind="—"; max_base_z="-99"
deadline_hop=$((SECONDS + TIMEOUT)); prev_line=""; i=0
while (( SECONDS < deadline_hop )); do
  i=$((i + 1))
  ph="$(last_phase)"; det="$(last_detail)"
  # republica a cada ~5 s enquanto não arrancou (fase vazia ou IDLE)
  if [[ -z "$ph" || "$ph" == "IDLE" ]]; then
    if (( (i - 1) % 5 == 0 )); then publish_hop; fi
  else
    started=true
  fi

  mode_name="$(read_locomotion_mode)"
  mo_xyz="$(last_xyz "$TFMO_F")"; [[ -z "$mo_xyz" ]] && mo_xyz="0 0 0"
  mo_mag="$(tf_xyz_magnitude "$mo_xyz")"
  mb_xyz="$(last_xyz "$TFMB_F")"
  mb_z="$(echo "$mb_xyz" | awk '{print $3}')"

  # AERIAL: por modo OU por fase do hop (CRUISE/LANDING).
  if [[ "${mode_name:-}" == "aerial" ]]; then saw_aerial=true; fi
  case "$ph" in CRUISE|LANDING) saw_aerial=true ;; esac

  # autoridade dinâmica: map→odom afasta-se da identidade no voo (carrega a altitude)
  if [[ "$saw_aerial" == "true" ]]; then
    [[ -n "$mb_z" && "$mb_z" != "nan" ]] && ok_aerial_tf=true
    if awk -v m="$mo_mag" 'BEGIN { exit (m > 0.08) ? 0 : 1 }'; then
      ok_aerial_tf_authority=true; aerial_tf_kind="dinâmica (|T|>0.08m)"
    fi
  fi
  # z máximo de map→base ao longo de TODA a corrida (pico = altitude de cruzeiro).
  if [[ -n "$mb_z" && "$mb_z" != "nan" ]] \
     && awk -v z="$mb_z" -v mx="$max_base_z" 'BEGIN{exit (z>mx)?0:1}'; then
    max_base_z="$mb_z"
  fi

  line="  [$(date +%H:%M:%S)] hop=${ph:-?} mode=${mode_name:-?} |map→odom|=${mo_mag} z(map→base)=${mb_z:-?}"
  if [[ "$line" != "$prev_line" ]]; then echo "$line"; prev_line="$line"; fi

  case "$ph" in
    DONE)   ok_hop=true; break ;;
    FAILED) echo "  FAIL: hop executor reportou FAILED (${det})"; break ;;
  esac
  sleep 1
done
if [[ "$started" != "true" && "$ok_hop" != "true" ]]; then
  echo "  ✗ salto não arrancou (MAVLink 5760 não ligou?) — ver $LOGDIR/hop.log"
fi

if awk -v z="$max_base_z" 'BEGIN { exit (z > 0.5) ? 0 : 1 }'; then ok_aerial_z=true; fi
echo "  ── resumo do monitor ──"
echo "  z(map→base) máximo observado = ${max_base_z} m"
[[ "$saw_aerial" == "true" ]]            && echo "  ✓ [check-2] AERIAL observado" || echo "  ✗ [check-2] AERIAL nunca observado"
[[ "$ok_aerial_tf" == "true" ]]          && echo "  ✓ [check-2a] map→base resolve em AERIAL (cadeia TF íntegra)" || echo "  ✗ [check-2a] map→base ausente em AERIAL"
[[ "$ok_aerial_tf_authority" == "true" ]] && echo "  ✓ [check-2b] autoridade ${aerial_tf_kind}" || echo "  ✗ [check-2b] map→odom não-dinâmico (autoridade inactiva?)"
[[ "$ok_aerial_z" == "true" ]]           && echo "  ✓ [check-3] z subiu no ar (max ${max_base_z} m)" || echo "  ✗ [check-3] z não subiu (max ${max_base_z} m)"
[[ "$ok_hop" == "true" ]]                && echo "  ✓ [check-4] salto DONE" || echo "  ✗ [check-4] salto não chegou a DONE"

# ── check-5: posição final vs alvo ───────────────────────────────────────
echo ""
echo "[check-5] Posição final vs alvo (${LX}, ${LY}) tol=${POS_TOL}m…"
ok_land_pos=false
final_x=""; final_y=""; final_z=""
for _ in $(seq 1 10); do
  pose_block="$(timeout 2 ros2 topic echo /state/pose_fused --once 2>/dev/null \
    | sed -n '/position:/,/orientation:/p' || true)"
  if [[ -n "$pose_block" ]]; then
    final_x="$(echo "$pose_block" | awk '/x:/ {print $2; exit}')"
    final_y="$(echo "$pose_block" | awk '/y:/ {print $2; exit}')"
    final_z="$(echo "$pose_block" | awk '/z:/ {print $2; exit}')"
    if [[ -n "$final_x" && -n "$final_y" ]]; then
      dist="$(awk -v x="$final_x" -v y="$final_y" -v lx="$LX" -v ly="$LY" \
        'BEGIN { printf "%.3f", sqrt((x-lx)^2 + (y-ly)^2) }')"
      if awk -v d="$dist" -v t="$POS_TOL" 'BEGIN { exit (d <= t) ? 0 : 1 }'; then
        ok_land_pos=true
        echo "  ✓ [check-5] pose=(${final_x}, ${final_y}, ${final_z}) dist=${dist}m"
      else
        echo "  ✗ [check-5] pose=(${final_x}, ${final_y}, ${final_z}) dist=${dist}m > tol=${POS_TOL}m"
      fi
      break
    fi
  fi
  sleep 2
done
[[ -n "$final_x" ]] || echo "  WARN [check-5] /state/pose_fused indisponível"

# ── Resultado ─────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  EKF SE(3) — resultados"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [check-0] TF odom→base_link      : $([ "$ok_tf" == true ] && echo "OK" || echo "FAIL")"
echo "  [check-1] /state/odometry pré-voo: $([ "$ok_odom" == true ] && echo "OK" || echo "WARN")"
echo "  [check-2a] map→odom em AERIAL     : $([ "$ok_aerial_tf" == true ] && echo "OK" || echo "FAIL")"
echo "  [check-2b] autoridade map→odom    : $([ "$ok_aerial_tf_authority" == true ] && echo "OK (${aerial_tf_kind})" || echo "FAIL (${aerial_tf_kind})")"
echo "  [check-3] EKF z em AERIAL         : $([ "$ok_aerial_z" == true ] && echo "OK" || echo "WARN")"
echo "  [check-4] Salto completo (DONE)   : $([ "$ok_hop" == true ] && echo "OK" || echo "FAIL")"
echo "  [check-5] Posição final vs alvo   : $([ "$ok_land_pos" == true ] && echo "OK" || echo "WARN")"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  logs: $LOGDIR"
echo "  rviz: ${RVIZ_CFG}"
echo "  teste: forest test ekf-se3 [--assert] [--no-rviz] [--lx ${LX} --ly ${LY} --alt ${ALT}]"

if [[ "$ok_tf" == "true" && "$ok_aerial_tf" == "true" && "$ok_aerial_tf_authority" == "true" && "$ok_hop" == "true" ]]; then
  echo "SUCCESS: EKF SE(3) — TF + map→odom dinâmico + salto DONE"
  if [[ "$ok_land_pos" != "true" ]]; then
    echo "NOTE: posição final fora de tol — ver check-5 (correcção prevista Fase 1)"
  fi
  exit 0
fi
echo "FAIL: ver logs em $LOGDIR"
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
