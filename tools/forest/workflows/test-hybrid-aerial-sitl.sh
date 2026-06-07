#!/usr/bin/env bash
# forest test hybrid-aerial-sitl — valida hover estável sob ArduPilot SITL + Gazebo.
#
# M1 (default): quad padrão de fábrica (gazebo-iris) → prova que autopiloto+EKF resolvem
# o hover que o controlador em malha aberta não conseguia.
#
# Uso: forest test hybrid-aerial-sitl [--assert] [--world W] [--frame F]
#                                     [--alt M] [--hover SEC] [--tol M] [--gui] [--keep]
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
# shellcheck source=../lib/log.bash
source "${FOREST_ROOT}/lib/log.bash"

REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
AP_ENV="${REPO_ROOT}/scripts/autopilot/autopilot_env.sh"
HOVER_CHECK="${REPO_ROOT}/scripts/autopilot/sitl_hover_check.py"

# ── Defaults ─────────────────────────────────────────────────────────
ASSERT=false
WORLD="iris_runway.sdf"
FRAME="gazebo-iris"
ALT=5.0
HOVER=20.0
TOL=0.5
GUI=true        # Gazebo GUI ON por omissão (para veres o voo); --headless para CI
KEEP=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assert) ASSERT=true ;;
    --world) shift; WORLD="${1:?}" ;;
    --frame) shift; FRAME="${1:?}" ;;
    --alt) shift; ALT="${1:?}" ;;
    --hover) shift; HOVER="${1:?}" ;;
    --tol) shift; TOL="${1:?}" ;;
    --gui) GUI=true ;;
    --headless) GUI=false ;;
    --extra-parm) shift; EXTRA_PARM="${1:?}" ;;
    --pre-rotate) PRE_ROTATE=true ;;
    --keep) KEEP=true ;;
    -h|--help)
      echo "Usage: forest test hybrid-aerial-sitl [--assert] [--world W] [--frame F] [--alt M] [--hover SEC] [--tol M] [--headless] [--keep]"
      echo "  M1: quad padrão sob ArduPilot SITL; assere hover estável (métricas §3.2)."
      echo "  Gazebo GUI abre por omissão; usa --headless para CI/assert sem janela."
      exit 0 ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── Pré-requisitos ───────────────────────────────────────────────────
AP_BIN="${ARDUPILOT_DIR:-$HOME/ardupilot}/build/sitl/bin/arducopter"
if [[ ! -x "$AP_BIN" ]]; then
  echo "ERROR: ArduPilot SITL em falta ($AP_BIN)." >&2
  echo "  Instala: bash scripts/autopilot/install_ardupilot_sitl.sh  (na tua sessão; usa sudo)" >&2
  exit 1
fi
[[ -f "$AP_ENV" ]] && source "$AP_ENV"
if ! command -v sim_vehicle.py >/dev/null; then
  echo "ERROR: sim_vehicle.py não no PATH (source scripts/autopilot/autopilot_env.sh)." >&2
  exit 1
fi

# Python com pymavlink (venv do ArduPilot)
VENV_PY="$HOME/venv-ardupilot/bin/python3"
PYBIN="$VENV_PY"
[[ -x "$PYBIN" ]] || PYBIN="python3"
if ! "$PYBIN" -c "import pymavlink" 2>/dev/null; then
  echo "ERROR: pymavlink indisponível (esperado em $VENV_PY)." >&2
  exit 1
fi

export GZ_PARTITION="${GZ_PARTITION:-forest_sitl_$$}"   # isola da tua outra sessão gz
export GZ_SIM_RESOURCE_PATH="${HOME}/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"

LOGDIR="$(mktemp -d "${FOREST_STATE_DIR:-/tmp}/sitl.XXXXXX")"
GZ_PID=""; SITL_PID=""

cleanup() {
  echo "[cleanup] a terminar processos…"
  [[ -n "$SITL_PID" ]] && kill "$SITL_PID" 2>/dev/null || true
  [[ -n "$GZ_PID" ]] && kill "$GZ_PID" 2>/dev/null || true
  pkill -TERM -x arducopter 2>/dev/null || true
  pkill -TERM -f 'sim[_]vehicle' 2>/dev/null || true
  sleep 1
  pkill -KILL -x arducopter 2>/dev/null || true
  pkill -KILL -f 'sim[_]vehicle' 2>/dev/null || true
  fuser -k 5760/tcp 2>/dev/null || true
  [[ "$KEEP" == "true" ]] && echo "[--keep] logs: $LOGDIR"
}
trap cleanup EXIT

forest_log_section "Hybrid aerial SITL — $FRAME @ ${ALT}m (partition $GZ_PARTITION)"

# ── 1. Gazebo ────────────────────────────────────────────────────────
GZ_FLAGS="-r"; [[ "$GUI" == "true" ]] || GZ_FLAGS="-s -r"
echo "[1/3] Gazebo: gz sim $GZ_FLAGS $WORLD"
# shellcheck disable=SC2086
gz sim $GZ_FLAGS "$WORLD" > "$LOGDIR/gz.log" 2>&1 &
GZ_PID=$!
sleep 8
if ! kill -0 "$GZ_PID" 2>/dev/null; then
  echo "ERROR: Gazebo morreu no arranque. tail:" >&2; tail -5 "$LOGDIR/gz.log" >&2; exit 1
fi

# ── 2. ArduPilot SITL ────────────────────────────────────────────────
echo "[2/3] ArduPilot SITL: sim_vehicle.py -v ArduCopter -f $FRAME --model JSON -w"
# --model JSON: o ardupilot_gazebo (Harmonic) usa o backend FDM JSON, NÃO o 'gazebo' antigo.
#   Sem isto, o SITL fica à espera de uma conexão que nunca chega → sem heartbeat.
# -w + --add-param-file: limpa o eeprom e garante FRAME_CLASS/FRAME_TYPE do frame
#   (senão o arm falha com "Motors: Check frame class and type").
FRAME_PARM="${ARDUPILOT_DIR:-$HOME/ardupilot}/Tools/autotest/default_params/${FRAME}.parm"
ADD_PARM=""; [[ -f "$FRAME_PARM" ]] && ADD_PARM="--add-param-file=$FRAME_PARM"
# Param file extra (tuning do nosso quad), carregado DEPOIS do frame (sobrepõe).
if [[ -n "${EXTRA_PARM:-}" && -f "$EXTRA_PARM" ]]; then
  ADD_PARM="$ADD_PARM --add-param-file=$EXTRA_PARM"
  echo "    + params extra: $EXTRA_PARM"
fi
# shellcheck disable=SC2086
sim_vehicle.py -v ArduCopter -f "$FRAME" --model JSON -w $ADD_PARM --no-mavproxy --no-rebuild -I0 \
  > "$LOGDIR/sitl.log" 2>&1 &
SITL_PID=$!
echo "    a aguardar SITL (TCP 5760; -w faz wipe+reboot)…"
sleep 18

# ── 2b. Transição morphing (opcional): pernas + rodar lagartas a ±90° via gz ──
if [[ "${PRE_ROTATE:-false}" == "true" ]]; then
  echo "[2b] Transição: deploy pernas → rodar lagartas ±90° (antes do arm)…"
  TBASE="/model/marble_hd2/hybrid"
  for n in fl fr rl rr; do
    gz topic -t "$TBASE/support_leg_$n/cmd_pos" -m gz.msgs.Double -p "data: 0.17" 2>/dev/null || true
  done
  sleep 3
  gz topic -t "$TBASE/left_track_yaw/cmd_pos"  -m gz.msgs.Double -p "data: 1.5707963"  2>/dev/null || true
  gz topic -t "$TBASE/right_track_yaw/cmd_pos" -m gz.msgs.Double -p "data: -1.5707963" 2>/dev/null || true
  echo "    a aguardar rotação das lagartas (6 s)…"
  sleep 6
fi

# ── 3. Validação de hover (MAVLink) ──────────────────────────────────
echo "[3/3] Hover check (arm → takeoff ${ALT}m → amostra ${HOVER}s)…"
set +e
"$PYBIN" "$HOVER_CHECK" --connect tcp:127.0.0.1:5760 \
  --alt "$ALT" --hover-sec "$HOVER" --tol "$TOL"
RC=$?
set -e

echo ""
echo "logs: $LOGDIR (gz.log, sitl.log)"
if [[ "$RC" -eq 0 ]]; then
  echo "SUCCESS: hover estável sob ArduPilot (frame=$FRAME)"
  exit 0
fi
echo "FAIL: hover não validado (ver logs)"
[[ "$ASSERT" == "true" ]] && exit 1
exit 0
