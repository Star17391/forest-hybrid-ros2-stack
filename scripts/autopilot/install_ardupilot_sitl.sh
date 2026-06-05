#!/usr/bin/env bash
# M0/M1 — ArduPilot SITL + ardupilot_gazebo plugin (Ubuntu 24.04, gz Harmonic / Sim 8).
# Idempotente: clona só se faltar, recompila o que for preciso.
# Correr na TUA sessão (precisa de sudo + rede). NÃO correr em background.
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GZ_DIR="${ARDUPILOT_GZ_DIR:-$HOME/ardupilot_gazebo}"
GZ_VER=8  # Harmonic

say() { printf "\n\033[1;32m[ardupilot-sitl]\033[0m %s\n" "$*"; }
warn() { printf "\n\033[1;33m[ardupilot-sitl]\033[0m %s\n" "$*"; }

# ── 1. Clonar ArduPilot ──────────────────────────────────────────────
if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
  say "A clonar ArduPilot para $ARDUPILOT_DIR (com submódulos)…"
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
else
  say "ArduPilot já existe em $ARDUPILOT_DIR — a atualizar submódulos…"
  git -C "$ARDUPILOT_DIR" submodule update --init --recursive
fi

# ── 2. Pré-requisitos (apt, pip) ─────────────────────────────────────
say "A instalar pré-requisitos ArduPilot (pode pedir sudo)…"
cd "$ARDUPILOT_DIR"
# -y: assume yes; instala toolchain ARM, python deps, MAVProxy, pymavlink
Tools/environment_install/install-prereqs-ubuntu.sh -y
# Carrega PATH atualizado (MAVProxy etc.) sem reabrir shell
# shellcheck disable=SC1090
[ -f "$HOME/.ardupilot_env" ] && source "$HOME/.ardupilot_env" || true
[ -f "$HOME/.profile" ] && source "$HOME/.profile" || true

# ── 3. Build SITL (ArduCopter) ───────────────────────────────────────
say "A configurar e compilar SITL (ArduCopter)…"
./waf configure --board sitl
./waf copter

# ── 4. Plugin ardupilot_gazebo (Harmonic) ────────────────────────────
say "A instalar dependências do plugin Gazebo…"
sudo apt-get update
# GStreamer dev: o ardupilot_gazebo exige gstreamer-1.0 + gstreamer-app-1.0 (streaming de câmara).
sudo apt-get install -y "libgz-sim${GZ_VER}-dev" rapidjson-dev cmake build-essential \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

if [ ! -d "$ARDUPILOT_GZ_DIR/.git" ]; then
  say "A clonar ardupilot_gazebo para $ARDUPILOT_GZ_DIR…"
  git clone https://github.com/ArduPilot/ardupilot_gazebo "$ARDUPILOT_GZ_DIR"
else
  say "ardupilot_gazebo já existe — a atualizar…"
  git -C "$ARDUPILOT_GZ_DIR" pull --ff-only || warn "pull falhou (ok se tiveres edições locais)"
fi

say "A compilar o plugin ardupilot_gazebo…"
mkdir -p "$ARDUPILOT_GZ_DIR/build"
cd "$ARDUPILOT_GZ_DIR/build"
export GZ_VERSION=harmonic
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"

# ── 5. Verificação ───────────────────────────────────────────────────
say "Verificação:"
SITL_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
[ -x "$SITL_BIN" ] && say "  ✓ SITL: $SITL_BIN" || warn "  ✗ SITL não encontrado em $SITL_BIN"
PLUGIN_SO="$ARDUPILOT_GZ_DIR/build/libArduPilotPlugin.so"
[ -f "$PLUGIN_SO" ] && say "  ✓ plugin: $PLUGIN_SO" || warn "  ✗ plugin não encontrado em $PLUGIN_SO"
command -v sim_vehicle.py >/dev/null && say "  ✓ sim_vehicle.py no PATH" || warn "  ✗ sim_vehicle.py não no PATH (corre: source ~/.profile)"

say "Concluído. Próximo: 'source scripts/autopilot/autopilot_env.sh' e testar M1 (ver README)."
