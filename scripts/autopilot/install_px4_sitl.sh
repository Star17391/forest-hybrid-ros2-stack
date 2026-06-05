#!/usr/bin/env bash
# M0/M1 — PX4-Autopilot SITL com Gazebo (gz Harmonic) + agente uXRCE-DDS para ROS 2.
# Idempotente. Correr na TUA sessão (sudo + rede). NÃO correr em background.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
AGENT_DIR="${AGENT_DIR:-$HOME/Micro-XRCE-DDS-Agent}"

say() { printf "\n\033[1;34m[px4-sitl]\033[0m %s\n" "$*"; }
warn() { printf "\n\033[1;33m[px4-sitl]\033[0m %s\n" "$*"; }

# ── 1. Clonar PX4 ────────────────────────────────────────────────────
if [ ! -d "$PX4_DIR/.git" ]; then
  say "A clonar PX4-Autopilot para $PX4_DIR (recursivo)…"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$PX4_DIR"
else
  say "PX4 já existe em $PX4_DIR — a atualizar submódulos…"
  git -C "$PX4_DIR" submodule update --init --recursive
fi

# ── 2. Pré-requisitos PX4 ────────────────────────────────────────────
say "A instalar pré-requisitos PX4 (pode pedir sudo; inclui toolchain + gz)…"
cd "$PX4_DIR"
bash ./Tools/setup/ubuntu.sh
# Nota: PX4 main usa gz Harmonic por omissão em 24.04 (compatível com gz-sim 8 do sistema).

# ── 3. Build SITL + gz (x500 padrão para validar M1) ─────────────────
say "A compilar PX4 SITL (alvo gz_x500)… (a primeira vez é demorado)"
# Não lança a sim aqui (faz isso no teste); só compila o alvo:
DONT_RUN=1 make px4_sitl gz_x500

# ── 4. Agente uXRCE-DDS (ponte PX4 ↔ ROS 2) ──────────────────────────
say "A instalar o Micro-XRCE-DDS-Agent (ponte ROS 2)…"
if [ ! -d "$AGENT_DIR/.git" ]; then
  git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
fi
mkdir -p "$AGENT_DIR/build"
cd "$AGENT_DIR/build"
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"
sudo make install
sudo ldconfig /usr/local/lib/

# ── 5. px4_msgs para ROS 2 (opcional, p/ M3) ─────────────────────────
say "Nota: para integração ROS 2 (M3) precisarás de 'px4_msgs' no teu workspace."
say "      git clone https://github.com/PX4/px4_msgs.git em src/ e colcon build."

# ── 6. Verificação ───────────────────────────────────────────────────
say "Verificação:"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[ -x "$PX4_BIN" ] && say "  ✓ PX4 SITL: $PX4_BIN" || warn "  ✗ PX4 binário não encontrado em $PX4_BIN"
command -v MicroXRCEAgent >/dev/null && say "  ✓ MicroXRCEAgent no PATH" || warn "  ✗ MicroXRCEAgent não no PATH"

say "Concluído. Teste M1: 'cd $PX4_DIR && make px4_sitl gz_x500' (deve abrir gz com um x500 a pairar)."
