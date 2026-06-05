#!/usr/bin/env bash
# Exporta PATH/GZ env das toolchains autopiloto. Fazer: source este ficheiro.
# Idempotente; seguro se uma das toolchains ainda não existir.

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GZ_DIR="${ARDUPILOT_GZ_DIR:-$HOME/ardupilot_gazebo}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

# ── ArduPilot: sim_vehicle.py / autotest no PATH ─────────────────────
if [ -d "$ARDUPILOT_DIR" ]; then
  export PATH="$ARDUPILOT_DIR/Tools/autotest:$PATH"
fi

# ── ardupilot_gazebo: plugin + modelos/mundos para o gz encontrar ────
if [ -d "$ARDUPILOT_GZ_DIR/build" ]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GZ_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GZ_DIR/models:$ARDUPILOT_GZ_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"
fi

# ── ForestGen (mantém os nossos modelos/mundos visíveis em paralelo) ──
export GZ_SIM_RESOURCE_PATH="$HOME/Projetos/Gazebo/ForestGen/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "[autopilot_env] ArduPilot=$([ -d "$ARDUPILOT_DIR" ] && echo ok || echo ausente) "\
"ardupilot_gazebo=$([ -d "$ARDUPILOT_GZ_DIR/build" ] && echo ok || echo ausente) "\
"PX4=$([ -d "$PX4_DIR" ] && echo ok || echo ausente)"
