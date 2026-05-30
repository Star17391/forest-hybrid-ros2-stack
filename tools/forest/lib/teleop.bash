# shellcheck shell=bash
[[ -n "${_FOREST_TELEOP_LOADED:-}" ]] && return 0
_FOREST_TELEOP_LOADED=1

# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

forest_open_teleop_panel() {
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY não definido — painel Tk precisa de ambiente gráfico." >&2
    return 1
  fi
  if ! python3 -c "import tkinter" 2>/dev/null; then
    echo "ERROR: python3-tk em falta (sudo apt install python3-tk)" >&2
    return 1
  fi
  forest_log_section "Teleop panel (→ /forest_gen/cmd_vel)"
  echo "  Gazebo em PLAY; mantém botões premidos para mover."
  echo ""
  ros2 run forest_sim_bridge forest_teleop_panel
}
