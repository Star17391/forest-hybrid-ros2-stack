# shellcheck shell=bash
[[ -n "${_FOREST_MISSION_LOADED:-}" ]] && return 0
_FOREST_MISSION_LOADED=1

# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

forest_open_mission_panel() {
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY não definido — painel Tk precisa de ambiente gráfico." >&2
    return 1
  fi
  if ! python3 -c "import tkinter" 2>/dev/null; then
    echo "ERROR: python3-tk em falta (sudo apt install python3-tk)" >&2
    return 1
  fi
  forest_log_section "Mission panel (GOTO / PATROL → /mission/command)"
  echo "  Não uses: ros2 topic pub /planning/mission_goal …"
  echo "  Fecha o painel para terminar (modo foreground)."
  echo ""
  ros2 run forest_sim_bridge forest_mission_panel
}
