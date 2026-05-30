# shellcheck shell=bash
[[ -n "${_FOREST_RANDOM_MOVE_LOADED:-}" ]] && return 0
_FOREST_RANDOM_MOVE_LOADED=1

# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"

forest_random_move_usage() {
  cat <<EOF
forest random_move — exploração aleatória contínua (→ /forest_gen/cmd_vel)

Requer stack a correr (forest up sim-lidar3d-test -d, etc.) e Gazebo em PLAY.

Options (pass-through para o nó ROS):
  --linear SPEED       Velocidade linear máx. (default: 0.45)
  --angular SPEED      Velocidade angular máx. (default: 0.55)
  --segment-min SEC    Duração mínima de cada segmento (default: 3.5)
  --segment-max SEC    Duração máxima de cada segmento (default: 9.0)
  -h, --help

Exemplo:
  forest up sim-lidar3d-test -d --world forest_gentle_trees_rocks
  forest random_move
EOF
}

forest_run_random_move() {
  local linear="" angular="" seg_min="" seg_max=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --linear) linear="$2"; shift 2 ;;
      --angular) angular="$2"; shift 2 ;;
      --segment-min) seg_min="$2"; shift 2 ;;
      --segment-max) seg_max="$2"; shift 2 ;;
      -h|--help) forest_random_move_usage; return 0 ;;
      *) echo "Unknown option: $1" >&2; forest_random_move_usage >&2; return 2 ;;
    esac
  done

  forest_source_ros || return 1
  if ! ros2 topic list 2>/dev/null | grep -q "/forest_gen/cmd_vel"; then
    echo "WARNING: /forest_gen/cmd_vel não visível — confirma sim + bridge" >&2
  fi

  local -a extra=()
  [[ -n "$linear" ]] && extra+=(-p "linear_speed:=${linear}")
  [[ -n "$angular" ]] && extra+=(-p "angular_speed:=${angular}")
  [[ -n "$seg_min" ]] && extra+=(-p "segment_min_s:=${seg_min}")
  [[ -n "$seg_max" ]] && extra+=(-p "segment_max_s:=${seg_max}")

  forest_log_section "Random explore (segmentos longos, cmd_vel contínuo)"
  echo "  Ctrl+C para parar. Gazebo em PLAY."
  echo ""
  ros2 run forest_sim_bridge forest_random_explore "${extra[@]}"
}
