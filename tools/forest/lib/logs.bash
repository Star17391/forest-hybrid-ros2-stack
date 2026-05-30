# shellcheck shell=bash
[[ -n "${_FOREST_LOGS_LOADED:-}" ]] && return 0
_FOREST_LOGS_LOADED=1

# shellcheck source=session.bash
source "$(dirname "${BASH_SOURCE[0]}")/session.bash"

forest_logs_usage() {
  cat <<'EOF'
forest logs [layer] [options]

Show ROS/Gazebo output captured during `forest up` (layers log to a file, not the terminal).

Options:
  -f, --follow     Tail -f (live)
  -n N             Last N lines (default 80)
  --grep PATTERN   Filter lines (grep -E)

Layers: sim, nav, mission, … (default: all *.log in session dir)

Examples:
  forest logs sim -f
  forest logs --grep 'WARN|ERROR'
  forest logs sim -n 200
EOF
}

forest_logs_cmd() {
  local follow=false
  local lines=80
  local grep_pat=""
  local -a layers=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -f|--follow)
        follow=true
        shift
        ;;
      -n)
        shift
        lines="${1:-80}"
        shift
        ;;
      --grep)
        shift
        grep_pat="${1:-}"
        shift
        ;;
      -h|--help)
        forest_logs_usage
        return 0
        ;;
      -*)
        echo "Unknown option: $1" >&2
        forest_logs_usage >&2
        return 2
        ;;
      *)
        layers+=("$1")
        shift
        ;;
    esac
  done

  if ! forest_session_active; then
    echo "forest: no active session — logs only exist while a session is up or after up -d" >&2
    echo "  Tip: forest up sim-sensors-only -d   then   forest logs sim -f" >&2
    # Still allow reading last session dir from state if file exists but inactive
    if [[ ! -f "${FOREST_SESSION_FILE:-}" ]]; then
      return 1
    fi
  fi

  local log_dir
  log_dir="$(forest_session_read_field log_dir 2>/dev/null || true)"
  if [[ -z "$log_dir" || ! -d "$log_dir" ]]; then
    echo "ERROR: log directory not found (log_dir=${log_dir:-empty})" >&2
    return 1
  fi

  local -a files=()
  if [[ ${#layers[@]} -gt 0 ]]; then
    local lid
    for lid in "${layers[@]}"; do
      if [[ -f "${log_dir}/${lid}.log" ]]; then
        files+=("${log_dir}/${lid}.log")
      else
        echo "WARNING: missing ${log_dir}/${lid}.log" >&2
      fi
    done
  else
    local f
    for f in "${log_dir}"/*.log; do
      [[ -f "$f" ]] && files+=("$f")
    done
  fi

  if [[ ${#files[@]} -eq 0 ]]; then
    echo "ERROR: no log files in ${log_dir}" >&2
    return 1
  fi

  echo "Logs: ${files[*]}"
  if [[ "$follow" == "true" ]]; then
    if [[ -n "$grep_pat" ]]; then
      tail -n "$lines" -f "${files[@]}" | grep -E --line-buffered "$grep_pat" || true
    else
      tail -n "$lines" -f "${files[@]}"
    fi
    return 0
  fi

  if [[ -n "$grep_pat" ]]; then
    tail -n "$lines" "${files[@]}" | grep -E "$grep_pat" || true
  else
    tail -n "$lines" "${files[@]}"
  fi
}
