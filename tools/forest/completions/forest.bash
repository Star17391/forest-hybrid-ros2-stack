# Bash completion for the forest CLI.
# Install: bash tools/forest/completions/install.sh
# After CLI changes: bash tools/forest/completions/sync_completions.sh

[[ -n "${_FOREST_COMPLETION_LOADED:-}" ]] && return 0
_FOREST_COMPLETION_LOADED=1

_COMP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=commands.bash
source "${_COMP_DIR}/commands.bash"

_forest_root() {
  if [[ -n "${FOREST_ROOT:-}" && -d "${FOREST_ROOT}/profiles" ]]; then
    printf '%s' "$FOREST_ROOT"
    return
  fi
  local here
  here="$(cd "${_COMP_DIR}/.." && pwd)"
  printf '%s' "$here"
}

_forest_profiles() {
  local root profiles_dir f base
  root="$(_forest_root)"
  profiles_dir="${root}/profiles"
  [[ -d "$profiles_dir" ]] || return
  for f in "${profiles_dir}"/*.yaml; do
    [[ -f "$f" ]] || continue
    base="$(basename "$f" .yaml)"
    printf '%s\n' "$base"
  done
  for f in "${profiles_dir}"/*.profile.bash; do
    [[ -f "$f" ]] || continue
    base="$(basename "$f" .profile.bash)"
    printf '%s\n' "$base"
  done
}

_forest_worlds() {
  local fg dir f base
  fg="${FORESTGEN_PATH:-$HOME/Projetos/Gazebo/ForestGen}"
  dir="${fg}/worlds"
  [[ -d "$dir" ]] || return
  for f in "${dir}"/*.sdf; do
    [[ -f "$f" ]] || continue
    base="$(basename "$f" .sdf)"
    printf '%s\n' "$base"
    printf '%s\n' "$(basename "$f")"
  done
}

_forest_log_layers() {
  local state="${FOREST_STATE_DIR:-${XDG_RUNTIME_DIR:-/tmp}/forest}/session.state.json"
  if [[ -f "$state" ]]; then
    python3 - "$state" <<'PY' 2>/dev/null || true
import json, sys
with open(sys.argv[1]) as f:
    d = json.load(f)
for L in d.get("layers", []):
    print(L.get("id", ""))
PY
    return
  fi
  printf '%s\n' sim nav mission
}

_forest_completion() {
  local cur prev
  cur="${COMP_WORDS[COMP_CWORD]}"
  prev="${COMP_WORDS[COMP_CWORD - 1]}"

  local cmd=""
  if (( COMP_CWORD >= 1 )); then
    cmd="${COMP_WORDS[1]}"
  fi

  # forest <TAB>
  if (( COMP_CWORD == 1 )); then
    COMPREPLY=( $(compgen -W "${FOREST_CLI_TOP_LEVEL}" -- "$cur") )
    return 0
  fi

  case "$cmd" in
    profile)
      if [[ "$prev" == "profile" ]]; then
        COMPREPLY=( $(compgen -W "${FOREST_CLI_PROFILE_SUBS}" -- "$cur") )
      elif [[ "$prev" == "validate" ]]; then
        local profiles
        profiles="$(_forest_profiles)"
        COMPREPLY=( $(compgen -W "$profiles" -- "$cur") )
      fi
      ;;
    test)
      if [[ "$prev" == "test" ]]; then
        local wflows
        wflows="$(find "$(_forest_root)/workflows" -name 'test-*.sh' -printf '%f\n' 2>/dev/null | sed 's/^test-//;s/\.sh$//')"
        COMPREPLY=( $(compgen -W "$wflows" -- "$cur") )
      fi
      ;;
    diag)
      if [[ "$prev" == "diag" ]]; then
        COMPREPLY=( $(compgen -W "${FOREST_CLI_DIAG_SUBS}" -- "$cur") )
      fi
      ;;
    completion)
      if [[ "$prev" == "completion" ]]; then
        COMPREPLY=( $(compgen -W "${FOREST_CLI_COMPLETION_SUBS}" -- "$cur") )
      fi
      ;;
    attach)
      if [[ "$prev" == "attach" ]]; then
        COMPREPLY=( $(compgen -W "${FOREST_CLI_ATTACH_SUBS}" -- "$cur") )
      fi
      ;;
    logs)
      if [[ "$prev" == "logs" ]]; then
        local layers
        layers="$(_forest_log_layers)"
        COMPREPLY=( $(compgen -W "$layers ${FOREST_CLI_LOGS_OPTS}" -- "$cur") )
      else
        COMPREPLY=( $(compgen -W "${FOREST_CLI_LOGS_OPTS}" -- "$cur") )
      fi
      ;;
    panel|teleop)
      COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
      ;;
    status|help)
      COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
      ;;
    down)
      COMPREPLY=( $(compgen -W "--no-verify --force -h --help" -- "$cur") )
      ;;
    cleanup)
      COMPREPLY=( $(compgen -W "--hybrid -h --help" -- "$cur") )
      ;;
    world)
      if [[ "$prev" == "world" ]]; then
        COMPREPLY=( $(compgen -W "list -h --help" -- "$cur") )
      fi
      ;;
    up)
      if [[ "$prev" == "--world" || "$prev" == "-w" ]]; then
        local worlds
        worlds="$(_forest_worlds)"
        COMPREPLY=( $(compgen -W "$worlds" -- "$cur") )
      elif [[ "$prev" == "up" ]]; then
        local profiles
        profiles="$(_forest_profiles)"
        COMPREPLY=( $(compgen -W "$profiles" -- "$cur") )
      else
        COMPREPLY=( $(compgen -W "${FOREST_CLI_UP_OPTS}" -- "$cur") )
      fi
      ;;
    capture)
      if [[ "$prev" == "capture" || "$prev" == "--out" ]]; then
        local worlds
        worlds="$(_forest_worlds)"
        COMPREPLY=( $(compgen -W "$worlds" -- "$cur") )
      else
        COMPREPLY=( $(compgen -W "${FOREST_CLI_CAPTURE_OPTS}" -- "$cur") )
      fi
      ;;
    *)
      COMPREPLY=()
      ;;
  esac
  return 0
}

complete -o default -o bashdefault -F _forest_completion forest
