# shellcheck shell=bash
[[ -n "${_FOREST_PROFILE_LOADED:-}" ]] && return 0
_FOREST_PROFILE_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=mission.bash
source "$(dirname "${BASH_SOURCE[0]}")/mission.bash"

forest_profile_resolve() {
  local name="$1"
  local yaml="${FOREST_ROOT}/profiles/${name}.yaml"
  local bash_legacy="${FOREST_ROOT}/profiles/${name}.profile.bash"

  # legacy/sim-mvp-nav → profiles/legacy/sim-mvp-nav.yaml
  if [[ "$name" == legacy/* ]]; then
    yaml="${FOREST_ROOT}/profiles/${name}.yaml"
  fi

  if [[ "${FOREST_USE_LEGACY_PROFILES:-}" == "1" && -f "$bash_legacy" ]]; then
    printf '%s' "$bash_legacy"
    return 0
  fi
  if [[ -f "$yaml" ]]; then
    printf '%s' "$yaml"
    return 0
  fi
  if [[ -f "$bash_legacy" ]]; then
    printf '%s' "$bash_legacy"
    return 0
  fi
  return 1
}

forest_profile_load() {
  local name="$1"
  local panel_only="${2:-false}"
  local resolved
  if ! resolved="$(forest_profile_resolve "$name")"; then
    echo "ERROR: profile not found: $name" >&2
    echo "  (procura ${name}.yaml ou ${name}.profile.bash em tools/forest/profiles/)" >&2
    return 1
  fi

  if [[ "$resolved" == *.yaml ]]; then
    local tmp
    tmp="$(mktemp "${FOREST_STATE_DIR}/profile_${name}.XXXXXX")"
    if ! python3 "${FOREST_ROOT}/lib/profile.py" emit-bash "$resolved" "$panel_only" >"$tmp"; then
      rm -f "$tmp"
      return 1
    fi
    # shellcheck disable=SC1090
    source "$tmp"
    rm -f "$tmp"
    return 0
  fi

  # shellcheck disable=SC1090
  source "$resolved"
  if ! declare -f forest_profile_up >/dev/null; then
    echo "ERROR: legacy profile must define forest_profile_up(): $resolved" >&2
    return 1
  fi
  return 0
}

forest_profile_list() {
  python3 "${FOREST_ROOT}/lib/profile.py" list --profiles-dir "${FOREST_ROOT}/profiles"
}

forest_profile_panel_only_mode() {
  local -a nodes=("$@")
  forest_source_ros || return 1
  local n
  for n in "${nodes[@]}"; do
    if ! ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/${n}"; then
      echo "ERROR: nó /${n} em falta — arranca o stack antes de --panel-only" >&2
      return 1
    fi
  done
  forest_open_mission_panel
}

forest_profile_run_pre_start() {
  local mode="${forest_profile_pre_start:-cleanup_hybrid}"
  case "$mode" in
    none) ;;
    cleanup) forest_run_cleanup ;;
    cleanup_hybrid | *) forest_run_cleanup --hybrid ;;
  esac
}
