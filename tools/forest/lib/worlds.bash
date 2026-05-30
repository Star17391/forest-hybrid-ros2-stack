# shellcheck shell=bash
[[ -n "${_FOREST_WORLDS_LOADED:-}" ]] && return 0
_FOREST_WORLDS_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"

forest_forestgen_worlds_dir() {
  local fg="${FORESTGEN_PATH:-$HOME/Projetos/Gazebo/ForestGen}"
  if [[ -d "${fg}/worlds" ]]; then
    printf '%s/worlds' "$fg"
    return 0
  fi
  return 1
}

# Normaliza nome para launch arg (sempre termina em .sdf se for só basename).
forest_world_normalize() {
  local w="$1"
  if [[ -z "$w" ]]; then
    return 1
  fi
  if [[ "$w" == */* ]]; then
    printf '%s' "$w"
    return 0
  fi
  if [[ "$w" == *.sdf ]]; then
    printf '%s' "$w"
  else
    printf '%s.sdf' "$w"
  fi
}

forest_world_list() {
  local dir
  if ! dir="$(forest_forestgen_worlds_dir)"; then
    echo "ERROR: FORESTGEN worlds not found. Set FORESTGEN_PATH." >&2
    return 1
  fi
  local f base
  for f in "${dir}"/*.sdf; do
    [[ -f "$f" ]] || continue
    base="$(basename "$f" .sdf)"
    printf '%s  (%s)\n' "$base" "$(basename "$f")"
  done | sort
}

forest_world_names_for_completion() {
  local dir f base
  if ! dir="$(forest_forestgen_worlds_dir)"; then
    return 0
  fi
  for f in "${dir}"/*.sdf; do
    [[ -f "$f" ]] || continue
    base="$(basename "$f" .sdf)"
    printf '%s\n' "$base"
    printf '%s\n' "$(basename "$f")"
  done
}

forest_world_apply_override() {
  local w
  w="$(forest_world_normalize "$1")" || return 1
  export FOREST_LAUNCH_OVERRIDES="${FOREST_LAUNCH_OVERRIDES:+$FOREST_LAUNCH_OVERRIDES,}world:=${w}"
  echo "World override: ${w}"
}
