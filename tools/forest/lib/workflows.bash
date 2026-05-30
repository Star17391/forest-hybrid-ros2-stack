# shellcheck shell=bash
[[ -n "${_FOREST_WORKFLOWS_LOADED:-}" ]] && return 0
_FOREST_WORKFLOWS_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"
# shellcheck source=mission.bash
source "$(dirname "${BASH_SOURCE[0]}")/mission.bash"

forest_workflow_list() {
  local wf
  for wf in "${FOREST_ROOT}/workflows"/*.sh; do
    [[ -f "$wf" ]] || continue
    basename "$wf" .sh
  done | sort
}

forest_run_workflow() {
  local name="$1"
  shift
  local script="${FOREST_ROOT}/workflows/${name}.sh"
  if [[ ! -f "$script" ]]; then
    echo "ERROR: workflow not found: $name (expected $script)" >&2
    echo "Workflows:" >&2
    forest_workflow_list | sed 's/^/  - /' >&2 || true
    return 1
  fi
  forest_log_section "Workflow: $name"
  bash "$script" "$@"
}
