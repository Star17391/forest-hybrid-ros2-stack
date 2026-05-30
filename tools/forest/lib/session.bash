# shellcheck shell=bash
[[ -n "${_FOREST_SESSION_LOADED:-}" ]] && return 0
_FOREST_SESSION_LOADED=1

# shellcheck source=env.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
# shellcheck source=log.bash
source "$(dirname "${BASH_SOURCE[0]}")/log.bash"
# shellcheck source=lifecycle.bash
source "$(dirname "${BASH_SOURCE[0]}")/lifecycle.bash"
# shellcheck source=launch.bash
source "$(dirname "${BASH_SOURCE[0]}")/launch.bash"
# shellcheck source=profile.bash
source "$(dirname "${BASH_SOURCE[0]}")/profile.bash"

forest_session_active() {
  [[ -f "$FOREST_SESSION_FILE" ]]
}

forest_session_break_stale_lock() {
  if [[ ! -d "${FOREST_LOCK_FILE}.d" ]]; then
    return 0
  fi
  if forest_session_active; then
    return 0
  fi
  echo "WARNING: removing stale forest lock (no active session)" >&2
  rmdir "${FOREST_LOCK_FILE}.d" 2>/dev/null || rm -rf "${FOREST_LOCK_FILE}.d"
}

forest_session_acquire_lock() {
  mkdir -p "$(dirname "$FOREST_LOCK_FILE")"
  forest_session_break_stale_lock
  if ! mkdir "${FOREST_LOCK_FILE}.d" 2>/dev/null; then
    echo "ERROR: another forest operation in progress (lock)" >&2
    echo "       If stuck: forest down --force" >&2
    return 1
  fi
  return 0
}

forest_session_release_lock() {
  rmdir "${FOREST_LOCK_FILE}.d" 2>/dev/null || true
}

forest_session_read_field() {
  local field="$1"
  python3 - "$FOREST_SESSION_FILE" "$field" <<'PY'
import json, sys
path, field = sys.argv[1], sys.argv[2]
with open(path) as f:
    d = json.load(f)
v = d.get(field)
if v is None:
    sys.exit(1)
if isinstance(v, (list, dict)):
    print(json.dumps(v))
else:
    print(v)
PY
}

forest_session_write_state() {
  local profile="$1"
  local log_dir="$2"
  local foreground="${3:-true}"
  python3 - "$FOREST_SESSION_FILE" "$profile" "$log_dir" "$foreground" <<'PY'
import json, sys, datetime
out, profile, log_dir, fg = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4] == "true"
state = {
    "version": 1,
    "profile": profile,
    "started_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "log_dir": log_dir,
    "foreground": fg,
    "layers": [],
}
with open(out, "w") as f:
    json.dump(state, f, indent=2)
PY
}

forest_session_register_layer() {
  local layer_id="$1"
  local pgid="$2"
  local leader_pid="$3"
  shift 3
  local cmd="$*"
  python3 - "$FOREST_SESSION_FILE" "$layer_id" "$pgid" "$leader_pid" "$cmd" <<'PY'
import json, sys
path, lid, pgid_s, pid_s, cmd = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5]
pgid = int(pgid_s) if str(pgid_s).strip() else 0
pid = int(pid_s) if str(pid_s).strip() else 0
with open(path) as f:
    d = json.load(f)
d.setdefault("layers", []).append({
    "id": lid,
    "pgid": pgid,
    "leader_pid": pid,
    "command": cmd,
})
with open(path, "w") as f:
    json.dump(d, f, indent=2)
PY
}

forest_session_status() {
  if ! forest_session_active; then
    echo "forest: no active session"
    return 0
  fi
  echo "forest: active session"
  echo "  state:  $FOREST_SESSION_FILE"
  echo "  profile: $(forest_session_read_field profile 2>/dev/null || echo '?')"
  echo "  started: $(forest_session_read_field started_at 2>/dev/null || echo '?')"
  echo "  logs:    $(forest_session_read_field log_dir 2>/dev/null || echo '?')"
  echo "  layers:"
  python3 - "$FOREST_SESSION_FILE" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    d = json.load(f)
for L in d.get("layers", []):
    print(f"    - {L['id']}: pgid={L['pgid']} pid={L['leader_pid']}")
PY
  if forest_source_ros 2>/dev/null; then
    echo "  ros nodes (sample):"
    ros2 node list 2>/dev/null | head -20 | sed 's/^/    /'
  fi
}

forest_session_down_unlocked() {
  local do_verify="${1:-true}"

  if ! forest_session_active; then
    forest_log_section "No session state — running cleanup only"
    forest_source_ros || true
    if [[ "$do_verify" == "true" ]]; then
      forest_ensure_clean || return 1
    else
      forest_run_cleanup --hybrid || true
    fi
    return 0
  fi

  local log_dir
  log_dir="$(forest_session_read_field log_dir 2>/dev/null || echo '')"
  forest_log_section "Stopping session (logs: ${log_dir:-none})"

  forest_source_ros || true

  while IFS= read -r _layer_line; do
    [[ -z "$_layer_line" ]] && continue
    local _lid _pgid
    _lid="${_layer_line%% *}"
    _pgid="${_layer_line##* }"
    forest_kill_pgid "$_pgid" "$_lid"
  done < <(python3 - "$FOREST_SESSION_FILE" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    layers = json.load(f).get("layers", [])
for L in reversed(layers):
    print(L["id"], L["pgid"])
PY
)

  rm -f "$FOREST_SESSION_FILE"
  if [[ "$do_verify" == "true" ]]; then
    forest_ensure_clean || return 1
  else
    forest_run_cleanup --hybrid || true
  fi
  forest_log_section "Session stopped"
  return 0
}

forest_session_down() {
  local do_verify="${1:-true}"
  local break_lock="${2:-false}"
  if [[ "$break_lock" == "true" ]]; then
    forest_session_break_stale_lock
    rmdir "${FOREST_LOCK_FILE}.d" 2>/dev/null || rm -rf "${FOREST_LOCK_FILE}.d" 2>/dev/null || true
  fi
  forest_session_acquire_lock || return 1
  trap 'forest_session_release_lock' EXIT
  forest_session_down_unlocked "$do_verify"
}

forest_session_up_profile() {
  local profile="$1"
  local detach="${2:-false}"
  local panel_only="${3:-false}"

  if forest_session_active; then
    echo "ERROR: session already active — run 'forest down' first" >&2
    forest_session_status
    return 1
  fi

  forest_session_acquire_lock || return 1
  trap 'forest_session_release_lock' EXIT

  if ! forest_profile_load "$profile"; then
    forest_session_release_lock
    trap - EXIT
    return 1
  fi
  if ! declare -f forest_profile_up >/dev/null; then
    echo "ERROR: profile must define forest_profile_up()" >&2
    forest_session_release_lock
    trap - EXIT
    return 1
  fi

  forest_source_ros || return 1

  if [[ "$panel_only" == "true" ]]; then
    if ! forest_profile_up "true"; then
      forest_session_release_lock
      trap - EXIT
      return 1
    fi
    forest_session_release_lock
    trap - EXIT
    return 0
  fi

  forest_profile_run_pre_start
  forest_session_log_dir
  forest_session_write_state "$profile" "$FOREST_SESSION_LOG_DIR" "$([[ "$detach" == "true" ]] && echo false || echo true)"

  if ! forest_profile_up "false"; then
    forest_session_down_unlocked false
    forest_session_release_lock
    trap - EXIT
    return 1
  fi

  if [[ -n "${forest_profile_wait_nodes:-}" ]]; then
    forest_wait_for_nodes "${forest_profile_wait_nodes[@]}" || {
      echo "WARNING: wait_nodes incomplete — stack may still be starting" >&2
    }
  fi

  forest_session_release_lock
  trap - EXIT

  echo ""
  echo "Session up: profile=${profile} logs=${FOREST_SESSION_LOG_DIR}"
  if [[ -n "${forest_profile_launch_hint:-}" ]]; then
    echo "  ▸ ${forest_profile_launch_hint}"
  fi
  echo "  forest status | forest down | forest panel | forest teleop"

  if [[ "$detach" == "true" ]]; then
    echo "  (detached — use 'forest diag tf-audit' or 'forest diag tf' to check TF)"
    return 0
  fi

  if [[ "${forest_profile_ui_panel_foreground:-}" == "true" ]]; then
    echo ""
    echo "  Press PLAY in Gazebo, then use the mission panel."
    echo "  Close the panel or Ctrl+C to shutdown."
    echo ""
    trap 'echo ""; echo "Interrupt — forest down"; forest_session_down true; exit 130' INT TERM
    forest_open_mission_panel || true
    forest_session_down true
    return 0
  fi

  # Foreground: wait on first layer PGID, trap INT/TERM
  local first_pgid
  first_pgid="$(python3 - "$FOREST_SESSION_FILE" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    layers = json.load(f).get("layers", [])
print(layers[0]["pgid"] if layers else "")
PY
)"
  trap 'echo ""; echo "Interrupt — forest down"; forest_session_down true; exit 130' INT TERM
  if [[ -n "$first_pgid" ]]; then
    while kill -0 "-${first_pgid}" 2>/dev/null; do
      sleep 1
    done
  fi
  forest_session_down true
}
