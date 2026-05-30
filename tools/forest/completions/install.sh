#!/usr/bin/env bash
# Install forest CLI (PATH + bash Tab completion) into ~/.bashrc.
set -euo pipefail

HYBRID_WS="${HYBRID_WS:-$HOME/Projetos/Tese/forest-hybrid-ros2-stack}"
FOREST_BIN="${HYBRID_WS}/tools/forest/bin"
COMP_FILE="${HYBRID_WS}/tools/forest/completions/forest.bash"
SYNC_SCRIPT="${HYBRID_WS}/tools/forest/completions/sync_completions.sh"
MARKER="# forest CLI (PATH + completion)"
WRAPPER_MARKER="# forest CLI wrapper (completion refresh)"
BASHRC="${HOME}/.bashrc"
WRAPPER_FILE="${HYBRID_WS}/tools/forest/completions/forest_wrapper.bash"
REFRESH_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --refresh-only) REFRESH_ONLY=true ;;
    -h|--help)
      echo "Usage: $0 [--refresh-only]"
      echo "  Installs PATH + completion in ~/.bashrc (once)."
      echo "  --refresh-only  Only reload Tab completion in the current shell."
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -x "${FOREST_BIN}/forest" ]]; then
  echo "ERROR: forest not found at ${FOREST_BIN}/forest" >&2
  exit 1
fi
if [[ ! -f "$COMP_FILE" ]]; then
  echo "ERROR: missing $COMP_FILE" >&2
  exit 1
fi

if [[ "$REFRESH_ONLY" == "true" ]]; then
  # shellcheck source=sync_completions.sh
  source "$SYNC_SCRIPT"
  exit 0
fi

# shellcheck source=sync_completions.sh
source "$SYNC_SCRIPT"

_install_forest_bashrc_block() {
  {
    echo ""
    echo "$MARKER"
    echo "export PATH=\"${FOREST_BIN}:\$PATH\""
    echo "[[ -f \"${COMP_FILE}\" ]] && source \"${COMP_FILE}\""
    echo "$WRAPPER_MARKER"
    echo "[[ -f \"${WRAPPER_FILE}\" ]] && source \"${WRAPPER_FILE}\""
  } >>"$BASHRC"
}

if grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
  echo "PATH + completion already in $BASHRC"
  if ! grep -qF "$WRAPPER_MARKER" "$BASHRC" 2>/dev/null; then
    {
      echo ""
      echo "$WRAPPER_MARKER"
      echo "[[ -f \"${WRAPPER_FILE}\" ]] && source \"${WRAPPER_FILE}\""
    } >>"$BASHRC"
    echo "Added completion-refresh wrapper to $BASHRC"
  else
    echo "Wrapper already in $BASHRC"
  fi
else
  _install_forest_bashrc_block
  echo "Added PATH + completion + wrapper to $BASHRC"
fi

if grep -qF "# forest CLI bash completion" "$BASHRC" 2>/dev/null \
  && ! grep -qF "${FOREST_BIN}" "$BASHRC" 2>/dev/null; then
  {
    echo ""
    echo "# forest CLI PATH (added by install.sh upgrade)"
    echo "export PATH=\"${FOREST_BIN}:\$PATH\""
  } >>"$BASHRC"
  echo "Added missing PATH (completion was already present)"
fi

echo ""
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  [[ -f "$WRAPPER_FILE" ]] && source "$WRAPPER_FILE"
  echo "Tab completion active (forest up --world, forest world, forest logs)."
  echo "  forest completion refresh  → recarrega Tab nesta shell"
  echo "Other terminals:  source ~/.bashrc"
else
  echo "Para activar o Tab nesta shell:"
  echo "  source ~/.bashrc"
  echo "  # ou: source ${COMP_FILE} && source ${WRAPPER_FILE}"
  echo "Novos terminais:  source ~/.bashrc"
fi
