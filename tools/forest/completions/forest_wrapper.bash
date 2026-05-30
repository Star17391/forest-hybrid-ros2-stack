# shellcheck shell=bash
# Wrapper: `forest completion refresh` recarrega Tab na shell actual (não num subprocess).
# Instalado por completions/install.sh no ~/.bashrc.

forest() {
  case "${1-}:${2-}" in
    completion:refresh|completion:sync)
      local _sync
      _sync="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sync_completions.sh"
      # shellcheck source=sync_completions.sh
      source "$_sync"
      return 0
      ;;
  esac
  command forest "$@"
}
