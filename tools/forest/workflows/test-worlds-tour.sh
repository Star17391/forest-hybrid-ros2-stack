#!/usr/bin/env bash
# Tour visual de mundos: abre cada mundo no Gazebo + RViz (um de cada vez) para
# inspeção manual do terreno/perceção. NÃO faz asserts — é validação visual.
#
# Uso:
#   forest test worlds-tour                          # conjunto default, Enter p/ avançar
#   forest test worlds-tour --hold 30                # 30 s por mundo (sem Enter)
#   forest test worlds-tour --profile sim-lidar3d-experimental
#   forest test worlds-tour forest_flat forest_rugged forest_gentle_trees_rocks
#
# O que observar em cada mundo:
#   - Gazebo: o terreno/heightmap e árvores/rochas do mundo.
#   - RViz: nuvem "Exp CSF ground" (verde) deve acompanhar o relevo;
#           "Exp clusters" nas árvores/rochas.
#   - "Traversability costmap": NO Sprint 0 está VAZIO (cinzento) — só ganha
#     terreno no Sprint 2 (malha) e obstáculos no Sprint 1.
set -uo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/env.bash
source "${FOREST_ROOT}/lib/env.bash"
FOREST_BIN="${FOREST_ROOT}/bin/forest"

PROFILE="sim-traversability"
HOLD=""                 # vazio => interativo (Enter); número => segundos por mundo
declare -a WORLDS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile) shift; PROFILE="${1:?--profile requer nome}" ;;
    --hold) shift; HOLD="${1:?--hold requer segundos}" ;;
    -h|--help)
      sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    -*) echo "Opção desconhecida: $1" >&2; exit 2 ;;
    *) WORLDS+=("$1") ;;
  esac
  shift
done

# Default: relevo crescente + obstáculos (plano → gentle → rugged → com árvores/rochas).
if [[ ${#WORLDS[@]} -eq 0 ]]; then
  WORLDS=(
    forest_flat
    forest_gentle
    forest_rugged
    forest_gentle_trees_rocks
    forest_rugged_trees_rocks
  )
fi

echo "=== worlds-tour: ${#WORLDS[@]} mundos | profile=${PROFILE} | hold=${HOLD:-Enter} ==="
echo "    Mundos: ${WORLDS[*]}"
echo ""

cleanup() { "${FOREST_BIN}" down --force >/dev/null 2>&1 || true; }
trap 'echo; echo "Tour interrompido."; cleanup; exit 130' INT

i=0
for world in "${WORLDS[@]}"; do
  i=$((i + 1))
  echo "──────────────────────────────────────────────────────────────"
  echo "[$i/${#WORLDS[@]}] Mundo: ${world}"
  echo "──────────────────────────────────────────────────────────────"

  # Limpar o mundo anterior antes de subir o próximo.
  "${FOREST_BIN}" down --force >/dev/null 2>&1 || true

  if ! "${FOREST_BIN}" up "${PROFILE}" -d --world "${world}"; then
    echo "  AVISO: falha ao abrir '${world}' (timeout/erro). A saltar." >&2
    continue
  fi

  echo ""
  echo "  ▶ Gazebo + RViz abertos para '${world}'. Inspeciona o terreno."
  if [[ -n "${HOLD}" ]]; then
    echo "    (a aguardar ${HOLD}s…)"
    sleep "${HOLD}"
  else
    if [[ $i -lt ${#WORLDS[@]} ]]; then
      read -r -p "    Enter para o PRÓXIMO mundo (Ctrl-C para sair)… " _
    else
      read -r -p "    Último mundo. Enter para terminar e fazer cleanup… " _
    fi
  fi
done

echo ""
echo "=== Tour concluído — cleanup ==="
cleanup
echo "Done."
