# shellcheck shell=bash
# forest capture — arranca um mundo de captura de visão, vigia a contagem de
# imagens e faz `forest down` automaticamente quando o alvo é atingido.
# Tu conduzes (teleop/missão); isto só pára e fecha a sim quando há imagens que
# cheguem.
[[ -n "${_FOREST_CAPTURE_LOADED:-}" ]] && return 0
_FOREST_CAPTURE_LOADED=1

source "$(dirname "${BASH_SOURCE[0]}")/env.bash"
source "$(dirname "${BASH_SOURCE[0]}")/worlds.bash"

# Alvo de imagens por mundo (cf. orçamento sugerido). Fallback p/ mundos não listados.
forest_capture_default_target() {
  case "$1" in
    vision_logs_only)      echo 450 ;;
    vision_bushes_only)    echo 400 ;;
    vision_rocks_only)     echo 350 ;;
    vision_trees_only)     echo 300 ;;
    vision_mixed_gentle)   echo 350 ;;
    vision_mixed_moderate) echo 400 ;;
    vision_mixed_steep)    echo 350 ;;
    *)                     echo 400 ;;
  esac
}

forest_capture_usage() {
  cat <<'EOF'
forest capture <mundo> [alvo] — captura até ao alvo de imagens e fecha a sim

  Arranca o profile sim-vision-capture com <mundo>, espera o labeler, e fica a
  vigiar a pasta do dataset. Quando o nº TOTAL de imagens (train+val) chega ao
  alvo, faz `forest down` automaticamente. Conduz tu com teleop ou a tua missão.

Uso:
  forest capture vision_logs_only            # alvo por-mundo (450)
  forest capture vision_mixed_steep 600      # alvo explícito
  forest capture vision_rocks_only --poll 3  # verifica a cada 3 s

Opções:
  --profile P     Profile a arrancar (default: sim-vision-capture)
  --out DIR       Raiz do dataset (default: ~/datasets/forest_vision_labels)
                  A pasta efetiva é <DIR>/<mundo> (uma por mundo).
  --poll S        Intervalo de verificação em segundos (default: 5)
  --stall S       Avisa se não houver imagens novas durante S s (default: 120; 0=off)
  --no-down       Não fazer forest down no fim (só pára de vigiar)
  -h, --help      Esta ajuda
EOF
}

# Conta imagens (train+val) na pasta do dataset.
_forest_capture_count() {
  local d="$1" n=0
  local s
  for s in train val; do
    if [[ -d "${d}/images/${s}" ]]; then
      n=$(( n + $(find "${d}/images/${s}" -maxdepth 1 -name '*.jpg' 2>/dev/null | wc -l) ))
    fi
  done
  echo "$n"
}

forest_run_capture() {
  local profile="sim-vision-capture"
  local out_base="${FOREST_VISION_OUT:-$HOME/datasets/forest_vision_labels}"
  local poll=5 stall=120 do_down=true
  local world="" target=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) forest_capture_usage; return 0 ;;
      --profile) shift; profile="${1:?--profile requer nome}" ;;
      --out)     shift; out_base="${1:?--out requer caminho}" ;;
      --poll)    shift; poll="${1:?--poll requer segundos}" ;;
      --stall)   shift; stall="${1:?--stall requer segundos}" ;;
      --no-down) do_down=false ;;
      -*) echo "Opção desconhecida: $1" >&2; forest_capture_usage >&2; return 2 ;;
      *)
        if [[ -z "$world" ]]; then world="$1"
        elif [[ -z "$target" ]]; then target="$1"
        else echo "Argumento a mais: $1" >&2; return 2; fi
        ;;
    esac
    shift
  done

  if [[ -z "$world" ]]; then
    echo "ERROR: falta o nome do mundo." >&2
    forest_capture_usage >&2
    return 2
  fi
  world="${world%.sdf}"   # aceita nome com ou sem .sdf
  [[ -z "$target" ]] && target="$(forest_capture_default_target "$world")"
  if ! [[ "$target" =~ ^[0-9]+$ ]] || (( target <= 0 )); then
    echo "ERROR: alvo inválido: $target" >&2; return 2
  fi

  local out_dir="${out_base}/${world}"
  local forest_bin="${FOREST_ROOT}/bin/forest"

  local start_n; start_n="$(_forest_capture_count "$out_dir")"
  echo "=== forest capture ==="
  echo "  mundo:   ${world}"
  echo "  profile: ${profile}"
  echo "  pasta:   ${out_dir}"
  echo "  alvo:    ${target} imagens (já lá estão ${start_n})"
  echo ""

  if (( start_n >= target )); then
    echo "Já tens ${start_n} ≥ ${target} imagens. Nada a capturar."
    return 0
  fi

  # Arranca o stack (detached) com o mundo escolhido.
  echo ">> forest up ${profile} -d --world ${world}"
  if ! "$forest_bin" up "$profile" -d --world "$world"; then
    echo "ERROR: 'forest up' falhou." >&2
    return 1
  fi

  # Garante teardown se o utilizador interromper (Ctrl-C) ou ao sair.
  local _torn_down=false
  _forest_capture_teardown() {
    $_torn_down && return 0
    _torn_down=true
    if $do_down; then
      echo ""
      echo ">> forest down"
      "$forest_bin" down || true
    else
      echo "(--no-down: sim deixada a correr)"
    fi
  }
  trap '_forest_capture_teardown; trap - INT TERM EXIT; return 130' INT TERM

  forest_source_ros || { _forest_capture_teardown; return 1; }

  # Espera o labeler aparecer.
  echo -n "A aguardar o nó /gz_auto_labeler "
  local waited=0
  until ros2 node list 2>/dev/null | tr -d ' ' | grep -qx "/gz_auto_labeler"; do
    sleep 2; waited=$(( waited + 2 )); echo -n "."
    if (( waited >= 60 )); then
      echo ""; echo "ERROR: labeler não arrancou em 60 s." >&2
      _forest_capture_teardown; return 1
    fi
  done
  echo " ok"
  echo ""
  echo "CONDUZ AGORA (outro terminal):  forest teleop"
  echo "Pára e fecha sozinho ao chegar a ${target} imagens. (Ctrl-C cancela + down.)"
  echo ""

  # Vigia a contagem.
  local last_n="$start_n" last_change_t; last_change_t="$(date +%s)"
  local n now warned_stall=false
  while :; do
    sleep "$poll"
    n="$(_forest_capture_count "$out_dir")"
    now="$(date +%s)"
    if (( n != last_n )); then
      last_n="$n"; last_change_t="$now"; warned_stall=false
    fi
    printf '\r  %d / %d imagens (+%d nesta sessão)        ' "$n" "$target" "$(( n - start_n ))"
    if (( n >= target )); then
      echo ""; echo ""
      echo "ALVO ATINGIDO: ${n} ≥ ${target} imagens."
      break
    fi
    if (( stall > 0 )) && ! $warned_stall && (( now - last_change_t >= stall )); then
      echo ""
      echo "  AVISO: sem imagens novas há ${stall}s — o robô está parado? (conduz com teleop)"
      warned_stall=true
    fi
  done

  trap - INT TERM
  _forest_capture_teardown
  echo ""
  echo "Captura concluída: ${last_n} imagens em ${out_dir}"
  echo "Rever: forest diag vision-labels --render 24   (ou por-mundo: --output-dir ${out_dir})"
  return 0
}
