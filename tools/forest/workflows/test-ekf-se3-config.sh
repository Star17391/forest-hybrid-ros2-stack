#!/usr/bin/env bash
# forest test ekf-se3-config — validação estática EKF SE(3) (sem Gazebo).
#
# Verifica alinhamento ekf_local/global, imu_sanitize, freeze legacy, cylinder comments.
# Uso: forest test ekf-se3-config [--repo PATH]
set -euo pipefail

FOREST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${FOREST_ROOT}/../.." && pwd)"
SCRIPT="${REPO_ROOT}/tools/diagnostics/ekf_se3_config_validate.py"

REPO="$REPO_ROOT"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo) shift; REPO="${1:?}" ;;
    -h|--help)
      echo "Usage: forest test ekf-se3-config [--repo PATH]"
      exit 0
      ;;
    *) echo "Unknown: $1" >&2; exit 2 ;;
  esac
  shift
done

[[ -f "$SCRIPT" ]] || { echo "ERROR: missing $SCRIPT" >&2; exit 1; }
python3 "$SCRIPT" --repo "$REPO"
