#!/usr/bin/env bash
# Generate acados NMPC solver for forest_navigation_ros2 (Fase NMPC-0).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OCP_DIR="$ROOT/src/navigation_stack/forest_navigation_ros2/ocp"
TERA="$ROOT/tools/third_party/bin/t_renderer"

: "${ACADOS_SOURCE_DIR:=$HOME/acados}"
export ACADOS_SOURCE_DIR
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ACADOS_SOURCE_DIR}/lib"

if [[ -f "$HOME/acados/venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/acados/venv/bin/activate"
fi

TERA_SRC="${ACADOS_SOURCE_DIR}/interfaces/acados_template/tera_renderer"

build_bundled_tera() {
  if ! command -v cargo >/dev/null 2>&1; then
    return 1
  fi
  echo "Building t_renderer from bundled acados source..."
  mkdir -p "$(dirname "$TERA")"
  (cd "$TERA_SRC" && cargo build --release)
  install -m 755 "$TERA_SRC/target/release/t_renderer" "$TERA"
}

if [[ ! -x "$TERA" ]] || [[ ! -s "$TERA" ]]; then
  build_bundled_tera || true
fi

if [[ ! -x "$TERA" ]] || [[ ! -s "$TERA" ]]; then
  echo "t_renderer not found or empty at $TERA"
  echo "Install once:"
  echo "  mkdir -p $(dirname "$TERA")"
  echo "  curl -fsSL -o $TERA \\"
  echo "    https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64"
  echo "  chmod +x $TERA"
  echo "Or install Rust (cargo) and re-run this script to build from acados bundled source."
  exit 1
fi

GEN_DIR="$ROOT/src/navigation_stack/forest_navigation_ros2/acados_generated/skid_steer_kin"
if [[ -d "$GEN_DIR" ]]; then
  rm -f "$GEN_DIR/libacados_ocp_solver_skid_steer_kin.so"
fi

export TERA_PATH="$TERA"

cd "$OCP_DIR"
python3 generate_skid_steer_ocp.py "$@"

echo "Done. Rebuild: cd $ROOT && colcon build --packages-select forest_navigation_ros2"
