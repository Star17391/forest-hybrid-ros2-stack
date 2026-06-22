#!/usr/bin/env bash
# Compila o probe contra o código de perceção REAL e corre-o.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INC="$HERE/../../include"
BIN="$HERE/probe"
PCL_INC="$(ls -d /usr/include/pcl-* | head -1)"

if [ ! -x "$BIN" ] || [ "$HERE/probe.cpp" -nt "$BIN" ] || [ "$INC/forest_3d_perception/cylinder_fit.hpp" -nt "$BIN" ]; then
  g++ -O2 -std=c++17 -I "$INC" -I "$PCL_INC" -I /usr/include/eigen3 \
    "$HERE/probe.cpp" -o "$BIN"
fi
"$BIN" "$@"
