#!/usr/bin/env bash
# Copia o ONNX treinado para o pacote ROS2 (após forest-export-onnx).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="${1:-$HOME/Projetos/Tese/forest-semantic-training/artifacts/model.onnx}"
DST="$ROOT/src/perception_stack/forest_semantic_segmentation/models/forest_semantic.onnx"
if [[ ! -f "$SRC" ]]; then
  echo "Missing ONNX: $SRC" >&2
  exit 1
fi
mkdir -p "$(dirname "$DST")"
cp -f "$SRC" "$DST"
echo "Synced $SRC -> $DST"
ls -lh "$DST"
