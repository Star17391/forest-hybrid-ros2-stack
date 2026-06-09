#!/usr/bin/env bash
# Vendor CSF (Cloth Simulation Filter) for offline builds.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/src/perception_stack/forest_3d_perception/third_party/csf"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

git clone --depth 1 https://github.com/jianboqi/CSF.git "$TMP/csf"
mkdir -p "$DEST"
cp -a "$TMP/csf/src/"* "$DEST/"
echo "CSF vendored to $DEST"
ls -la "$DEST"
