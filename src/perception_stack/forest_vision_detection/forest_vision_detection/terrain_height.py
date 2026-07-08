#!/usr/bin/env python3
"""Terrain height lookup from a ForestGen terrain STL (no ROS, no trimesh).

Used to clip rock label boxes at the ground: rocks are placed partly buried
(vertical jitter + anchor), so the full mesh silhouette extends well below the
visible rock. Sampling the terrain height under each rock vertex lets the
labeller keep only the above-ground part.

Parses binary STL with numpy and answers height(x, y) via nearest grid vertex
(the terrain is a dense regular grid, so nearest-vertex is accurate to the cell).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - scipy expected in ROS env
    cKDTree = None


def _terrain_model_name(world_sdf: str) -> str | None:
    """Pull the custom_terrain model name referenced by a world SDF."""
    import re
    m = re.search(r"model://(custom_terrain[\w-]*)", Path(world_sdf).read_text())
    return m.group(1) if m else None


def terrain_stl_for_world(world_sdf: str, forestgen_models: Path) -> Path | None:
    name = _terrain_model_name(world_sdf)
    if not name:
        return None
    stl = forestgen_models / name / "meshes" / "terrain.stl"
    return stl if stl.is_file() else None


class TerrainHeightSampler:
    def __init__(self, stl_path: str | Path):
        verts = self._load_binary_stl_vertices(Path(stl_path))
        self._xy = verts[:, :2]
        self._z = verts[:, 2]
        self._tree = cKDTree(self._xy) if cKDTree is not None else None

    @staticmethod
    def _load_binary_stl_vertices(path: Path) -> np.ndarray:
        data = np.fromfile(path, dtype=np.uint8)
        n = int(np.frombuffer(data[80:84].tobytes(), dtype="<u4")[0])
        rec = data[84:84 + n * 50].reshape(n, 50)
        # bytes 12..48 of each 50-byte record = 3 vertices × 3 float32
        v = np.frombuffer(rec[:, 12:48].tobytes(), dtype="<f4").reshape(n * 3, 3)
        return v.astype(np.float64)

    def height(self, xy: np.ndarray) -> np.ndarray:
        """Nearest-vertex terrain height for an (N, 2) array of x,y points."""
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if self._tree is not None:
            _, idx = self._tree.query(xy)
            return self._z[idx]
        # Fallback without scipy: brute-force nearest (slow, rarely used).
        out = np.empty(len(xy))
        for i, p in enumerate(xy):
            out[i] = self._z[np.argmin(np.sum((self._xy - p) ** 2, axis=1))]
        return out
