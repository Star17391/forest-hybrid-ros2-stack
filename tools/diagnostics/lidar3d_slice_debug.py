#!/usr/bin/env python3
"""Monitor slice trunk pipeline rejections (30s teleop-friendly).

Usage (stack running with trunk_method=slice):
  source install/setup.bash
  python3 tools/diagnostics/lidar3d_slice_debug.py --duration 30

While it runs: use `forest teleop` or `forest random_move` near/far from trees.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


@dataclass
class Agg:
    frames: int = 0
    with_trunks: int = 0
    band_sum: float = 0.0
    cols_sum: float = 0.0
    stems_sum: float = 0.0
    c2d_sum: float = 0.0
    rej_cluster_small_sum: float = 0.0
    funnel_cells_sum: float = 0.0
    rej: dict[str, int] = field(default_factory=dict)
    best_slices_max: int = 0
    best_cont_max: float = 0.0
    best_drift_max: float = 0.0
    best_bottom_dz_last: float = 0.0
    last_dominant: str = ""

    def add_rej(self, key: str, n: int) -> None:
        if n:
            self.rej[key] = self.rej.get(key, 0) + n

    def dominant_reject(self) -> tuple[str, int]:
        if not self.rej:
            return "none", 0
        k = max(self.rej, key=self.rej.get)
        return k, self.rej[k]


class SliceDebugMonitor(Node):
    def __init__(self, duration: float, interval: float) -> None:
        super().__init__(
            "lidar3d_slice_debug",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._interval = interval
        self._t0 = time.monotonic()
        self._last_print = 0.0
        self._agg = Agg()
        self.create_subscription(String, "/perception/lidar3d/debug_stats", self._on_dbg, 10)
        self.create_timer(0.5, self._tick)
        print(f"Monitoring /perception/lidar3d/debug_stats for {duration:.0f}s …")
        print("  Use forest teleop / random_move and approach trees.\n")

    def _on_dbg(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get("trunk_method") != "slice":
            return

        a = self._agg
        a.frames += 1
        n_trunk = int(d.get("n_trunk", 0))
        if n_trunk > 0:
            a.with_trunks += 1
        a.band_sum += float(d.get("n_ndsm_band", 0))
        a.cols_sum += float(d.get("n_columns_found", 0))
        a.stems_sum += float(d.get("slice_n_stems", 0))
        a.c2d_sum += float(d.get("slice_n_2d_clusters", 0))
        a.rej_cluster_small_sum += float(d.get("funnel_rej_cluster_small", 0))
        a.funnel_cells_sum += float(d.get("funnel_grid_cells", 0))

        a.add_rej("cluster_2d_small", int(d.get("funnel_rej_cluster_small", 0)))
        a.add_rej("sparse_slices", int(d.get("slice_rej_sparse_slices", 0)))
        a.add_rej("cont_score", int(d.get("slice_rej_cont_score", 0)))
        a.add_rej("cont_persist", int(d.get("slice_rej_cont_persist", 0)))
        a.add_rej("cont_drift", int(d.get("slice_rej_cont_drift", 0)))
        a.add_rej("ground_dz", int(d.get("slice_rej_ground_dz", 0)))
        a.add_rej("ground_cell", int(d.get("slice_rej_ground_cell", 0)))
        a.add_rej("cylinder", int(d.get("slice_rej_cylinder", 0)))

        bs = int(d.get("slice_best_rej_slices", 0))
        a.best_slices_max = max(a.best_slices_max, bs)
        a.best_cont_max = max(a.best_cont_max, float(d.get("slice_best_cont", 0)))
        a.best_drift_max = max(a.best_drift_max, float(d.get("slice_best_drift", 0)))
        a.best_bottom_dz_last = float(d.get("slice_best_bottom_dz", 0))

        dom, _ = self._dominant_frame(d)
        a.last_dominant = dom

    def _dominant_frame(self, d: dict) -> tuple[str, int]:
        counts = {
            "cluster_2d_small": int(d.get("funnel_rej_cluster_small", 0)),
            "sparse_slices": int(d.get("slice_rej_sparse_slices", 0)),
            "cont_score": int(d.get("slice_rej_cont_score", 0)),
            "cont_persist": int(d.get("slice_rej_cont_persist", 0)),
            "cont_drift": int(d.get("slice_rej_cont_drift", 0)),
            "ground_dz": int(d.get("slice_rej_ground_dz", 0)),
            "ground_cell": int(d.get("slice_rej_ground_cell", 0)),
            "cylinder": int(d.get("slice_rej_cylinder", 0)),
        }
        if not any(counts.values()):
            return "none", 0
        k = max(counts, key=counts.get)
        return k, counts[k]

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last_print >= self._interval:
            self._print_live()
            self._last_print = now
        if now - self._t0 >= self._duration:
            self._print_final()
            rclpy.shutdown()

    def _print_live(self) -> None:
        a = self._agg
        if a.frames == 0:
            print("  … waiting for debug_stats (is lidar3d_segmentation_node running?)")
            return
        dom, dom_n = a.dominant_reject()
        print(
            f"  [{time.monotonic() - self._t0:5.1f}s] frames={a.frames} "
            f"trunk_frames={a.with_trunks} "
            f"avg_band={a.band_sum / a.frames:.0f} "
            f"avg_c2d={a.c2d_sum / a.frames:.1f} "
            f"avg_cols={a.cols_sum / a.frames:.1f} "
            f"rej_c2d_small={a.rej_cluster_small_sum / a.frames:.0f} "
            f"best_slices={a.best_slices_max} "
            f"best_cont={a.best_cont_max:.2f} "
            f"dominant_rej={dom}({dom_n})"
        )

    def _print_final(self) -> None:
        a = self._agg
        print("\n" + "=" * 68)
        print(f"  SLICE DEBUG SUMMARY ({self._duration:.0f}s)")
        print("=" * 68)
        if a.frames == 0:
            print("  No slice debug_stats received.")
            print("  Check: FOREST_LIDAR3D_TRUNK_SLICE=1 and node running.")
            return
        dom, dom_n = a.dominant_reject()
        print(f"  Frames:           {a.frames}")
        print(f"  Frames w/ trunks: {a.with_trunks} ({100 * a.with_trunks / a.frames:.1f}%)")
        print(f"  Avg nDSM band:    {a.band_sum / a.frames:.0f} pts")
        print(f"  Avg 2D clusters:  {a.c2d_sum / a.frames:.1f}")
        print(f"  Avg grid cells:   {a.funnel_cells_sum / a.frames:.0f}")
        print(f"  Avg rej c2d<min:  {a.rej_cluster_small_sum / a.frames:.0f}/frame")
        print(f"  Avg stems tracked:{a.stems_sum / a.frames:.1f}")
        print(f"  Avg accepted:     {a.cols_sum / a.frames:.2f} cols/frame")
        print(f"  Best near-miss:   slices={a.best_slices_max} cont={a.best_cont_max:.2f} "
              f"drift={a.best_drift_max:.2f} bottom_dz={a.best_bottom_dz_last:.2f}m")
        print(f"  Dominant reject:  {dom} (total events ~{dom_n})")
        print("\n  Rejection totals (summed over frames):")
        for k in sorted(a.rej, key=a.rej.get, reverse=True):
            print(f"    {k:16s} {a.rej[k]}")
        print("\n  Interpretation:")
        hints = {
            "cluster_2d_small": "BUG/heurística: BFS funde células mas <min_pts — corrigido se avg_c2d>0.",
            "sparse_slices": "Clusters não ligam ≥4 fatias — aproxima-te / rodeia o tronco.",
            "cont_score": "Continuidade baixa — copa fragmentada ou associação XY falhou.",
            "cont_persist": "Ocupação vertical baixa (fatias ligadas / altura spanned) — era bug n_slices/total_slices.",
            "cont_drift": "Espalhamento XY entre fatias (LiDAR esparso) — usar trunk_voxel 0.05 e drift RMS.",
            "ground_dz": "Base fora da banda solo–tronco (dz); ver slice_best_bottom_dz.",
            "ground_cell": "Base não ligada ao solo conectado (ground connectivity).",
            "cylinder": "Cylinder fit falhou (altura/raio/rmse/inliers).",
            "none": "Sem rejeições registadas — verificar band=0 ou outro método.",
        }
        print(f"    → {hints.get(dom, '?')}")
        print("=" * 68 + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Slice trunk rejection monitor")
    ap.add_argument("--duration", type=float, default=30.0, help="Seconds (default 30)")
    ap.add_argument("--interval", type=float, default=2.0, help="Print interval (default 2s)")
    args = ap.parse_args()

    rclpy.init()
    node = SliceDebugMonitor(args.duration, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._print_final()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
