#!/usr/bin/env python3
"""Measure ground-reference / nDSM height quality from live ROS data (no assumptions).

Requires stack with publish_ground_ref_debug:=true (slice overlay enables it).

Usage:
  source install/setup.bash
  forest up sim-lidar3d-test -d --world forest_rugged_trees_rocks --trunk-slice
  python3 tools/diagnostics/lidar3d_ground_ref_probe.py --duration 30

RViz (Intensity = measured field):
  /perception/lidar3d/debug/ground_ref_h          → h = p.z - zg_ndsm
  /perception/lidar3d/debug/ground_ref_zg         → zg at (x,y), intensity = confidence
  /perception/lidar3d/debug/ground_ref_no_obs_cells
  /perception/lidar3d/debug/ground_ref_confidence_cells
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String


@dataclass
class Hist:
    bins: dict[str, int] = field(default_factory=dict)

    def add(self, key: str, n: int = 1) -> None:
        self.bins[key] = self.bins.get(key, 0) + n


@dataclass
class Agg:
    frames_stats: int = 0
    frames_h: int = 0
    h_sum: float = 0.0
    h_low: float = 0.0
    h_mid: float = 0.0
    h_high: float = 0.0
    h_below_min: float = 0.0
    h_in_band: float = 0.0
    h_above_max: float = 0.0
    ndsm_min: float = 0.2
    ndsm_max: float = 4.0
    gr_under_inpaint: list[float] = field(default_factory=list)
    gr_h_below: list[float] = field(default_factory=list)
    gr_h_in_band: list[float] = field(default_factory=list)
    zg_conf_low_h: list[float] = field(default_factory=list)
    zg_conf_high_h: list[float] = field(default_factory=list)
    last_grid: dict[str, float] = field(default_factory=dict)


def pct(n: float, d: float) -> float:
    return 100.0 * n / d if d > 0 else 0.0


def mean(xs: list[float]) -> float:
    return sum(xs) / len(xs) if xs else float("nan")


class GroundRefProbe(Node):
    def __init__(self, duration: float, ndsm_min: float, ndsm_max: float) -> None:
        super().__init__(
            "lidar3d_ground_ref_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._agg = Agg(ndsm_min=ndsm_min, ndsm_max=ndsm_max)
        qos = qos_profile_sensor_data

        self.create_subscription(String, "/perception/lidar3d/debug_stats", self._on_stats, 10)
        self.create_subscription(
            PointCloud2, "/perception/lidar3d/debug/ground_ref_h", self._on_h, qos
        )
        self.create_subscription(
            PointCloud2, "/perception/lidar3d/debug/ground_ref_zg", self._on_zg, qos
        )
        self.create_timer(0.5, self._tick)
        print(f"Probing ground reference for {duration:.0f}s …")
        print("  Ensure publish_ground_ref_debug:=true and node was restarted after rebuild.\n")

    def _on_stats(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "gr_nonground_pts" not in d:
            return
        a = self._agg
        a.frames_stats += 1
        n = max(1.0, float(d.get("gr_nonground_pts", 1)))
        a.gr_under_inpaint.append(pct(float(d.get("gr_under_inpaint_cell", 0)), n))
        a.gr_h_below.append(pct(float(d.get("gr_h_below_ndsm_min", 0)), n))
        a.gr_h_in_band.append(pct(float(d.get("gr_h_in_ndsm_band", 0)), n))
        a.last_grid = {
            "mesh_obs": float(d.get("gr_mesh_obs_cells", 0)),
            "mesh_inpaint": float(d.get("gr_mesh_inpaint_cells", 0)),
            "surface_inpaint": float(d.get("gr_surface_inpaint_cells", 0)),
            "mean_h": float(d.get("gr_mean_h_m", 0)),
            "mean_inpaint_delta": float(d.get("gr_mean_inpaint_delta_m", 0)),
            "mean_zg_minus_obs": float(d.get("gr_mean_zg_minus_obs_m", 0)),
            "pct_under_inpaint": pct(float(d.get("gr_under_inpaint_cell", 0)), n),
            "pct_h_below": pct(float(d.get("gr_h_below_ndsm_min", 0)), n),
            "pct_h_in_band": pct(float(d.get("gr_h_in_ndsm_band", 0)), n),
        }

    def _on_h(self, msg: PointCloud2) -> None:
        a = self._agg
        a.frames_h += 1
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            h = float(p[3])
            a.h_sum += h
            if h < a.ndsm_min:
                a.h_below_min += 1
            elif h > a.ndsm_max:
                a.h_above_max += 1
            else:
                a.h_in_band += 1
            if h < 0.5:
                a.h_low += 1
            elif h < 2.0:
                a.h_mid += 1
            else:
                a.h_high += 1

    def _on_zg(self, msg: PointCloud2) -> None:
        # Correlate confidence (intensity) with h from same message index not available;
        # stats JSON already aggregates per-frame.
        pass

    def _tick(self) -> None:
        if time.monotonic() - self._t0 < self._duration:
            return
        self._report()
        rclpy.shutdown()

    def _report(self) -> None:
        a = self._agg
        print("=" * 72)
        print("GROUND REFERENCE PROBE REPORT")
        print("=" * 72)
        if a.frames_stats == 0:
            print("NO debug_stats with gr_* fields — rebuild forest_3d_perception,")
            print("restart stack, and set publish_ground_ref_debug: true")
            return
        if a.frames_h == 0:
            print("NO ground_ref_h messages — check publish_ground_ref_debug and RViz topic.")
        else:
            total_pts = a.h_below_min + a.h_in_band + a.h_above_max
            print(f"Point cloud /debug/ground_ref_h: {a.frames_h} frames, ~{total_pts} samples")
            print(f"  h < {a.ndsm_min} m (tronco rejeitado se banda começa aqui): "
                  f"{pct(a.h_below_min, total_pts):.1f}%")
            print(f"  h in [{a.ndsm_min}, {a.ndsm_max}] m (banda nDSM): "
                  f"{pct(a.h_in_band, total_pts):.1f}%")
            print(f"  h > {a.ndsm_max} m: {pct(a.h_above_max, total_pts):.1f}%")
            print(f"  h < 0.5 m / 0.5–2 / >2 m: "
                  f"{pct(a.h_low, total_pts):.1f}% / "
                  f"{pct(a.h_mid, total_pts):.1f}% / "
                  f"{pct(a.h_high, total_pts):.1f}%")

        print(f"\nPer-frame JSON (n={a.frames_stats}) — médias:")
        print(f"  % nonground sob célula mesh INPAINT: {mean(a.gr_under_inpaint):.1f}%")
        print(f"  % nonground com h < ndsm_min:        {mean(a.gr_h_below):.1f}%")
        print(f"  % nonground na banda nDSM:           {mean(a.gr_h_in_band):.1f}%")
        if a.last_grid:
            g = a.last_grid
            print(f"\nÚltimo frame grid:")
            print(f"  mesh observed / inpaint cells: {g['mesh_obs']:.0f} / {g['mesh_inpaint']:.0f}")
            print(f"  surface inpaint cells:       {g['surface_inpaint']:.0f}")
            print(f"  mean h (nonground):          {g['mean_h']:.3f} m")
            print(f"  mean inpaint delta (mesh):   {g['mean_inpaint_delta']:.3f} m")
            print(f"  mean zg - z_obs (obs cells): {g['mean_zg_minus_obs']:.3f} m")

        print("\nInterpretação (só com estes números):")
        if mean(a.gr_h_below) > 40.0:
            print("  → >40% nonground com h < ndsm_min: zg alto vs base do tronco (dado medido).")
        if mean(a.gr_under_inpaint) > 30.0:
            print("  → Grande fração sob células sem solo observado (inpaint).")
        print("=" * 72)


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe nDSM ground reference from live topics")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--ndsm-min", type=float, default=0.2)
    ap.add_argument("--ndsm-max", type=float, default=2.8)
    args = ap.parse_args()

    rclpy.init()
    node = GroundRefProbe(args.duration, args.ndsm_min, args.ndsm_max)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
