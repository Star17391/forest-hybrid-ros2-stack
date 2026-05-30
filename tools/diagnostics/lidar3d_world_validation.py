#!/usr/bin/env python3
"""Validação LiDAR 3D para um mundo (stack já a correr).

Agrega num único relatório:
- TF map→laser (stack monitor)
- Tópicos segmentation (seg audit)
- Estatísticas quantitativas /perception/lidar3d/debug_stats (rejeições tronco)

Uso (sessão forest up):
  python3 tools/diagnostics/lidar3d_world_validation.py \\
    --world-id W3 --world-name forest_rugged_trees_rocks \\
    --duration 40 --out-dir reports/lidar3d_validation/run/W3
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


@dataclass
class TopicStats:
    count: int = 0
    total_pts: int = 0


@dataclass
class DebugAgg:
    frames: int = 0
    ok_frames: int = 0
    tf_fail: int = 0
    status_counts: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    ground_plane_rejected: int = 0
    n_clusters_sum: int = 0
    n_trunk_sum: int = 0
    reject_height: int = 0
    reject_radius: int = 0
    reject_verticality: int = 0
    reject_small: int = 0
    n_voxel_sum: int = 0
    n_ground_sum: int = 0
    n_nonground_sum: int = 0
    n_holes_sum: int = 0
    grid_coverage_sum: float = 0.0
    grid_mean_dz_sum: float = 0.0
    grid_unknown_sum: int = 0
    ground_method: str = ""
    gcr_sum: float = 0.0
    suspended_pct_sum: float = 0.0
    n_ground_raw_sum: int = 0
    n_suspended_sum: int = 0
    mean_dz_connected_sum: float = 0.0
    trunk_method: str = ""
    n_columns_sum: int = 0
    n_tracks_sum: int = 0
    reject_column_sparse: int = 0
    reject_cylinder_rmse: int = 0


class WorldValidation(Node):
    def __init__(self, duration: float, world_id: str, world_name: str, out_dir: Path | None):
        super().__init__(
            "lidar3d_world_validation",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._world_id = world_id
        self._world_name = world_name
        self._out_dir = out_dir
        self._t0 = time.monotonic()

        self._tf_ok = 0
        self._tf_fail = 0
        self._static_edges: set[tuple[str, str]] = set()

        self._input = TopicStats()
        self._ground = TopicStats()
        self._trunks = TopicStats()
        self._obstacles = TopicStats()
        self._debug = DebugAgg()

        qos = qos_profile_sensor_data
        self.create_subscription(PointCloud2, "/sensors/lidar/points", self._on_input, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/ground", self._on_ground, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/trunks", self._on_trunks, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/obstacles", self._on_obstacles, qos)
        self.create_subscription(String, "/perception/lidar3d/debug_stats", self._on_debug, 10)
        self.create_subscription(TFMessage, "/tf_static", self._on_static, TF_STATIC_QOS)

        from tf2_ros import Buffer, TransformListener

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._timer = self.create_timer(0.5, self._probe_tf)

    def _on_static(self, msg: TFMessage):
        for t in msg.transforms:
            self._static_edges.add((t.header.frame_id, t.child_frame_id))

    def _n_pts(self, msg: PointCloud2) -> int:
        return int(msg.width) * int(msg.height)

    def _on_input(self, msg: PointCloud2):
        self._input.count += 1
        self._input.total_pts += self._n_pts(msg)

    def _on_ground(self, msg: PointCloud2):
        self._ground.count += 1
        self._ground.total_pts += self._n_pts(msg)

    def _on_trunks(self, msg: PointCloud2):
        self._trunks.count += 1
        self._trunks.total_pts += self._n_pts(msg)

    def _on_obstacles(self, msg: PointCloud2):
        self._obstacles.count += 1
        self._obstacles.total_pts += self._n_pts(msg)

    def _on_debug(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._debug.frames += 1
        status = str(d.get("status", ""))
        if status:
            self._debug.status_counts[status] += 1
        if status == "ok":
            self._debug.ok_frames += 1
        elif status == "tf_fail":
            self._debug.tf_fail += 1
        if d.get("ground_plane_rejected"):
            self._debug.ground_plane_rejected += 1
        self._debug.n_clusters_sum += int(d.get("n_clusters", 0))
        self._debug.n_trunk_sum += int(d.get("n_trunk", 0))
        self._debug.reject_height += int(d.get("reject_height", 0))
        self._debug.reject_radius += int(d.get("reject_radius", 0))
        self._debug.reject_verticality += int(d.get("reject_verticality", 0))
        self._debug.reject_small += int(d.get("reject_cluster_small", 0))
        self._debug.n_voxel_sum += int(d.get("n_voxel", 0))
        self._debug.n_ground_sum += int(d.get("n_ground", 0))
        self._debug.n_nonground_sum += int(d.get("n_nonground", 0))
        self._debug.n_holes_sum += int(d.get("n_holes", 0))
        if d.get("ground_method"):
            self._debug.ground_method = str(d["ground_method"])
        if d.get("trunk_method"):
            self._debug.trunk_method = str(d["trunk_method"])
        self._debug.n_columns_sum += int(d.get("n_columns_found", 0))
        self._debug.n_tracks_sum += int(d.get("n_tracks", 0))
        self._debug.reject_column_sparse += int(d.get("reject_column_sparse", 0))
        self._debug.reject_cylinder_rmse += int(d.get("reject_cylinder_rmse", 0))
        if status == "ok":
            self._debug.grid_coverage_sum += float(d.get("grid_coverage_pct", 0.0))
            self._debug.grid_mean_dz_sum += float(d.get("grid_mean_abs_dz_ground_m", 0.0))
            self._debug.grid_unknown_sum += int(d.get("grid_unknown_pts", 0))
            gc = d.get("ground_connectivity")
            if gc is True or (isinstance(gc, str) and gc.lower() == "true"):
                self._debug.gcr_sum += float(d.get("gcr_pct", 0.0))
                self._debug.suspended_pct_sum += float(d.get("suspended_pct_of_raw", 0.0))
                self._debug.n_ground_raw_sum += int(d.get("n_ground_raw", 0))
                self._debug.n_suspended_sum += int(d.get("n_suspended", 0))
                self._debug.mean_dz_connected_sum += float(d.get("mean_abs_dz_connected_m", 0.0))

    def _probe_tf(self):
        try:
            if self._buf.can_transform(
                "map", "laser", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            ):
                self._tf_ok += 1
            else:
                self._tf_fail += 1
        except Exception:
            self._tf_fail += 1

        if time.monotonic() - self._t0 >= self._duration:
            self._finish()
            rclpy.shutdown()

    def _finish(self):
        total_tf = self._tf_ok + self._tf_fail
        tf_pct = 100.0 * self._tf_ok / total_tf if total_tf else 0.0

        seg_total = self._ground.total_pts + self._trunks.total_pts + self._obstacles.total_pts
        g_pct = 100.0 * self._ground.total_pts / seg_total if seg_total else 0.0
        t_pct = 100.0 * self._trunks.total_pts / seg_total if seg_total else 0.0
        o_pct = 100.0 * self._obstacles.total_pts / seg_total if seg_total else 0.0

        avg_clusters = (
            self._debug.n_clusters_sum / self._debug.ok_frames if self._debug.ok_frames else 0.0
        )
        avg_trunk = (
            self._debug.n_trunk_sum / self._debug.ok_frames if self._debug.ok_frames else 0.0
        )
        avg_grid_cov = (
            self._debug.grid_coverage_sum / self._debug.ok_frames
            if self._debug.ok_frames
            else 0.0
        )
        avg_grid_dz = (
            self._debug.grid_mean_dz_sum / self._debug.ok_frames
            if self._debug.ok_frames
            else 0.0
        )
        ground_method = self._debug.ground_method or "unknown"
        using_grid = ground_method == "grid"

        issues: list[str] = []
        warnings: list[str] = []
        if self._input.count == 0:
            issues.append("no_lidar_input")
        if tf_pct < 90.0 and total_tf > 5:
            issues.append(f"tf_map_laser_low_{tf_pct:.0f}pct")
        if self._ground.count == 0 and self._input.count > 3:
            issues.append("no_ground_output")
            if self._debug.frames == 0:
                issues.append("no_debug_stats")
            elif self._debug.ok_frames == 0:
                top = sorted(self._debug.status_counts.items(), key=lambda x: -x[1])[:3]
                issues.append(f"debug_status_{top[0][0]}" if top else "debug_no_ok_status")
        if self._debug.ok_frames == 0 and self._debug.frames > 0:
            issues.append("seg_no_ok_frames")
        if not using_grid and self._debug.ok_frames > 0:
            issues.append("ground_method_not_grid")

        seg_total_debug = self._debug.n_ground_sum + self._debug.n_nonground_sum
        debug_ground_pct = (
            100.0 * self._debug.n_ground_sum / seg_total_debug if seg_total_debug else 100.0
        )

        avg_gcr = (
            self._debug.gcr_sum / self._debug.ok_frames if self._debug.ok_frames else 0.0
        )
        avg_susp = (
            self._debug.suspended_pct_sum / self._debug.ok_frames
            if self._debug.ok_frames
            else 0.0
        )
        avg_dz_conn = (
            self._debug.mean_dz_connected_sum / self._debug.ok_frames
            if self._debug.ok_frames
            else 0.0
        )

        if using_grid and self._debug.ok_frames > 0:
            # Robot estático: só ~20–45% das células 30×30 m recebem hits LiDAR (normal).
            if avg_grid_cov < 12.0:
                issues.append(f"grid_coverage_low_{avg_grid_cov:.0f}pct")
            if avg_grid_dz > 0.18 and avg_dz_conn <= 0.0:
                issues.append(f"grid_ground_fit_poor_dz_{avg_grid_dz:.3f}m")
            if avg_dz_conn > 0.12:
                issues.append(f"ground_connected_dz_high_{avg_dz_conn:.3f}m")
            # Sprint 1 — Fase A (W0): solo conectado estável
            if self._world_id == "W0":
                if avg_gcr < 88.0:
                    issues.append(f"gcr_low_{avg_gcr:.0f}pct")
                if avg_susp > 12.0:
                    issues.append(f"suspended_high_{avg_susp:.0f}pct")
            if self._world_id in ("W1", "W2", "W3") and debug_ground_pct > 98.0:
                issues.append(f"seg_collapsed_to_ground_{debug_ground_pct:.0f}pct")
            if self._world_id == "W2" and o_pct < 2.0 and debug_ground_pct > 95.0:
                issues.append("w2_obstacle_pct_too_low_vs_rugged_terrain")
            if self._world_id == "W3" and o_pct > 75.0:
                issues.append(f"w3_obstacle_pct_high_{o_pct:.0f}pct")
            if self._world_id == "W0" and g_pct < 70.0:
                issues.append(f"w0_ground_pct_low_{g_pct:.0f}pct")

        if self._world_id in ("W1", "W3") and self._trunks.total_pts == 0:
            warnings.append("zero_trunks_expected_until_phase_1c")
        if self._debug.ok_frames > 0 and avg_trunk < 0.1 and self._world_id in ("W1", "W3"):
            warnings.append("trunk_clusters_near_zero")

        dom_reject = "none"
        if self._debug.ok_frames > 0:
            rejects = {
                "radius": self._debug.reject_radius,
                "verticality": self._debug.reject_verticality,
                "height": self._debug.reject_height,
                "small": self._debug.reject_small,
            }
            dom_reject = max(rejects, key=rejects.get)  # type: ignore[arg-type]

        verdict = "PASS" if not issues else "FAIL"
        trunk_verdict = "FAIL" if warnings else "PASS"

        summary = {
            "world_id": self._world_id,
            "world_name": self._world_name,
            "duration_s": self._duration,
            "verdict": verdict,
            "trunk_verdict": trunk_verdict,
            "issues": issues,
            "warnings": warnings,
            "tf": {
                "map_to_laser_ok_pct": round(tf_pct, 2),
                "probes": total_tf,
            },
            "topics": {
                "input_clouds": self._input.count,
                "input_pts": self._input.total_pts,
                "ground_msgs": self._ground.count,
                "ground_pts": self._ground.total_pts,
                "trunk_msgs": self._trunks.count,
                "trunk_pts": self._trunks.total_pts,
                "obstacle_msgs": self._obstacles.count,
                "obstacle_pts": self._obstacles.total_pts,
                "ground_pct": round(g_pct, 1),
                "trunk_pct": round(t_pct, 1),
                "obstacle_pct": round(o_pct, 1),
            },
            "debug_stats": {
                "frames": self._debug.frames,
                "ok_frames": self._debug.ok_frames,
                "tf_fail_frames": self._debug.tf_fail,
                "ground_method": ground_method,
                "debug_status_counts": dict(self._debug.status_counts),
                "ground_plane_rejected_frames": self._debug.ground_plane_rejected,
                "avg_clusters_per_ok_frame": round(avg_clusters, 2),
                "avg_trunk_clusters_per_ok_frame": round(avg_trunk, 2),
                "avg_grid_coverage_pct": round(avg_grid_cov, 1),
                "avg_grid_mean_abs_dz_ground_m": round(avg_grid_dz, 4),
                "avg_grid_unknown_pts": round(
                    self._debug.grid_unknown_sum / self._debug.ok_frames
                    if self._debug.ok_frames
                    else 0.0,
                    1,
                ),
                "debug_ground_class_pct": round(debug_ground_pct, 1),
                "debug_nonground_pts_total": self._debug.n_nonground_sum,
                "avg_gcr_pct": round(avg_gcr, 1),
                "avg_suspended_pct_of_raw": round(avg_susp, 1),
                "avg_mean_abs_dz_connected_m": round(avg_dz_conn, 4),
                "total_suspended_pts": self._debug.n_suspended_sum,
                "reject_height_total": self._debug.reject_height,
                "reject_radius_total": self._debug.reject_radius,
                "reject_verticality_total": self._debug.reject_verticality,
                "reject_small_total": self._debug.reject_small,
                "dominant_trunk_reject_reason": dom_reject,
                "trunk_method": self._debug.trunk_method or "cluster",
                "avg_columns_per_ok_frame": round(
                    self._debug.n_columns_sum / self._debug.ok_frames
                    if self._debug.ok_frames
                    else 0.0,
                    2,
                ),
                "avg_tracks_per_ok_frame": round(
                    self._debug.n_tracks_sum / self._debug.ok_frames
                    if self._debug.ok_frames
                    else 0.0,
                    2,
                ),
                "reject_column_sparse_total": self._debug.reject_column_sparse,
                "reject_cylinder_rmse_total": self._debug.reject_cylinder_rmse,
            },
            "tf_static_edges": [list(e) for e in sorted(self._static_edges)],
        }

        print(f"\n{'='*64}")
        print(f"  {self._world_id} — {self._world_name} ({self._duration:.0f}s)")
        print(f"{'='*64}")
        print(f"  VERDICT (ground/grid): {verdict}")
        print(f"  VERDICT (trunks): {trunk_verdict}")
        if issues:
            print(f"  Issues: {', '.join(issues)}")
        if warnings:
            print(f"  Warnings: {', '.join(warnings)}")
        print(f"  Ground method: {ground_method}")
        if self._debug.trunk_method:
            avg_cols = (
                self._debug.n_columns_sum / self._debug.ok_frames
                if self._debug.ok_frames
                else 0.0
            )
            avg_tracks = (
                self._debug.n_tracks_sum / self._debug.ok_frames
                if self._debug.ok_frames
                else 0.0
            )
            print(f"  Trunk method: {self._debug.trunk_method}")
            print(
                f"  Column debug: avg_columns={avg_cols:.1f} avg_tracks={avg_tracks:.1f} "
                f"rej_sparse={self._debug.reject_column_sparse} rej_cyl_rmse={self._debug.reject_cylinder_rmse}"
            )
        print(f"  TF map→laser: {tf_pct:.1f}% ({self._tf_ok}/{total_tf})")
        print(
            f"  Seg ratios: ground={g_pct:.1f}% trunk={t_pct:.1f}% obstacle={o_pct:.1f}%"
        )
        if using_grid:
            print(
                f"  Grid: coverage={avg_grid_cov:.1f}% mean|dz|_ground={avg_grid_dz:.4f} m"
            )
            print(
                f"  Connectivity: GCR={avg_gcr:.1f}% suspended={avg_susp:.1f}% "
                f"|dz|_conn={avg_dz_conn:.4f} m"
            )
        print(
            f"  Trunk debug: avg_clusters={avg_clusters:.1f} avg_trunk/frame={avg_trunk:.2f}"
        )
        print(
            f"  Rejects (total): radius={self._debug.reject_radius} "
            f"vert={self._debug.reject_verticality} height={self._debug.reject_height}"
        )
        print(f"  Dominant reject: {dom_reject}")
        print(f"{'='*64}\n")

        if self._out_dir:
            self._out_dir.mkdir(parents=True, exist_ok=True)
            path = self._out_dir / "world_summary.json"
            path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print(f"Wrote {path}")

        self._exit_code = 0 if verdict == "PASS" else 1


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--world-id", required=True)
    p.add_argument("--world-name", required=True)
    p.add_argument("--out-dir", type=str, default="")
    args = p.parse_args()
    out = Path(args.out_dir) if args.out_dir else None

    rclpy.init()
    node = WorldValidation(args.duration, args.world_id, args.world_name, out)
    node._exit_code = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node._finish()
    finally:
        code = getattr(node, "_exit_code", 1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(code)


if __name__ == "__main__":
    main()
