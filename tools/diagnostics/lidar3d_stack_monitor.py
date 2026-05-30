#!/usr/bin/env python3
"""Monitor integrado do stack LiDAR 3D: TF, hz, latência, handoff bootstrap/EKF.

Uso (sim a correr, Gazebo PLAY, use_sim_time):
  source install/setup.bash
  forest diag lidar3d-stack --duration 45

Saída: relatório no terminal + opcional JSON em reports/phase3d/
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
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
    last_stamp_ns: int = 0
    gaps_over_500ms: int = 0
    _prev_stamp_ns: int = field(default=-1, repr=False)

    def observe(self, stamp_ns: int) -> None:
        if stamp_ns > 0 and self._prev_stamp_ns > 0:
            dt_ms = (stamp_ns - self._prev_stamp_ns) / 1e6
            if dt_ms > 500.0:
                self.gaps_over_500ms += 1
        self._prev_stamp_ns = stamp_ns
        self.last_stamp_ns = stamp_ns
        self.count += 1


class Lidar3dStackMonitor(Node):
    def __init__(self, duration: float, out_dir: Path | None) -> None:
        from rclpy.parameter import Parameter

        super().__init__(
            "lidar3d_stack_monitor",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._out_dir = out_dir

        self._static_edges: set[tuple[str, str]] = set()
        self._odom_base_ekf = 0
        self._odom_base_boot = 0
        self._tf_chain_ok = 0
        self._tf_chain_fail = 0

        self._topics: dict[str, TopicStats] = defaultdict(TopicStats)

        self.create_subscription(TFMessage, "/tf_static", self._on_static, TF_STATIC_QOS)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.create_subscription(PointCloud2, "/sensors/lidar/points", self._on_pts, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, "/perception/lidar/points_labeled", self._on_labeled, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/sensors/wheel_odometry", self._on_wheel, 10)
        self.create_subscription(Odometry, "/state/odometry", self._on_state_odom, 10)

        self._timer = self.create_timer(0.5, self._probe_tf)
        self._done_code: int | None = None

    def _on_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._static_edges.add((t.header.frame_id, t.child_frame_id))

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            p, c = t.header.frame_id, t.child_frame_id
            if p == "odom" and c == "marble_hd2/base_link":
                # Heuristic: bootstrap often publishes before EKF stabilizes
                if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
                    self._odom_base_boot += 1
                else:
                    self._odom_base_ekf += 1

    def _observe_topic(self, key: str, stamp_ns: int) -> None:
        self._topics[key].observe(stamp_ns)

    def _on_pts(self, msg: PointCloud2) -> None:
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._observe_topic("/sensors/lidar/points", ns)

    def _on_labeled(self, msg: PointCloud2) -> None:
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._observe_topic("/perception/lidar/points_labeled", ns)

    def _on_wheel(self, msg: Odometry) -> None:
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._observe_topic("/sensors/wheel_odometry", ns)

    def _on_state_odom(self, msg: Odometry) -> None:
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._observe_topic("/state/odometry", ns)

    def _probe_tf(self) -> None:
        from tf2_ros import Buffer, TransformListener

        if not hasattr(self, "_buf"):
            self._buf = Buffer()
            self._listener = TransformListener(self._buf, self)

        try:
            if self._buf.can_transform(
                "map", "laser", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            ):
                self._tf_chain_ok += 1
            else:
                self._tf_chain_fail += 1
        except Exception:
            self._tf_chain_fail += 1

        if time.monotonic() - self._t0 >= self._duration:
            self._done_code = self._report()
            self.destroy_timer(self._timer)
            rclpy.shutdown()

    def _hz(self, key: str) -> float | None:
        st = self._topics[key]
        if st.count < 2 or st.last_stamp_ns <= 0:
            return None
        return st.count / self._duration

    def _report(self) -> int:
        print(f"\n=== lidar3d_stack_monitor ({self._duration:.0f}s) ===\n")

        print("TF static edges:")
        for e in sorted(self._static_edges):
            print(f"  {e[0]} -> {e[1]}")

        need_static = {("map", "odom"), ("marble_hd2/base_link", "laser")}
        missing = need_static - self._static_edges
        if missing:
            print(f"  MISSING static: {missing}")

        total_tf = self._tf_chain_ok + self._tf_chain_fail
        pct = 100.0 * self._tf_chain_ok / total_tf if total_tf else 0.0
        print(f"\nTF chain map->laser (0.5s polling): {self._tf_chain_ok}/{total_tf} OK ({pct:.1f}%)")

        print("\nTopic rates (approx):")
        for key in (
            "/sensors/lidar/points",
            "/perception/lidar/points_labeled",
            "/sensors/wheel_odometry",
            "/state/odometry",
        ):
            hz = self._hz(key)
            gaps = self._topics[key].gaps_over_500ms
            print(f"  {key}: hz≈{hz if hz else '—'}  gaps>500ms={gaps}")

        print(f"\nodom->base_link TF messages (heuristic): ekf_like={self._odom_base_ekf} boot_like={self._odom_base_boot}")

        verdict = []
        if ("marble_hd2/base_link", "laser") not in self._static_edges:
            verdict.append("FAIL: base_link->laser static missing")
        if pct < 95.0 and total_tf > 10:
            verdict.append(f"FAIL: map->laser TF success {pct:.1f}% < 95%")
        hz_pts = self._hz("/sensors/lidar/points")
        if hz_pts is None or hz_pts < 2.0:
            verdict.append("FAIL: /sensors/lidar/points hz < 2")
        if not verdict:
            print("\nVERDICT: PASS (pipeline estável em steady-state)")
            code = 0
        else:
            for v in verdict:
                print(f"\n{v}", file=sys.stderr)
            code = 1

        if self._out_dir:
            self._out_dir.mkdir(parents=True, exist_ok=True)
            out = {
                "duration_s": self._duration,
                "static_edges": [list(e) for e in sorted(self._static_edges)],
                "tf_chain_ok_pct": pct,
                "topics": {k: {"count": self._topics[k].count, "hz": self._hz(k)} for k in self._topics},
                "verdict": verdict,
            }
            path = self._out_dir / f"stack_monitor_{int(time.time())}.json"
            path.write_text(json.dumps(out, indent=2), encoding="utf-8")
            print(f"\nJSON: {path}")

        return code


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=45.0)
    p.add_argument("--out-dir", type=str, default="")
    args = p.parse_args()
    out = Path(args.out_dir) if args.out_dir else None

    rclpy.init()
    node = Lidar3dStackMonitor(args.duration, out)
    code = 0
    try:
        while rclpy.ok() and node._done_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = node._done_code if node._done_code is not None else node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
