#!/usr/bin/env python3
"""Análise de latência e taxa dos sensores vs EKF (Fase 0).

Mede intervalos entre mensagens e atraso stamp→recepção.

Uso:
  forest diag ekf-latency --duration 30
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

_DIAG_DIR = Path(__file__).resolve().parent
if str(_DIAG_DIR) not in sys.path:
    sys.path.insert(0, str(_DIAG_DIR))

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from ros_time_util import header_stamp_sec, latency_ms


@dataclass
class StreamStats:
    topic: str
    count: int
    duration_s: float
    rate_hz: float
    dt_mean_ms: float
    dt_p95_ms: float
    dt_max_ms: float
    latency_mean_ms: float
    latency_p95_ms: float
    latency_max_ms: float


def _interval_stats(
    stamps: list[float], latencies_ms: list[float]
) -> StreamStats | None:
    if len(stamps) < 2:
        return None
    dts = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
    lats = list(latencies_ms)
    dts.sort()
    lats.sort()
    duration = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / duration if duration > 1e-6 else 0.0

    def p95(vals: list[float]) -> float:
        return vals[int(0.95 * (len(vals) - 1))] if vals else float("nan")

    return StreamStats(
        topic="",
        count=len(stamps),
        duration_s=duration,
        rate_hz=rate,
        dt_mean_ms=sum(dts) / len(dts),
        dt_p95_ms=p95(dts),
        dt_max_ms=dts[-1],
        latency_mean_ms=sum(lats) / len(lats),
        latency_p95_ms=p95(lats),
        latency_max_ms=lats[-1],
    )


class LatencyAnalyzer(Node):
    def __init__(
        self, duration: float, output_dir: Path, use_sim_time: bool = True
    ) -> None:
        super().__init__(
            "ekf_latency_analyzer",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time),
            ],
        )
        self._duration = duration
        self._output_dir = output_dir
        self._t0 = time.monotonic()
        self._imu_stamps: list[float] = []
        self._imu_lat_ms: list[float] = []
        self._wheel_stamps: list[float] = []
        self._wheel_lat_ms: list[float] = []
        self._ekf_stamps: list[float] = []
        self._ekf_lat_ms: list[float] = []
        self._use_sim = False

        qos = qos_profile_sensor_data
        self.create_subscription(Imu, "/sensors/imu/data", self._on_imu, qos)
        self.create_subscription(Odometry, "/sensors/wheel_odometry", self._on_wheel, 10)
        self.create_subscription(Odometry, "/state/odometry", self._on_ekf, 10)

    def _maybe_done(self) -> None:
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            raise SystemExit(0)

    def _on_imu(self, msg: Imu) -> None:
        self._imu_stamps.append(header_stamp_sec(msg))
        self._imu_lat_ms.append(latency_ms(msg, self))
        self._maybe_done()

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_stamps.append(header_stamp_sec(msg))
        self._wheel_lat_ms.append(latency_ms(msg, self))
        self._maybe_done()

    def _on_ekf(self, msg: Odometry) -> None:
        self._ekf_stamps.append(header_stamp_sec(msg))
        self._ekf_lat_ms.append(latency_ms(msg, self))
        self._maybe_done()

    def _report(self) -> None:
        rows: list[StreamStats] = []
        sim_param = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self._use_sim = sim_param

        for topic, stamps, lats in (
            ("/sensors/imu/data", self._imu_stamps, self._imu_lat_ms),
            ("/sensors/wheel_odometry", self._wheel_stamps, self._wheel_lat_ms),
            ("/state/odometry", self._ekf_stamps, self._ekf_lat_ms),
        ):
            st = _interval_stats(stamps, lats)
            if st is None:
                print(f"AVISO: poucas amostras em {topic}", file=sys.stderr)
                continue
            st.topic = topic
            rows.append(st)

        self._output_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "duration_requested_s": self._duration,
            "clock_looks_like_sim_time": self._use_sim,
            "note": "latency usa relógio ROS (alinhado com use_sim_time)",
            "streams": [asdict(r) for r in rows],
        }
        out_json = self._output_dir / "latency.json"
        out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

        print(f"\n=== ekf_latency_analyzer ({self._duration:.0f}s) ===")
        print(f"Output: {self._output_dir.resolve()}")
        for st in rows:
            print(f"\n{st.topic}")
            print(f"  count={st.count}  rate≈{st.rate_hz:.1f} Hz  span={st.duration_s:.1f}s")
            print(f"  Δt: mean={st.dt_mean_ms:.1f} ms  p95={st.dt_p95_ms:.1f} ms  max={st.dt_max_ms:.1f} ms")
            lat_ok = st.latency_mean_ms < 5000.0
            if lat_ok:
                print(
                    f"  latency ROS clock: mean={st.latency_mean_ms:.1f} ms  "
                    f"p95={st.latency_p95_ms:.1f} ms  max={st.latency_max_ms:.1f} ms"
                )
            else:
                print(
                    "  latency: N/A (relógio desalinhado — usa --use-sim-time; Δt acima é fiável)"
                )
        if self._use_sim:
            print("\n(use_sim_time=true no nó de diagnóstico)")

        self._try_plot(rows)

    def _try_plot(self, rows: list[StreamStats]) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("Plot: omitido (python3-matplotlib)")
            return
        if not rows:
            return
        fig, ax = plt.subplots(figsize=(8, 4))
        labels = [r.topic.split("/")[-1] or r.topic for r in rows]
        p95 = [r.dt_p95_ms for r in rows]
        ax.bar(labels, p95, color=["#2ecc71", "#3498db", "#e74c3c"][: len(rows)])
        ax.set_ylabel("Δt p95 [ms]")
        ax.set_title("Intervalo entre mensagens (p95)")
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(self._output_dir / "latency_dt_p95.png", dpi=120)
        plt.close(fig)
        print(f"Plot: {self._output_dir / 'latency_dt_p95.png'}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--output-dir", type=Path, default=None)
    p.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = p.parse_args()

    if args.output_dir is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output_dir = Path("reports/phase0") / f"{stamp}_latency"

    rclpy.init()
    node = LatencyAnalyzer(args.duration, args.output_dir, args.use_sim_time)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if (args.output_dir / "latency.json").is_file() else 1


if __name__ == "__main__":
    sys.exit(main())
