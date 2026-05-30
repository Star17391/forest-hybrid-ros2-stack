#!/usr/bin/env python3
"""Benchmark Fase 0: /state/pose_fused vs ground truth Gazebo (world_tf_full).

Requer sim a correr (forest up sim-mvp-nav -d) com Gazebo em PLAY e movimento
(teleop, patrol, ou cmd_vel). Grava CSV, JSON e opcionalmente PNG.

Uso:
  forest diag pose-benchmark --label ekf_wheel_only --duration 90
  forest diag pose-benchmark --output-dir reports/run1 --duration 120
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_DIAG_DIR = Path(__file__).resolve().parent
if str(_DIAG_DIR) not in sys.path:
    sys.path.insert(0, str(_DIAG_DIR))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

from gz_world_tf_pick import GzMarblePosePicker
from pose_metrics_lib import (
    compute_pose_errors,
    try_plot_trajectories,
    write_csv,
    write_metrics_json,
    yaw_from_quat,
)


class PoseBenchmark(Node):
    def __init__(
        self,
        model: str,
        duration: float,
        label: str,
        output_dir: Path,
        use_sim_time: bool = True,
    ) -> None:
        super().__init__(
            "pose_benchmark",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time),
            ],
        )
        self._model = model
        self._duration = duration
        self._label = label
        self._output_dir = output_dir
        self._t0 = time.monotonic()
        self._gz_picker = GzMarblePosePicker(model, f"{model}/base_link")
        self._gz_pose: tuple[float, float, float] | None = None
        self._fused_series: list[tuple[float, float, float, float]] = []
        self._gz_series: list[tuple[float, float, float, float]] = []
        self._fused_msgs = 0
        self._gz_msgs = 0
        self._fused_before_gz = 0
        self._gt_origin_skips = 0

        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_fused, 10)
        for topic in ("/forest_gen/gz/world_tf_full", "/forest_gen/gz/world_tf"):
            self.create_subscription(TFMessage, topic, self._on_gz, 10)

    def _on_fused(self, msg: PoseStamped) -> None:
        self._fused_msgs += 1
        if self._gz_pose is None:
            self._fused_before_gz += 1
            return
        now = time.monotonic()
        if now - self._t0 >= self._duration:
            self._finish()
            raise SystemExit(0)

        t = now - self._t0
        p = msg.pose.position
        q = msg.pose.orientation
        fused = (t, p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        gx, gy, gyaw = self._gz_pose

        self._fused_series.append(fused)
        self._gz_series.append((t, gx, gy, gyaw))

    def _on_gz(self, msg: TFMessage) -> None:
        self._gz_msgs += 1
        picked = self._gz_picker.pick_xy_yaw(msg)
        if picked is None:
            self._gt_origin_skips += 1
            return
        label = self._gz_picker.last_label
        if label and not self._gz_pose:
            self.get_logger().info(f"GT pose latched: {label}")
        self._gz_pose = picked

    def _finish(self) -> None:
        stats = compute_pose_errors(self._fused_series, self._gz_series)
        out = self._output_dir
        out.mkdir(parents=True, exist_ok=True)

        if stats is None:
            print(
                "ERRO: amostras insuficientes (<2 pares alinhados).",
                file=sys.stderr,
            )
            print(
                f"  pose_fused msgs: {self._fused_msgs}  "
                f"(antes de GT: {self._fused_before_gz})",
                file=sys.stderr,
            )
            print(
                f"  world_tf msgs: {self._gz_msgs}  "
                f"GT: {self._gz_picker.last_label or '(nenhum)'}",
                file=sys.stderr,
            )
            if self._gz_msgs == 0:
                print(
                    "  Sem /forest_gen/gz/world_tf(_full) — Gazebo em PLAY? "
                    "Relança sim ou espera ~10s após PLAY.",
                    file=sys.stderr,
                )
            elif self._fused_msgs == 0:
                print(
                    "  Sem /state/pose_fused — confirma forest up sim-mvp-nav e EKF.",
                    file=sys.stderr,
                )
            else:
                print(
                    "  GT ou fused chegaram tarde — repete com robô já em movimento.",
                    file=sys.stderr,
                )
            return

        write_csv(out / "trajectories.csv", self._fused_series, self._gz_series)
        write_metrics_json(
            out / "metrics.json",
            self._label,
            stats,
            extra={
                "model": self._model,
                "gt_frame": self._gz_picker.last_label,
                "pose_fused_msgs": self._fused_msgs,
                "world_tf_msgs": self._gz_msgs,
                "gt_origin_skips": self._gt_origin_skips,
                "gt_avg_speed_mps": (
                    stats.gt_path_m / stats.duration_s if stats.duration_s > 1e-3 else 0.0
                ),
            },
        )
        plotted = try_plot_trajectories(out / "trajectories.png", self._fused_series, self._gz_series, self._label)

        print(f"\n=== pose_benchmark [{self._label}] ===")
        print(f"Output: {out.resolve()}")
        gt_speed = stats.gt_path_m / max(stats.duration_s, 1e-3)
        print(
            f"Samples: {stats.samples}  duration: {stats.duration_s:.1f}s  "
            f"GT: {self._gz_picker.last_label}"
        )
        print(f"GT path: {stats.gt_path_m:.2f} m  fused path: {stats.fused_path_m:.2f} m  "
              f"(GT speed ~{gt_speed:.2f} m/s)")
        if gt_speed > 8.0:
            print(
                "AVISO: GT path implausível (>8 m/s) — repetir benchmark após actualizar gz_world_tf_pick"
            )
        if self._gt_origin_skips:
            print(f"GT origin skips: {self._gt_origin_skips}")
        print(
            f"|Δpos| RMSE={stats.pos_rmse_m:.3f} m  mean={stats.pos_mean_m:.3f} m  "
            f"max={stats.pos_max_m:.3f} m  p95={stats.pos_p95_m:.3f} m  end={stats.end_pos_error_m:.3f} m"
        )
        print(
            f"|Δyaw| RMSE={stats.yaw_rmse_deg:.1f}°  mean={stats.yaw_mean_deg:.1f}°  "
            f"max={stats.yaw_max_deg:.1f}°  p95={stats.yaw_p95_deg:.1f}°"
        )
        print(f"Drift (end_err / GT path): {stats.drift_pct:.2f}%")
        if not plotted:
            print("Plot: omitido (instala python3-matplotlib para trajectories.png)")

    def spin_with_timer(self) -> None:
        while rclpy.ok() and time.monotonic() - self._t0 < self._duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._finish()


def main() -> int:
    p = argparse.ArgumentParser(description="Fase 0 pose benchmark vs Gazebo GT")
    p.add_argument("--duration", type=float, default=90.0, help="Segundos de recolha")
    p.add_argument("--model", default="marble_hd2")
    p.add_argument("--label", default="ekf_unknown", help="Tag no metrics.json (ex. ekf_wheel_only)")
    p.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Relógio ROS = sim (default True com forest up)",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Pasta de saída (default: reports/phase0/TIMESTAMP_LABEL)",
    )
    args = p.parse_args()

    if args.output_dir is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        safe = args.label.replace("/", "_").replace(" ", "_")
        args.output_dir = Path("reports/phase0") / f"{stamp}_{safe}"

    rclpy.init()
    node = PoseBenchmark(
        args.model, args.duration, args.label, args.output_dir, args.use_sim_time
    )
    try:
        node.spin_with_timer()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if (args.output_dir / "metrics.json").is_file() else 1


if __name__ == "__main__":
    sys.exit(main())
