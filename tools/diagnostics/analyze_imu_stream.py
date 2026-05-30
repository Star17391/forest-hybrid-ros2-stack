#!/usr/bin/env python3
"""Análise quantitativa de /sensors/imu/data_raw (ou outro tópico Imu).

Uso (sim a correr, Play no Gazebo):
  python3 scripts/diagnostics/analyze_imu_stream.py --duration 5
  python3 scripts/diagnostics/analyze_imu_stream.py --topic /sensors/imu/data --duration 10

Saída: estatísticas, validação de quaternion, covariâncias, |g|, taxa.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


def sensor_qos() -> QoSProfile:
    """Match ros_gz_bridge SENSOR_DATA + forest_sensors_cpp (Best Effort)."""
    return qos_profile_sensor_data


def quat_norm(w, x, y, z):
    return math.sqrt(w * w + x * x + y * y + z * z)


def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuAnalyzer(Node):
    def __init__(self, topic: str, duration: float, qos: QoSProfile) -> None:
        super().__init__("imu_stream_analyzer")
        self._duration = duration
        self._t0 = time.monotonic()
        self._done = False
        self._count = 0
        self._bad_quat = 0
        self._zero_ang_cov = 0
        self._zero_ori_cov = 0
        self._nan_gyro = 0
        self._g_norms: list[float] = []
        self._wz: list[float] = []
        self._yaws: list[float] = []
        self._stamps: list[float] = []
        self._frame_id = ""

        self.create_subscription(Imu, topic, self._cb, qos)
        self.create_timer(duration, self._on_timeout)
        self.get_logger().info(
            f"Listening {topic} for {duration:.1f}s (QoS reliability="
            f"{'RELIABLE' if qos.reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'}) ..."
        )

    def _cb(self, msg: Imu) -> None:
        self._count += 1
        self._frame_id = msg.header.frame_id
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        n = quat_norm(w, x, y, z)
        ori_unknown = msg.orientation_covariance[0] < 0.0
        if not math.isfinite(n) or n < 1e-6 or abs(n - 1.0) > 0.05:
            self._bad_quat += 1
        if not ori_unknown and msg.orientation_covariance[0] <= 1e-12:
            self._zero_ori_cov += 1
        if msg.angular_velocity_covariance[0] <= 1e-12:
            self._zero_ang_cov += 1

        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        if not all(map(math.isfinite, (gx, gy, gz))):
            self._nan_gyro += 1

        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        if all(map(math.isfinite, (ax, ay, az))):
            self._g_norms.append(math.sqrt(ax * ax + ay * ay + az * az))

        self._wz.append(gz)
        if math.isfinite(n) and n > 1e-6:
            self._yaws.append(yaw_from_quat(x, y, z, w))

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._stamps.append(t)

    def _on_timeout(self) -> None:
        if self._done:
            return
        self._done = True
        self._report()
        rclpy.shutdown()

    def _report(self) -> None:
        if self._count == 0:
            print(
                "\n=== IMU stream analysis ===\n"
                "Samples: 0 — nenhuma mensagem recebida.\n"
                "  • Gazebo em Play?\n"
                "  • Sim a correr (diag_imu ou sim_mvp)?\n"
                "  • source install/setup.bash\n"
                "  • QoS: use --reliable só para tópicos Reliable (não /sensors/imu/data_raw)."
            )
            return
        hz = 0.0
        if len(self._stamps) >= 2:
            dt = self._stamps[-1] - self._stamps[0]
            if dt > 0:
                hz = (len(self._stamps) - 1) / dt

        def stats(vals: list[float]):
            if not vals:
                return "n/a"
            vals = sorted(vals)
            return (
                f"min={vals[0]:.4f} max={vals[-1]:.4f} "
                f"mean={sum(vals)/len(vals):.4f} p95={vals[int(0.95*(len(vals)-1))]:.4f}"
            )

        print("\n=== IMU stream analysis ===")
        print(f"Samples: {self._count}  rate≈{hz:.1f} Hz  frame_id={self._frame_id!r}")
        print(f"Invalid quaternion: {self._bad_quat} ({100*self._bad_quat/max(1,self._count):.1f}%)")
        print(f"Zero orientation covariance: {self._zero_ori_cov}")
        print(f"Zero angular_velocity covariance: {self._zero_ang_cov}")
        print(f"NaN gyro samples: {self._nan_gyro}")
        print(f"|g| m/s²: {stats(self._g_norms)}  (expect ~9.8 at rest)")
        print(f"gyro.z rad/s: {stats(self._wz)}")
        if self._yaws:
            dyaw = max(self._yaws) - min(self._yaws)
            print(f"yaw span rad: {dyaw:.4f} ({math.degrees(dyaw):.2f}°)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/sensors/imu/data_raw")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--reliable",
        action="store_true",
        help="Subscrever com QoS Reliable (ex. tópicos internos); default = sensor Best Effort",
    )
    parser.add_argument(
        "--no-use-sim-time",
        action="store_true",
        help="Relógio de parede (default: use_sim_time=true com /clock do Gazebo)",
    )
    args = parser.parse_args()

    qos = (
        QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        if args.reliable
        else sensor_qos()
    )

    rclpy.init()
    node = ImuAnalyzer(args.topic, args.duration, qos)
    if not args.no_use_sim_time:
        from rclpy.parameter import Parameter

        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node._done:
            node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
