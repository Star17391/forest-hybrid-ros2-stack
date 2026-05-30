#!/usr/bin/env python3
"""YDLidar X4 — fdpo-ros-stack protocol (sdpo_driver_laser_2d / YDLIDARX4.cpp)."""

from __future__ import annotations

import math
import threading
import time
from enum import Enum, auto

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class _State(Enum):
    PH1 = auto()
    PH2 = auto()
    CT = auto()
    LSN = auto()
    FSA1 = auto()
    FSA2 = auto()
    LSA1 = auto()
    LSA2 = auto()
    CS1 = auto()
    CS2 = auto()
    SI1 = auto()
    SI2 = auto()
    IDLE = auto()


def _norm_ang_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FdpoYdlidarX4Node(Node):
    def __init__(self) -> None:
        super().__init__("fdpo_ydlidar_x4")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 128000)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("dist_min", 0.12)
        self.declare_parameter("dist_max", 10.0)
        self.declare_parameter("start_settle_ms", 300)
        self.declare_parameter("after_stop_ms", 100)
        self.declare_parameter("start_attempts", 5)
        self.declare_parameter("between_start_ms", 250)

        self._port = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baudrate").value)
        self._frame_id = self.get_parameter("frame_id").value
        scan_topic = self.get_parameter("scan_topic").value
        self._dist_min = float(self.get_parameter("dist_min").value)
        self._dist_max = float(self.get_parameter("dist_max").value)

        self._state = _State.IDLE
        self._sample_count = 0
        self._pkg_zero = False
        self._pkg_num_samples = 0
        self._raw_start_ang = 0
        self._raw_end_ang = 0
        self._raw_dist: list[int] = []
        self._dist_buf: list[float] = []
        self._ang_buf: list[float] = []
        self._start_ang = 0.0
        self._end_ang = 0.0
        self._scan_count = 0
        self._running = True

        self._pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        self._fdpo_start_scan()
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.get_logger().info(
            f"fdpo YDLidar X4 {self._port}@{self._baud} -> {scan_topic}"
        )

    def _fdpo_start_scan(self) -> None:
        settle = int(self.get_parameter("start_settle_ms").value)
        after_stop = int(self.get_parameter("after_stop_ms").value)
        attempts = int(self.get_parameter("start_attempts").value)
        between = int(self.get_parameter("between_start_ms").value)
        self._ser.reset_input_buffer()
        time.sleep(max(0, settle) / 1000.0)
        self._ser.write(bytes([0xA5, 0x65]))
        time.sleep(max(0, after_stop) / 1000.0)
        for i in range(max(1, attempts)):
            self._ser.write(bytes([0xA5, 0x60]))
            if i + 1 < attempts and between > 0:
                time.sleep(between / 1000.0)

    def _read_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                chunk = self._ser.read(512)
            except serial.SerialException as exc:
                self.get_logger().error(str(exc))
                time.sleep(0.5)
                continue
            for b in chunk:
                self._on_byte(b)

    def _on_byte(self, ch: int) -> None:
        st = self._state

        if st == _State.PH1:
            self._state = _State.PH2 if ch == 0x55 else _State.IDLE
        elif st == _State.PH2:
            self._state = _State.CT
        elif st == _State.CT:
            self._state = _State.LSN
        elif st == _State.LSN:
            self._state = _State.FSA1
        elif st == _State.FSA1:
            self._state = _State.FSA2
        elif st == _State.FSA2:
            self._state = _State.LSA1
        elif st == _State.LSA1:
            self._state = _State.LSA2
        elif st == _State.LSA2:
            self._state = _State.CS1
        elif st == _State.CS1:
            self._state = _State.CS2
        elif st == _State.CS2:
            self._state = _State.SI1
            self._sample_count = 0
            self._raw_dist = []
        elif st == _State.SI1:
            self._state = _State.SI2
        elif st == _State.SI2:
            if self._sample_count >= self._pkg_num_samples:
                if ch == 0xAA:
                    self._state = _State.PH1
                    self._process_packet()
                else:
                    self._state = _State.IDLE
            else:
                self._state = _State.SI1
        elif st == _State.IDLE and ch == 0xAA:
            self._state = _State.PH1

        st = self._state
        if st == _State.CT:
            if ch & 0x01:
                self._pkg_zero = True
                self._publish_scan()
                self._dist_buf.clear()
                self._ang_buf.clear()
            else:
                self._pkg_zero = False
        elif st == _State.LSN:
            self._pkg_num_samples = ch & 0xFF
        elif st == _State.FSA1:
            self._raw_start_ang = ch & 0xFF
        elif st == _State.FSA2:
            self._start_ang = ((self._raw_start_ang | (ch << 8)) >> 1) / 64.0
        elif st == _State.LSA1:
            self._raw_end_ang = ch & 0xFF
        elif st == _State.LSA2:
            self._end_ang = ((self._raw_end_ang | (ch << 8)) >> 1) / 64.0
        elif st == _State.SI1:
            if len(self._raw_dist) <= self._sample_count:
                self._raw_dist.append(0)
            self._raw_dist[self._sample_count] = ch & 0xFF
        elif st == _State.SI2:
            self._raw_dist[self._sample_count] |= ch << 8
            self._sample_count += 1

    def _process_packet(self) -> None:
        if self._pkg_zero or self._sample_count == 0:
            return
        n = self._sample_count
        delta = self._end_ang - self._start_ang
        if self._start_ang > self._end_ang:
            delta += 360.0
        for i in range(n):
            dist_mm = self._raw_dist[i] / 4.0
            if self._raw_dist[i] == 0:
                corr = 0.0
            else:
                corr = math.degrees(
                    math.atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm))
                )
            ang = -(
                self._start_ang + delta * i / max(1, n - 1) + corr
            )
            dist_m = dist_mm / 1000.0
            if self._dist_min <= dist_m <= self._dist_max:
                self._dist_buf.append(dist_m)
                self._ang_buf.append(_norm_ang_rad(math.radians(ang)))

    def _publish_scan(self) -> None:
        if len(self._dist_buf) < 10:
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = min(self._ang_buf)
        msg.angle_max = max(self._ang_buf)
        n = len(self._ang_buf)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / max(1, n - 1)
        msg.range_min = self._dist_min
        msg.range_max = self._dist_max
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / max(1, n)
        msg.ranges = [float("inf")] * n
        for a, d in zip(self._ang_buf, self._dist_buf):
            idx = int(round((a - msg.angle_min) / msg.angle_increment))
            idx = max(0, min(n - 1, idx))
            if math.isfinite(msg.ranges[idx]):
                msg.ranges[idx] = min(msg.ranges[idx], d) if msg.ranges[idx] != float("inf") else d
            else:
                msg.ranges[idx] = d
        self._pub.publish(msg)
        self._scan_count += 1
        if self._scan_count <= 5 or self._scan_count % 20 == 0:
            self.get_logger().info(f"scan {self._scan_count}: {n} pts")

    def destroy_node(self) -> None:
        self._running = False
        try:
            self._ser.write(bytes([0xA5, 0x65]))
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FdpoYdlidarX4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
