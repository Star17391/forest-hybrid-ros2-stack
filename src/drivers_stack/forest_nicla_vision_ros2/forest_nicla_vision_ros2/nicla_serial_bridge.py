#!/usr/bin/env python3
"""ROS 2 bridge: Nicla Vision (USB serial or Wi-Fi) -> camera + IMU topics."""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool, Header

from forest_nicla_vision_ros2.camera_info_util import camera_info_from_intrinsics
from forest_nicla_vision_ros2.protocol import FrameType, find_nicla_serial_port, rgb565_to_rgb8
from forest_nicla_vision_ros2.nicla_transport import (
    NiclaDeviceClient,
    create_nicla_client,
    jpeg_to_rgb8,
)


class NiclaSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("nicla_serial_bridge")

        self.declare_parameter("transport", "serial")
        self.declare_parameter("image_encoding", "jpeg")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baud_rate", 921600)
        self.declare_parameter("wifi_host", "")
        self.declare_parameter("wifi_port", 9876)
        self.declare_parameter("frame_id", "nicla_camera_optical_frame")
        self.declare_parameter("imu_frame_id", "nicla_imu_link")
        self.declare_parameter("image_rate_hz", 3.0)
        self.declare_parameter("imu_rate_hz", 25.0)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("imu_topic", "/sensors/imu/data")
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("serial_timeout_s", 5.0)
        self.declare_parameter("reconnect_backoff_max_s", 10.0)
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("health_topic", "/sensors/nicla_serial/connected")
        self.declare_parameter("camera_fx", 280.0)
        self.declare_parameter("camera_fy", 280.0)
        self.declare_parameter("camera_cx", 160.0)
        self.declare_parameter("camera_cy", 120.0)
        self.declare_parameter("camera_distortion_model", "plumb_bob")
        self.declare_parameter("camera_distortion", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("imu_accel_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("imu_gyro_signs", [1.0, 1.0, 1.0])

        self._transport = (
            self.get_parameter("transport").get_parameter_value().string_value.strip().lower()
        )
        self._image_encoding = (
            self.get_parameter("image_encoding").get_parameter_value().string_value.strip().lower()
        )
        port = self.get_parameter("serial_port").get_parameter_value().string_value
        if not port and self._transport == "serial":
            port = find_nicla_serial_port() or ""
        self._serial_port = port
        self._baud = int(self.get_parameter("baud_rate").value)
        self._wifi_host = self.get_parameter("wifi_host").get_parameter_value().string_value
        self._wifi_port = int(self.get_parameter("wifi_port").value)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._imu_frame_id = self.get_parameter("imu_frame_id").get_parameter_value().string_value
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._serial_timeout_s = float(self.get_parameter("serial_timeout_s").value)
        self._reconnect_backoff_max_s = float(self.get_parameter("reconnect_backoff_max_s").value)
        self._publish_camera_info = bool(self.get_parameter("publish_camera_info").value)
        self._accel_signs = list(self.get_parameter("imu_accel_signs").value)
        self._gyro_signs = list(self.get_parameter("imu_gyro_signs").value)
        self._camera_fx = float(self.get_parameter("camera_fx").value)
        self._camera_fy = float(self.get_parameter("camera_fy").value)
        self._camera_cx = float(self.get_parameter("camera_cx").value)
        self._camera_cy = float(self.get_parameter("camera_cy").value)
        self._camera_distortion_model = (
            self.get_parameter("camera_distortion_model").get_parameter_value().string_value
        )
        self._camera_distortion = list(self.get_parameter("camera_distortion").value)
        self._use_jpeg = self._image_encoding in ("jpeg", "jpg")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._image_pub = self.create_publisher(Image, image_topic, qos)

        self._camera_info_pub: Optional[rclpy.publisher.Publisher] = None
        if self._publish_camera_info:
            info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            self._camera_info_pub = self.create_publisher(CameraInfo, info_topic, qos)

        self._imu_pub: Optional[rclpy.publisher.Publisher] = None
        if self._publish_imu:
            imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
            self._imu_pub = self.create_publisher(Imu, imu_topic, qos)

        health_topic = self.get_parameter("health_topic").get_parameter_value().string_value
        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._health_pub = self.create_publisher(Bool, health_topic, health_qos)
        self._health_timer = self.create_timer(2.0, self._publish_health_periodic)

        image_rate = float(self.get_parameter("image_rate_hz").value)
        if image_rate > 0.0:
            self._image_timer = self.create_timer(1.0 / image_rate, self._on_image_timer)
        else:
            self._image_timer = None

        imu_rate = float(self.get_parameter("imu_rate_hz").value)
        if self._publish_imu and imu_rate > 0.0:
            self._imu_timer = self.create_timer(1.0 / imu_rate, self._on_imu_timer)
        else:
            self._imu_timer = None

        self._client: Optional[NiclaDeviceClient] = None
        self._device_jpeg_ok = False
        self._io_lock = threading.Lock()
        self._reconnect_backoff_s = 0.5
        self._next_connect_mono = 0.0
        self._connected = False
        self._publish_health_periodic()

        enc = "jpeg" if self._use_jpeg else "rgb565"
        self.get_logger().info(
            f"Nicla Phase 4: transport={self._transport} encoding={enc} "
            f"image {image_rate:.2f} Hz imu {imu_rate:.1f} Hz"
        )

    def _publish_health_periodic(self) -> None:
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = self._connected
        try:
            self._health_pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass

    def _publish_health(self, connected: bool) -> None:
        if self._connected != connected:
            self._connected = connected
            self._publish_health_periodic()

    def _ensure_client(self) -> Optional[NiclaDeviceClient]:
        if self._client:
            return self._client

        now = time.monotonic()
        if now < self._next_connect_mono:
            return None

        try:
            client = create_nicla_client(
                self._transport,
                serial_port=self._serial_port,
                baud_rate=self._baud,
                wifi_host=self._wifi_host,
                wifi_port=self._wifi_port,
                timeout_s=self._serial_timeout_s,
            )
            client.open()
            if not client.ping():
                client.close()
                raise RuntimeError("PING did not return PONG")
            status = client.status()
            self._device_jpeg_ok = "enc=rgb888" in status
            self._client = client
            self._reconnect_backoff_s = 0.5
            self._publish_health(True)
            self.get_logger().info(f"connected ({self._transport}): {status}")
            if self._use_jpeg and not self._device_jpeg_ok:
                self.get_logger().warning(
                    "Nicla firmware lacks enc=rgb888 (old JPEG encoder). "
                    "Using RGB565 SNAP for correct colors — reflash sensor firmware. "
                    "On Wi-Fi, lower image_rate_hz (~0.5–1.0)."
                )
            return self._client
        except Exception as exc:  # noqa: BLE001
            self._publish_health(False)
            self._next_connect_mono = now + self._reconnect_backoff_s
            self._reconnect_backoff_s = min(
                self._reconnect_backoff_s * 2.0, self._reconnect_backoff_max_s
            )
            self.get_logger().warning(
                f"connect failed ({self._transport}): {exc} "
                f"(retry in {self._reconnect_backoff_s:.1f}s)",
                throttle_duration_sec=5.0,
            )
            return None

    def _reset_client(self) -> None:
        if self._client:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
        self._client = None
        self._device_jpeg_ok = False
        self._publish_health(False)
        self._next_connect_mono = time.monotonic() + self._reconnect_backoff_s

    def _apply_imu_signs(
        self, accel: tuple[float, float, float], gyro: tuple[float, float, float]
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        ax, ay, az = accel
        gx, gy, gz = gyro
        sx, sy, sz = (self._accel_signs + [1.0, 1.0, 1.0])[:3]
        gx_s, gy_s, gz_s = (self._gyro_signs + [1.0, 1.0, 1.0])[:3]
        return (ax * sx, ay * sy, az * sz), (gx * gx_s, gy * gy_s, gz * gz_s)

    def _on_image_timer(self) -> None:
        if not self._io_lock.acquire(blocking=False):
            return
        try:
            client = self._ensure_client()
            if not client:
                return
            try:
                request_device_jpeg = self._use_jpeg and self._device_jpeg_ok
                frame = client.snap(jpeg=request_device_jpeg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"SNAP failed: {exc}")
                self._reset_client()
                return
        finally:
            self._io_lock.release()

        self.get_logger().info(
            f"SNAP {frame.frame_type.name} {frame.width}x{frame.height} "
            f"payload={len(frame.payload)} B",
            throttle_duration_sec=10.0,
        )
        if frame.frame_type == FrameType.IMAGE_JPEG:
            try:
                rgb8, width, height = jpeg_to_rgb8(frame.payload)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"jpeg decode failed: {exc}")
                return
            self._publish_image(width, height, rgb8)
        elif frame.frame_type == FrameType.IMAGE_RGB565:
            if self._use_jpeg and self._device_jpeg_ok:
                self._device_jpeg_ok = False
                self.get_logger().warning(
                    "Nicla JPEG encode unavailable (RAM); using RGB565 SNAP only",
                    throttle_duration_sec=60.0,
                )
            try:
                rgb8 = rgb565_to_rgb8(frame.payload, frame.width, frame.height)
            except ValueError as exc:
                self.get_logger().error(f"rgb565 decode failed: {exc}")
                return
            self._publish_image(frame.width, frame.height, rgb8)

    def _on_imu_timer(self) -> None:
        if not self._imu_pub:
            return
        if not self._io_lock.acquire(blocking=False):
            return
        try:
            client = self._ensure_client()
            if not client:
                return
            try:
                sample = client.read_imu()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"IMU failed: {exc}", throttle_duration_sec=5.0)
                self._reset_client()
                return
        finally:
            self._io_lock.release()

        accel, gyro = self._apply_imu_signs(
            sample.linear_acceleration, sample.angular_velocity
        )
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        self._imu_pub.publish(msg)

    def _publish_image(self, width: int, height: int, rgb8: bytes) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = list(rgb8)
        self._image_pub.publish(msg)

        if self._camera_info_pub:
            cx = self._camera_cx if width >= 320 else width / 2.0
            cy = self._camera_cy if height >= 240 else height / 2.0
            info = camera_info_from_intrinsics(
                width,
                height,
                self._frame_id,
                self._camera_fx,
                self._camera_fy,
                cx,
                cy,
                self._camera_distortion_model,
                self._camera_distortion,
            )
            info.header.stamp = stamp
            self._camera_info_pub.publish(info)

    def destroy_node(self) -> bool:
        if self._client:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
            self._client = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NiclaSerialBridge()
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
