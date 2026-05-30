"""Serial framing for the Nicla Vision sensor node (firmware forest_nicla_sensor_node)."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import serial

FRAME_MAGIC = b"NICLAv1"
FRAME_HEADER_SIZE = len(FRAME_MAGIC) + 1 + 2 + 2 + 4  # magic + type + w + h + len
FRAME_TRAILER_SIZE = 2  # crc16

USB_VID_ARDUINO = 0x2341
USB_PID_NICLA_VISION_HS = 0x045F
USB_PID_NICLA_VISION_HS_ALT = 0x055F  # Virtual Comm Port (variant)
USB_PID_NICLA_VISION_RUNTIME = 0x025F  # runtime USB identity (some cores)


class FrameType(IntEnum):
    IMAGE_RGB565 = 0x01
    IMU_SAMPLE = 0x02
    IMAGE_JPEG = 0x03


IMU_PAYLOAD_SIZE = 28  # uint32 timestamp + 6 float32 (little-endian)
IMU_STRUCT = struct.Struct("<Iffffff")


@dataclass(frozen=True)
class NiclaFrame:
    frame_type: FrameType
    width: int
    height: int
    payload: bytes


@dataclass(frozen=True)
class ImuSample:
    timestamp_ms: int
    linear_acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def rgb565_to_rgb8(buffer: bytes, width: int, height: int) -> bytes:
    """Decode GC2145 / mbed Camera RGB565 (big-endian per pixel, Arduino visualizer)."""
    expected = width * height * 2
    if len(buffer) < expected:
        raise ValueError(f"rgb565 buffer too small: {len(buffer)} < {expected}")
    out = bytearray(width * height * 3)
    idx = 0
    for i in range(0, expected, 2):
        # Nicla Vision + Arduino Camera library: 16-bit pixels are big-endian on the wire.
        word = (buffer[i] << 8) | buffer[i + 1]
        r = ((word >> 11) & 0x1F) << 3
        g = ((word >> 5) & 0x3F) << 2
        b = (word & 0x1F) << 3
        out[idx] = r
        out[idx + 1] = g
        out[idx + 2] = b
        idx += 3
    return bytes(out)


def parse_imu_payload(payload: bytes) -> ImuSample:
    if len(payload) < IMU_PAYLOAD_SIZE:
        raise ValueError(f"IMU payload too small: {len(payload)}")
    timestamp_ms, ax, ay, az, gx, gy, gz = IMU_STRUCT.unpack_from(payload)
    return ImuSample(
        timestamp_ms=timestamp_ms,
        linear_acceleration=(ax, ay, az),
        angular_velocity=(gx, gy, gz),
    )


def find_nicla_serial_port() -> Optional[str]:
    """Return /dev/ttyACM* for an attached Nicla Vision, if present."""
    try:
        import serial.tools.list_ports
    except ImportError:
        return None

    for port in serial.tools.list_ports.comports():
        if port.vid == USB_VID_ARDUINO and port.pid in (
            USB_PID_NICLA_VISION_HS,
            USB_PID_NICLA_VISION_HS_ALT,
            USB_PID_NICLA_VISION_RUNTIME,
        ):
            return port.device
        desc = (port.description or "").lower()
        if "nicla vision" in desc:
            return port.device
    return None
