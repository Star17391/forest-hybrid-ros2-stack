"""Nicla Vision transport layer: USB serial and Wi-Fi TCP (shared NICLAv1 protocol)."""

from __future__ import annotations

import io
import socket
import time
from abc import ABC, abstractmethod
from typing import Optional

import serial

from forest_nicla_vision_ros2.protocol import (
    FRAME_HEADER_SIZE,
    FRAME_TRAILER_SIZE,
    FrameType,
    ImuSample,
    NiclaFrame,
    crc16_ccitt_false,
    find_nicla_serial_port,
    parse_imu_payload,
    rgb565_to_rgb8,
)


class NiclaDeviceClient(ABC):
    """Host client for NICLAv1 command protocol over serial or TCP."""

    def __init__(self, timeout_s: float = 5.0) -> None:
        self._timeout_s = timeout_s

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def _write(self, data: bytes) -> None: ...

    @abstractmethod
    def _read(self, size: int, deadline: float) -> bytes: ...

    def __enter__(self) -> "NiclaDeviceClient":
        self.open()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def send_command(self, command: str) -> None:
        self._write((command.strip() + "\n").encode("ascii"))

    def read_line(self, timeout_s: Optional[float] = None) -> str:
        deadline = time.monotonic() + (timeout_s if timeout_s is not None else self._timeout_s)
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read(1, deadline)
            if not chunk:
                continue
            if chunk == b"\n":
                return buf.decode("ascii", errors="replace").strip()
            if chunk != b"\r":
                buf.extend(chunk)
        raise TimeoutError("timed out waiting for line response")

    def ping(self) -> bool:
        self.send_command("PING")
        return self.read_line() == "PONG"

    def status(self) -> str:
        self.send_command("STATUS")
        return self.read_line()

    def snap(self, jpeg: bool = False) -> NiclaFrame:
        self.send_command("SNAP_JPEG" if jpeg else "SNAP")
        line = self.read_line(timeout_s=max(self._timeout_s, 20.0))
        if line.startswith("ERR"):
            raise RuntimeError(line)
        if not line.startswith("OK"):
            raise RuntimeError(f"unexpected SNAP ack: {line!r}")
        timeout = 25.0 if jpeg else 35.0
        return self._read_binary_frame(frame_timeout_s=max(self._timeout_s, timeout))

    def snap_jpeg(self) -> NiclaFrame:
        frame = self.snap(jpeg=True)
        if frame.frame_type != FrameType.IMAGE_JPEG:
            raise ValueError(f"expected JPEG frame, got {frame.frame_type}")
        return frame

    def read_imu(self) -> ImuSample:
        self.send_command("IMU")
        line = self.read_line(timeout_s=max(self._timeout_s, 2.0))
        if line.startswith("ERR"):
            raise RuntimeError(line)
        if line != "OK":
            raise RuntimeError(f"unexpected IMU ack: {line!r}")
        frame = self._read_binary_frame(frame_timeout_s=max(self._timeout_s, 5.0))
        if frame.frame_type != FrameType.IMU_SAMPLE:
            raise ValueError(f"expected IMU frame, got {frame.frame_type}")
        return parse_imu_payload(frame.payload)

    def _read_binary_frame(self, frame_timeout_s: Optional[float] = None) -> NiclaFrame:
        deadline = time.monotonic() + (
            frame_timeout_s if frame_timeout_s is not None else max(self._timeout_s, 15.0)
        )
        header = self._read_exactly(FRAME_HEADER_SIZE, deadline)
        if not header.startswith(b"NICLAv1"):
            raise ValueError(f"bad frame magic: {header[:16]!r}")

        import struct

        frame_type_raw, width, height, length = struct.unpack_from("<BHH I", header, 7)
        frame_type = FrameType(frame_type_raw)
        payload = self._read_exactly(length, deadline)
        trailer = self._read_exactly(FRAME_TRAILER_SIZE, deadline)
        (crc_rx,) = struct.unpack("<H", trailer)
        crc_tx = crc16_ccitt_false(payload)
        if crc_rx != crc_tx:
            raise ValueError(f"CRC mismatch: rx=0x{crc_rx:04x} tx=0x{crc_tx:04x}")
        return NiclaFrame(frame_type=frame_type, width=width, height=height, payload=payload)

    def _read_exactly(self, size: int, deadline: float) -> bytes:
        out = bytearray()
        while len(out) < size:
            if time.monotonic() > deadline:
                raise TimeoutError(f"timed out reading {size} bytes (got {len(out)})")
            chunk = self._read(size - len(out), deadline)
            if chunk:
                out.extend(chunk)
        return bytes(out)


class NiclaSerialClient(NiclaDeviceClient):
    def __init__(self, port: str, baud: int = 921600, timeout_s: float = 5.0) -> None:
        super().__init__(timeout_s=timeout_s)
        self._port = port
        self._baud = baud
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            self._port,
            self._baud,
            timeout=self._timeout_s,
            write_timeout=self._timeout_s,
        )
        time.sleep(0.8)
        self._drain_banner()

    def _drain_banner(self) -> None:
        if not self._ser:
            return
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            waiting = getattr(self._ser, "in_waiting", 0) or 0
            if waiting <= 0:
                time.sleep(0.05)
                continue
            line = self._ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("READY"):
                return

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def _write(self, data: bytes) -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("serial port is not open")
        self._ser.write(data)
        self._ser.flush()

    def _read(self, size: int, deadline: float) -> bytes:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("serial port is not open")
        remaining = max(0.0, deadline - time.monotonic())
        self._ser.timeout = min(self._timeout_s, remaining) if remaining > 0 else 0.05
        return self._ser.read(size)


class NiclaWifiClient(NiclaDeviceClient):
    def __init__(self, host: str, port: int = 9876, timeout_s: float = 5.0) -> None:
        super().__init__(timeout_s=timeout_s)
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None

    def open(self) -> None:
        if self._sock:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self._timeout_s)
        sock.connect((self._host, self._port))
        self._sock = sock

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def _write(self, data: bytes) -> None:
        if not self._sock:
            raise RuntimeError("wifi socket is not open")
        self._sock.sendall(data)

    def _read(self, size: int, deadline: float) -> bytes:
        if not self._sock:
            raise RuntimeError("wifi socket is not open")
        remaining = max(0.0, deadline - time.monotonic())
        self._sock.settimeout(min(self._timeout_s, remaining) if remaining > 0 else 0.05)
        try:
            return self._sock.recv(size)
        except socket.timeout:
            return b""


def create_nicla_client(
    transport: str,
    *,
    serial_port: str = "",
    baud_rate: int = 921600,
    wifi_host: str = "",
    wifi_port: int = 9876,
    timeout_s: float = 5.0,
) -> NiclaDeviceClient:
    mode = transport.strip().lower()
    if mode == "serial":
        port = serial_port or find_nicla_serial_port() or ""
        if not port:
            raise ValueError("serial transport requires a port or auto-detected Nicla")
        return NiclaSerialClient(port, baud=baud_rate, timeout_s=timeout_s)
    if mode == "wifi":
        if not wifi_host:
            raise ValueError("wifi transport requires wifi_host (Nicla IP)")
        return NiclaWifiClient(wifi_host, port=wifi_port, timeout_s=timeout_s)
    raise ValueError(f"unknown transport: {transport}")


def jpeg_to_rgb8(jpeg_bytes: bytes) -> tuple[bytes, int, int]:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("JPEG decode requires python3-pil (Pillow)") from exc
    image = Image.open(io.BytesIO(jpeg_bytes))
    if image.mode != "RGB":
        image = image.convert("RGB")
    width, height = image.size
    return image.tobytes(), width, height
