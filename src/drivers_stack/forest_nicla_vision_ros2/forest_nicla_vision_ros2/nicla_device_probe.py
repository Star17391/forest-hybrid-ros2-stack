#!/usr/bin/env python3
"""Validate Nicla Vision over serial or Wi-Fi (PING, STATUS, IMU, SNAP / SNAP_JPEG)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from forest_nicla_vision_ros2.nicla_transport import (
    FrameType,
    create_nicla_client,
    find_nicla_serial_port,
    jpeg_to_rgb8,
    rgb565_to_rgb8,
)


def _write_ppm(path: Path, rgb8: bytes, width: int, height: int) -> None:
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + rgb8)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Probe Arduino Nicla Vision sensor node")
    parser.add_argument("--transport", choices=("serial", "wifi"), default="serial")
    parser.add_argument("--port", default="", help="Serial device (serial transport)")
    parser.add_argument("--wifi-host", default="", help="Nicla IP (wifi transport)")
    parser.add_argument("--wifi-port", type=int, default=9876)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--snap", action="store_true", help="Request one camera frame")
    parser.add_argument("--jpeg", action="store_true", help="Use SNAP_JPEG (Phase 4 firmware)")
    parser.add_argument("--imu", action="store_true", help="Request one IMU sample")
    parser.add_argument(
        "--out",
        default="/tmp/nicla_snap.ppm",
        help="PPM output path when --snap is set",
    )
    parser.add_argument(
        "--color-check",
        action="store_true",
        help="One session: STATUS + RGB565 SNAP + JPEG SNAP (for nicla_jpeg_color_check.sh)",
    )
    parser.add_argument(
        "--out-jpeg",
        default="/tmp/nicla_color_jpeg.ppm",
        help="PPM path for JPEG leg when --color-check is set",
    )
    args = parser.parse_args(argv)

    serial_port = args.port or find_nicla_serial_port() or ""
    if args.transport == "serial" and not serial_port:
        print("Nicla serial port not found", file=sys.stderr)
        return 1
    if args.transport == "wifi" and not args.wifi_host:
        print("--wifi-host required for wifi transport", file=sys.stderr)
        return 1

    print(f"transport={args.transport} jpeg={args.jpeg}")

    try:
        with create_nicla_client(
            args.transport,
            serial_port=serial_port,
            baud_rate=args.baud,
            wifi_host=args.wifi_host,
            wifi_port=args.wifi_port,
        ) as client:
            if not client.ping():
                print("PING failed", file=sys.stderr)
                return 2
            print("PING -> PONG")
            status = client.status()
            print(f"STATUS -> {status}")
            device_jpeg_ok = "enc=rgb888" in status
            if args.jpeg and not device_jpeg_ok:
                print(
                    "WARN: firmware missing enc=rgb888 — SNAP_JPEG uses a broken encoder. "
                    "Using RGB565 SNAP (correct colors). Reflash nicla_sensor_node.ino.",
                    file=sys.stderr,
                )

            if args.imu:
                sample = client.read_imu()
                ax, ay, az = sample.linear_acceleration
                gx, gy, gz = sample.angular_velocity
                print(f"IMU t={sample.timestamp_ms} ms acc=({ax:.2f},{ay:.2f},{az:.2f})")

            def process_snap(
                frame, out_path: Path, want_jpeg: bool, dt: float
            ) -> int:
                if frame.frame_type == FrameType.IMAGE_JPEG:
                    jpg_path = out_path.with_suffix(".jpg")
                    jpg_path.write_bytes(frame.payload)
                    rgb8, width, height = jpeg_to_rgb8(frame.payload)
                    kind = f"jpeg device ({len(frame.payload)} B) raw->{jpg_path}"
                elif frame.frame_type == FrameType.IMAGE_RGB565:
                    rgb8 = rgb565_to_rgb8(frame.payload, frame.width, frame.height)
                    width, height = frame.width, frame.height
                    if want_jpeg and not device_jpeg_ok:
                        kind = (
                            f"rgb565 ({len(frame.payload)} B, "
                            "used instead of broken SNAP_JPEG)"
                        )
                    else:
                        kind = f"rgb565 ({len(frame.payload)} B)"
                else:
                    print(f"unexpected frame type {frame.frame_type}", file=sys.stderr)
                    return 3
                _write_ppm(out_path, rgb8, width, height)
                print(f"SNAP -> {width}x{height} {kind} in {dt:.2f}s -> {out_path}")
                return 0

            if args.color_check:
                t0 = time.monotonic()
                frame565 = client.snap(jpeg=False)
                dt565 = time.monotonic() - t0
                out565 = Path(args.out)
                if process_snap(frame565, out565, want_jpeg=False, dt=dt565) != 0:
                    return 3
                time.sleep(0.4)
                t0 = time.monotonic()
                frame_jpeg = client.snap(jpeg=device_jpeg_ok)
                dt_jpeg = time.monotonic() - t0
                out_jpeg = Path(args.out_jpeg)
                if process_snap(frame_jpeg, out_jpeg, want_jpeg=True, dt=dt_jpeg) != 0:
                    return 3
                return 0

            if not args.snap:
                return 0

            t0 = time.monotonic()
            frame = client.snap(jpeg=args.jpeg and device_jpeg_ok)
            dt = time.monotonic() - t0

            out = Path(args.out)
            return process_snap(frame, out, want_jpeg=args.jpeg, dt=dt)
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
