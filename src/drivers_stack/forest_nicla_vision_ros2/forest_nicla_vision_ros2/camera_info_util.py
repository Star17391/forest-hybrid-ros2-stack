"""Build sensor_msgs/CameraInfo for the Nicla Vision (placeholder intrinsics until calibration)."""

from __future__ import annotations

from sensor_msgs.msg import CameraInfo


def camera_info_from_intrinsics(
    width: int,
    height: int,
    frame_id: str,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    distortion_model: str = "plumb_bob",
    d: list[float] | None = None,
) -> CameraInfo:
    """Create CameraInfo with pinhole K and zero distortion unless provided."""
    if d is None:
        d = [0.0, 0.0, 0.0, 0.0, 0.0]

    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.distortion_model = distortion_model
    msg.d = list(d)
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.height = 0
    msg.roi.width = 0
    msg.roi.do_rectify = False
    return msg
