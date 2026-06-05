"""Seleccionar pose MARBLE HD2 em /forest_gen/gz/world_tf(_full) (Pose_V).

Alinha com marble_pose_from_gz.py: frame nomeado ou índice estável
(z≈spawn, baixo spin — evita latch na hélice).
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GzMarblePosePicker:
    def __init__(
        self,
        model_name: str = "marble_hd2",
        child_frame: str = "marble_hd2/base_link",
        seed_z: float = 0.35,
    ) -> None:
        self._model = model_name
        self._child = child_frame
        self._seed_z = seed_z
        self._latched_index: int | None = None
        self._index_history: dict[int, list[tuple[float, float, float, float]]] = {}
        self._samples = 0
        self._last_label: str = ""

    @property
    def last_label(self) -> str:
        return self._last_label

    def _matches_model(self, child_frame_id: str) -> bool:
        if not child_frame_id:
            return False
        m = self._model
        return (
            child_frame_id == m
            or child_frame_id == self._child
            or child_frame_id.startswith(m + "/")
            or child_frame_id.startswith(m + "::")
        )

    def _pick_named(self, msg: TFMessage):
        candidates = [tf for tf in msg.transforms if self._matches_model(tf.child_frame_id)]
        if not candidates:
            return None

        def rank(child: str) -> tuple[int, int]:
            if child in (self._child, f"{self._model}/base_link", f"{self._model}::base_link"):
                return (0, len(child))
            if child == self._model:
                return (1, len(child))
            return (2, len(child))

        candidates.sort(key=lambda tf: rank(tf.child_frame_id))
        chosen = candidates[0]
        self._last_label = chosen.child_frame_id or "named"
        return chosen.transform

    def _stability_cost(self, idx: int, geom: Transform) -> float:
        z = float(geom.translation.z)
        if z < 0.12 or z > 6.0:
            return float("inf")
        x, y = geom.translation.x, geom.translation.y
        if abs(x) < 1e-4 and abs(y) < 1e-4:
            return float("inf")
        yaw = yaw_from_quat(
            geom.rotation.x, geom.rotation.y, geom.rotation.z, geom.rotation.w
        )
        hist = self._index_history.setdefault(idx, [])
        hist.append((x, y, yaw, z))
        if len(hist) > 12:
            del hist[:-12]
        if len(hist) < 4:
            return float("inf")
        max_dyaw = 0.0
        max_dxy = 0.0
        for i in range(1, len(hist)):
            max_dyaw = max(
                max_dyaw, abs(_normalize_angle(hist[i][2] - hist[i - 1][2]))
            )
            max_dxy = max(
                max_dxy,
                math.hypot(hist[i][0] - hist[i - 1][0], hist[i][1] - hist[i - 1][1]),
            )
        return abs(z - self._seed_z) + max_dyaw * 8.0 + max_dxy * 2.0

    def _pick_unnamed(self, msg: TFMessage):
        if not msg.transforms:
            return None

        if self._latched_index is not None:
            idx = self._latched_index
            if idx < len(msg.transforms):
                self._last_label = f"index[{idx}]"
                return msg.transforms[idx].transform
            return None

        self._samples += 1
        best_idx: int | None = None
        best_cost = float("inf")
        for idx, tf in enumerate(msg.transforms):
            cost = self._stability_cost(idx, tf.transform)
            if cost < best_cost:
                best_cost = cost
                best_idx = idx

        if best_idx is not None and best_cost < float("inf") and self._samples >= 8:
            self._latched_index = best_idx
            self._last_label = f"index[{best_idx}]"
            return msg.transforms[best_idx].transform

        if best_idx is not None and self._samples < 8:
            self._last_label = f"index[{best_idx}]?prelatch"
            return msg.transforms[best_idx].transform
        return None

    def pick_xy_yaw(self, msg: TFMessage) -> tuple[float, float, float] | None:
        geom = self._pick_named(msg)
        if geom is None:
            geom = self._pick_unnamed(msg)
        if geom is None:
            return None

        x, y = geom.translation.x, geom.translation.y
        if self._latched_index is not None and abs(x) < 1e-6 and abs(y) < 1e-6:
            return None

        yaw = yaw_from_quat(
            geom.rotation.x, geom.rotation.y, geom.rotation.z, geom.rotation.w
        )
        return (x, y, yaw)
