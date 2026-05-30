"""Seleccionar pose MARBLE HD2 em /forest_gen/gz/world_tf(_full) (Pose_V).

Alinha com forest_sim_bridge/marble_pose_from_gz.py: frame nomeado ou índice
com movimento (evita alternar entre (0,0) e pose real no mesmo TFMessage).
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
    ) -> None:
        self._model = model_name
        self._child = child_frame
        self._latched_index: int | None = None
        self._index_origin: dict[int, tuple[float, float, float]] = {}
        self._index_motion: dict[int, float] = {}
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

    def _motion_score(self, idx: int, geom: Transform) -> float:
        x, y = geom.translation.x, geom.translation.y
        yaw = yaw_from_quat(
            geom.rotation.x, geom.rotation.y, geom.rotation.z, geom.rotation.w
        )
        if idx not in self._index_origin:
            self._index_origin[idx] = (x, y, yaw)
        ox, oy, oyaw = self._index_origin[idx]
        score = (x - ox) ** 2 + (y - oy) ** 2 + (
            abs(_normalize_angle(yaw - oyaw)) * 0.35
        ) ** 2
        self._index_motion[idx] = max(self._index_motion.get(idx, 0.0), score)
        return score

    def _pick_unnamed(self, msg: TFMessage):
        if not msg.transforms:
            return None

        if self._latched_index is not None:
            idx = self._latched_index
            if idx < len(msg.transforms):
                self._last_label = f"index[{idx}]"
                return msg.transforms[idx].transform
            return None

        best_idx: int | None = None
        best_score = -1.0
        for idx, tf in enumerate(msg.transforms):
            self._motion_score(idx, tf.transform)
            hist = self._index_motion.get(idx, 0.0)
            if hist > best_score:
                best_score = hist
                best_idx = idx

        if best_idx is not None and best_score > 0.01:
            self._latched_index = best_idx
            self._last_label = f"index[{best_idx}]"
            return msg.transforms[best_idx].transform

        if best_idx is not None:
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
        # Pose_V: entrada (0,0) costuma ser mundo/origem — não usar para benchmark.
        if self._latched_index is not None and abs(x) < 1e-6 and abs(y) < 1e-6:
            return None

        yaw = yaw_from_quat(
            geom.rotation.x, geom.rotation.y, geom.rotation.z, geom.rotation.w
        )
        return (x, y, yaw)
