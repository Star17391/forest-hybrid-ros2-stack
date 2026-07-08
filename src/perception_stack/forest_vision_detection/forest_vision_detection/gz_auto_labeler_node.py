#!/usr/bin/env python3
"""Gazebo ground-truth auto-labeler for vision object detection.

Parses a ForestGen SDF world file to extract static object poses (trees, rocks,
bushes, fallen logs). Subscribes to the front camera and uses TF2 to transform
world objects into image space, saving YOLO-format labels alongside each frame.

Drive the robot with a planned waypoint mission (or any controller) so diverse
viewpoints are captured — this node is agnostic to *how* the robot moves, it just
labels whatever the camera sees. Labels are auto-generated; the user only does a
quick review pass (delete bad frames) rather than annotating from scratch.

Per-model 3D bounding boxes are measured from the collision meshes (metres),
then scaled by each instance's <scale> and rotated by its yaw. Camera projection
uses the real CameraInfo intrinsics. Optional depth-based occlusion rejection
drops objects hidden behind nearer geometry.

YOLO label format per line: class_id cx cy w h  (all normalised [0,1])
Classes: 0=tree 1=rock 2=bush 3=fallen_log
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzTransportNode

    _GZ_TRANSPORT_OK = True
except ImportError:  # pragma: no cover - gz python bindings absent
    _GZ_TRANSPORT_OK = False


# ── Geometry core (pure, no ROS) ─────────────────────────────────────────────
from forest_vision_detection.geometry import (  # noqa: E402
    CLASS_NAMES,
    Projected,
    WorldObject,
    parse_world_sdf,
    _R_OPT_BODY,
    _pose_to_matrix,
    _project_object,
    _rpy_to_quat,
    _tf_to_matrix,
)


# ── ROS2 node ───────────────────────────────────────────────────────────────

class GzAutoLabelerNode(Node):
    """Auto-labels camera frames using Gazebo ground-truth object poses.

    Prereq: forest up sim-vision-capture -d --world <world>
    Then drive a planned waypoint mission so the camera sees varied scenes.
    """

    def __init__(self) -> None:
        super().__init__("gz_auto_labeler")

        self.declare_parameter("world_sdf", "")
        self.declare_parameter("output_dir", "/tmp/forest_vision_labels")
        # "trunk" = narrow box on the stem only (no canopy); "full" = whole tree.
        self.declare_parameter("tree_target", "trunk")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_frame", "marble_hd2/camera_front_optical")
        # Pose source: "gz_truth" reads the robot's ground-truth pose straight from
        # Gazebo (immune to EKF/odometry drift); "tf" looks up map→camera via TF2.
        self.declare_parameter("pose_source", "gz_truth")
        self.declare_parameter("gz_pose_topic", "/world/unified_world/pose/info")
        self.declare_parameter("robot_model_name", "marble_hd2")
        # Static base_link→camera offset (from marble_sensor_tf_static / model.sdf).
        self.declare_parameter("camera_xyz", [0.40, 0.0, 0.24])
        self.declare_parameter("camera_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("depth_topic", "/forest_gen/camera_front/depth")
        self.declare_parameter("max_range_m", 18.0)
        self.declare_parameter("min_bbox_pixels", 15)
        self.declare_parameter("save_interval_s", 0.8)
        # Motion gate: only save when the camera viewpoint actually changed since
        # the last saved frame. Avoids flooding the dataset with near-identical
        # frames while the robot is stationary or crawling. A frame is saved when
        # EITHER threshold is exceeded. Set both to 0.0 to disable (time-only).
        self.declare_parameter("min_translation_m", 0.25)
        self.declare_parameter("min_rotation_deg", 8.0)
        self.declare_parameter("tf_timeout_s", 0.15)
        # The static TF frame "..._optical" is actually body-oriented (RPY=0).
        # Keep true unless the stack is fixed to publish a real optical frame.
        self.declare_parameter("apply_optical_rotation", True)
        # Depth-based occlusion rejection (needs depth_topic publishing).
        self.declare_parameter("use_depth_occlusion", True)
        self.declare_parameter("occlusion_depth_ratio", 0.65)
        self.declare_parameter("occlusion_min_range_m", 4.0)
        # Train/val split: contiguous blocks of frames go entirely to one split
        # (avoids near-duplicate consecutive frames leaking across the split).
        self.declare_parameter("val_block_period", 7)
        self.declare_parameter("block_size", 15)
        # Debug overlay: save annotated copies for visual verification.
        self.declare_parameter("draw_debug", False)

        sdf_path = self.get_parameter("world_sdf").value
        self._out = Path(self.get_parameter("output_dir").value)
        self._world_frame = self.get_parameter("world_frame").value
        self._cam_frame = self.get_parameter("camera_frame").value
        self._pose_source = str(self.get_parameter("pose_source").value)
        self._robot_name = str(self.get_parameter("robot_model_name").value)
        cam_xyz = list(self.get_parameter("camera_xyz").value)
        cam_rpy = list(self.get_parameter("camera_rpy").value)
        self._T_base_cam = _pose_to_matrix(
            cam_xyz[0], cam_xyz[1], cam_xyz[2],
            *_rpy_to_quat(cam_rpy[0], cam_rpy[1], cam_rpy[2]),
        )
        self._max_range = float(self.get_parameter("max_range_m").value)
        self._min_bbox = int(self.get_parameter("min_bbox_pixels").value)
        self._save_interval = float(self.get_parameter("save_interval_s").value)
        self._min_trans = float(self.get_parameter("min_translation_m").value)
        self._min_rot = math.radians(float(self.get_parameter("min_rotation_deg").value))
        self._tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        self._apply_opt = bool(self.get_parameter("apply_optical_rotation").value)
        self._use_depth = bool(self.get_parameter("use_depth_occlusion").value)
        self._occ_ratio = float(self.get_parameter("occlusion_depth_ratio").value)
        self._occ_min_range = float(self.get_parameter("occlusion_min_range_m").value)
        self._val_period = max(1, int(self.get_parameter("val_block_period").value))
        self._block_size = max(1, int(self.get_parameter("block_size").value))
        self._draw_debug = bool(self.get_parameter("draw_debug").value)

        if not sdf_path or not Path(sdf_path).is_file():
            self.get_logger().error(
                f"Parameter 'world_sdf' missing or not a file: {sdf_path!r}. Shutting down."
            )
            raise SystemExit(1)

        self._sdf_path = sdf_path
        self._poses_csv = None
        tree_target = str(self.get_parameter("tree_target").value)
        # Terrain sampler so rock boxes are clipped at the ground (rocks are
        # placed partly buried — without this the box covers the buried part too).
        ground_fn = None
        try:
            from .terrain_height import TerrainHeightSampler, terrain_stl_for_world
            stl = terrain_stl_for_world(sdf_path, Path(sdf_path).resolve().parent.parent / "models")
            if stl is not None:
                ground_fn = TerrainHeightSampler(stl).height
                self.get_logger().info(f"Rock ground-clip terrain: {stl.name}")
            else:
                self.get_logger().warn("Terrain STL not found — rocks not clipped at ground.")
        except Exception as exc:
            self.get_logger().warn(f"Terrain sampler unavailable ({exc}); rocks not clipped.")
        self._world_objects = parse_world_sdf(sdf_path, tree_target=tree_target, ground_fn=ground_fn)
        counts: dict[str, int] = {}
        for obj in self._world_objects.values():
            counts[obj.cls] = counts.get(obj.cls, 0) + 1
        self.get_logger().info(
            f"World '{Path(sdf_path).name}': {counts} total={len(self._world_objects)} "
            f"tree_target={tree_target}"
        )

        for split in ("train", "val"):
            (self._out / "images" / split).mkdir(parents=True, exist_ok=True)
            (self._out / "labels" / split).mkdir(parents=True, exist_ok=True)
        if self._draw_debug:
            (self._out / "debug").mkdir(parents=True, exist_ok=True)

        self._K: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._last_save = 0.0
        self._last_pose: np.ndarray | None = None   # camera position (world) of last saved frame
        self._last_fwd: np.ndarray | None = None     # camera forward axis (world) of last saved frame
        self._skipped_motion = 0
        # Resume from any frames already in output_dir so repeated capture
        # sessions accumulate instead of overwriting frame_000000 onwards.
        self._frame_count = self._resume_frame_count()
        if self._frame_count:
            self.get_logger().info(
                f"Resuming capture: {self._frame_count} frames already in {self._out}"
            )
        self._skipped_tf = 0
        self._occluded = 0

        # Ground-truth robot pose from Gazebo (immune to EKF drift).
        self._gt_lock = threading.Lock()
        self._T_world_base_gt: np.ndarray | None = None
        self._gz_node: GzTransportNode | None = None
        self._use_gz_truth = self._pose_source == "gz_truth" and _GZ_TRANSPORT_OK
        if self._pose_source == "gz_truth" and not _GZ_TRANSPORT_OK:
            self.get_logger().warn(
                "pose_source=gz_truth but gz.transport bindings unavailable; "
                "falling back to TF lookup (subject to odometry drift)."
            )
        if self._use_gz_truth:
            gz_topic = str(self.get_parameter("gz_pose_topic").value)
            self._gz_node = GzTransportNode()
            if self._gz_node.subscribe(Pose_V, gz_topic, self._on_gz_pose):
                self.get_logger().info(
                    f"GT pose: {gz_topic} (entity '{self._robot_name}')"
                )
            else:
                self.get_logger().error(
                    f"gz.transport subscribe failed on {gz_topic}; using TF fallback."
                )
                self._use_gz_truth = False

        # TF fallback (also used when pose_source=tf).
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        img_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        # Gazebo sensor topics are BEST_EFFORT; match it or no messages arrive.
        sensor_qos = rclpy.qos.qos_profile_sensor_data

        self.create_subscription(CameraInfo, info_topic, self._on_cam_info, sensor_qos)
        self.create_subscription(Image, img_topic, self._on_image, sensor_qos)
        if self._use_depth:
            self.create_subscription(Image, depth_topic, self._on_depth, sensor_qos)

        self.get_logger().info(
            f"Auto-labeler → {self._out}  interval={self._save_interval:.1f}s "
            f"range={self._max_range:.0f}m pose={'gz_truth' if self._use_gz_truth else 'tf'} "
            f"optical_rot={self._apply_opt} depth_occ={self._use_depth}"
        )

    def _on_gz_pose(self, msg: "Pose_V") -> None:
        """gz.transport callback (separate thread): cache robot GT world pose."""
        for p in msg.pose:
            if p.name == self._robot_name:
                q = p.orientation
                t = p.position
                T = _pose_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
                with self._gt_lock:
                    self._T_world_base_gt = T
                return

    # ── callbacks ──

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if self._K is None:
            self._K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"CameraInfo: {msg.width}×{msg.height} "
                f"fx={self._K[0,0]:.1f} fy={self._K[1,1]:.1f}"
            )

    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding in ("32FC1", "32FC"):
                d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == "16UC1":
                d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            else:
                return
            self._depth = d
        except Exception:
            self._depth = None

    def _is_occluded(self, p: Projected, img_w: int, img_h: int) -> bool:
        if not self._use_depth or self._depth is None:
            return False
        if p.depth < self._occ_min_range:
            return False
        dh, dw = self._depth.shape
        u = int(round(p.u_px * dw / img_w))
        v = int(round(p.v_px * dh / img_h))
        if not (0 <= u < dw and 0 <= v < dh):
            return False
        # Sample a small patch and take the median valid depth.
        u0, u1 = max(0, u - 2), min(dw, u + 3)
        v0, v1 = max(0, v - 2), min(dh, v + 3)
        patch = self._depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.1)]
        if valid.size == 0:
            return False
        measured = float(np.median(valid))
        return measured < self._occ_ratio * p.depth

    def _split_for_block(self) -> str:
        block = self._frame_count // self._block_size
        return "val" if (block % self._val_period == 0) else "train"

    def _camera_extrinsics(self, msg: Image) -> np.ndarray | None:
        """Return T_optcam_world (world → optical camera), or None if unavailable.

        Primary: ground-truth robot pose from Gazebo + static base→camera offset.
        Fallback: TF2 lookup of world→camera (subject to odometry drift).
        """
        if self._use_gz_truth:
            with self._gt_lock:
                T_world_base = self._T_world_base_gt
            if T_world_base is None:
                self._skipped_tf += 1
                return None
            T_world_cambody = T_world_base @ self._T_base_cam
            T_cambody_world = np.linalg.inv(T_world_cambody)
        else:
            try:
                tf = self._tf_buf.lookup_transform(
                    self._world_frame, self._cam_frame, msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
                )
            except Exception:
                self._skipped_tf += 1
                return None
            T_cambody_world = np.linalg.inv(_tf_to_matrix(tf))

        if self._apply_opt:
            R = np.eye(4)
            R[:3, :3] = _R_OPT_BODY
            return R @ T_cambody_world               # world → true optical
        return T_cambody_world

    def _on_image(self, msg: Image) -> None:
        if self._K is None:
            return
        now = time.monotonic()
        if now - self._last_save < self._save_interval:
            return

        T_optcam_world = self._camera_extrinsics(msg)
        if T_optcam_world is None:
            return

        # Motion gate: skip frames whose viewpoint barely changed since the last
        # saved one (keeps the dataset diverse instead of full of duplicates).
        if self._min_trans > 0.0 or self._min_rot > 0.0:
            R = T_optcam_world[:3, :3]
            cam_pos = -R.T @ T_optcam_world[:3, 3]   # camera position in world
            fwd = R[2, :]                             # optical +Z (view direction) in world
            if self._last_pose is not None:
                moved = float(np.linalg.norm(cam_pos - self._last_pose))
                cos = float(np.clip(np.dot(fwd, self._last_fwd), -1.0, 1.0))
                turned = math.acos(cos)
                if moved < self._min_trans and turned < self._min_rot:
                    self._skipped_motion += 1
                    return
        else:
            cam_pos = fwd = None

        projections: list[Projected] = []
        for obj in self._world_objects.values():
            pr = _project_object(
                obj, T_optcam_world, self._K, msg.width, msg.height,
                self._max_range, self._min_bbox,
            )
            if pr is None:
                continue
            if self._is_occluded(pr, msg.width, msg.height):
                self._occluded += 1
                continue
            projections.append(pr)

        if not projections:
            return

        try:
            bgr = self._decode_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return
        if bgr is None:
            return

        split = self._split_for_block()
        stem = f"frame_{self._frame_count:06d}"
        img_path = self._out / "images" / split / f"{stem}.jpg"
        lbl_path = self._out / "labels" / split / f"{stem}.txt"

        cv2.imwrite(str(img_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with lbl_path.open("w") as f:
            for p in projections:
                f.write(f"{p.cls_id} {p.cx:.6f} {p.cy:.6f} {p.bw:.6f} {p.bh:.6f}\n")

        # Log the camera extrinsics so the dataset can be re-labelled offline
        # (e.g. change tree_target or box dims) without re-running the sim.
        self._log_pose(split, stem, T_optcam_world, msg.width, msg.height)

        if self._draw_debug:
            self._save_debug(bgr.copy(), projections, msg.width, msg.height, stem)

        self._frame_count += 1
        self._last_save = now
        if cam_pos is not None:                  # commit motion-gate reference
            self._last_pose = cam_pos
            self._last_fwd = fwd
        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f"Saved {self._frame_count} frames  "
                f"(tf_skip={self._skipped_tf} occluded={self._occluded} "
                f"motion_skip={self._skipped_motion})"
            )

    # ── resume / accumulation ──

    def _resume_frame_count(self) -> int:
        """Highest existing frame index + 1, so re-runs into the same dir append."""
        highest = -1
        for split in ("train", "val"):
            d = self._out / "images" / split
            if not d.is_dir():
                continue
            for p in d.glob("frame_*.jpg"):
                try:
                    highest = max(highest, int(p.stem.split("_")[1]))
                except (IndexError, ValueError):
                    continue
        return highest + 1

    # ── pose logging (for offline re-labelling) ──

    def _init_pose_log(self, sdf_path: str, w: int, h: int) -> None:
        import json
        meta = {
            "world_sdf": str(sdf_path),
            "K": self._K.reshape(-1).tolist(),
            "width": int(w),
            "height": int(h),
            "max_range_m": self._max_range,
            "min_bbox_pixels": self._min_bbox,
        }
        (self._out / "meta.json").write_text(json.dumps(meta, indent=2))
        # Append so repeated sessions into the same dir keep earlier poses; only
        # write the header when starting a fresh (or headerless) file.
        poses_path = self._out / "poses.csv"
        fresh = not poses_path.is_file() or poses_path.stat().st_size == 0
        self._poses_csv = poses_path.open("a")
        if fresh:
            self._poses_csv.write("split,stem," + ",".join(f"m{i}" for i in range(16)) + "\n")

    def _log_pose(self, split: str, stem: str, T: np.ndarray, w: int, h: int) -> None:
        if self._poses_csv is None:
            self._init_pose_log(self._sdf_path, w, h)
        flat = ",".join(f"{v:.8e}" for v in T.reshape(-1))
        self._poses_csv.write(f"{split},{stem},{flat}\n")
        self._poses_csv.flush()

    # ── helpers ──

    @staticmethod
    def _decode_bgr(msg: Image) -> np.ndarray | None:
        if msg.encoding in ("rgb8", "bgr8", "8UC3"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else arr
        return None

    def _save_debug(self, bgr, projs: list[Projected], w: int, h: int, stem: str) -> None:
        colours = [(0, 200, 0), (0, 0, 230), (0, 180, 180), (200, 0, 200)]
        for p in projs:
            x1 = int((p.cx - p.bw / 2) * w)
            y1 = int((p.cy - p.bh / 2) * h)
            x2 = int((p.cx + p.bw / 2) * w)
            y2 = int((p.cy + p.bh / 2) * h)
            c = colours[p.cls_id % len(colours)]
            cv2.rectangle(bgr, (x1, y1), (x2, y2), c, 1)
            cv2.putText(bgr, CLASS_NAMES[p.cls_id], (x1, max(10, y1 - 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, c, 1)
        cv2.imwrite(str(self._out / "debug" / f"{stem}.jpg"), bgr)


def main() -> None:
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    try:
        node = GzAutoLabelerNode()
    except SystemExit:
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
