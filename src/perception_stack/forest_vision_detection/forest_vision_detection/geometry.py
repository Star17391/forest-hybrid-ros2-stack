#!/usr/bin/env python3
"""Pure-geometry core for the vision auto-labeler (no ROS dependency).

SDF world parsing, per-model 3D bounding boxes (full tree vs trunk-only),
and pinhole projection into image space. Imported by both the ROS node
(gz_auto_labeler_node) and the offline re-labeller (relabel_offline).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

try:  # baked convex hulls for tight rock silhouette boxes (no trimesh at runtime)
    from .rock_hulls import _ROCK_HULLS
except ImportError:  # standalone / non-package import
    try:
        from rock_hulls import _ROCK_HULLS
    except ImportError:
        _ROCK_HULLS: dict[str, np.ndarray] = {}


# ── Class taxonomy ──────────────────────────────────────────────────────────

CLASS_NAMES = ["tree", "rock", "bush", "fallen_log"]
CLASS_ID = {n: i for i, n in enumerate(CLASS_NAMES)}

# Model URI / name-prefix → semantic class
_PREFIX_TO_CLASS: dict[str, str] = {
    "tree": "tree",
    "rock": "rock",
    "bush": "bush",
    "fallen_log": "fallen_log",
}


@dataclass(frozen=True)
class ModelBBox:
    """Axis-aligned bbox in the model's local frame (metres, unit scale).

    hx/hy = half-widths in x/y; z_lo/z_hi = vertical extent (z up).
    """
    hx: float
    hy: float
    z_lo: float
    z_hi: float


# Per-URI dimensions measured from the ForestGen collision meshes (metres).
# Trees: full canopy bbox, measured from *_col.dae (base at z=0). Rocks: rock.stl.
_URI_DIMS: dict[str, ModelBBox] = {
    "Tree1": ModelBBox(3.96, 5.22, 0.0, 12.25),
    "Tree2": ModelBBox(3.90, 3.71, 0.0, 11.67),
    "Tree3": ModelBBox(4.78, 5.35, 0.0, 12.28),
    "Tree4": ModelBBox(3.90, 3.31, 0.0, 9.47),
    "Tree5": ModelBBox(2.64, 2.64, 0.0, 11.88),
    "Tree6": ModelBBox(5.06, 4.17, 0.0, 14.85),
    "Rock1": ModelBBox(0.45, 0.47, -0.50, 0.48),
    "Rock2": ModelBBox(0.87, 0.96, -0.91, 1.03),
    "Rock3": ModelBBox(1.80, 1.58, -1.82, 1.63),
}

# TRUNK-only bbox per tree URI: half-width = trunk radius × ~1.4 margin; height
# from ground (z=0) up to where the canopy starts (measured from the radial
# profile). Gives a narrow vertical box on the stem, not the wide canopy.
# Tree5's foliage reaches the base (no distinct trunk in the mesh) → conservative.
_URI_TRUNK_DIMS: dict[str, ModelBBox] = {
    "Tree1": ModelBBox(0.48, 0.48, 0.0, 4.3),
    "Tree2": ModelBBox(0.45, 0.45, 0.0, 4.7),
    "Tree3": ModelBBox(0.38, 0.38, 0.0, 4.9),
    "Tree4": ModelBBox(0.60, 0.60, 0.0, 5.2),
    "Tree5": ModelBBox(0.50, 0.50, 0.0, 3.5),
    "Tree6": ModelBBox(0.53, 0.53, 0.0, 3.7),
}
_TRUNK_FALLBACK = ModelBBox(0.5, 0.5, 0.0, 4.0)

# Fallback per-class dims for inline models (bush spheres r≈0.6; fallen_log
# is a cylinder r=0.15 L=1.8 whose axis is local-z, later laid down by roll≈90°).
_CLASS_FALLBACK_DIMS: dict[str, ModelBBox] = {
    "tree": ModelBBox(4.0, 4.0, 0.0, 12.0),
    "rock": ModelBBox(0.9, 0.9, -0.9, 0.9),
    "bush": ModelBBox(0.65, 0.65, -0.6, 0.7),
    "fallen_log": ModelBBox(0.15, 0.15, -0.9, 0.9),
}


# ── SDF parsing ─────────────────────────────────────────────────────────────

@dataclass
class WorldObject:
    name: str
    cls: str
    cls_id: int
    pos: np.ndarray              # [x, y, z] in SDF world frame (= ROS map)
    corners_world: np.ndarray    # (8, 3) bbox corners already in world frame


def _rpy_to_rotmat(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _bbox_corners(bbox: ModelBBox, scale: float) -> np.ndarray:
    hx, hy = bbox.hx * scale, bbox.hy * scale
    zl, zh = bbox.z_lo * scale, bbox.z_hi * scale
    corners = []
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (zl, zh):
                corners.append([sx, sy, sz])
    return np.array(corners, dtype=np.float64)


def _local_points(uri: str, cls: str, bbox: ModelBBox, scale: float) -> np.ndarray:
    """Mesh-frame points whose projected 2D bbox = the object's label box.

    Rocks are low-poly and rotated, so the 8-corner AABB of the full mesh is far
    larger than the visible silhouette. For rock URIs with a baked convex hull we
    use the hull points instead — the 2D bbox of the projected hull equals the
    tight silhouette (extreme projected points are always hull vertices). All
    other models keep the AABB corners.
    """
    if cls == "rock":
        hull = _ROCK_HULLS.get(_uri_basename(uri))
        if hull is not None:
            return hull * scale
    return _bbox_corners(bbox, scale)


def _parse_pose(text: str) -> tuple[np.ndarray, np.ndarray]:
    vals = [float(v) for v in text.split()]
    return np.array(vals[:3]), np.array(vals[3:6])


def _class_from_name(name: str) -> str | None:
    for prefix, cls in _PREFIX_TO_CLASS.items():
        rest = name[len(prefix):]
        if name.startswith(prefix) and (rest == "" or rest[0] == "_"):
            return cls
    return None


def _uri_basename(uri: str) -> str:
    # model://Tree6  → Tree6   ;   model://Tree6/meshes/x.dae → Tree6
    s = uri.replace("model://", "").strip("/")
    return s.split("/")[0] if s else ""


def _dims_for(uri: str, cls: str, tree_target: str) -> ModelBBox:
    base = _uri_basename(uri)
    if cls == "tree" and tree_target == "trunk":
        return _URI_TRUNK_DIMS.get(base, _TRUNK_FALLBACK)
    if base in _URI_DIMS:
        return _URI_DIMS[base]
    return _CLASS_FALLBACK_DIMS[cls]


def parse_world_sdf(sdf_path: str, tree_target: str = "trunk",
                    ground_fn=None) -> dict[str, WorldObject]:
    """Parse ForestGen SDF and return labelled static objects.

    tree_target: "trunk" labels only the tree stem (narrow box up to the canopy);
                 "full" labels the whole tree including the canopy.
    ground_fn:   optional callable (N,2)->(N,) terrain height. When given, rock
                 silhouette vertices below the terrain are lifted to the ground
                 line, so the label box covers only the above-ground part of the
                 rock (rocks are placed partly buried by the vertical jitter).
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        world = root

    objects: dict[str, WorldObject] = {}

    def _add(name: str, pose_text: str, scale: float, uri: str) -> None:
        cls = _class_from_name(name)
        if cls is None:
            return
        pos, rpy = _parse_pose(pose_text)
        bbox = _dims_for(uri, cls, tree_target)
        R = _rpy_to_rotmat(rpy)
        corners_local = _local_points(uri, cls, bbox, scale)
        corners_world = (R @ corners_local.T).T + pos
        # Clip rocks at the terrain: lift buried silhouette vertices to the ground
        # line so the label box only covers the visible (above-ground) part.
        if cls == "rock" and ground_fn is not None and uri:
            g = np.asarray(ground_fn(corners_world[:, :2]))
            corners_world[:, 2] = np.maximum(corners_world[:, 2], g)
        objects[name] = WorldObject(
            name=name, cls=cls, cls_id=CLASS_ID[cls],
            pos=pos, corners_world=corners_world,
        )

    # <include> elements (trees, rocks): carry <uri>, <name>, <pose>, <scale>
    for inc in world.findall("include"):
        name_el = inc.find("name")
        pose_el = inc.find("pose")
        if name_el is None or pose_el is None:
            continue
        uri_el = inc.find("uri")
        scale_el = inc.find("scale")
        scale_txt = scale_el.text.strip().split()[0] if scale_el is not None else "1.0"
        uri = uri_el.text.strip() if uri_el is not None else ""
        _add(name_el.text.strip(), pose_el.text.strip(), float(scale_txt), uri)

    # <model> elements (inline: bushes, fallen logs) — no uri, use class fallback
    for model in world.findall("model"):
        name = model.get("name", "")
        pose_el = model.find("pose")
        if pose_el is None:
            continue
        _add(name, pose_el.text.strip(), 1.0, "")

    return objects


# ── Geometry / projection ────────────────────────────────────────────────────

# Body (x-forward, y-left, z-up) → optical (z-forward, x-right, y-down).
# The static TF publishes camera_front_optical with RPY=(0,0,0) relative to
# base_link, i.e. it is a *body*-oriented frame, NOT a true optical frame.
# We rotate into optical convention so the pinhole projection below is valid.
_R_OPT_BODY = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def _quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [  2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
        [  2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


def _tf_to_matrix(tf_stamped) -> np.ndarray:
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    T = np.eye(4)
    T[:3, :3] = _quat_to_rotmat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _pose_to_matrix(px, py, pz, qx, qy, qz, qw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _quat_to_rotmat(qx, qy, qz, qw)
    T[:3, 3] = [px, py, pz]
    return T


@dataclass
class Projected:
    cls_id: int
    cx: float
    cy: float
    bw: float
    bh: float
    depth: float            # optical-frame depth of object centre (metres)
    u_px: float             # pixel centre (for occlusion sampling)
    v_px: float


def _project_object(
    obj: WorldObject,
    T_optcam_world: np.ndarray,   # 4×4: world → optical camera frame
    K: np.ndarray,
    img_w: int,
    img_h: int,
    max_range: float,
    min_bbox_px: int,
) -> Projected | None:
    corners_h = np.hstack([obj.corners_world, np.ones((len(obj.corners_world), 1))])
    corners_cam = (T_optcam_world @ corners_h.T).T[:, :3]

    centre_h = np.array([*obj.pos, 1.0])
    centre_cam = (T_optcam_world @ centre_h)[:3]
    depth = float(centre_cam[2])
    if depth <= 0.2 or depth > max_range:
        return None

    in_front = corners_cam[:, 2] > 0.05
    if not np.any(in_front):
        return None
    corners_cam = corners_cam[in_front]

    xc = corners_cam[:, 0] / corners_cam[:, 2]
    yc = corners_cam[:, 1] / corners_cam[:, 2]
    u = K[0, 0] * xc + K[0, 2]
    v = K[1, 1] * yc + K[1, 2]

    u_min = max(0.0, float(np.min(u)))
    v_min = max(0.0, float(np.min(v)))
    u_max = min(float(img_w), float(np.max(u)))
    v_max = min(float(img_h), float(np.max(v)))

    bw_px = u_max - u_min
    bh_px = v_max - v_min
    if bw_px < min_bbox_px or bh_px < min_bbox_px:
        return None
    if u_max <= 0 or v_max <= 0 or u_min >= img_w or v_min >= img_h:
        return None

    cx = max(0.0, min(1.0, (u_min + u_max) * 0.5 / img_w))
    cy = max(0.0, min(1.0, (v_min + v_max) * 0.5 / img_h))
    bw = min(1.0, bw_px / img_w)
    bh = min(1.0, bh_px / img_h)

    # Optical projection of the object *centre* for occlusion sampling.
    u_centre = float(K[0, 0] * centre_cam[0] / depth + K[0, 2])
    v_centre = float(K[1, 1] * centre_cam[1] / depth + K[1, 2])

    return Projected(obj.cls_id, cx, cy, bw, bh, depth, u_centre, v_centre)

