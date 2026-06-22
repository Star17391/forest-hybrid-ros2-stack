"""
Gera nuvens de pontos sintéticas mas FIÉIS, com ground-truth tronco/copa por ponto.

O gpu_lidar do Gazebo intersecta a mesh VISUAL (a colisão é um casco simplificado
só para a física). Por isso fazemos raycasting à mesh visual, que tem tronco real.

Ground-truth por ponto vem da GEOMETRIA atingida:
  - tronco = geometria opaca (alpha=255) que começa no solo (a casca/ramos),
  - copa   = qualquer geometria de folhagem (alpha=191),
  - solo   = plano de solo.
label: 0=tronco, 1=copa, 2=solo.

Saída:
  <out>.npz  -> {xyz, label, meta(json)} para análise Python
  <out>.xyzl -> texto "x y z label" para o probe C++ (perceção real)
"""
import argparse
import json
import os

import numpy as np
import trimesh

from lidar_model import rays_in_ground_frame, RANGE_MIN, RANGE_MAX

MODELS = "/home/star17391/Projetos/Gazebo/ForestGen/models"
LBL_TRUNK, LBL_CANOPY, LBL_GROUND = 0, 1, 2
DBH_Z = 1.3  # altura nominal do DBH [m]


def load_tree_visual(tid):
    """Devolve (mesh_concatenada, face_is_trunk[bool], axis_xy, trunk_mesh)."""
    s = trimesh.load(f"{MODELS}/Tree{tid}/meshes/Tree{tid}_visual.dae", force="scene")
    geoms = list(s.geometry.values())
    # Tronco = geometria opaca (alpha 255) com z_min mais baixo.
    def alpha(g):
        return int(g.visual.material.baseColorFactor[3])
    opaque = [g for g in geoms if alpha(g) == 255]
    pool = opaque if opaque else geoms
    trunk_geom = min(pool, key=lambda g: g.vertices[:, 2].min())

    meshes, face_is_trunk = [], []
    for g in geoms:
        meshes.append(g)
        face_is_trunk.append(np.full(len(g.faces), g is trunk_geom, dtype=bool))
    combined = trimesh.util.concatenate(meshes)
    face_is_trunk = np.concatenate(face_is_trunk)

    z = trunk_geom.vertices[:, 2]
    base = trunk_geom.vertices[z < z.min() + 0.3]
    axis_xy = base[:, :2].mean(axis=0)
    return combined, face_is_trunk, axis_xy, trunk_geom


def gt_trunk_diameter(trunk_geom, axis_xy):
    """Diâmetro GT do fuste na banda do DBH, das fatias da geometria de tronco."""
    sec = trunk_geom.section(plane_origin=[axis_xy[0], axis_xy[1], DBH_Z],
                             plane_normal=[0, 0, 1])
    if sec is None:
        v = trunk_geom.vertices
        z = v[:, 2]
        band = (z >= DBH_Z - 0.4) & (z <= DBH_Z + 0.4)
        v = v[band] if band.sum() >= 3 else v
        r = np.hypot(v[:, 0] - axis_xy[0], v[:, 1] - axis_xy[1])
        return float(2 * np.median(r))
    p = sec.vertices[:, :2]
    r = np.hypot(p[:, 0] - axis_xy[0], p[:, 1] - axis_xy[1])
    return float(2 * np.median(r))


def ground_plane(half=30.0):
    v = np.array([[-half, -half, 0], [half, -half, 0],
                  [half, half, 0], [-half, half, 0]], dtype=float)
    return trimesh.Trimesh(vertices=v, faces=[[0, 1, 2], [0, 2, 3]], process=False)


def raycast(tree_mesh, face_is_trunk, axis_xy, dist, azimuth_deg, with_ground=True):
    a = np.radians(azimuth_deg)
    rx = axis_xy[0] + dist * np.cos(a)
    ry = axis_xy[1] + dist * np.sin(a)
    yaw = np.arctan2(axis_xy[1] - ry, axis_xy[0] - rx)
    origin, dirs = rays_in_ground_frame(rx, ry, yaw)

    n_tree = len(tree_mesh.faces)
    scene = trimesh.util.concatenate([tree_mesh, ground_plane()]) if with_ground else tree_mesh
    locs, ray_i, tri_i = scene.ray.intersects_location(
        ray_origins=np.tile(origin, (len(dirs), 1)),
        ray_directions=dirs, multiple_hits=False)
    if len(locs) == 0:
        return np.zeros((0, 3)), np.zeros(0, int)
    d = np.linalg.norm(locs - origin, axis=1)
    keep = (d >= RANGE_MIN) & (d <= RANGE_MAX)
    locs, tri_i = locs[keep], tri_i[keep]

    label = np.full(len(locs), LBL_GROUND, int)
    on_tree = tri_i < n_tree
    label[on_tree] = np.where(face_is_trunk[tri_i[on_tree]], LBL_TRUNK, LBL_CANOPY)
    return locs, label


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tree", type=int, required=True)
    ap.add_argument("--dist", type=float, default=4.0)
    ap.add_argument("--azimuth", type=float, default=0.0)
    ap.add_argument("--no-ground", action="store_true")
    ap.add_argument("--out", type=str, required=True)
    args = ap.parse_args()

    mesh, face_is_trunk, axis_xy, trunk_geom = load_tree_visual(args.tree)
    diam = gt_trunk_diameter(trunk_geom, axis_xy)
    xyz, label = raycast(mesh, face_is_trunk, axis_xy, args.dist, args.azimuth,
                         with_ground=not args.no_ground)

    meta = dict(tree=args.tree, dist=args.dist, azimuth=args.azimuth,
                gt_diameter_m=diam, axis_x=float(axis_xy[0]), axis_y=float(axis_xy[1]),
                n_trunk=int((label == LBL_TRUNK).sum()),
                n_canopy=int((label == LBL_CANOPY).sum()),
                n_ground=int((label == LBL_GROUND).sum()))
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez(args.out, xyz=xyz.astype(np.float32), label=label.astype(np.int32),
             meta=json.dumps(meta))
    with open(os.path.splitext(args.out)[0] + ".xyzl", "w") as f:
        for p, l in zip(xyz, label):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {l}\n")
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
