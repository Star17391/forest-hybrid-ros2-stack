#!/usr/bin/env python3
"""Re-label a captured dataset offline — no simulation needed.

Reads the per-frame camera extrinsics logged during capture (poses.csv + meta.json)
and re-projects the world objects with the *current* bbox definitions and
tree_target. Overwrites the YOLO label .txt files in place.

Use this whenever you tweak tree_target or the box dimensions in
gz_auto_labeler_node.py — instead of re-running the sim, just re-project.

Usage:
  python3 -m forest_vision_detection.relabel_offline \
      --dataset ~/datasets/forest_vision_labels \
      --tree-target trunk
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from forest_vision_detection.geometry import (
    parse_world_sdf, _project_object, CLASS_ID, CLASS_NAMES,
)
from forest_vision_detection.terrain_height import (
    TerrainHeightSampler, terrain_stl_for_world,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline re-labelling of a captured dataset")
    parser.add_argument("--dataset", type=Path, required=True,
                        help="Dataset root (must contain poses.csv + meta.json)")
    parser.add_argument("--tree-target", default="trunk", choices=["trunk", "full"])
    parser.add_argument("--world-sdf", default="",
                        help="Override world SDF (default: from meta.json)")
    parser.add_argument("--classes", default="",
                        help="Comma-separated classes to re-project (e.g. 'rock'). "
                             "Only these classes' boxes are replaced; existing boxes of "
                             "other classes are kept as-is (preserves their depth-occlusion "
                             "filtering). Default: re-project all classes.")
    args = parser.parse_args()

    only_ids: set[int] | None = None
    if args.classes.strip():
        only_ids = set()
        for c in args.classes.split(","):
            c = c.strip()
            if c not in CLASS_ID:
                raise SystemExit(f"Unknown class {c!r}. Valid: {CLASS_NAMES}")
            only_ids.add(CLASS_ID[c])

    meta_path = args.dataset / "meta.json"
    poses_path = args.dataset / "poses.csv"
    if not meta_path.is_file() or not poses_path.is_file():
        raise SystemExit(
            f"Missing meta.json/poses.csv in {args.dataset}. "
            "This dataset was captured before pose-logging was enabled — re-capture once."
        )

    meta = json.loads(meta_path.read_text())
    sdf = args.world_sdf or meta["world_sdf"]
    K = np.array(meta["K"], dtype=np.float64).reshape(3, 3)
    W, H = int(meta["width"]), int(meta["height"])
    max_range = float(meta.get("max_range_m", 18.0))
    min_bbox = int(meta.get("min_bbox_pixels", 15))

    # Terrain sampler to clip rocks at the ground (rocks are placed partly buried).
    ground_fn = None
    stl = terrain_stl_for_world(sdf, Path(sdf).resolve().parent.parent / "models")
    if stl is not None:
        ground_fn = TerrainHeightSampler(stl).height
        print(f"Terrain clip: {stl.name} (rocks clipped at ground)")
    else:
        print("WARNING: terrain STL not found — rocks NOT clipped at ground.")

    objects = parse_world_sdf(sdf, tree_target=args.tree_target, ground_fn=ground_fn)
    print(f"World: {sdf}  tree_target={args.tree_target}  objects={len(objects)}")

    lines = poses_path.read_text().splitlines()
    header = lines[0].split(",")
    n_rewritten = 0
    n_boxes = 0
    for row in lines[1:]:
        if not row.strip():
            continue
        parts = row.split(",")
        split, stem = parts[0], parts[1]
        T = np.array([float(x) for x in parts[2:18]], dtype=np.float64).reshape(4, 4)

        labels = []
        for obj in objects.values():
            if only_ids is not None and obj.cls_id not in only_ids:
                continue
            pr = _project_object(obj, T, K, W, H, max_range, min_bbox)
            if pr is not None:
                labels.append(pr)

        lbl_path = args.dataset / "labels" / split / f"{stem}.txt"
        lbl_path.parent.mkdir(parents=True, exist_ok=True)

        # When only re-projecting some classes, keep the existing boxes of the
        # other classes (preserves their depth-occlusion filtering from capture).
        kept: list[str] = []
        if only_ids is not None and lbl_path.is_file():
            for ln in lbl_path.read_text().splitlines():
                p = ln.split()
                if len(p) == 5 and int(p[0]) not in only_ids:
                    kept.append(ln)

        with lbl_path.open("w") as f:
            for ln in kept:
                f.write(ln + "\n")
            for p in labels:
                f.write(f"{p.cls_id} {p.cx:.6f} {p.cy:.6f} {p.bw:.6f} {p.bh:.6f}\n")
        n_rewritten += 1
        n_boxes += len(labels) + len(kept)

    print(f"Re-labelled {n_rewritten} frames, {n_boxes} boxes total "
          f"({n_boxes/max(1,n_rewritten):.1f}/frame).")
    print("Note: offline re-labelling skips depth-occlusion (depth not stored).")
    print(f"Review: python3 .../vision_labels_audit.py --output-dir {args.dataset} --render 24")


if __name__ == "__main__":
    main()
