#!/usr/bin/env python3
"""
Audit tool for auto-labeled vision dataset.

Modes:
  --live      Monitor the auto-labeler while it runs (prints stats every 10s)
  --render N  Draw labels onto N random frames → <output_dir>/review/ + a grid
  (default)   One-shot: read the output_dir and report class distribution

Usage:
  python3 vision_labels_audit.py [--output-dir DIR] [--duration S] [--live]
  python3 vision_labels_audit.py --render 24 [--output-dir DIR]
"""

from __future__ import annotations

import argparse
import os
import random
import time
from collections import Counter
from pathlib import Path


CLASS_NAMES = ["tree", "rock", "bush", "fallen_log"]
_CLASS_COLOURS = [(0, 200, 0), (0, 0, 230), (0, 180, 180), (200, 0, 200)]


def _count_labels(labels_dir: Path) -> tuple[Counter, int, int]:
    cls_count: Counter = Counter()
    total_frames = 0
    empty_frames = 0
    for split in ("train", "val"):
        d = labels_dir / split
        if not d.is_dir():
            continue
        for lbl in d.glob("*.txt"):
            lines = [l.strip() for l in lbl.read_text().splitlines() if l.strip()]
            if not lines:
                empty_frames += 1
                continue
            total_frames += 1
            for line in lines:
                cls_id = int(line.split()[0])
                name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"cls_{cls_id}"
                cls_count[name] += 1
    return cls_count, total_frames, empty_frames


def _split_sizes(output_dir: Path) -> dict[str, int]:
    sizes = {}
    for split in ("train", "val"):
        imgs = list((output_dir / "images" / split).glob("*.jpg"))
        sizes[split] = len(imgs)
    return sizes


def _print_report(output_dir: Path) -> None:
    sizes = _split_sizes(output_dir)
    total_imgs = sum(sizes.values())
    cls_count, total_frames, empty = _count_labels(output_dir / "labels")

    print(f"\n{'='*54}")
    print(f"  Vision dataset  →  {output_dir}")
    print(f"{'='*54}")
    print(f"  Images   train={sizes.get('train',0):4d}  val={sizes.get('val',0):4d}  total={total_imgs}")
    print(f"  Frames with labels: {total_frames}   empty: {empty}")
    print()

    if cls_count:
        total_boxes = sum(cls_count.values())
        print(f"  {'Class':<14}  {'Boxes':>6}  {'%':>6}  {'avg/frame':>9}")
        print(f"  {'-'*14}  {'-'*6}  {'-'*6}  {'-'*9}")
        for name in CLASS_NAMES:
            n = cls_count.get(name, 0)
            pct = 100.0 * n / total_boxes if total_boxes else 0.0
            avg = n / total_frames if total_frames else 0.0
            print(f"  {name:<14}  {n:>6}  {pct:>5.1f}%  {avg:>9.2f}")
        print(f"  {'TOTAL':<14}  {total_boxes:>6}")
    else:
        print("  No labels found yet.")
    print()


def _live_monitor(output_dir: Path, duration: float) -> None:
    print(f"Live monitor for {duration:.0f}s — output: {output_dir}")
    print("(Ctrl-C to stop early)\n")
    t0 = time.monotonic()
    prev_total = 0
    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= duration:
                break
            sizes = _split_sizes(output_dir)
            total = sum(sizes.values())
            rate = (total - prev_total) / 10.0
            prev_total = total
            print(
                f"  [{elapsed:5.0f}s]  images={total:4d}  "
                f"train={sizes.get('train',0)}  val={sizes.get('val',0)}  "
                f"rate={rate:.1f} f/10s"
            )
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass
    _print_report(output_dir)


def _render_labels(output_dir: Path, n: int) -> None:
    """Draw YOLO labels onto N random frames and build a review grid."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("ERROR: opencv + numpy required for --render. pip install opencv-python numpy")
        return

    pairs: list[tuple[Path, Path]] = []
    for split in ("train", "val"):
        for img in (output_dir / "images" / split).glob("*.jpg"):
            lbl = output_dir / "labels" / split / f"{img.stem}.txt"
            if lbl.is_file():
                pairs.append((img, lbl))
    if not pairs:
        print(f"No image/label pairs in {output_dir}")
        return

    random.shuffle(pairs)
    pairs = pairs[:n]
    review_dir = output_dir / "review"
    review_dir.mkdir(exist_ok=True)

    thumbs = []
    for img_path, lbl_path in pairs:
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        for line in lbl_path.read_text().splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            cid = int(parts[0])
            cx, cy, bw, bh = (float(x) for x in parts[1:])
            x1, y1 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
            x2, y2 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
            col = _CLASS_COLOURS[cid % len(_CLASS_COLOURS)]
            cv2.rectangle(bgr, (x1, y1), (x2, y2), col, 1)
            cv2.putText(bgr, CLASS_NAMES[cid], (x1, max(10, y1 - 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
        out = review_dir / f"review_{img_path.stem}.jpg"
        cv2.imwrite(str(out), bgr)
        thumbs.append(cv2.resize(bgr, (320, 240)))

    if thumbs:
        cols = 4
        rows = (len(thumbs) + cols - 1) // cols
        canvas = np.zeros((rows * 240, cols * 320, 3), dtype=np.uint8)
        for i, t in enumerate(thumbs):
            r, c = divmod(i, cols)
            canvas[r*240:(r+1)*240, c*320:(c+1)*320] = t
        grid_path = review_dir / "_grid.jpg"
        cv2.imwrite(str(grid_path), canvas)
        print(f"\nRendered {len(thumbs)} annotated frames → {review_dir}")
        print(f"  Open the mosaic:  xdg-open {grid_path}")
        print(f"  Individual frames: {review_dir}/review_*.jpg")
        print("  Colours: tree=green  rock=red  bush=teal  fallen_log=magenta")


def _check_ros_output_dir() -> Path | None:
    """Try to get output_dir from the running gz_auto_labeler node via ros2 param."""
    try:
        import subprocess
        result = subprocess.run(
            ["ros2", "param", "get", "/gz_auto_labeler", "output_dir"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            if "String value is:" in line:
                return Path(line.split("String value is:")[-1].strip())
    except Exception:
        pass
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit auto-labeled vision dataset")
    parser.add_argument(
        "--output-dir", "-o",
        type=Path,
        default=None,
        help="Dataset root (images/ + labels/ subdirs). Auto-detected from ROS param if omitted.",
    )
    parser.add_argument("--duration", "-d", type=float, default=60.0,
                        help="Live monitor duration in seconds (default 60)")
    parser.add_argument("--live", action="store_true",
                        help="Monitor live capture rate, then print report")
    parser.add_argument("--render", type=int, default=0, metavar="N",
                        help="Draw labels onto N random frames → <output_dir>/review/")
    args = parser.parse_args()

    out_dir = args.output_dir
    if out_dir is None:
        out_dir = _check_ros_output_dir()
    if out_dir is None:
        out_dir = Path.home() / "datasets" / "forest_vision_labels"
        print(f"Auto-detected output dir: {out_dir}")

    if not out_dir.exists():
        print(f"Output dir not found: {out_dir}")
        print("Is the sim running?  forest up sim-vision-capture -d ...")
        return

    if args.render > 0:
        _print_report(out_dir)
        _render_labels(out_dir, args.render)
        return
    if args.live:
        _live_monitor(out_dir, args.duration)
    else:
        _print_report(out_dir)

    # Final advice
    sizes = _split_sizes(out_dir)
    total = sum(sizes.values())
    if total < 100:
        print(f"  Only {total} images. Drive a waypoint mission to collect more.")
    elif total < 400:
        print(f"  {total} images. Recommend 400+ for good YOLO training.")
    else:
        print(f"  {total} images — ready to review and train!")
        print(f"  Next: review labels, then cd forest-vision-training && python train.py")


if __name__ == "__main__":
    main()
