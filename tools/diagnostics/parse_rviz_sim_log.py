#!/usr/bin/env python3
"""Parse sim.log for RViz/Ogre/clock/TF timeline evidence."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


PATTERNS = {
    "rviz_start": re.compile(r"\[rviz2-\d+\]: process started"),
    "rviz_died": re.compile(r"\[ERROR\] \[rviz2-\d+\]: process has died"),
    "drawable_fail": re.compile(r"\[rviz2-\d+\] failed to create drawable"),
    "ogre_crash": re.compile(r"Ogre::InternalErrorException|GL vertex buffer"),
    "clock_bridge": re.compile(r"Creating GZ->ROS Bridge:.*(/clock|clock)"),
    "points_labeled_sub": re.compile(r"Subscribing to: /perception/lidar/points_labeled"),
    "ekf_nan": re.compile(r"NaNs were detected|TF_NAN_INPUT"),
    "tf_echo": re.compile(r"Translation:"),
}


def parse_log(path: Path) -> dict[str, list[int]]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    hits: dict[str, list[int]] = {k: [] for k in PATTERNS}
    for i, line in enumerate(lines, 1):
        for key, pat in PATTERNS.items():
            if pat.search(line):
                hits[key].append(i)
    return hits


def report(path: Path, hits: dict[str, list[int]]) -> None:
    print(f"=== {path} ({sum(len(v) for v in hits.values())} events) ===")
    for key, lines in hits.items():
        if not lines:
            continue
        print(f"  {key}: lines {lines[:5]}{'...' if len(lines) > 5 else ''} ({len(lines)} total)")

    # Causal ordering check
    rviz = hits["rviz_start"]
    clock = hits["clock_bridge"]
    drawable = hits["drawable_fail"]
    died = hits["rviz_died"]
    if rviz and clock and min(drawable + died or [10**9]) < min(clock):
        print("  [VERDICT] Drawable/crash BEFORE /clock bridge — H1 CONFIRMED")
    if hits["ogre_crash"]:
        print("  [VERDICT] Ogre GL vertex buffer crash present")
    if len(hits["points_labeled_sub"]) > 1:
        print(f"  [VERDICT] Duplicate points_labeled subscribe ({len(hits['points_labeled_sub'])}x)")
    if hits["ekf_nan"]:
        print("  [VERDICT] EKF/TF NaN present — secondary TF poison")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("logs", nargs="+", type=Path)
    args = p.parse_args()
    for log in args.logs:
        if not log.is_file():
            print(f"Missing: {log}", file=sys.stderr)
            return 1
        report(log, parse_log(log))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
