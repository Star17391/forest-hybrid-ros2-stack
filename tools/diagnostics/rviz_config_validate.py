#!/usr/bin/env python3
"""Validate RViz config YAML for Forest MVP stability rules."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


RULES = {
    "time_sync": "SyncSource: /clock",
    "no_window_geometry_required": "Window Geometry",  # optional but noted
}

RISKY_PATTERNS = [
    (re.compile(r"Style:\s*Flat Squares"), "LaserScan/PointCloud Style Flat Squares (high GL cost)"),
    (re.compile(r"Depth:\s*30"), "QoS/history Depth 30 (retains many scans in GPU)"),
    (re.compile(r"Color Transformer:\s*Intensity"), "Intensity transformer on PointCloud2 (fragile with label channel)"),
    (re.compile(r"Channel Name:\s*label"), "PointCloud2 label channel (verify display disabled at startup)"),
]


def validate(path: Path) -> int:
    text = path.read_text(encoding="utf-8", errors="replace")
    errors: list[str] = []
    warnings: list[str] = []
    info: list[str] = []

    if "Visualization Manager:" not in text:
        errors.append("Missing 'Visualization Manager:' section")
    if "Fixed Frame: map" not in text and "Fixed Frame:" not in text:
        warnings.append("No Fixed Frame: map (check Global Options)")

    if RULES["time_sync"] not in text:
        warnings.append("No Time panel SyncSource /clock — sim TF may desync")
    else:
        info.append("Time sync /clock: OK")

    if RULES["no_window_geometry_required"] not in text:
        warnings.append("No Window Geometry — dock layout will vary each start (desconfigurada)")
    else:
        info.append("Window Geometry: present")
        if "QMainWindow State:" not in text:
            warnings.append(
                "Window Geometry without QMainWindow State — left dock may expand to full width"
            )
        else:
            info.append("QMainWindow State: present (dock widths saved)")

    extra_panels = sum(
        1
        for name in ("Selection", "Tool Properties", "Views")
        if f"Name: {name}" in text
    )
    if extra_panels >= 2:
        warnings.append(
            f"{extra_panels} extra side panels (Selection/Tool/Views) — use Displays+Time only "
            "for narrow left dock"
        )
    panel_count = len(re.findall(r"^\s+- Class: rviz_common/", text, re.MULTILINE))
    if panel_count <= 2:
        info.append(f"Panel count: {panel_count} (minimal — good for narrow layout)")
    else:
        info.append(f"Panel count: {panel_count}")

    enabled = len(re.findall(r"^\s+Enabled:\s*true", text, re.MULTILINE))
    disabled = len(re.findall(r"^\s+Enabled:\s*false", text, re.MULTILINE))
    info.append(f"Displays enabled={enabled} disabled={disabled}")

    for pat, msg in RISKY_PATTERNS:
        if pat.search(text):
            warnings.append(msg)

    topics = re.findall(r"Value:\s*(/\S+)", text)
    info.append(f"Topics referenced: {len(topics)}")
    for t in sorted(set(topics)):
        info.append(f"  - {t}")

    print(f"=== {path} ===")
    for line in info:
        print(f"  [info] {line}")
    for line in warnings:
        print(f"  [WARN] {line}")
    for line in errors:
        print(f"  [ERR]  {line}")

    return 1 if errors else (2 if warnings else 0)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("rviz_files", nargs="+", type=Path)
    args = p.parse_args()
    worst = 0
    for f in args.rviz_files:
        if not f.is_file():
            print(f"Missing: {f}", file=sys.stderr)
            worst = 1
            continue
        worst = max(worst, validate(f))
    return min(worst, 1)


if __name__ == "__main__":
    sys.exit(main())
