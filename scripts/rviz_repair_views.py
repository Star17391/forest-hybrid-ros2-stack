#!/usr/bin/env python3
"""Repair RViz configs after panel strip (restore Views: header, clean Window Geometry)."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGET_DIRS = [
    ROOT / "src/conf/forest_hybrid_conf/config",
    ROOT / "src/sim_bridge/forest_sim_bridge/config",
    ROOT / "src/drivers_stack/forest_lidar_ros2/config",
    ROOT / "src/navigation_stack/forest_navigation_ros2/config",
    ROOT / "src/perception_stack/forest_semantic_segmentation/config",
]


def repair_views_header(text: str) -> str:
    """Insert missing '  Views:' before orphan orbit view blocks."""
    if "  Views:\n    Current:" in text:
        return text
    text = re.sub(
        r"(  Value: true\n)(    Current:)",
        r"\1  Views:\n\2",
        text,
    )
    lines = text.splitlines()
    out: list[str] = []
    section: str | None = None
    for i, line in enumerate(lines):
        if re.match(r"^  [A-Za-z].*:$", line) and not line.startswith("    "):
            section = line.strip().rstrip(":")
        if (
            line == "    Current:"
            and section == "Tools"
            and (not out or out[-1] != "  Views:")
        ):
            out.append("  Views:")
            section = "Views"
        out.append(line)
    return "\n".join(out) + "\n"


def remove_camera_image_display(text: str) -> str:
    """Drop optional Camera Image display (causes empty white preview dock)."""
    pattern = re.compile(
        r"    - Alpha: 1\n"
        r"      Class: rviz_default_plugins/Image\n"
        r"      Enabled: false\n"
        r"      Name: Camera \(optional\)\n"
        r"      Topic:\n"
        r"        Depth: 5\n"
        r"        Durability Policy: Volatile\n"
        r"        History Policy: Keep Last\n"
        r"        Reliability Policy: Best Effort\n"
        r"        Value: /camera/image_raw\n",
        re.MULTILINE,
    )
    return pattern.sub("", text)


def clean_window_geometry(text: str) -> str:
    lines = text.splitlines()
    if "Window Geometry:" not in lines:
        return text
    wg_idx = lines.index("Window Geometry:")
    head = lines[: wg_idx + 1]
    tail = lines[wg_idx + 1 :]
    cleaned: list[str] = []
    for line in tail:
        stripped = line.strip()
        if stripped == "collapsed: true" and (
            not cleaned or cleaned[-1].strip() in ("Hide Right Dock: true", "Hide Left Dock: false")
        ):
            continue
        if stripped == "collapsed: true" and cleaned and cleaned[-1].strip() == "collapsed: true":
            continue
        cleaned.append(line)
    return "\n".join(head + cleaned) + "\n"


def repair_file(path: Path) -> bool:
    original = path.read_text(encoding="utf-8")
    updated = original
    updated = remove_camera_image_display(updated)
    updated = repair_views_header(updated)
    updated = clean_window_geometry(updated)
    if updated != original:
        path.write_text(updated, encoding="utf-8")
        return True
    return False


def main() -> int:
    changed = 0
    for d in TARGET_DIRS:
        if not d.is_dir():
            continue
        for path in sorted(d.glob("*.rviz")):
            if repair_file(path):
                print(f"repaired: {path}")
                changed += 1
    print(f"done ({changed} files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
