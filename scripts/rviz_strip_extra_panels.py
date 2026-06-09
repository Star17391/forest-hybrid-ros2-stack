#!/usr/bin/env python3
"""Remove Selection / Views / Tool Properties panels from Forest RViz configs."""
from __future__ import annotations

from pathlib import Path

DROP_PANEL_CLASSES = {
    "rviz_common/Selection",
    "rviz_common/Views",
    "rviz_common/Tool Properties",
}
ROOT = Path(__file__).resolve().parents[1]
TARGET_DIRS = [
    ROOT / "src/conf/forest_hybrid_conf/config",
    ROOT / "src/sim_bridge/forest_sim_bridge/config",
    ROOT / "src/drivers_stack/forest_lidar_ros2/config",
    ROOT / "src/navigation_stack/forest_navigation_ros2/config",
    ROOT / "src/perception_stack/forest_semantic_segmentation/config",
]


def strip_file(path: Path) -> bool:
    lines = path.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    i = 0
    changed = False
    while i < len(lines):
        line = lines[i]
        if line.strip() == "- Class: rviz_common/Selection":
            changed = True
            i += 1
            while i < len(lines) and not lines[i].startswith("  - Class:"):
                i += 1
            continue
        if line.strip() == "- Class: rviz_common/Views":
            changed = True
            i += 1
            while i < len(lines) and not lines[i].startswith("  - Class:"):
                i += 1
            continue
        if line.strip() == "- Class: rviz_common/Tool Properties":
            changed = True
            i += 1
            while i < len(lines) and not lines[i].startswith("  - Class:"):
                i += 1
            continue
        # Only strip dock geometry keys (not Visualization Manager "  Views:").
        if (
            line.startswith("  Selection:")
            or line.startswith("  Tool Properties:")
            or (line.startswith("  Views:") and i > 0 and lines[i - 1].strip() != "Value: true")
        ):
            changed = True
            i += 1
            if line.startswith("  Views:") or line.startswith("  Selection:") or line.startswith(
                "  Tool Properties:"
            ):
                while i < len(lines) and lines[i].startswith("    collapsed:"):
                    i += 1
            continue
        if line.startswith("  QMainWindow State:"):
            changed = True
            i += 1
            while i < len(lines) and lines[i].startswith("    "):
                i += 1
            continue
        if line.strip() == "Hide Right Dock: false":
            out.append("  Hide Right Dock: true")
            changed = True
            i += 1
            continue
        out.append(line)
        i += 1
    if not changed:
        return False
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return True


def main() -> int:
    updated = 0
    for d in TARGET_DIRS:
        if not d.is_dir():
            continue
        for path in sorted(d.glob("*.rviz")):
            if strip_file(path):
                print(f"stripped: {path}")
                updated += 1
    print(f"done ({updated} files updated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
