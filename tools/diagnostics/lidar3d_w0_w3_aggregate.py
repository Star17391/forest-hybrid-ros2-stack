#!/usr/bin/env python3
"""Agrega world_summary.json de uma corrida W0–W3."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_dir", type=Path, help="Pasta da corrida (contém W0/, W1/, …)")
    args = p.parse_args()
    run_dir = args.run_dir
    if not run_dir.is_dir():
        print(f"ERROR: not a directory: {run_dir}", file=sys.stderr)
        return 2

    worlds = []
    for sub in sorted(run_dir.iterdir()):
        summary = sub / "world_summary.json"
        if summary.is_file():
            worlds.append(json.loads(summary.read_text(encoding="utf-8")))

    if not worlds:
        print(f"ERROR: no world_summary.json under {run_dir}", file=sys.stderr)
        return 2

    overall_fail = any(w.get("verdict") == "FAIL" for w in worlds)
    report = {
        "run_dir": str(run_dir.resolve()),
        "overall_verdict": "FAIL" if overall_fail else "PASS",
        "worlds": worlds,
    }
    out_path = run_dir / "run_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    md_path = run_dir / "RUN_REPORT.md"
    lines = [
        f"# LiDAR 3D validation — {run_dir.name}",
        "",
        f"**Overall:** {report['overall_verdict']}",
        "",
        "| World | Mundo | Ground | Trunks | TF % | G% | O% | grid cov% | |dz|g | Issues |",
        "|-------|-------|--------|--------|------|----|----|-----------|-------|--------|",
    ]
    for w in worlds:
        t = w.get("topics", {})
        d = w.get("debug_stats", {})
        tf = w.get("tf", {})
        issues = ", ".join(w.get("issues", [])) or "—"
        lines.append(
            f"| {w.get('world_id')} | `{w.get('world_name')}` | {w.get('verdict')} | "
            f"{w.get('trunk_verdict', '—')} | {tf.get('map_to_laser_ok_pct', '—')}% | "
            f"{t.get('ground_pct', '—')}% | {t.get('obstacle_pct', '—')}% | "
            f"{d.get('avg_grid_coverage_pct', '—')}% | "
            f"{d.get('avg_grid_mean_abs_dz_ground_m', '—')} | {issues} |"
        )
    lines.extend(
        [
            "",
            "Baseline RANSAC: `docs/reports/lidar3d_validation/20260526_150334/`.",
            "Expectativas grelha 2.5D: `docs/reports/lidar3d_validation/EXPECTED_OUTPUT.md`.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"\nOverall: {report['overall_verdict']}")
    print(f"  {out_path}")
    print(f"  {md_path}\n")
    for w in worlds:
        print(f"  {w['world_id']}: {w['verdict']} — {', '.join(w.get('issues', [])) or 'ok'}")
    return 1 if overall_fail else 0


if __name__ == "__main__":
    sys.exit(main())
