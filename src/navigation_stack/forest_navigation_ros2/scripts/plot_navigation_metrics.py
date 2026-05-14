#!/usr/bin/env python3
"""Gera gráficos a partir do CSV de métricas do navigation_node."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path) -> dict[str, list[float]]:
    cols: dict[str, list[float]] = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                if k == "leg_id":
                    continue
                cols.setdefault(k, []).append(float(v))
    return cols


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot forest navigation metrics")
    parser.add_argument("csv", nargs="?", default="/tmp/forest_navigation_metrics.csv")
    parser.add_argument("-o", "--output", default="/tmp/forest_navigation_plots.png")
    args = parser.parse_args()

    path = Path(args.csv)
    if not path.is_file():
        raise SystemExit(f"CSV não encontrado: {path}")

    data = load_csv(path)
    t = data.get("t_sec", [])
    if not t:
        raise SystemExit("CSV vazio ou sem coluna t_sec")

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Forest Navigation MVP — tracking metrics")

    axes[0, 0].plot(t, data.get("lateral_error_m", []), label="lateral error [m]")
    axes[0, 0].set_xlabel("t [s]")
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].plot(t, data.get("heading_error_rad", []), label="heading error [rad]")
    axes[0, 1].set_xlabel("t [s]")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].plot(t, data.get("v_cmd", []), label="v_cmd")
    axes[1, 0].plot(t, data.get("w_cmd", []), label="w_cmd")
    axes[1, 0].set_xlabel("t [s]")
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    axes[1, 1].plot(t, data.get("goal_dist_m", []), label="goal distance [m]")
    axes[1, 1].plot(t, data.get("progress", []), label="progress")
    axes[1, 1].set_xlabel("t [s]")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    if data.get("x") and data.get("y"):
        fig2, ax2 = plt.subplots(figsize=(6, 6))
        ax2.plot(data["x"], data["y"], "-", label="trace")
        ax2.set_aspect("equal", adjustable="box")
        ax2.set_xlabel("x [m]")
        ax2.set_ylabel("y [m]")
        ax2.legend()
        ax2.grid(True)
        trace_out = Path(args.output).with_name("forest_navigation_trace.png")
        fig2.savefig(trace_out, dpi=150, bbox_inches="tight")
        print(f"Trace plot: {trace_out}")

    fig.tight_layout()
    fig.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"Metrics plot: {args.output}")


if __name__ == "__main__":
    main()
