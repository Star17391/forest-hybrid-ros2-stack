"""Métricas partilhadas para benchmark de pose (Fase 0)."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a: float, b: float) -> float:
    d = abs(a - b)
    return min(d, 2.0 * math.pi - d)


def percentile(sorted_vals: Sequence[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    idx = int(p * (len(sorted_vals) - 1))
    return float(sorted_vals[idx])


def path_length(xy: Sequence[tuple[float, float]]) -> float:
    if len(xy) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(xy, xy[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


@dataclass
class PoseErrorStats:
    samples: int
    duration_s: float
    pos_rmse_m: float
    pos_mean_m: float
    pos_max_m: float
    pos_p95_m: float
    yaw_rmse_deg: float
    yaw_mean_deg: float
    yaw_max_deg: float
    yaw_p95_deg: float
    gt_path_m: float
    fused_path_m: float
    end_pos_error_m: float
    drift_pct: float

    def to_dict(self) -> dict:
        return asdict(self)


def compute_pose_errors(
    fused_xy: Sequence[tuple[float, float, float]],
    gz_xy: Sequence[tuple[float, float, float]],
) -> PoseErrorStats | None:
    """fused/gz rows: (t, x, y, yaw). Alinhamento por índice (mesmo tick)."""
    n = min(len(fused_xy), len(gz_xy))
    if n < 2:
        return None

    dpos: list[float] = []
    dyaw: list[float] = []
    for i in range(n):
        _, fx, fy, fyaw = fused_xy[i]
        _, gx, gy, gyaw = gz_xy[i]
        dpos.append(math.hypot(fx - gx, fy - gy))
        dyaw.append(math.degrees(angle_diff(fyaw, gyaw)))

    dpos_sorted = sorted(dpos)
    dyaw_sorted = sorted(dyaw)
    duration = fused_xy[n - 1][0] - fused_xy[0][0]

    gt_xy = [(r[1], r[2]) for r in gz_xy[:n]]
    fused_xy2 = [(r[1], r[2]) for r in fused_xy[:n]]

    gt_len = path_length(gt_xy)
    end_err = dpos[-1]
    drift_pct = (end_err / gt_len * 100.0) if gt_len > 1e-3 else float("nan")

    return PoseErrorStats(
        samples=n,
        duration_s=duration,
        pos_rmse_m=math.sqrt(sum(e * e for e in dpos) / n),
        pos_mean_m=sum(dpos) / n,
        pos_max_m=dpos_sorted[-1],
        pos_p95_m=percentile(dpos_sorted, 0.95),
        yaw_rmse_deg=math.sqrt(sum(e * e for e in dyaw) / n),
        yaw_mean_deg=sum(dyaw) / n,
        yaw_max_deg=dyaw_sorted[-1],
        yaw_p95_deg=percentile(dyaw_sorted, 0.95),
        gt_path_m=gt_len,
        fused_path_m=path_length(fused_xy2),
        end_pos_error_m=end_err,
        drift_pct=drift_pct,
    )


def write_csv(path: Path, fused_xy, gz_xy) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    n = min(len(fused_xy), len(gz_xy))
    with path.open("w", encoding="utf-8") as f:
        f.write("t,fused_x,fused_y,fused_yaw,gz_x,gz_y,gz_yaw,pos_err_m,yaw_err_deg\n")
        for i in range(n):
            t, fx, fy, fyaw = fused_xy[i]
            _, gx, gy, gyaw = gz_xy[i]
            pe = math.hypot(fx - gx, fy - gy)
            ye = math.degrees(angle_diff(fyaw, gyaw))
            f.write(
                f"{t:.6f},{fx:.6f},{fy:.6f},{fyaw:.6f},"
                f"{gx:.6f},{gy:.6f},{gyaw:.6f},{pe:.6f},{ye:.6f}\n"
            )


def write_metrics_json(path: Path, label: str, stats: PoseErrorStats, extra: dict | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"label": label, "metrics": stats.to_dict()}
    if extra:
        payload["extra"] = extra
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def try_plot_trajectories(
    out_png: Path,
    fused_xy,
    gz_xy,
    label: str,
) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    n = min(len(fused_xy), len(gz_xy))
    if n < 2:
        return False

    fx = [r[1] for r in fused_xy[:n]]
    fy = [r[2] for r in fused_xy[:n]]
    gx = [r[1] for r in gz_xy[:n]]
    gy = [r[2] for r in gz_xy[:n]]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(gx, gy, "g-", label="Gazebo GT", linewidth=1.5)
    axes[0].plot(fx, fy, "b--", label="pose_fused", linewidth=1.2)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title(f"Trajetórias — {label}")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    errs = [
        math.hypot(fused_xy[i][1] - gz_xy[i][1], fused_xy[i][2] - gz_xy[i][2])
        for i in range(n)
    ]
    ts = [fused_xy[i][0] - fused_xy[0][0] for i in range(n)]
    axes[1].plot(ts, errs, "r-", linewidth=1.0)
    axes[1].set_xlabel("t [s]")
    axes[1].set_ylabel("|Δpos| [m]")
    axes[1].set_title("Erro posição vs tempo")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True
