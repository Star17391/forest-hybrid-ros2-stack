#!/usr/bin/env python3
"""Compara dois metrics.json da Fase 0 (A/B wheel vs wheel+IMU)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_metrics(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data.get("metrics") or data


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("baseline", type=Path, help="metrics.json ekf_wheel_only")
    p.add_argument("candidate", type=Path, help="metrics.json ekf_local")
    args = p.parse_args()

    for path in (args.baseline, args.candidate):
        if not path.is_file():
            print(f"ERRO: ficheiro em falta: {path}", file=sys.stderr)
            return 1

    b = load_metrics(args.baseline)
    c = load_metrics(args.candidate)

    print("\n=== Fase 0 A/B — pose benchmark ===\n")
    print(f"{'métrica':<22} {'baseline':>12} {'candidate':>12} {'Δ%':>10}")
    print("-" * 58)

    keys = [
        ("pos_rmse_m", "RMSE pos [m]", False),
        ("pos_p95_m", "p95 pos [m]", False),
        ("yaw_rmse_deg", "RMSE yaw [°]", False),
        ("yaw_p95_deg", "p95 yaw [°]", False),
        ("drift_pct", "drift [%]", False),
        ("end_pos_error_m", "erro final [m]", False),
    ]

    for key, label, higher_better in keys:
        bv = float(b.get(key, float("nan")))
        cv = float(c.get(key, float("nan")))
        if bv != 0 and not (bv != bv or cv != cv):
            pct = (cv - bv) / abs(bv) * 100.0
            if not higher_better:
                pct = -pct  # lower is better → positive Δ% means improvement
            print(f"{label:<22} {bv:12.3f} {cv:12.3f} {pct:+9.1f}%")
        else:
            print(f"{label:<22} {bv:12.3f} {cv:12.3f}       n/a")

    print("\nInterpretação: Δ% positivo = candidate melhor (menor erro); negativo = candidate pior.")
    print("Critério Fase 0 E1: yaw RMSE ou p95 ↓ ≥15% em traj. com curvas, sem piorar recta.")

    pos_b = float(b.get("pos_rmse_m", float("nan")))
    pos_c = float(c.get("pos_rmse_m", float("nan")))
    yaw_b = float(b.get("yaw_rmse_deg", float("nan")))
    yaw_c = float(c.get("yaw_rmse_deg", float("nan")))
    if pos_c <= pos_b * 0.85 and yaw_c <= yaw_b * 0.85:
        print("VERDICT: candidate PASS (pos e yaw melhoraram ≥15%)")
    elif pos_c > pos_b * 1.2:
        print("VERDICT: candidate FAIL — posição pior que baseline (manter wheel_only ou rever IMU/EKF)")
    elif yaw_c > yaw_b:
        print("VERDICT: candidate FAIL — yaw pior que baseline (IMU não ajuda nesta config)")
    else:
        print("VERDICT: inconclusivo — repetir A/B com ekf_mode correcto nos perfis")
    return 0


if __name__ == "__main__":
    sys.exit(main())
