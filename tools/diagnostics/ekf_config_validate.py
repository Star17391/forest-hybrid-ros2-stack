#!/usr/bin/env python3
"""Valida ficheiros ekf*.yaml do robot_localization.

O `robot_localization` espera matrizes 15x15 para `process_noise_covariance` e
`initial_estimate_covariance` (225 floats). Quando passamos só uma diagonal
parcial (ex: 45 valores em 3 linhas), o ROS YAML parser cria um `vector<double>`
mais curto. O EKF lê 225 entradas → memória não inicializada → estado inicial
**NaN** → publica TF NaN para sempre.

Esta ferramenta deteta esse erro antes do runtime.

Uso:
    python3 tools/diagnostics/ekf_config_validate.py \
        src/localization_mapping_stack/forest_state_estimation/config/*.yaml
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml

REQUIRED_15x15 = ("process_noise_covariance", "initial_estimate_covariance")
EXPECTED = 15 * 15

REQUIRED_FRAMES = ("map_frame", "odom_frame", "base_link_frame", "world_frame")


def validate_file(path: Path) -> list[str]:
    errors: list[str] = []
    if not path.is_file():
        return [f"file not found: {path}"]
    try:
        data = yaml.safe_load(path.read_text())
    except Exception as exc:
        return [f"YAML parse error: {exc}"]

    if not isinstance(data, dict):
        return ["root not a dict"]

    for node, sub in data.items():
        if not isinstance(sub, dict):
            continue
        params = sub.get("ros__parameters", sub)
        if not isinstance(params, dict):
            continue
        for key in REQUIRED_15x15:
            if key not in params:
                continue
            val = params[key]
            if not isinstance(val, list):
                errors.append(f"{path}: {key} is not a list")
                continue
            n = len(val)
            if n != EXPECTED:
                errors.append(
                    f"{path}: {key} has {n} entries (expected {EXPECTED} = 15x15) "
                    f"-> robot_localization will read uninit memory -> NaN state"
                )

        for fr in REQUIRED_FRAMES:
            if "publish_tf" in params and params.get("publish_tf") and fr not in params:
                errors.append(f"{path}: publish_tf=true but {fr} missing")

    return errors


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print("usage: ekf_config_validate.py <ekf.yaml> [...]")
        return 2

    total_err = 0
    for p in paths:
        errs = validate_file(p)
        if errs:
            print(f"=== FAIL {p} ===")
            for e in errs:
                print(f"  [error] {e}")
            total_err += len(errs)
        else:
            print(f"=== OK   {p} ===")

    if total_err:
        print(f"\n{total_err} problema(s) detectado(s). Corrige antes de fazer rebuild.")
        return 1
    print("\nTodos os EKF configs OK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
