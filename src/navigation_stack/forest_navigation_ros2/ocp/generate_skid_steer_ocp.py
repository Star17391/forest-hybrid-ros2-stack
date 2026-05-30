#!/usr/bin/env python3
"""Generate acados OCP solver for forest NMPC local controller (skid-steer kin.)."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from acados_template.utils import get_tera

from skid_steer_model import export_skid_steer_kin_model

DEFAULT_N = 25
DEFAULT_TF = 2.0
DEFAULT_V_MAX = 0.5
DEFAULT_W_MAX = 1.0


def ensure_tera_renderer() -> None:
    repo_root = Path(__file__).resolve().parents[4]
    bundled = repo_root / "tools" / "third_party" / "bin" / "t_renderer"
    if bundled.is_file() and os.access(bundled, os.X_OK) and bundled.stat().st_size > 0:
        os.environ["TERA_PATH"] = str(bundled)
        return

    acados_root = Path(os.environ.get("ACADOS_SOURCE_DIR", Path.home() / "acados"))
    tera_src = acados_root / "interfaces" / "acados_template" / "tera_renderer"
    tera_bin = tera_src / "target" / "release" / "t_renderer"
    if tera_src.is_dir() and shutil.which("cargo"):
        import subprocess

        subprocess.run(
            ["cargo", "build", "--release"],
            cwd=tera_src,
            check=True,
        )
        bundled.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(tera_bin, bundled)
        bundled.chmod(0o755)
        os.environ["TERA_PATH"] = str(bundled)
        return

    get_tera(force_download=True)


def build_ocp(
    *,
    n_horizon: int = DEFAULT_N,
    tf: float = DEFAULT_TF,
    v_max: float = DEFAULT_V_MAX,
    w_max: float = DEFAULT_W_MAX,
) -> AcadosOcp:
    ocp = AcadosOcp()
    model = export_skid_steer_kin_model()
    ocp.model = model

    ny = 6
    ny_e = 3

    ocp.solver_options.N_horizon = n_horizon
    ocp.solver_options.tf = tf

    w_stage = np.diag([10.0, 10.0, 5.0, 1.0, 0.5, 0.05])
    w_terminal = np.diag([20.0, 20.0, 10.0])

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = w_stage
    ocp.cost.W_e = w_terminal
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)
    ocp.parameter_values = np.zeros(4)

    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([0.0, -w_max])
    ocp.constraints.ubu = np.array([v_max, w_max])
    # Initial state equality x(0) = x0 — bounds updated at runtime in C++.
    ocp.constraints.idxbx_0 = np.array([0, 1, 2])
    ocp.constraints.lbx_0 = np.zeros(3)
    ocp.constraints.ubx_0 = np.zeros(3)

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 1

    return ocp


def smoke_test_solver(json_path: Path) -> None:
    solver = AcadosOcpSolver(None, json_file=str(json_path), build=False, generate=False)

    x0 = np.array([0.0, 0.0, 0.0])
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)

    for k in range(solver.N):
        xref = np.array([0.5 * (k + 1), 0.0, 0.0, 0.4])
        solver.set(k, "p", xref)

    status = solver.solve()
    if status != 0:
        raise RuntimeError(f"acados smoke test failed with status {status}")

    u0 = solver.get(0, "u")
    print(f"smoke_test OK u0=[v={u0[0]:.3f}, omega={u0[1]:.3f}]")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "acados_generated" / "skid_steer_kin",
    )
    parser.add_argument("--N", type=int, default=DEFAULT_N)
    parser.add_argument("--tf", type=float, default=DEFAULT_TF)
    parser.add_argument("--skip-test", action="store_true")
    args = parser.parse_args()

    ensure_tera_renderer()

    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    ocp = build_ocp(n_horizon=args.N, tf=args.tf)
    json_path = out_dir / "acados_ocp.json"
    ocp.code_gen_opts.code_export_directory = str(out_dir)
    ocp.code_gen_opts.json_file = str(json_path)

    AcadosOcpSolver(
        ocp,
        json_file=str(json_path),
        build=True,
        generate=True,
    )

    meta = {
        "model": "skid_steer_kin",
        "N": args.N,
        "tf": args.tf,
        "dt": args.tf / args.N,
        "nx": 3,
        "nu": 2,
        "np": 4,
    }
    (out_dir / "forest_nmpc_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"Generated acados solver in {out_dir}")

    if not args.skip_test:
        smoke_test_solver(json_path)


if __name__ == "__main__":
    main()
