"""
Bateria de testes de isolação de tronco (T0–T3) nas 6 árvores visuais.

Para cada (árvore, distância, azimute): gera a nuvem rotulada, corre o probe C++
(perceção real) e mede:
  - contaminação de copa na banda do DBH (objetivo: 0 pontos),
  - erro do DBH vs GT,
  - estabilidade do raio entre vistas (CV).

GT do diâmetro (T0): circle-fit de Taubin na secção mais baixa e limpa do tronco.
"""
import json
import subprocess
import sys

import numpy as np

sys.path.insert(0, ".")
import gen_cloud as gc

HERE = "/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack/src/perception_stack/forest_3d_perception/tools/canopy_isolation"
PROBE = f"{HERE}/run_probe.sh"
DISTS = [3.0, 4.0, 6.0, 8.0]
AZIMUTHS = [0.0, 90.0, 180.0, 270.0]


def taubin(x, y):
    x = np.asarray(x, float); y = np.asarray(y, float)
    mx, my = x.mean(), y.mean()
    u, v = x - mx, y - my
    z = u * u + v * v
    Mxx, Myy, Mxy = (u * u).mean(), (v * v).mean(), (u * v).mean()
    Mxz, Myz, Mzz = (u * z).mean(), (v * z).mean(), (z * z).mean()
    Mz = Mxx + Myy; Cov = Mxx * Myy - Mxy * Mxy; Vz = Mzz - Mz * Mz
    A3 = 4 * Mz; A2 = -3 * Mz * Mz - Mzz
    A1 = Vz * Mz + 4 * Cov * Mz - Mxz * Mxz - Myz * Myz
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Vz * Cov
    xx, yy = 0.0, A0
    for _ in range(99):
        Dy = A1 + xx * (2 * A2 + 3 * A3 * xx)
        if abs(Dy) < 1e-18:
            break
        xn = xx - yy / Dy
        yn = A0 + xn * (A1 + xn * (A2 + xn * A3))
        if abs(yn) >= abs(yy):
            break
        xx, yy = xn, yn
    det = xx * xx - xx * Mz + Cov
    xc = (Mxz * (Myy - xx) - Myz * Mxy) / det / 2
    yc = (Myz * (Mxx - xx) - Mxz * Mxy) / det / 2
    return float(np.sqrt(xc * xc + yc * yc + Mz))


def gt_diameter(tid):
    """T0: diâmetro GT na secção limpa mais baixa do fuste (Taubin)."""
    _, _, axis_xy, trunk_geom = gc.load_tree_visual(tid)
    best = None
    for zc in [0.6, 0.8, 1.0, 1.2, 1.3]:
        sec = trunk_geom.section(plane_origin=[axis_xy[0], axis_xy[1], zc],
                                 plane_normal=[0, 0, 1])
        if sec is None or len(sec.vertices) < 6:
            continue
        p = sec.vertices[:, :2]
        d = 2 * taubin(p[:, 0], p[:, 1])
        # mantém a secção mais baixa plausível (tronco, não ramos): a menor.
        if best is None or d < best:
            best = d
    return best if best else float("nan")


def run(tree, dist, az, gt):
    """tree = (mesh, face_is_trunk, axis_xy) pré-carregado uma vez por árvore."""
    mesh, fit, axis_xy = tree
    xyz, label = gc.raycast(mesh, fit, axis_xy, dist, az, with_ground=True)
    xyzl = f"{HERE}/out/scene.xyzl"
    with open(xyzl, "w") as f:
        for p, l in zip(xyz, label):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {l}\n")
    r = subprocess.run([PROBE, xyzl, f"{gt:.4f}"], capture_output=True, text=True)
    return json.loads(r.stdout.strip().splitlines()[-1])


def main():
    print(f"{'tree':>4} {'GTd':>5} | {'dist':>4} {'az':>4} | {'reject':>10} "
          f"{'dbh':>5} {'err':>6} {'band':>4} {'canopy':>6}")
    worst_canopy = 0
    radii = {t: [] for t in range(1, 7)}
    n_total = n_accept = n_clean = 0
    rows = []
    for tid in range(1, 7):
        gt = gt_diameter(tid)
        mesh, fit, axis_xy, _ = gc.load_tree_visual(tid)  # carrega 1× por árvore
        tree = (mesh, fit, axis_xy)
        for dist in DISTS:
            for az in AZIMUTHS:
                j = run(tree, dist, az, gt)
                n_total += 1
                canopy = j["band_canopy"]
                worst_canopy = max(worst_canopy, canopy)
                if canopy == 0:
                    n_clean += 1
                if j["reject"] == "Accepted":
                    n_accept += 1
                    radii[tid].append(j["radius"])
                flag = "" if canopy == 0 else "  <<< COPA"
                print(f"{tid:>4} {gt:>5.2f} | {dist:>4.0f} {az:>4.0f} | "
                      f"{j['reject']:>10} {j['dbh']:>5.2f} {j['err']:>+6.2f} "
                      f"{j['n_band']:>4} {canopy:>6}{flag}")
                rows.append((tid, dist, az, gt, j))
    print("\n=== RESUMO ===")
    print(f"cenas: {n_total} | aceites: {n_accept} | SEM copa na banda: {n_clean}/{n_total}"
          f" | pior contaminação: {worst_canopy} pontos")
    for tid in range(1, 7):
        rs = radii[tid]
        if len(rs) >= 2:
            cv = 100 * np.std(rs) / np.mean(rs)
            print(f"  Tree{tid}: raio médio={np.mean(rs):.3f} CV={cv:.1f}% (n={len(rs)})")
    with open(f"{HERE}/out/results.json", "w") as f:
        json.dump([dict(tree=int(t), dist=float(d), az=float(a), gt=float(g), **j)
                   for (t, d, a, g, j) in rows], f)
    print(f"\nresultados -> {HERE}/out/results.json")
    return rows


if __name__ == "__main__":
    main()
