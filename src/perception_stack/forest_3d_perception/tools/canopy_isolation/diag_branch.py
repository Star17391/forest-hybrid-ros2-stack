"""
Diagnóstico FOCADO (poucas amostras): os pontos medidos (azuis) para o DBH são
mesmo do FUSTE vertical, ou entram RAMOS adjacentes / folhas?

Refina o ground-truth: dentro da geometria de tronco (casca), separa o FUSTE
vertical dos RAMOS, fatia a fatia — um ponto é fuste se estiver perto do centro
local da coluna; senão é ramo. Depois marca, na banda azul medida, quantos pontos
são fuste / ramo / folha.

Não corre a bateria. Gera 1 figura com zoom na zona do peito por amostra.
"""
import json
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, ".")
import gen_cloud as gc

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = f"{HERE}/run_probe.sh"
GT = {1: 0.48, 2: 0.50, 3: 0.41, 4: 0.51, 5: 0.24, 6: 0.55}


def label_fuste_vs_branch(xyz, label, axis_xy):
    """Para os pontos de TRONCO (label 0), decide fuste (0) vs ramo (3) por fatia.

    Em cada fatia de 0.2 m, o centro local = mediana XY dos pontos de tronco; o raio
    do fuste = mediana das distâncias a esse centro. Pontos além de 2×esse raio +0.12 m
    (ou simplesmente longe do eixo) são RAMOS. Devolve novo array de labels:
    0=fuste, 1=copa(folha), 2=solo, 3=ramo.
    """
    out = label.copy()
    z = xyz[:, 2]
    trunk = label == 0
    if trunk.sum() < 3:
        return out
    zt = z[trunk]
    zmin, zmax = zt.min(), zt.max()
    idx_trunk = np.where(trunk)[0]
    h = 0.2
    nb = max(1, int(np.ceil((zmax - zmin) / h)))
    for b in range(nb):
        zlo, zhi = zmin + b * h, zmin + (b + 1) * h
        sel = idx_trunk[(z[idx_trunk] >= zlo) & (z[idx_trunk] < zhi)]
        if len(sel) < 2:
            continue
        cx = np.median(xyz[sel, 0])
        cy = np.median(xyz[sel, 1])
        r = np.hypot(xyz[sel, 0] - cx, xyz[sel, 1] - cy)
        rmed = np.median(r)
        lim = max(2.0 * rmed + 0.12, 0.25)  # além disto = ramo a sair da coluna
        branch = sel[r > lim]
        out[branch] = 3
    return out


def diag(tree_obj, tid, dist, az, ax):
    mesh, fit, axis_xy = tree_obj
    xyz, label = gc.raycast(mesh, fit, axis_xy, dist, az, with_ground=True)
    base = f"{HERE}/out/diag_t{tid}_d{dist:g}_a{az:g}"
    with open(base + ".xyzl", "w") as f:
        for p, l in zip(xyz, label):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {l}\n")
    r = subprocess.run([PROBE, base + ".xyzl", f"{GT[tid]:.4f}", base + ".band"],
                       capture_output=True, text=True)
    j = json.loads(r.stdout.strip().splitlines()[-1])
    band = (np.loadtxt(base + ".band").reshape(-1, 4)
            if os.path.getsize(base + ".band") else np.zeros((0, 4)))

    # GT refinado: fuste vs ramo
    lab2 = label_fuste_vs_branch(xyz, label, axis_xy)
    rad = np.hypot(xyz[:, 0] - axis_xy[0], xyz[:, 1] - axis_xy[1])

    COL = {0: "#1a9850", 1: "#f0a000", 2: "#cccccc", 3: "#d73027"}
    NAME = {0: "fuste vertical (GT)", 1: "folha (GT)", 2: "solo", 3: "RAMO (GT)"}
    for lab in [2, 1, 3, 0]:
        s = lab2 == lab
        ax.scatter(rad[s], xyz[s, 2], s=9, c=COL[lab], alpha=0.55, label=NAME[lab])

    # classificar cada ponto da banda azul: fuste / ramo / folha
    n_fuste = n_ramo = n_folha = 0
    if len(band):
        # mapear cada ponto da banda ao seu label refinado (por coincidência XYZ)
        key = {(round(x, 4), round(y, 4), round(z, 4)): lab2[i]
               for i, (x, y, z) in enumerate(xyz)}
        blabs = [key.get((round(p[0], 4), round(p[1], 4), round(p[2], 4)), 0) for p in band]
        blabs = np.array(blabs)
        n_fuste = int((blabs == 0).sum())
        n_ramo = int((blabs == 3).sum())
        n_folha = int((blabs == 1).sum())
        br = np.hypot(band[:, 0] - axis_xy[0], band[:, 1] - axis_xy[1])
        ax.scatter(br, band[:, 2], s=40, facecolors="#08306b", edgecolors="w",
                   linewidths=0.6, zorder=5, label="MEDIDO p/ DBH (azul)")

    ax.set_xlim(0, 0.9)
    ax.set_ylim(0, 3.0)
    ax.set_xlabel("dist. horizontal ao eixo [m]")
    ax.set_ylabel("z [m]")
    bad = n_ramo + n_folha
    col = "green" if bad == 0 else "crimson"
    ax.set_title(f"Tree{tid} d={dist:g} az={az:g}  | medidos: fuste={n_fuste} "
                 f"RAMO={n_ramo} folha={n_folha}", fontsize=9, color=col)
    ax.legend(fontsize=6.5, loc="upper right")
    return n_fuste, n_ramo, n_folha


def main():
    samples = [(1, 4, 270), (2, 4, 270), (3, 4, 270)]
    if len(sys.argv) > 1:  # ex: python diag_branch.py 1:4:0 5:3:90
        samples = [tuple(int(x) for x in a.split(":")) for a in sys.argv[1:]]
    fig, axes = plt.subplots(1, len(samples), figsize=(5.2 * len(samples), 5))
    if len(samples) == 1:
        axes = [axes]
    cache = {}
    print(f"{'amostra':>16} | fuste  RAMO  folha")
    for ax, (tid, d, a) in zip(axes, samples):
        if tid not in cache:
            m, fi, ax_xy, _ = gc.load_tree_visual(tid)
            cache[tid] = (m, fi, ax_xy)
        nf, nr, no = diag(cache[tid], tid, d, a, ax)
        print(f"  Tree{tid} d{d} az{a:>3} | {nf:>5} {nr:>5} {no:>5}"
              + ("   <<< RAMO/FOLHA NA MEDIÇÃO" if (nr + no) else ""))
    fig.suptitle("Diagnóstico: os pontos AZUIS (medidos p/ DBH) são fuste, ramo ou folha?",
                 fontsize=11)
    fig.tight_layout()
    out = f"{HERE}/out/diag_branch.png"
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("escrito", out)


if __name__ == "__main__":
    main()
