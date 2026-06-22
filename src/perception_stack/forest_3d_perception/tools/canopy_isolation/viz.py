"""
Visualização da isolação tronco/copa.

Para cada cena: corre o probe (perceção real), que exporta os pontos selecionados
para a banda do DBH, e desenha:
  - vista lateral (distância horizontal ao eixo  vs  Z),
  - vista de topo (X-Y), com zoom no tronco,
colorindo por GT (tronco=verde, copa=laranja, solo=cinza) e marcando a PRETO os
pontos que a perceção escolheu para medir o DBH. Objetivo visível: zero marcas
pretas sobre pontos laranja (copa).
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
COL = {0: "#1a9850", 1: "#f08000", 2: "#999999"}
NAME = {0: "tronco (GT)", 1: "copa (GT)", 2: "solo"}


def scene(tid, dist, az):
    mesh, fit, axis_xy, trunk_geom = gc.load_tree_visual(tid)
    xyz, label = gc.raycast(mesh, fit, axis_xy, dist, az, with_ground=True)
    base = f"{HERE}/out/viz_t{tid}_d{dist:g}_a{az:g}"
    with open(base + ".xyzl", "w") as f:
        for p, l in zip(xyz, label):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {l}\n")
    r = subprocess.run([PROBE, base + ".xyzl", f"{GT[tid]:.4f}", base + ".band"],
                       capture_output=True, text=True)
    j = json.loads(r.stdout.strip().splitlines()[-1])
    band = np.loadtxt(base + ".band").reshape(-1, 4) if os.path.getsize(base + ".band") else np.zeros((0, 4))
    return xyz, label, band, axis_xy, j


def draw(ax_side, ax_top, tid, dist, az):
    xyz, label, band, axis_xy, j = scene(tid, dist, az)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rad = np.hypot(x - axis_xy[0], y - axis_xy[1])
    bandset = set(map(tuple, np.round(band[:, :3], 4))) if len(band) else set()

    def is_band(px, py, pz):
        return (round(px, 4), round(py, 4), round(pz, 4)) in bandset

    # vista lateral: raio ao eixo vs z. Pontos DESCARTADOS a cores ténues; os
    # CONSIDERADOS TRONCO (medidos para o DBH) realçados a azul-escuro.
    ax_side.scatter(rad[label == 2], z[label == 2], s=4, c=COL[2], alpha=0.25,
                    label="descartado: solo")
    ax_side.scatter(rad[label == 1], z[label == 1], s=7, c=COL[1], alpha=0.55,
                    label="descartado: copa")
    s0 = label == 0
    ax_side.scatter(rad[s0], z[s0], s=7, c=COL[0], alpha=0.45,
                    label="tronco (não medido)")
    if len(band):
        br = np.hypot(band[:, 0] - axis_xy[0], band[:, 1] - axis_xy[1])
        ax_side.scatter(br, band[:, 2], s=34, marker="o", facecolors="#08306b",
                        edgecolors="w", linewidths=0.6, zorder=5,
                        label="CONSIDERADO TRONCO (medido p/ DBH)")
    ax_side.set_xlim(0, 1.4)
    ax_side.set_ylim(0, min(max(z.max() + 0.3, 3.0), 7))
    ax_side.set_xlabel("distância horizontal ao eixo do tronco [m]")
    ax_side.set_ylabel("z (altura) [m]")
    canopy = j["band_canopy"]
    ok = "OK" if canopy == 0 else f"{canopy} PONTOS DE COPA!"
    title = (f"Tree{tid}  d={dist:g}m az={az:g}°   Ø medido={j['dbh']:.2f}m "
             f"(GT {GT[tid]:.2f})   copa-na-medição: {ok}")
    ax_side.set_title(title, fontsize=9, color="green" if canopy == 0 else "crimson")
    ax_side.legend(fontsize=6.5, loc="upper right", framealpha=0.9)

    # vista de topo: zoom no tronco
    m = (label != 2) & (z < 2.0)
    ax_top.scatter(x[m & (label == 1)], y[m & (label == 1)], s=8, c=COL[1], alpha=0.5)
    ax_top.scatter(x[m & (label == 0)], y[m & (label == 0)], s=8, c=COL[0], alpha=0.5)
    if len(band):
        ax_top.scatter(band[:, 0], band[:, 1], s=34, facecolors="#08306b",
                       edgecolors="w", linewidths=0.6, zorder=5)
    ax_top.plot(axis_xy[0], axis_xy[1], "r+", ms=13, mew=2, label="eixo GT")
    ax_top.set_xlim(axis_xy[0] - 1.0, axis_xy[0] + 1.0)
    ax_top.set_ylim(axis_xy[1] - 1.0, axis_xy[1] + 1.0)
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("x [m]"); ax_top.set_ylabel("y [m]")
    ax_top.set_title("vista de topo (z<2m)", fontsize=8)
    ax_top.legend(fontsize=6, loc="upper right")
    return j


def summary_chart(fname):
    import json as _j
    with open(f"{HERE}/out/results.json") as f:
        rows = _j.load(f)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    # esquerda: contaminação de copa por cena (deve ser tudo 0)
    canopy = [r["band_canopy"] for r in rows]
    a1.bar(range(len(rows)), canopy, color=["#1a9850" if c == 0 else "#d73027" for c in canopy])
    a1.set_title(f"Pontos de COPA na medição do DBH — {len(rows)} cenas\n"
                 f"({sum(c == 0 for c in canopy)}/{len(rows)} com ZERO copa)", fontsize=10)
    a1.set_xlabel("cena (árvore × distância × azimute)"); a1.set_ylabel("nº pontos de copa")
    a1.set_ylim(0, max(1, max(canopy) + 1))
    # direita: erro do DBH por árvore (só aceites)
    data, labels = [], []
    for tid in range(1, 7):
        errs = [r["err"] for r in rows if r["tree"] == tid and r["reject"] == "Accepted"]
        if errs:
            data.append(errs); labels.append(f"T{tid}\nGT{GT[tid]:.2f}")
    a2.axhline(0, color="grey", lw=0.8)
    a2.boxplot(data, tick_labels=labels, showmeans=True)
    a2.set_title("Erro do DBH vs GT por árvore (cenas aceites) [m]", fontsize=10)
    a2.set_ylabel("Ø medido − Ø GT [m]")
    fig.tight_layout(); fig.savefig(fname, dpi=110, bbox_inches="tight")
    print("escrito", fname)


def montage(scenes, fname, suptitle):
    n = len(scenes)
    fig, axes = plt.subplots(n, 2, figsize=(11, 3.1 * n))
    if n == 1:
        axes = axes.reshape(1, 2)
    for i, (tid, d, a) in enumerate(scenes):
        draw(axes[i, 0], axes[i, 1], tid, d, a)
    fig.suptitle(suptitle, fontsize=12, y=1.0)
    fig.tight_layout()
    fig.savefig(fname, dpi=110, bbox_inches="tight")
    print("escrito", fname)


if __name__ == "__main__":
    # uma vista representativa por árvore (limpa)
    montage([(t, 4.0, 270.0) for t in range(1, 7)],
            f"{HERE}/out/isolation_per_tree.png",
            "Isolação tronco/copa — 1 vista por árvore (pretas = pontos medidos p/ DBH)")
    # caso difícil Tree5 (copa baixa) a várias distâncias
    montage([(5, 3.0, 90.0), (5, 4.0, 180.0), (5, 8.0, 180.0)],
            f"{HERE}/out/isolation_tree5_hard.png",
            "Tree5 (copa baixa) — casos difíceis")
    if os.path.exists(f"{HERE}/out/results.json"):
        summary_chart(f"{HERE}/out/isolation_summary.png")
