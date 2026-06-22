"""
PROTÓTIPO (Python) — comparar variantes do novo critério de tronco.

Critério acordado:
  - Tronco = ESTRUTURA CONTÍNUA que pode CURVAR/INCLINAR (a Tree1 é curva ao longo
    do z) — continuidade da LINHA-DE-CENTRO, NÃO retidão. Sem limite de altura/largura.
  - Por fatia, o tronco é o aglomerado espacial que CONTINUA a linha-de-centro vinda
    de baixo; os RAMOS são lobos laterais separados -> removidos.
  - DBH ao longo de TODO o fuste, por ajuste de círculo (recupera o eixo real).

Duas variantes (o utilizador escolhe pela comparação):
  (B) seguir a coluna toda, limpando os ramos em cada fatia.
  (C) idem, mas PARAR quando a estrutura do tronco acaba (sem aglomerado contínuo).

Não usa ground-truth no algoritmo; o GT só colore a figura.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, ".")
import gen_cloud as gc

HERE = os.path.dirname(os.path.abspath(__file__))
GT = {1: 0.48, 2: 0.50, 3: 0.41, 4: 0.51, 5: 0.24, 6: 0.55}
SLICE_H = 0.15
LINK_D = 0.18   # distância de ligação p/ aglomerar pontos da MESMA fatia [m]


# ---------------- ajuste de círculo (Taubin + refinamento de Landau) -------------
def fit_circle(xs, ys):
    x = np.asarray(xs, float); y = np.asarray(ys, float)
    if len(x) < 3:
        return float(np.mean(x)) if len(x) else 0.0, float(np.mean(y)) if len(y) else 0.0, 0.0
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
    if abs(det) < 1e-12:
        return mx, my, float(np.median(np.hypot(u, v)))
    cx = (Mxz * (Myy - xx) - Myz * Mxy) / det / 2 + mx
    cy = (Myz * (Mxx - xx) - Mxz * Mxy) / det / 2 + my
    # refinamento geométrico de Landau (corrige viés de arco parcial)
    for _ in range(12):
        d = np.hypot(x - cx, y - cy)
        good = d > 1e-9
        if good.sum() == 0:
            break
        mr = d[good].mean()
        ncx = x[good].mean() - mr * ((x[good] - cx) / d[good]).mean()
        ncy = y[good].mean() - mr * ((y[good] - cy) / d[good]).mean()
        if np.hypot(ncx - cx, ncy - cy) < 1e-6:
            cx, cy = ncx, ncy
            break
        cx, cy = ncx, ncy
    r = float(np.median(np.hypot(x - cx, y - cy)))
    return cx, cy, r


# ---------------- aglomeração espacial de uma fatia (componentes ligadas) --------
def slice_clusters(P, link_d=LINK_D):
    """Componentes ligadas de pontos 2D por distância. Devolve lista de arrays de idx."""
    n = len(P)
    if n == 0:
        return []
    parent = list(range(n))
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for i in range(n):
        for j in range(i + 1, n):
            if (P[i, 0] - P[j, 0]) ** 2 + (P[i, 1] - P[j, 1]) ** 2 <= link_d * link_d:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[ri] = rj
    comp = {}
    for i in range(n):
        comp.setdefault(find(i), []).append(i)
    return [np.array(v) for v in comp.values()]


# ---------------- extração do fuste (linha-de-centro contínua + limpeza) ----------
def robust_zbase(xyz):
    """z do pé do tronco, robusto a clusters dominados pela copa.

    Perto do MÍNIMO de z só existe tronco (a copa está em cima), por isso a âncora XY
    vem da banda mais baixa e o z_base é o min-z dos pontos perto dessa âncora.
    """
    z = xyz[:, 2]
    zmin = z.min()
    low = xyz[z <= zmin + 0.8][:, :2]
    if len(low) == 0:
        return zmin
    anchor = np.array([np.median(low[:, 0]), np.median(low[:, 1])])
    near = np.hypot(xyz[:, 0] - anchor[0], xyz[:, 1] - anchor[1]) < 0.5
    return float(z[near].min()) if near.any() else zmin


SUSTAIN = 3      # nº de fatias seguidas "abertas" para confirmar a base da copa
EXT_PCT = 95     # percentil do raio usado como "extensão" da fatia


def track_column(xyz, z_base):
    """Rastreia a coluna do tronco fatia a fatia (linha-de-centro que pode curvar).

    Para CADA fatia devolve um registo: z central, centro (x,y) da linha (extrapolado
    quando não há componente que continue), índices da componente FINA que continua a
    coluna, raio limpo dessa componente e a extensão (percentil EXT_PCT) de TODOS os
    pontos da fatia em relação ao centro — o sinal que dispara a base da copa.
    Não pára: vai até ao topo; quem decide o corte é find_crown_base().
    """
    z = xyz[:, 2]
    zmax = z.max()
    nb = max(1, int(np.ceil((zmax - z_base) / SLICE_H)))
    bins = [[] for _ in range(nb)]
    for i in range(len(xyz)):
        if z[i] >= z_base:
            b = min(nb - 1, int((z[i] - z_base) / SLICE_H))
            bins[b].append(i)

    # componentes (aglomerados) de cada fatia com pontos: (z, idx, centro, n)
    sl_b, comps = [], []     # comps[k] = lista de (idx_array, centro_xy, n)
    for b in range(nb):
        if not bins[b]:
            continue
        ids = np.array(bins[b])
        P = xyz[ids][:, :2]
        cs = []
        for comp in slice_clusters(P):
            cc = np.array([np.median(P[comp, 0]), np.median(P[comp, 1])])
            cs.append((ids[comp], cc, len(comp)))
        sl_b.append(b)
        comps.append(cs)
    if not comps:
        return []

    # ---- DP (Viterbi): escolhe em cada fatia o aglomerado que forma o FIO contínuo
    # mais suave desde a base. Âncora = maior aglomerado da fatia base (o tronco domina
    # junto ao solo). Custo de transição = deslocamento lateral; portão cresce com o
    # nº de fatias saltadas. Um desvio para um ramo e volta custa mais que ficar
    # vertical -> o caminho ótimo segue o tronco e ignora lobos de ramo.
    INF = 1e18
    K = len(comps)
    dp = [[INF] * len(comps[k]) for k in range(K)]
    bk = [[-1] * len(comps[k]) for k in range(K)]
    j0 = int(np.argmax([n for (_, _, n) in comps[0]]))
    dp[0][j0] = 0.0
    for k in range(1, K):
        gap = sl_b[k] - sl_b[k - 1]
        if gap > 4:                       # quebra vertical grande -> não liga (fim do fio)
            continue
        gate = min(0.30, 0.16 + 0.45 * gap)   # portão pequeno: não salta p/ a copa
        for j, (_, cj, _) in enumerate(comps[k]):
            for i, (_, ci, _) in enumerate(comps[k - 1]):
                if dp[k - 1][i] >= INF:
                    continue
                d = float(np.hypot(*(cj - ci)))
                if d <= gate and dp[k - 1][i] + d < dp[k][j]:
                    dp[k][j] = dp[k - 1][i] + d
                    bk[k][j] = i
    # fim do fio = fatia mais alta alcançável; entre empates, menor custo
    end_k = max(k for k in range(K) if min(dp[k]) < INF)
    end_j = int(np.argmin(dp[end_k]))
    chosen = {}
    k, j = end_k, end_j
    while k >= 0 and j >= 0:
        chosen[k] = j
        j = bk[k][j]
        k -= 1

    slices = []
    last_center = comps[0][j0][1].copy()
    r = max(0.03, np.median(np.hypot(
        xyz[comps[0][j0][0], 0] - last_center[0],
        xyz[comps[0][j0][0], 1] - last_center[1])))
    for k in range(K):
        zc = z_base + (sl_b[k] + 0.5) * SLICE_H
        P = np.array([c[1] for c in comps[k]])    # centros (não usado p/ ext)
        all_ids = np.concatenate([c[0] for c in comps[k]])
        if k in chosen:
            ids_k, center, _ = comps[k][chosen[k]]
            rr = np.hypot(xyz[ids_k, 0] - center[0], xyz[ids_k, 1] - center[1])
            r_clean = max(np.percentile(rr, 40), 0.02)
            last_center = center.copy()
            r = 0.6 * r + 0.4 * r_clean
            col_idx = ids_k
        else:
            center, r_clean, col_idx = last_center.copy(), r, np.array([], int)
        rad_all = np.hypot(xyz[all_ids, 0] - center[0], xyz[all_ids, 1] - center[1])
        ext = float(np.percentile(rad_all, EXT_PCT))
        slices.append(dict(z=zc, center=center, idx=col_idx, r=r_clean,
                           ext=ext, n=len(all_ids)))
    return slices


def find_crown_base(slices):
    """z da base da copa: 1ª fatia onde a extensão "abre" de forma SUSTENTADA.

    Referência do tronco = mediana do raio limpo das fatias baixas. A copa começa
    quando a extensão (todos os pontos) passa de max(0.5 m, 3×r_ref) durante pelo
    menos SUSTAIN fatias seguidas. Devolve o z dessa 1ª fatia (corte é < z_copa).
    Se nunca dispara, devolve +inf (usa a coluna toda).
    """
    if not slices:
        return float("inf")
    low = [s["r"] for s in slices[:max(3, len(slices) // 3)] if s["idx"].size]
    r_ref = np.median(low) if low else 0.15
    thr = max(0.5, 3.0 * r_ref)
    run0 = None
    for k, s in enumerate(slices):
        if s["ext"] > thr:
            if run0 is None:
                run0 = k
            if k - run0 + 1 >= SUSTAIN:
                return slices[run0]["z"]
        else:
            run0 = None
    return float("inf")


def extract(xyz, stop_at_end=True, z_base=None):
    """Isola o fuste: coluna rastreada -> corte na base da copa -> fit robusto + trim.

    stop_at_end é ignorado (compat. com chamadas antigas); o corte é sempre a base
    da copa detetada. Devolve (idx_fuste, idx_ramo, centerline, diâmetro).
    """
    if z_base is None:
        z_base = robust_zbase(xyz)
    slices = track_column(xyz, z_base)
    if not slices:
        return np.array([], int), np.array([], int), np.zeros((0, 4)), 0.0
    z_copa = find_crown_base(slices)

    # coluna candidata = componentes finas abaixo da base da copa
    column, centerline = [], []
    for s in slices:
        if s["z"] >= z_copa:
            break
        for i in s["idx"]:
            column.append((i, s["center"][0], s["center"][1]))
        if s["idx"].size:
            centerline.append((s["z"], s["center"][0], s["center"][1], s["r"]))

    # remoção pela SUPERFÍCIE usando o EIXO VERDADEIRO (circle-fit), não o centróide
    # do arco. Recentramos cada arco pela sua centróide local, ajustamos UM círculo
    # -> offset centróide→eixo (dx,dy) + raio real r_trunk. Um ramo fica para lá de
    # r_trunk a partir do eixo -> removido (todo o corpo, não só a ponta).
    fuste, branch, r_trunk = [], [], 0.0
    # tudo o que sobrou acima da copa é, por definição, não-fuste
    cut = {i for (i, _, _) in column}
    branch += [s_i for s in slices if s["z"] >= z_copa for s_i in s["idx"].tolist()]
    if len(column) >= 5:
        rel = np.array([[xyz[i, 0] - cx, xyz[i, 1] - cy] for (i, cx, cy) in column])
        dx, dy, r_trunk = fit_circle(rel[:, 0], rel[:, 1])
        for _ in range(2):
            keep = np.hypot(rel[:, 0] - dx, rel[:, 1] - dy) <= 1.35 * r_trunk + 0.03
            if keep.sum() >= 5:
                dx, dy, r_trunk = fit_circle(rel[keep, 0], rel[keep, 1])
        lim = 1.35 * r_trunk + 0.03
        for k, (i, cx, cy) in enumerate(column):
            d = np.hypot(rel[k, 0] - dx, rel[k, 1] - dy)
            (fuste if d <= lim else branch).append(i)
    else:
        fuste += [i for (i, _, _) in column]
    return np.array(fuste, int), np.array(branch, int), np.array(centerline), 2 * r_trunk


def dbh_from_fuste(xyz, idx_fuste, centerline):
    """Diâmetro por ajuste de círculo nos pontos do fuste, recentrados pela linha."""
    if len(idx_fuste) < 4 or len(centerline) == 0:
        return 0.0
    # recentrar cada ponto pelo centro da sua fatia -> alinha o arco de toda a coluna
    cl = centerline
    xs, ys = [], []
    for i in idx_fuste:
        j = np.argmin(np.abs(cl[:, 0] - xyz[i, 2]))
        xs.append(xyz[i, 0] - cl[j, 1])
        ys.append(xyz[i, 1] - cl[j, 2])
    _, _, r = fit_circle(xs, ys)
    return 2 * r


# ---------------- GT só p/ colorir (fuste vs ramo) ----------
def gt_split(xyz, label, axis_xy):
    out = label.copy()
    z = xyz[:, 2]
    tr = np.where(label == 0)[0]
    if len(tr) < 3:
        return out
    nb = max(1, int(np.ceil((z[tr].max() - z[tr].min()) / 0.2)))
    for b in range(nb):
        zlo = z[tr].min() + b * 0.2
        sel = tr[(z[tr] >= zlo) & (z[tr] < zlo + 0.2)]
        if len(sel) < 2:
            continue
        c = np.array([np.median(xyz[sel, 0]), np.median(xyz[sel, 1])])
        rr = np.hypot(xyz[sel, 0] - c[0], xyz[sel, 1] - c[1])
        out[sel[rr > max(2.0 * np.median(rr) + 0.12, 0.25)]] = 3
    return out


def panel(ax, xyz, label, axis_xy, tid, d, a):
    f, br, cl, dbh = extract(xyz)
    z_copa = find_crown_base(track_column(xyz, robust_zbase(xyz)))
    lab2 = gt_split(xyz, label, axis_xy)
    rad = np.hypot(xyz[:, 0] - axis_xy[0], xyz[:, 1] - axis_xy[1])
    COL = {0: "#1a9850", 1: "#f0a000", 3: "#d73027"}
    NM = {0: "fuste(GT)", 1: "folha(GT)", 3: "ramo(GT)"}
    for lab in [1, 3, 0]:
        s = lab2 == lab
        ax.scatter(rad[s], xyz[s, 2], s=7, c=COL[lab], alpha=0.4, label=NM[lab])
    if len(f):
        ax.scatter(rad[f], xyz[f, 2], s=26, facecolors="#08306b", edgecolors="w",
                   linewidths=0.5, zorder=5, label="fuste medido")
    if len(br):
        ax.scatter(rad[br], xyz[br, 2], s=20, marker="x", c="k", alpha=0.5, zorder=6,
                   label="removido")
    if np.isfinite(z_copa):
        ax.axhline(z_copa, color="#6a3d9a", ls="--", lw=1.2, label=f"base copa z={z_copa:.2f}")
    ax.set_xlim(0, 1.0); ax.set_ylim(0, max(3.0, xyz[:, 2].max() * 0.6))
    ax.set_xlabel("dist. horiz. eixo [m]"); ax.set_ylabel("z [m]")
    nbad = int(np.sum(np.isin(f, np.where(lab2 == 3)[0]))) + \
           int(np.sum(np.isin(f, np.where(lab2 == 1)[0])))
    ax.set_title(f"Tree{tid} d{d:g} az{a:g}  Ø={dbh:.2f} (GT {GT[tid]:.2f})  "
                 f"fuste={len(f)} ramo-no-medido={nbad}", fontsize=8.5,
                 color="green" if nbad == 0 else "crimson")
    ax.legend(fontsize=5.5, loc="upper right")
    return len(f), nbad, dbh


def run(samples):
    n = len(samples)
    ncol = 3 if n > 1 else 1
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.5 * ncol, 4.8 * nrow))
    axes = np.array(axes).reshape(-1)
    cache = {}
    print(f"{'amostra':>16} | fuste  ramo-no-medido  Ø(m)   GT   |Δ|")
    tot_bad = 0
    for k, (tid, d, a) in enumerate(samples):
        if tid not in cache:
            m, fi, axy, _ = gc.load_tree_visual(tid)
            cache[tid] = (m, fi, axy)
        mesh, fit, axis_xy = cache[tid]
        xyz, label = gc.raycast(mesh, fit, axis_xy, d, a, with_ground=True)
        nz = label != 2
        nf, nb, db = panel(axes[k], xyz[nz], label[nz], axis_xy, tid, d, a)
        tot_bad += nb
        flag = "" if nb == 0 else "  <<< RAMO/FOLHA"
        print(f"  Tree{tid} d{d:g} az{a:>3g} | {nf:>4} {nb:>10}      {db:.2f}  {GT[tid]:.2f}  "
              f"{abs(db - GT[tid]):.2f}{flag}")
    for k in range(n, len(axes)):
        axes[k].axis("off")
    print(f"\n=== total ramo/folha no medido: {tot_bad} ===")
    fig.suptitle("Isolação do fuste — fuste azul, removidos (×), base da copa (-- roxo)",
                 fontsize=12)
    fig.tight_layout()
    out = f"{HERE}/out/fuste_compare.png"
    fig.savefig(out, dpi=115, bbox_inches="tight")
    print("escrito", out)


if __name__ == "__main__":
    s = [(t, 4, 270) for t in range(1, 7)]   # 6 árvores, d=4 m, az=270
    if len(sys.argv) > 1:
        s = [tuple(int(x) for x in a.split(":")) for a in sys.argv[1:]]
    run(s)
