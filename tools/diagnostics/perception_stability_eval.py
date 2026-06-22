#!/usr/bin/env python3
"""Atribuição da INSTABILIDADE da perceção de troncos (Agente 2).

Responde a UMA pergunta de forma autónoma: o flicker (troncos a piscar) e o
jitter de posição das deteções em /perception/lidar/tree_landmarks vêm de
  A) o cilindro a alternar entre fit ACEITE e FALLBACK (a base muda de definição),
  B) o classificador a comutar TRUNK<->não-TRUNK na fronteira (sem histerese),
  C) o region-growing a perder a SEMENTE-no-solo do tronco neste frame (raiz a
     montante: sem semente, a região não cresce -> o tronco desaparece), ou
  D) recall limitado por ALCANCE/densidade (poucos troncos por scan, de partida)?
Também confirma E) se base_covariance/diameter_stddev já vêm preenchidos.

Não depende de o utilizador conduzir: CONDUZ o robô sozinho (reta+rotação+arco) e
mede tudo a partir de dois tópicos:
  - /perception/lidar/tree_landmarks               (TreeLandmarkArray, a SAÍDA)
  - /perception/lidar3d/experimental/debug_stats   (String JSON, o FUNIL interno)

O funil dá a cadeia por-frame
  n_voxel -> n_non_ground -> n_working -> n_seeds -> n_clusters
          -> n_trunk_classified -> n_trunk_accept (+ rejeições)
A causa dominante é a ETAPA cuja taxa de passagem é, ao mesmo tempo, mais baixa e
mais instável (maior CV), confirmada pela correlação seeds<->accepts (Causa C) e
pela fração de fallback (Causa A).

Uso (via CLI):  forest diag perception-stability [--duration 40] [--no-drive]
Uso (direto):   python3 perception_stability_eval.py --duration 40
Requer a sim a correr:  forest up sim-tree-slam -d --world forest_gentle_trees_rocks
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import defaultdict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CMD_VEL_TOPIC = "/forest_gen/cmd_vel"
PERCEPTION_TOPIC = "/perception/lidar/tree_landmarks"
DEBUG_STATS_TOPIC = "/perception/lidar3d/experimental/debug_stats"
EXP_NODE = "/lidar3d_experimental_node"

# Limiares de decisão (tunáveis por flag).
FLICKER_COV = 0.15        # CV da contagem de troncos abaixo do qual está estável
PASS_RATE_CV_HI = 0.20    # CV da taxa de passagem de uma etapa => etapa instável
NN_MATCH_M = 1.0          # NN entre frames considerado o mesmo tronco
FALLBACK_RATIO = 0.15     # diameter_stddev/diameter == 0.30*r/(2*r) = 0.15 (assinatura)
FALLBACK_TOL = 0.01


def cv(xs) -> float:
    xs = [float(x) for x in xs]
    if len(xs) < 2:
        return 0.0
    m = statistics.mean(xs)
    return statistics.pstdev(xs) / m if m > 1e-9 else 0.0


def mean(xs) -> float:
    return statistics.mean(xs) if xs else 0.0


def pct(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    k = min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))
    return s[k]


def pearson(xs, ys) -> float:
    n = min(len(xs), len(ys))
    if n < 3:
        return 0.0
    xs, ys = xs[:n], ys[:n]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-9 or sy < 1e-9:
        return 0.0
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return cov / (sx * sy)


class PerceptionStabilityEval(Node):
    def __init__(self, args) -> None:
        super().__init__("perception_stability_eval")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args

        # Funil interno (uma entrada por frame processado).
        self.funnel = defaultdict(list)   # chave -> série temporal
        self.funnel_msgs = 0
        self.binary_new = False           # True se o JSON do funil tem n_seeds

        # Saída (tree_landmarks).
        self.out_counts = []
        self.out_latency = []
        self.nn_jitter = []
        self.dbh_jitter = []
        self.fallback_toggles = 0         # nº de troncos (NN) que mudaram accept<->fallback
        self.nn_pairs = 0
        self.ranges = []                  # range de cada tronco detetado
        self.basecov_nonzero = 0
        self.basecov_total = 0
        self.diamstd_nonzero = 0
        self._prev = None                 # [(x,y,dbh,is_fallback)]
        self.out_msgs = 0

        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, DEBUG_STATS_TOPIC, self._on_funnel, rel)
        self._sub_perc(rel)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def _sub_perc(self, rel) -> None:
        try:
            from forest_hybrid_msgs.msg import TreeLandmarkArray
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"forest_hybrid_msgs indisponível ({exc}); corre via 'forest diag "
                "perception-stability' (faz source do install).")
            return
        self.create_subscription(TreeLandmarkArray, PERCEPTION_TOPIC, self._on_perc, rel)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- callbacks ----------------------------------------------------------
    def _on_funnel(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(d.get("status", "")).lower() != "ok":
            return  # só frames totalmente processados entram nas estatísticas
        if "n_seeds" in d:
            self.binary_new = True
        self.funnel_msgs += 1
        for k in ("n_voxel", "n_non_ground", "n_working", "n_seeds", "n_clusters",
                  "n_trunk_classified", "n_trunk_accept", "n_trunk_reject_radius",
                  "n_trunk_reject_height", "n_trunk_reject_verticality",
                  "n_trunk_reject_points", "dbh_stability_pct"):
            if k in d:
                self.funnel[k].append(d[k])

    def _on_perc(self, msg) -> None:
        self.out_msgs += 1
        self.out_counts.append(len(msg.trees))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            self.out_latency.append(self._now() - stamp)
        cur = []
        for tr in msg.trees:
            d = tr.diameter
            sd = tr.diameter_stddev
            is_fb = d > 1e-3 and abs(sd / d - FALLBACK_RATIO) < FALLBACK_TOL
            cur.append((tr.base.x, tr.base.y, d, is_fb))
            self.ranges.append(math.hypot(tr.base.x, tr.base.y))
            self.basecov_total += 1
            cov = list(tr.base_covariance)
            if any(abs(c) > 1e-12 for c in cov):
                self.basecov_nonzero += 1
            if sd > 1e-9:
                self.diamstd_nonzero += 1
        if self._prev:
            for (x, y, d, fb) in cur:
                best, best_d, best_fb = None, 1e9, None
                for (px, py, pd, pfb) in self._prev:
                    dd = math.hypot(x - px, y - py)
                    if dd < best_d:
                        best_d, best, best_fb = dd, pd, pfb
                if best is not None and best_d < NN_MATCH_M:
                    self.nn_pairs += 1
                    self.nn_jitter.append(best_d)
                    self.dbh_jitter.append(abs(d - best))
                    if fb != best_fb:
                        self.fallback_toggles += 1
        self._prev = cur

    # --- condução automática ------------------------------------------------
    def _drive(self, lin: float, ang: float) -> None:
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.cmd_pub.publish(tw)

    def run(self) -> dict:
        dur = self.args.duration
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            el = time.monotonic() - t0
            if self.args.drive:
                frac = el % 12.0
                if frac < 4.0:
                    self._drive(0.3, 0.0)
                elif frac < 6.0:
                    self._drive(0.0, 0.0)
                elif frac < 10.0:
                    self._drive(0.0, 0.5)
                else:
                    self._drive(0.15, 0.3)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.args.drive:
            for _ in range(5):
                self._drive(0.0, 0.0)
                time.sleep(0.02)
        return self._analyze()

    # --- análise + veredito -------------------------------------------------
    def _analyze(self) -> dict:
        f = self.funnel
        out = {"funnel_msgs": self.funnel_msgs, "out_msgs": self.out_msgs,
               "binary_new": self.binary_new}

        # Saída (o sintoma).
        cnts = self.out_counts
        out["output"] = {
            "trees_mean": round(mean(cnts), 2),
            "trees_min": min(cnts) if cnts else 0,
            "trees_max": max(cnts) if cnts else 0,
            "count_cov": round(cv(cnts), 3),
            "nn_jitter_p50_mm": round(1000 * statistics.median(self.nn_jitter)) if self.nn_jitter else 0,
            "nn_jitter_p95_mm": round(1000 * pct(self.nn_jitter, 95)) if self.nn_jitter else 0,
            "dbh_jitter_p50_mm": round(1000 * statistics.median(self.dbh_jitter)) if self.dbh_jitter else 0,
            "latency_p95_ms": round(1000 * pct(self.out_latency, 95)) if self.out_latency else 0,
            "range_p50_m": round(statistics.median(self.ranges), 1) if self.ranges else 0,
            "range_p95_m": round(pct(self.ranges, 95), 1) if self.ranges else 0,
        }

        # Funil: médias + CV por etapa.
        stage_keys = ["n_voxel", "n_non_ground", "n_working", "n_seeds",
                      "n_clusters", "n_trunk_classified", "n_trunk_accept"]
        out["funnel"] = {k: {"mean": round(mean(f.get(k, [])), 1),
                             "cv": round(cv(f.get(k, [])), 3)} for k in stage_keys}

        # Taxas de passagem por frame (média + CV). Etapa instável = passagem
        # baixa E CV alta (perde troncos de forma intermitente).
        def pass_series(num_k, den_k):
            num, den = f.get(num_k, []), f.get(den_k, [])
            n = min(len(num), len(den))
            return [num[i] / den[i] for i in range(n) if den[i] > 0]

        passes = {
            "seed_of_working": pass_series("n_seeds", "n_working"),
            "cluster_of_seed": pass_series("n_clusters", "n_seeds"),
            "classify_of_cluster": pass_series("n_trunk_classified", "n_clusters"),
            "accept_of_classify": pass_series("n_trunk_accept", "n_trunk_classified"),
        }
        out["pass_rates"] = {k: {"mean": round(mean(v), 3), "cv": round(cv(v), 3)}
                             for k, v in passes.items()}

        # Sinais por causa.
        seeds = f.get("n_seeds", [])
        clusters = f.get("n_clusters", [])
        cls = f.get("n_trunk_classified", [])
        acc = f.get("n_trunk_accept", [])
        seed_accept_corr = pearson(seeds, acc)

        # Causa A: fração de fallback por frame + toggling no output.
        frac_fallback = []
        for i in range(min(len(cls), len(acc))):
            if cls[i] > 0:
                frac_fallback.append((cls[i] - acc[i]) / cls[i])
        toggle_rate = self.fallback_toggles / self.nn_pairs if self.nn_pairs else 0.0

        out["causes"] = {
            "A_cylinder": {
                "frac_fallback_mean": round(mean(frac_fallback), 3),
                "accept_of_classify_cv": out["pass_rates"]["accept_of_classify"]["cv"],
                "fallback_toggle_rate": round(toggle_rate, 3),
            },
            "B_classify": {
                "classify_of_cluster_mean": out["pass_rates"]["classify_of_cluster"]["mean"],
                "classify_of_cluster_cv": out["pass_rates"]["classify_of_cluster"]["cv"],
                "cluster_cv": round(cv(clusters), 3),
                "trunk_classified_cv": round(cv(cls), 3),
            },
            "C_region_grow": {
                "seeds_cv": round(cv(seeds), 3),
                "clusters_cv": round(cv(clusters), 3),
                "seed_accept_corr": round(seed_accept_corr, 3),
                "seeds_mean": round(mean(seeds), 1),
            },
            "D_recall_range": {
                "clusters_mean": round(mean(clusters), 1),
                "trees_mean": out["output"]["trees_mean"],
                "range_p95_m": out["output"]["range_p95_m"],
            },
            "E_uncertainty": {
                "base_cov_nonzero_frac": round(self.basecov_nonzero / self.basecov_total, 3)
                if self.basecov_total else 0.0,
                "diam_stddev_nonzero_frac": round(self.diamstd_nonzero / self.basecov_total, 3)
                if self.basecov_total else 0.0,
                "samples": self.basecov_total,
            },
        }

        out["verdict"] = self._verdict(out, passes)
        return out

    def _verdict(self, o, passes) -> dict:
        reasons = []
        oc = o["output"]
        c = o["causes"]
        stable = oc["count_cov"] < self.args.flicker_cov and oc["nn_jitter_p95_mm"] < 200

        # E é ortogonal — reporta sempre.
        e = c["E_uncertainty"]
        if e["samples"] == 0:
            e_note = "E (incerteza): SEM troncos publicados — não avaliável."
        elif e["base_cov_nonzero_frac"] > 0.95 and e["diam_stddev_nonzero_frac"] > 0.95:
            e_note = (f"E (incerteza): RESOLVIDO — base_covariance não-nula em "
                      f"{100*e['base_cov_nonzero_frac']:.0f}% e diameter_stddev em "
                      f"{100*e['diam_stddev_nonzero_frac']:.0f}% dos troncos (critério #4 fechado).")
        else:
            e_note = (f"E (incerteza): base_covariance não-nula só em "
                      f"{100*e['base_cov_nonzero_frac']:.0f}% — preenchimento incompleto.")

        if not self.binary_new:
            return {"primary": "BINÁRIO ANTIGO",
                    "summary": "O funil não expõe n_seeds — o nó a correr não tem a "
                               "instrumentação. Faz rebuild de forest_3d_perception e relança a sim.",
                    "e_note": e_note}

        if stable:
            return {"primary": "ESTÁVEL (não reproduzido)",
                    "summary": f"count_cov={oc['count_cov']} nn_jitter_p95={oc['nn_jitter_p95_mm']}mm "
                               "abaixo dos limiares nesta janela. Aumenta --duration ou confirma o mundo.",
                    "e_note": e_note}

        # Ranquear a etapa que mais contribui para o flicker: passagem baixa + CV alta.
        # Score = CV da taxa de passagem * (1 - média da taxa) — penaliza etapas que
        # perdem muitos troncos de forma intermitente.
        stage_label = {
            "seed_of_working": "C (region-grow: poucas/instáveis SEMENTES no solo)",
            "cluster_of_seed": "C (region-grow: sementes não crescem em região)",
            "classify_of_cluster": "B (classify: comuta TRUNK na fronteira)",
            "accept_of_classify": "A (cilindro: alterna accept<->fallback)",
        }
        scored = []
        for k, series in passes.items():
            if not series:
                continue
            m, v = mean(series), cv(series)
            scored.append((v * (1.0 - m), k, m, v))
        scored.sort(reverse=True)

        if not scored:
            return {"primary": "INCONCLUSIVO",
                    "summary": "Funil sem dados suficientes (nenhum frame com clusters?).",
                    "e_note": e_note}

        top_score, top_k, top_m, top_v = scored[0]
        primary = stage_label[top_k]
        # Empate: se a 2ª etapa está a <20% do score da 1ª, são co-dominantes.
        if len(scored) > 1 and scored[1][0] > 0.8 * top_score:
            primary = f"{stage_label[top_k]}  +  {stage_label[scored[1][1]]} (co-dominantes)"

        # Causa C tem assinatura adicional: correlação seeds<->accepts alta + CV(seeds) alto.
        cc = c["C_region_grow"]
        if cc["seed_accept_corr"] > 0.5 and cc["seeds_cv"] > 0.2 and not top_k.startswith("seed"):
            reasons.append(
                f"NOTA: forte correlação seeds<->accepts ({cc['seed_accept_corr']}) com "
                f"CV(seeds)={cc['seeds_cv']} — Causa C (sementes) contribui MESMO não sendo "
                "a etapa de maior atrito; sem semente o tronco nem chega a cluster.")

        # Causa D: recall de base baixo (independente do flicker).
        d = c["D_recall_range"]
        recall_note = ""
        if d["clusters_mean"] < 4:
            recall_note = (f"PARALELO (Causa D): só {d['clusters_mean']} clusters/scan "
                           f"(range_p95={d['range_p95_m']}m) — recall limitado de partida, "
                           "ortogonal ao flicker. Mexer em max_range/voxel/min_region_pts.")

        # Causa A: fallback alto.
        a = c["A_cylinder"]
        # Separar RECALL (onde se perdem troncos, em magnitude) de FLICKER (onde a
        # contagem fica instável, em CV). São perguntas diferentes.
        fr = o["funnel"]
        stage_chain = [("n_working", "n_seeds"), ("n_seeds", "n_clusters"),
                       ("n_clusters", "n_trunk_classified"),
                       ("n_trunk_classified", "n_trunk_accept")]
        drops = []
        for a_k, b_k in stage_chain:
            am, bm = fr[a_k]["mean"], fr[b_k]["mean"]
            if am > 1e-6:
                drops.append((1.0 - bm / am, f"{a_k}->{b_k}", am, bm))
        drops.sort(reverse=True)
        if drops:
            dr, dname, dam, dbm = drops[0]
            reasons.append(
                f"Fonte de RECALL (maior perda de troncos): {dname} perde {100*dr:.0f}% "
                f"({dam:.1f}->{dbm:.1f}/scan).")
        reasons.append(
            f"Fonte de FLICKER (maior instabilidade): {primary} — passagem média={top_m:.2f} CV={top_v:.2f}.")
        reasons.append(
            f"Decomposição (passagem média|CV): "
            f"seed/work={o['pass_rates']['seed_of_working']['mean']}|{o['pass_rates']['seed_of_working']['cv']}, "
            f"clus/seed={o['pass_rates']['cluster_of_seed']['mean']}|{o['pass_rates']['cluster_of_seed']['cv']}, "
            f"cls/clus={o['pass_rates']['classify_of_cluster']['mean']}|{o['pass_rates']['classify_of_cluster']['cv']}, "
            f"acc/cls={o['pass_rates']['accept_of_classify']['mean']}|{o['pass_rates']['accept_of_classify']['cv']}.")
        if a["frac_fallback_mean"] > 0.2 or a["fallback_toggle_rate"] > 0.1:
            reasons.append(
                f"Causa A presente: fallback médio={a['frac_fallback_mean']} "
                f"toggle_rate={a['fallback_toggle_rate']} (a base muda de definição "
                "accept<->fallback -> jitter da base).")

        return {"primary": primary,
                "flicker": {"count_cov": oc["count_cov"], "nn_jitter_p95_mm": oc["nn_jitter_p95_mm"]},
                "reasons": reasons,
                "recall_note": recall_note,
                "e_note": e_note,
                "ranking": [{"stage": k, "score": round(s, 3), "pass_mean": round(m, 3),
                             "pass_cv": round(v, 3)} for (s, k, m, v) in scored]}


def confirm_node(node) -> bool:
    """True se o nó experimental responde a list_parameters."""
    from rcl_interfaces.srv import ListParameters
    cli = node.create_client(ListParameters, f"{EXP_NODE}/list_parameters")
    if cli.wait_for_service(timeout_sec=3.0):
        req = ListParameters.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        return fut.result() is not None
    return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Atribuição da instabilidade da perceção de troncos (autónomo).")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--drive", dest="drive", action="store_true", default=True)
    ap.add_argument("--no-drive", dest="drive", action="store_false")
    ap.add_argument("--out", default="/tmp/perception_stability.json")
    ap.add_argument("--flicker-cov", type=float, default=FLICKER_COV)
    args = ap.parse_args()

    rclpy.init()
    node = PerceptionStabilityEval(args)

    print("=" * 72)
    print("  PERCEPTION STABILITY — atribuição do flicker/jitter de troncos")
    print("=" * 72)
    up = confirm_node(node)
    print(f"  Nó {EXP_NODE} vivo: {'sim' if up else 'NÃO (a sim está a correr?)'}")
    print(f"  A capturar {args.duration:.0f}s "
          f"({'a conduzir o robô' if args.drive else 'condução manual'})…")

    result = node.run()

    def sec(t):
        print(f"\n── {t} " + "─" * max(0, 60 - len(t)))
    sec("SAÍDA  (/perception/lidar/tree_landmarks)")
    print("   ", result["output"])
    sec("FUNIL  (média|CV por etapa)")
    for k, v in result["funnel"].items():
        print(f"    {k:22s} mean={v['mean']:>8} cv={v['cv']}")
    sec("TAXAS DE PASSAGEM  (média|CV)")
    for k, v in result["pass_rates"].items():
        print(f"    {k:22s} mean={v['mean']:>6} cv={v['cv']}")
    sec("CAUSAS")
    for k, v in result["causes"].items():
        print(f"    {k}: {v}")

    v = result["verdict"]
    print("\n" + "=" * 72)
    print(f"  VEREDITO -> {v['primary']}")
    for r in v.get("reasons", []):
        print(f"    • {r}")
    if v.get("recall_note"):
        print(f"    • {v['recall_note']}")
    if v.get("summary"):
        print(f"    {v['summary']}")
    print(f"  {v.get('e_note', '')}")
    print("=" * 72)

    with open(args.out, "w") as fh:
        json.dump(result, fh, indent=2)
    print(f"  JSON: {args.out}\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
