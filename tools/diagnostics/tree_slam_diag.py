#!/usr/bin/env python3
"""Diagnóstico de atribuição do Tree-SLAM em modo GROUND.

Responde a UMA pergunta: o "solavanco" dos frames (map->odom a saltar) vem
(a) da estimação de estado a montante (odom->base_link não-suave),
(b) da perceção (deteções de tronco a piscar -> churn de associação), ou
(c) desta camada (backend a oscilar com entrada estável)?

É auto-contido: confirma que o binário novo do nó está a correr, CONDUZ o robô
sozinho (reta + rotação) para exercitar o SLAM, mede tudo a partir dos tópicos/TF,
e imprime um VEREDITO + grava JSON. Não depende de o utilizador conduzir bem.

Uso (via CLI):  forest diag tree-slam [--duration 40] [--no-drive] [--out PATH]
Uso (direto):   python3 tree_slam_diag.py --duration 40
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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "marble_hd2/base_link"
CMD_VEL_TOPIC = "/forest_gen/cmd_vel"
PERCEPTION_TOPIC = "/perception/lidar/tree_landmarks"
ODOM_TOPIC = "/state/odometry"
SLAM_STATUS_TOPIC = "/slam/status"
TREE_MAP_TOPIC = "/slam/tree_map"
NODE_NAME = "/tree_slam_node"

# Limiares de decisão (tunáveis por flag). Pensados para sim a ~0.3 m/s.
JUMP_XY_M = 0.05        # passo de TF entre amostras consecutivas considerado "salto"
JUMP_YAW_DEG = 3.0
FLICKER_COV = 0.25      # coef. de variação da contagem de troncos por scan
NN_JITTER_M = 0.20      # jitter médio de posição por tronco entre scans consecutivos


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def finite_transform(t) -> bool:
    tr, rot = t.transform.translation, t.transform.rotation
    vals = [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
    if not all(math.isfinite(v) for v in vals):
        return False
    n = rot.x ** 2 + rot.y ** 2 + rot.z ** 2 + rot.w ** 2
    return n > 1e-6


def pct(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    k = min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))
    return s[k]


def _perception_note(perc, slam) -> str:
    flick = (perc["count_cov"] > 0.25 or perc["nn_jitter_p95_mm"] > 200 or
             (perc["trees_max"] - perc["trees_min"]) > 3)
    if flick:
        return (f"Perceção a piscar (REAL, secundário): trees[{perc['trees_min']}..{perc['trees_max']}] "
                f"cov={perc['count_cov']} nn_jitter_p95={perc['nn_jitter_p95_mm']}mm, "
                f"média {perc['trees_mean']} troncos/scan -> backend sub-constrangido, "
                f"tracks a oscilar (decreases={slam['tracks_decreases']}). "
                "Atacar a estabilidade do stem-band clustering DEPOIS de resolver o duplo publisher.")
    return "Perceção estável nesta janela."


class TreeSlamDiag(Node):
    def __init__(self, args) -> None:
        super().__init__("tree_slam_diag")
        # Alinhar o relógio do nó com o tempo de sim (stamps das mensagens).
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args

        # Séries temporais de TF por par de frames: lista de (stamp, x, y, yaw).
        self.tf_series = defaultdict(list)
        self.tf_nan = defaultdict(int)

        # Perceção.
        self.perc_counts = []          # nº de troncos por mensagem
        self.perc_latency = []         # idade (now - stamp) por mensagem [s]
        self.perc_nn_jitter = []       # jitter de posição NN entre frames [m]
        self.perc_dbh_jitter = []      # jitter de DBH NN entre frames [m]
        self._perc_prev = None         # lista de (x,y,dbh) do frame anterior
        self.perc_msgs = 0

        # Odometria de estado.
        self.odom_speeds = []          # |v| linear [m/s]
        self.odom_yawrates = []        # |w| [rad/s]
        self.odom_msgs = 0

        # SLAM status / mapa.
        self.slam_status_n = []        # n_landmarks_tracked
        self.tree_map_n = []           # nº de landmarks publicados

        # QoS: tópicos normais (reliable volatile) + status (transient_local).
        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        tl = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(TFMessage, "/tf", self._on_tf, rel)
        self._sub_optional()

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._tl_qos = tl
        self.graph_info = {}  # preenchido por collect_graph() antes da captura

    def _sub_optional(self) -> None:
        """Subscreve tópicos cujas msgs dependem de forest_hybrid_msgs estar no path."""
        try:
            from forest_hybrid_msgs.msg import (
                SlamStatus,
                TrackedTreeLandmarkArray,
                TreeLandmarkArray,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"forest_hybrid_msgs indisponível ({exc}); perceção/slam não medidos. "
                "Corre via 'forest diag tree-slam' (faz source do install).")
            return
        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        tl = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TreeLandmarkArray, PERCEPTION_TOPIC, self._on_perc, rel)
        self.create_subscription(TrackedTreeLandmarkArray, TREE_MAP_TOPIC, self._on_tree_map, rel)
        self.create_subscription(SlamStatus, SLAM_STATUS_TOPIC, self._on_status, tl)
        # Odometria: nav_msgs sempre disponível.
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, rel)

    # --- callbacks ----------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in {(MAP_FRAME, ODOM_FRAME), (ODOM_FRAME, BASE_FRAME)}:
                continue
            if not finite_transform(t):
                self.tf_nan[key] += 1
                continue
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tr = t.transform.translation
            q = t.transform.rotation
            self.tf_series[key].append((stamp, tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w)))

    def _on_perc(self, msg) -> None:
        self.perc_msgs += 1
        self.perc_counts.append(len(msg.trees))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            self.perc_latency.append(self._now() - stamp)
        cur = [(tr.base.x, tr.base.y, tr.diameter) for tr in msg.trees]
        if self._perc_prev:
            for (x, y, d) in cur:
                best, best_d = None, 1e9
                for (px, py, pd) in self._perc_prev:
                    dd = math.hypot(x - px, y - py)
                    if dd < best_d:
                        best_d, best = dd, pd
                if best is not None and best_d < 1.0:  # mesmo tronco (NN < 1 m)
                    self.perc_nn_jitter.append(best_d)
                    self.perc_dbh_jitter.append(abs(d - best))
        self._perc_prev = cur

    def _on_odom(self, msg) -> None:
        self.odom_msgs += 1
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.odom_speeds.append(math.hypot(v.x, v.y))
        self.odom_yawrates.append(abs(w.z))

    def _on_status(self, msg) -> None:
        self.slam_status_n.append(int(msg.n_landmarks_tracked))

    def _on_tree_map(self, msg) -> None:
        self.tree_map_n.append(len(msg.trees))

    # --- condução automática ------------------------------------------------
    def _drive(self, lin: float, ang: float) -> None:
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.cmd_pub.publish(tw)

    def run(self) -> dict:
        dur = self.args.duration
        t0 = time.monotonic()
        phase_log = []
        # Padrão: reta -> parar -> rodar -> parar -> reta+roda, repetido.
        # (exercita tanto o caso de translação como o de rotação, onde o erro
        # da camada era maior.)
        while time.monotonic() - t0 < dur:
            el = time.monotonic() - t0
            if self.args.drive:
                frac = (el % 12.0)
                if frac < 4.0:
                    self._drive(0.3, 0.0); phase = "reta"
                elif frac < 6.0:
                    self._drive(0.0, 0.0); phase = "parado"
                elif frac < 10.0:
                    self._drive(0.0, 0.5); phase = "rotacao"
                else:
                    self._drive(0.15, 0.3); phase = "arco"
            else:
                phase = "manual"
            if not phase_log or phase_log[-1] != phase:
                phase_log.append(phase)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.args.drive:
            for _ in range(5):
                self._drive(0.0, 0.0)
                time.sleep(0.02)
        return self._analyze(phase_log)

    # --- análise + veredito -------------------------------------------------
    def _tf_steps(self, key):
        """Passos de translação/yaw entre amostras consecutivas de uma TF."""
        ser = self.tf_series.get(key, [])
        steps_xy, steps_yaw, dts = [], [], []
        for (s0), (s1) in zip(ser, ser[1:]):
            ds = s1[0] - s0[0]
            if ds <= 1e-6:
                continue
            steps_xy.append(math.hypot(s1[1] - s0[1], s1[2] - s0[2]))
            steps_yaw.append(abs(wrap(s1[3] - s0[3])))
            dts.append(ds)
        return steps_xy, steps_yaw, dts

    def _analyze(self, phase_log) -> dict:
        out = {"frames": {"map_odom": list((MAP_FRAME, ODOM_FRAME)),
                          "odom_base": list((ODOM_FRAME, BASE_FRAME))},
               "phases": phase_log}

        mo_xy, mo_yaw, mo_dt = self._tf_steps((MAP_FRAME, ODOM_FRAME))
        ob_xy, ob_yaw, ob_dt = self._tf_steps((ODOM_FRAME, BASE_FRAME))

        moving = bool(self.odom_speeds) and (pct(self.odom_speeds, 75) > 0.05 or
                                             pct(self.odom_yawrates, 75) > 0.05)

        def tf_block(xy, yaw, dt, nan):
            jumps = sum(1 for v in xy if v > self.args.jump_xy) + \
                sum(1 for v in yaw if math.degrees(v) > self.args.jump_yaw)
            secs = sum(dt) if dt else 0.0
            return {
                "samples": len(xy),
                "rate_hz": round(len(xy) / secs, 1) if secs > 0 else 0.0,
                "step_xy_p50_mm": round(1000 * statistics.median(xy)) if xy else 0,
                "step_xy_p95_mm": round(1000 * pct(xy, 95)) if xy else 0,
                "step_xy_max_mm": round(1000 * max(xy)) if xy else 0,
                "step_yaw_p95_deg": round(math.degrees(pct(yaw, 95)), 2) if yaw else 0,
                "step_yaw_max_deg": round(math.degrees(max(yaw)), 2) if yaw else 0,
                "jumps": jumps,
                "jumps_per_s": round(jumps / secs, 2) if secs > 0 else 0.0,
                "nan_count": nan,
            }

        out["map_odom"] = tf_block(mo_xy, mo_yaw, mo_dt, self.tf_nan.get((MAP_FRAME, ODOM_FRAME), 0))
        out["odom_base"] = tf_block(ob_xy, ob_yaw, ob_dt, self.tf_nan.get((ODOM_FRAME, BASE_FRAME), 0))

        # Perceção.
        cnts = self.perc_counts
        cov = (statistics.pstdev(cnts) / statistics.mean(cnts)) if cnts and statistics.mean(cnts) > 0 else 0.0
        out["perception"] = {
            "msgs": self.perc_msgs,
            "trees_mean": round(statistics.mean(cnts), 1) if cnts else 0,
            "trees_min": min(cnts) if cnts else 0,
            "trees_max": max(cnts) if cnts else 0,
            "count_cov": round(cov, 3),
            "nn_jitter_p50_mm": round(1000 * statistics.median(self.perc_nn_jitter)) if self.perc_nn_jitter else 0,
            "nn_jitter_p95_mm": round(1000 * pct(self.perc_nn_jitter, 95)) if self.perc_nn_jitter else 0,
            "dbh_jitter_p50_mm": round(1000 * statistics.median(self.perc_dbh_jitter)) if self.perc_dbh_jitter else 0,
            "latency_p50_ms": round(1000 * statistics.median(self.perc_latency)) if self.perc_latency else 0,
            "latency_p95_ms": round(1000 * pct(self.perc_latency, 95)) if self.perc_latency else 0,
        }

        # SLAM.
        status = self.slam_status_n
        decreases = sum(1 for a, b in zip(status, status[1:]) if b < a)
        out["slam"] = {
            "status_msgs": len(status),
            "tracks_min": min(status) if status else 0,
            "tracks_max": max(status) if status else 0,
            "tracks_decreases": decreases,  # quantas vezes o nº de tracks caiu (churn/death)
            "tree_map_final": self.tree_map_n[-1] if self.tree_map_n else 0,
            "tree_map_max": max(self.tree_map_n) if self.tree_map_n else 0,
        }
        out["robot_moved"] = moving
        out["odom_speed_p50_mps"] = round(statistics.median(self.odom_speeds), 3) if self.odom_speeds else 0
        out["graph"] = self.graph_info

        # Sinal de DUPLO PUBLISHER de map->odom: fração de amostras ~identidade
        # (map==odom) misturada com amostras longe -> /tf a alternar entre fontes.
        mo_ser = self.tf_series.get((MAP_FRAME, ODOM_FRAME), [])
        near_id = sum(1 for s in mo_ser if math.hypot(s[1], s[2]) < 0.05 and abs(wrap(s[3])) < 0.05)
        far = sum(1 for s in mo_ser if math.hypot(s[1], s[2]) > 0.20 or abs(wrap(s[3])) > 0.20)
        out["map_odom_dual"] = {
            "near_identity_frac": round(near_id / len(mo_ser), 2) if mo_ser else 0.0,
            "far_frac": round(far / len(mo_ser), 2) if mo_ser else 0.0,
            "rate_vs_expected": round(out["map_odom"]["rate_hz"] / 10.0, 1),  # tree_slam publica ~10Hz
        }

        out["verdict"] = self._verdict(out, moving)
        return out

    def _verdict(self, o, moving) -> dict:
        reasons = []
        layer = None

        ob, mo, perc, slam = o["odom_base"], o["map_odom"], o["perception"], o["slam"]
        g, dual = o.get("graph", {}), o["map_odom_dual"]

        # 0a. DUPLO PUBLISHER de map->odom (regra de ouro) — verificado ANTES de
        # tudo, porque um /tf a alternar entre fontes finge "instabilidade" de
        # qualquer camada a jusante e invalida os outros sinais.
        competitors = g.get("map_odom_competitors", [])
        # Gatilho COMPORTAMENTAL (não a config): só há duplo publisher ATIVO se o
        # /tf mostrar taxa anómala e/ou alternância identidade<->correção. NOTA: a
        # autoridade mantém sempre um broadcaster de /tf (usa-o no ar) e pode ter
        # ground_mode="identity" mas estar a CEDER no solo (auto-yield via
        # owns_map_to_odom) — por isso a config sozinha NÃO é prova.
        rate_anomaly = mo["rate_hz"] > 15.0  # >> 10 Hz do tree_slam => +1 publisher ativo
        bimodal = dual["near_identity_frac"] > 0.1 and dual["far_frac"] > 0.1
        config_risk = (g.get("authority_ground_mode") == "identity")
        if rate_anomaly or bimodal:
            return {
                "layer": "DUPLO PUBLISHER de map->odom (regra de ouro violada)",
                "moving": moving,
                "reasons": [
                    f"map->odom a {mo['rate_hz']}Hz (esperado ~10Hz do tree_slam) "
                    f"=> {dual['rate_vs_expected']}x publishers ATIVOS.",
                    f"amostras ~identidade={dual['near_identity_frac']} + longe={dual['far_frac']} "
                    "=> /tf alterna entre identidade e a correção do SLAM (o 'solavanco').",
                    f"autoridade ground_mode='{g.get('authority_ground_mode')}'; "
                    f"publishers de /tf: {competitors or 'n/d'}.",
                    "FIX: a autoridade deve ceder ao Tree-SLAM no solo (auto-yield via "
                    "owns_map_to_odom) — ou ground_mode='silent'. A perceção a piscar "
                    "é um problema REAL mas SEPARADO (ver abaixo).",
                ],
                "secondary": _perception_note(perc, slam),
            }
        if config_risk:
            reasons.append(
                f"NOTA: autoridade com ground_mode='identity' mas SEM duplo publisher "
                f"ativo (map->odom @{mo['rate_hz']}Hz, near_id={dual['near_identity_frac']}) "
                "=> auto-yield a funcionar; ground_mode é só um risco latente de config.")

        # 0b. Dados suficientes?
        if not moving:
            return {"layer": "INCONCLUSIVO",
                    "summary": "Robô não se mexeu (sem cmd_vel a chegar ou preso). "
                               "Corre com --drive (default) ou verifica /forest_gen/cmd_vel.",
                    "reasons": ["odom_speed ~0"]}
        if ob["nan_count"] > 0:
            return {"layer": "MONTANTE (EKF)",
                    "summary": "odom->base_link tem transforms NaN/inválidas — EKF a publicar lixo.",
                    "reasons": [f"odom_base nan={ob['nan_count']}"]}

        # 1. odom->base_link salta? -> camada de estimação de estado.
        odom_jumpy = ob["jumps_per_s"] > 0.5 or ob["step_xy_max_mm"] > 150 or ob["step_yaw_max_deg"] > 10
        # 2. perceção a piscar? -> association churn.
        perc_flicker = (perc["count_cov"] > self.args.flicker_cov or
                        perc["nn_jitter_p95_mm"] > self.args.nn_jitter * 1000 or
                        (perc["trees_max"] - perc["trees_min"]) > max(3, 0.5 * max(1, perc["trees_mean"])))
        # 3. map->odom salta? (o sintoma)
        mo_jumpy = mo["jumps_per_s"] > 0.5 or mo["step_xy_p95_mm"] > 60 or mo["step_yaw_p95_deg"] > 3.0
        latency_high = perc["latency_p95_ms"] > 120

        if odom_jumpy:
            layer = "MONTANTE (forest_state_estimation / EKF)"
            reasons.append(
                f"odom->base_link instável: max_xy={ob['step_xy_max_mm']}mm "
                f"max_yaw={ob['step_yaw_max_deg']}deg jumps/s={ob['jumps_per_s']}. "
                "Esta camada não corrige odom aos saltos — arranjar a montante primeiro.")
        elif perc_flicker and mo_jumpy:
            layer = "PERCEÇÃO (forest_3d_perception)"
            reasons.append(
                f"odom suave mas perceção a piscar: count_cov={perc['count_cov']} "
                f"trees[{perc['trees_min']}..{perc['trees_max']}] nn_jitter_p95={perc['nn_jitter_p95_mm']}mm "
                f"-> churn de associação (tracks_decreases={slam['tracks_decreases']}) -> map->odom salta.")
        elif mo_jumpy:
            layer = "ESTA CAMADA (forest_tree_slam / backend)"
            reasons.append(
                f"odom suave e perceção estável, mas map->odom salta: "
                f"p95_xy={mo['step_xy_p95_mm']}mm p95_yaw={mo['step_yaw_p95_deg']}deg "
                f"jumps/s={mo['jumps_per_s']} -> backend a oscilar (grafo mal condicionado / "
                "optimize por scan). Investigar keyframe wobble com diagnostics:=true.")
        else:
            layer = "ESTÁVEL (não reproduzido)"
            reasons.append(
                "Nenhuma camada mostrou instabilidade significativa nesta janela. "
                "Se vês solavanco no RViz, aumenta --duration ou confirma que estás a olhar para 'map'.")

        if latency_high and layer.startswith(("ESTA CAMADA", "PERCE")):
            reasons.append(
                f"NOTA: latência perceção alta (p95={perc['latency_p95_ms']}ms) — "
                "a sincronização odom/scan (Causa nº2) ainda contribui.")

        return {"layer": layer, "moving": moving, "reasons": reasons}


def confirm_new_binary(node) -> dict:
    """Confirma que o tree_slam_node novo está vivo via o param 'diagnostics'."""
    from rcl_interfaces.srv import ListParameters
    cli = node.create_client(ListParameters, f"{NODE_NAME}/list_parameters")
    info = {"node_up": False, "has_diagnostics_param": False}
    if cli.wait_for_service(timeout_sec=3.0):
        info["node_up"] = True
        req = ListParameters.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        if fut.result() is not None:
            names = list(fut.result().result.names)
            info["has_diagnostics_param"] = "diagnostics" in names
    return info


def get_string_param(node, target_node: str, param: str) -> str | None:
    """Lê um parâmetro string de outro nó (ou None se indisponível)."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f"{target_node}/get_parameters")
    if not cli.wait_for_service(timeout_sec=2.0):
        return None
    req = GetParameters.Request()
    req.names = [param]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    res = fut.result()
    if res is None or not res.values:
        return None
    v = res.values[0]
    return v.string_value if v.type == 4 else f"<type {v.type}>"


def list_tf_publishers(node) -> list:
    """Nomes dos nós que publicam em /tf (revela publishers concorrentes)."""
    try:
        eps = node.get_publishers_info_by_topic("/tf")
    except Exception:  # noqa: BLE001
        return []
    out = []
    for e in eps:
        ns = e.node_namespace if e.node_namespace != "/" else ""
        out.append(f"{ns}/{e.node_name}".replace("//", "/"))
    return sorted(set(out))


def collect_graph(node) -> dict:
    """Introspeção do grafo: publishers de /tf + ground_mode da autoridade.

    Causa candidata #0 do solavanco: DOIS publishers de map->odom em simultâneo
    (regra de ouro violada). Sinais: >1 publisher dinâmico de /tf entre
    {tree_slam_node, map_odom_authority_node, slam_toolbox}, e/ou a autoridade
    em ground_mode='identity' enquanto o Tree-SLAM também publica.
    """
    # Deixa o grafo estabilizar.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    pubs = list_tf_publishers(node)
    authority_mode = get_string_param(node, "/map_odom_authority_node", "ground_mode")
    tree_slam_pub = any("tree_slam" in p for p in pubs)
    authority_pub = any("map_odom_authority" in p for p in pubs)
    slamtb_pub = any("slam_toolbox" in p or "async_slam" in p for p in pubs)
    competitors = [p for p in pubs if any(
        k in p for k in ("tree_slam", "map_odom_authority", "slam_toolbox", "async_slam"))]
    return {
        "tf_publishers": pubs,
        "map_odom_competitors": competitors,
        "authority_ground_mode": authority_mode,
        "tree_slam_publishes_tf": tree_slam_pub,
        "authority_publishes_tf": authority_pub,
        "slam_toolbox_publishes_tf": slamtb_pub,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Diagnóstico de atribuição do Tree-SLAM (GROUND).")
    ap.add_argument("--duration", type=float, default=40.0, help="Janela de captura [s].")
    ap.add_argument("--drive", dest="drive", action="store_true", default=True,
                    help="Conduzir o robô automaticamente (default).")
    ap.add_argument("--no-drive", dest="drive", action="store_false",
                    help="Não conduzir (o utilizador conduz à mão).")
    ap.add_argument("--out", default="/tmp/tree_slam_diag.json", help="Caminho do JSON de saída.")
    ap.add_argument("--jump-xy", type=float, default=JUMP_XY_M, help="Limiar de salto XY [m].")
    ap.add_argument("--jump-yaw", type=float, default=JUMP_YAW_DEG, help="Limiar de salto yaw [deg].")
    ap.add_argument("--flicker-cov", type=float, default=FLICKER_COV)
    ap.add_argument("--nn-jitter", type=float, default=NN_JITTER_M)
    args = ap.parse_args()

    rclpy.init()
    node = TreeSlamDiag(args)

    print("=" * 72)
    print("  TREE-SLAM DIAG — atribuição do solavanco (map->odom)")
    print("=" * 72)
    binfo = confirm_new_binary(node)
    if not binfo["node_up"]:
        print("  [!] /tree_slam_node não responde — a sim/SLAM está a correr? (forest up sim-tree-slam)")
    else:
        tag = "SIM (binário novo)" if binfo["has_diagnostics_param"] else "NÃO (binário ANTIGO — rebuild!)"
        print(f"  Nó vivo: sim | binário com correções (param 'diagnostics'): {tag}")
    node.graph_info = collect_graph(node)
    comp = node.graph_info.get("map_odom_competitors", [])
    print(f"  Publishers de /tf concorrentes em map->odom: {comp or 'n/d'}")
    print(f"  Autoridade ground_mode: {node.graph_info.get('authority_ground_mode')}")
    print(f"  A capturar {args.duration:.0f}s "
          f"({'a conduzir o robô automaticamente' if args.drive else 'condução manual'})…")

    result = node.run()
    result["binary_check"] = binfo

    # --- relatório humano ---
    def sec(t):
        print(f"\n── {t} " + "─" * (68 - len(t)))
    sec("TF map->odom  (correção do SLAM = ESTA CAMADA)")
    print("   ", result["map_odom"])
    sec("TF odom->base_link  (EKF = MONTANTE; tem de ser SUAVE)")
    print("   ", result["odom_base"])
    sec("Perceção  (/perception/lidar/tree_landmarks)")
    print("   ", result["perception"])
    sec("SLAM  (/slam/status, /slam/tree_map)")
    print("   ", result["slam"])

    v = result["verdict"]
    print("\n" + "=" * 72)
    print(f"  VEREDITO -> {v['layer']}")
    for r in v.get("reasons", []):
        print(f"    • {r}")
    if "summary" in v:
        print(f"    {v['summary']}")
    if v.get("secondary"):
        print(f"  SECUNDÁRIO: {v['secondary']}")
    print("=" * 72)

    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"  JSON: {args.out}\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
