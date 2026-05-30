#!/usr/bin/env python3
"""Diagnóstico profundo do tilt/slope dos pontos LiDAR 3D.

Analisa:
1. TF real publicado (base_link→laser): quaternion, euler, pitch efectivo
2. Transform chain completo: map→odom→base_link→laser
3. Pontos brutos do sensor: min/max Z, média Z, slope aparente
4. Pontos após transform para base_link e map: idem
5. Timestamps: cloud stamp vs TF stamp — dessincronização
6. Causa raiz do "Could not transform from [laser] to [map]"

Pré-requisitos: sim 3D a correr (Gazebo PLAY), use_sim_time.
Uso:
  python3 tools/diagnostics/lidar3d_tilt_debug.py --duration 20
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass, field

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


def quat_to_euler(q):
    """Quaternion (x,y,z,w) → (roll, pitch, yaw) em radianos."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def extract_xyz_from_cloud(cloud_msg: PointCloud2, max_pts: int = 5000):
    """Extrai pontos (x, y, z) de um PointCloud2."""
    offsets = {}
    for f in cloud_msg.fields:
        offsets[f.name] = f.offset

    if "x" not in offsets or "y" not in offsets or "z" not in offsets:
        return np.empty((0, 3))

    step = cloud_msg.point_step
    n_total = cloud_msg.width * cloud_msg.height
    stride = max(1, n_total // max_pts)

    pts = []
    data = bytes(cloud_msg.data)
    for i in range(0, n_total, stride):
        off = i * step
        if off + step > len(data):
            break
        x = struct.unpack_from("<f", data, off + offsets["x"])[0]
        y = struct.unpack_from("<f", data, off + offsets["y"])[0]
        z = struct.unpack_from("<f", data, off + offsets["z"])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            pts.append((x, y, z))

    return np.array(pts) if pts else np.empty((0, 3))


class Lidar3dTiltDebug(Node):
    def __init__(self, duration: float):
        super().__init__(
            "lidar3d_tilt_debug",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)

        self._cloud_count = 0
        self._cloud_samples: list[dict] = []
        self._tf_base_laser_seen = False
        self._tf_base_laser_info: dict = {}
        self._tf_map_odom_info: dict = {}
        self._tf_odom_base_info: dict = {}
        self._transform_errors: list[str] = []
        self._transform_success = 0
        self._transform_fail = 0

        self.create_subscription(
            PointCloud2, "/sensors/lidar/points", self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(TFMessage, "/tf_static", self._on_static, TF_STATIC_QOS)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)

        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Tilt debug: a recolher dados por {duration:.0f}s...")

    def _on_static(self, msg: TFMessage):
        for t in msg.transforms:
            p, c = t.header.frame_id, t.child_frame_id
            q = t.transform.rotation
            tr = t.transform.translation
            if "base_link" in p and c == "laser":
                roll, pitch, yaw = quat_to_euler((q.x, q.y, q.z, q.w))
                self._tf_base_laser_seen = True
                self._tf_base_laser_info = {
                    "parent": p,
                    "child": c,
                    "translation": (tr.x, tr.y, tr.z),
                    "quat": (q.x, q.y, q.z, q.w),
                    "rpy_rad": (roll, pitch, yaw),
                    "rpy_deg": (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
                    "stamp": f"{t.header.stamp.sec}.{t.header.stamp.nanosec:09d}",
                }
            if p == "map" and c == "odom":
                self._tf_map_odom_info = {
                    "parent": p, "child": c,
                    "translation": (tr.x, tr.y, tr.z),
                }

    def _on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            p, c = t.header.frame_id, t.child_frame_id
            if p == "odom" and "base_link" in c:
                tr = t.transform.translation
                q = t.transform.rotation
                roll, pitch, yaw = quat_to_euler((q.x, q.y, q.z, q.w))
                self._tf_odom_base_info = {
                    "parent": p, "child": c,
                    "translation": (tr.x, tr.y, tr.z),
                    "rpy_deg": (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
                    "stamp": f"{t.header.stamp.sec}.{t.header.stamp.nanosec:09d}",
                }

    def _on_cloud(self, msg: PointCloud2):
        self._cloud_count += 1
        if self._cloud_count > 3 and len(self._cloud_samples) >= 5:
            return

        pts = extract_xyz_from_cloud(msg, max_pts=3000)
        if pts.shape[0] == 0:
            return

        cloud_stamp = rclpy.time.Time(
            seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec
        )

        # TF lookup: laser → map
        tf_laser_to_map_ok = False
        tf_error_msg = ""
        try:
            t = self._buf.lookup_transform(
                "map", msg.header.frame_id, cloud_stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tf_laser_to_map_ok = True
            self._transform_success += 1
        except TransformException as e:
            tf_error_msg = str(e)
            self._transform_fail += 1
            if len(self._transform_errors) < 10:
                self._transform_errors.append(tf_error_msg)

        # TF lookup: laser → base_link
        tf_to_base = None
        try:
            tf_to_base = self._buf.lookup_transform(
                "marble_hd2/base_link", msg.header.frame_id, cloud_stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            pass

        # Compute transformed points to base_link
        pts_in_base = None
        if tf_to_base is not None:
            tr = tf_to_base.transform.translation
            q = tf_to_base.transform.rotation
            # Build rotation matrix from quaternion
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
            ])
            t_vec = np.array([tr.x, tr.y, tr.z])
            pts_in_base = (R @ pts.T).T + t_vec

        # Z statistics in laser frame
        z_laser = pts[:, 2]
        x_laser = pts[:, 0]

        # Slope estimation: linear fit z = m*x + b (in laser frame, front view)
        fwd_mask = x_laser > 1.0  # only forward points
        slope_laser = None
        if fwd_mask.sum() > 50:
            from numpy.polynomial.polynomial import polyfit
            c = polyfit(x_laser[fwd_mask], z_laser[fwd_mask], 1)
            slope_laser = c[1]  # slope coefficient

        slope_base = None
        if pts_in_base is not None:
            z_base = pts_in_base[:, 2]
            x_base = pts_in_base[:, 0]
            fwd_base = x_base > 1.0
            if fwd_base.sum() > 50:
                from numpy.polynomial.polynomial import polyfit
                c = polyfit(x_base[fwd_base], z_base[fwd_base], 1)
                slope_base = c[1]

        sample = {
            "cloud_idx": self._cloud_count,
            "n_pts": pts.shape[0],
            "frame_id": msg.header.frame_id,
            "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            "laser_z_min": float(z_laser.min()),
            "laser_z_max": float(z_laser.max()),
            "laser_z_mean": float(z_laser.mean()),
            "laser_x_range": (float(x_laser.min()), float(x_laser.max())),
            "slope_laser": slope_laser,
            "tf_laser_to_map_ok": tf_laser_to_map_ok,
            "tf_error": tf_error_msg[:120] if tf_error_msg else "",
            "tf_to_base_ok": tf_to_base is not None,
        }
        if pts_in_base is not None:
            z_base = pts_in_base[:, 2]
            sample["base_z_min"] = float(z_base.min())
            sample["base_z_max"] = float(z_base.max())
            sample["base_z_mean"] = float(z_base.mean())
            sample["slope_base"] = slope_base
        self._cloud_samples.append(sample)

    def _tick(self):
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            rclpy.shutdown()

    def _report(self):
        print("\n" + "=" * 72)
        print("  DIAGNÓSTICO TILT / SLOPE — LiDAR 3D")
        print("=" * 72)

        # 1. TF base_link→laser
        print("\n── 1. TF base_link → laser (static) ──")
        if self._tf_base_laser_seen:
            info = self._tf_base_laser_info
            print(f"  Parent:      {info['parent']}")
            print(f"  Child:       {info['child']}")
            print(f"  Translation: x={info['translation'][0]:.4f}  y={info['translation'][1]:.4f}  z={info['translation'][2]:.4f}")
            print(f"  Quaternion:  x={info['quat'][0]:.6f}  y={info['quat'][1]:.6f}  z={info['quat'][2]:.6f}  w={info['quat'][3]:.6f}")
            print(f"  RPY (rad):   roll={info['rpy_rad'][0]:.6f}  pitch={info['rpy_rad'][1]:.6f}  yaw={info['rpy_rad'][2]:.6f}")
            print(f"  RPY (deg):   roll={info['rpy_deg'][0]:.2f}°  pitch={info['rpy_deg'][1]:.2f}°  yaw={info['rpy_deg'][2]:.2f}°")
            print(f"  Stamp:       {info['stamp']}")

            pitch_deg = info["rpy_deg"][1]
            if abs(pitch_deg) > 1.0:
                print(f"\n  ⚠ PITCH = {pitch_deg:.2f}° ≠ 0°")
                print(f"    Isto TILTA os pontos em RViz quando transformados para map/base_link.")
                print(f"    Positivo = sensor aponta para BAIXO (nose-down).")
                if pitch_deg > 0:
                    print(f"    → Pontos distantes ficam com Z NEGATIVO após transform (slope negativo).")
                else:
                    print(f"    → Pontos distantes ficam com Z POSITIVO após transform (slope positivo).")
        else:
            print("  ❌ NÃO RECEBIDO! A cadeia TF está incompleta.")
            print("     Causa provável: static_sensor_tf_node não arrancou ou EKF atrasado.")

        # 2. TF chain
        print("\n── 2. TF chain: map → odom → base_link → laser ──")
        if self._tf_map_odom_info:
            i = self._tf_map_odom_info
            print(f"  map → odom:       OK (t={i['translation']})")
        else:
            print("  map → odom:       ❌ NÃO VISTO")

        if self._tf_odom_base_info:
            i = self._tf_odom_base_info
            print(f"  odom → base_link: OK (rpy_deg={i['rpy_deg']}, stamp={i['stamp']})")
        else:
            print("  odom → base_link: ❌ NÃO VISTO (EKF / bootstrap offline)")

        if self._tf_base_laser_seen:
            print(f"  base_link → laser: OK (pitch={self._tf_base_laser_info['rpy_deg'][1]:.2f}°)")
        else:
            print(f"  base_link → laser: ❌ MISSING")

        # 3. Transform success/fail
        print(f"\n── 3. Transform laser → map (por cloud) ──")
        total = self._transform_success + self._transform_fail
        if total > 0:
            pct = 100 * self._transform_success / total
            print(f"  Sucesso: {self._transform_success}/{total} ({pct:.1f}%)")
        else:
            print(f"  Nenhuma cloud recebida.")

        if self._transform_errors:
            print(f"\n  Erros TF (primeiros {len(self._transform_errors)}):")
            for e in self._transform_errors[:5]:
                print(f"    → {e}")

        # 4. Point cloud analysis
        print(f"\n── 4. Análise dos pontos (amostras: {len(self._cloud_samples)}) ──")
        for s in self._cloud_samples[:3]:
            print(f"\n  Cloud #{s['cloud_idx']} ({s['n_pts']} pts, frame={s['frame_id']}, stamp={s['stamp']})")
            print(f"    [LASER frame] Z: min={s['laser_z_min']:.3f}  max={s['laser_z_max']:.3f}  mean={s['laser_z_mean']:.3f}")
            print(f"    [LASER frame] X range: {s['laser_x_range'][0]:.2f} .. {s['laser_x_range'][1]:.2f}")
            if s["slope_laser"] is not None:
                print(f"    [LASER frame] Slope (z/x, pts x>1m): {s['slope_laser']:.4f} ({math.degrees(math.atan(s['slope_laser'])):.2f}°)")
            if s.get("base_z_min") is not None:
                print(f"    [BASE frame]  Z: min={s['base_z_min']:.3f}  max={s['base_z_max']:.3f}  mean={s['base_z_mean']:.3f}")
            if s.get("slope_base") is not None:
                print(f"    [BASE frame]  Slope (z/x, pts x>1m): {s['slope_base']:.4f} ({math.degrees(math.atan(s['slope_base'])):.2f}°)")
            print(f"    TF laser→map: {'✓' if s['tf_laser_to_map_ok'] else '✗ ' + s['tf_error'][:80]}")
            print(f"    TF laser→base: {'✓' if s['tf_to_base_ok'] else '✗'}")

        # 5. Diagnóstico final
        print(f"\n── 5. DIAGNÓSTICO ──")
        issues = []

        if not self._tf_base_laser_seen:
            issues.append("CRÍTICO: TF base_link→laser não publicado. O RViz/classify não conseguem transformar.")

        if self._transform_fail > 0 and self._transform_success == 0:
            issues.append("CRÍTICO: NENHUMA transform laser→map teve sucesso.")
            issues.append("  → Cadeia TF incompleta: falta map→odom OU odom→base OU base→laser.")

        if self._tf_base_laser_seen:
            pitch = self._tf_base_laser_info["rpy_deg"][1]
            if abs(pitch) > 1.0:
                issues.append(
                    f"TILT: pitch={pitch:.2f}° no TF base→laser. "
                    f"O sensor SDF TAMBÉM tem pitch={pitch:.2f}° na <pose>."
                )
                issues.append(
                    "  O ponto-chave: o sensor Gazebo JÁ inclui o tilt na sua <pose>."
                )
                issues.append(
                    "  Ao publicar TF com O MESMO pitch, estamos a dizer ao RViz que o frame "
                    "laser está rodado — o que CORRIGE a inclinação (pontos do chão ficam planos)."
                )
                issues.append(
                    "  SE os pontos aparecem inclinados, hipóteses:"
                )
                issues.append(
                    "    a) TF está a falhar (RViz mostra frame laser sem rotação → slope visível)"
                )
                issues.append(
                    "    b) Sinal do pitch invertido (TF diz 'nariz para cima' quando sensor aponta para baixo)"
                )
                issues.append(
                    "    c) O sensor Gazebo NÃO está no frame esperado (bridge frame override)"
                )

        for s in self._cloud_samples[:3]:
            if s.get("slope_base") is not None and abs(s["slope_base"]) > 0.02:
                slope_deg = math.degrees(math.atan(s["slope_base"]))
                issues.append(
                    f"SLOPE no frame BASE: {slope_deg:.2f}° — chão NÃO está plano após TF!"
                )
                if self._tf_base_laser_info:
                    pitch_tf = self._tf_base_laser_info["rpy_deg"][1]
                    if abs(slope_deg + pitch_tf) < 2.0:
                        issues.append(
                            "  → Slope ≈ -pitch_TF: o TF está a ADICIONAR inclinação em vez de REMOVER."
                        )
                        issues.append(
                            "  → FIX PROVÁVEL: pitch no YAML deve ser NEGATIVO (-0.218) ou ZERO."
                        )
                    elif abs(slope_deg - pitch_tf) < 2.0:
                        issues.append(
                            "  → Slope ≈ +pitch_TF: pitch DUPLICADO (Gazebo + TF ambos aplicam tilt)."
                        )
                break

        if not self._cloud_samples:
            issues.append("NENHUMA cloud recebida em /sensors/lidar/points — bridge offline?")

        if self._transform_fail > 0 and self._transform_success > 0:
            pct_fail = 100 * self._transform_fail / (self._transform_fail + self._transform_success)
            if pct_fail > 20:
                issues.append(
                    f"INSTABILIDADE: {pct_fail:.0f}% das transforms falharam — "
                    "timestamps desalinhados ou TF intermitente."
                )

        if issues:
            for iss in issues:
                print(f"  • {iss}")
        else:
            print("  ✓ Sem problemas detectados. Pontos devem aparecer planos em RViz.")

        # 6. Recomendações
        print(f"\n── 6. ACÇÃO RECOMENDADA ──")
        if not self._tf_base_laser_seen:
            print("  1. Verificar se static_sensor_tf_node está a correr:")
            print("     ros2 node list | grep static_sensor")
            print("  2. Se não está: verificar launch timing (lidar_tf_early em sim_gazebo)")
        elif self._transform_fail > 0 and self._transform_success == 0:
            print("  1. Verificar map→odom: ros2 run tf2_ros tf2_echo map odom")
            print("  2. Verificar odom→base: ros2 run tf2_ros tf2_echo odom marble_hd2/base_link")
            print("  3. Se odom→base falha: EKF/bootstrap não arrancou")
        else:
            found_slope = any(
                s.get("slope_base") is not None and abs(s["slope_base"]) > 0.02
                for s in self._cloud_samples[:3]
            )
            if found_slope:
                print("  O slope no frame base indica pitch errado no TF.")
                print("  Opção A: mudar pitch para 0 no extrinsics YAML (se Gazebo já corrige)")
                print("  Opção B: mudar pitch para NEGATIVO (se sinal invertido)")
                print("  Opção C: verificar se Gazebo publica dados no frame do LINK (não do sensor)")
                print("")
                print("  TESTE RÁPIDO para confirmar:")
                print("    ros2 run tf2_ros tf2_echo marble_hd2/base_link laser")
                print("    → Se pitch ≈ +12.5°: esse é o valor actual")
                print("    → Mudar pitch: 0.0 no YAML e rebuild → ver se pontos ficam planos")
            else:
                print("  Pipeline parece correcto. Verificar visualização em RViz:")
                print("  - Fixed frame = map")
                print("  - Topic = /sensors/lidar/points (frame laser)")
                print("  - Se ainda com slope: verificar se robot não está em rampa no mundo")

        print("\n" + "=" * 72)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=20.0)
    args = p.parse_args()

    rclpy.init()
    node = Lidar3dTiltDebug(args.duration)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
