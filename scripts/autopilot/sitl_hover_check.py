#!/usr/bin/env python3
"""Valida hover estável num ArduPilot SITL via MAVLink (arm → takeoff → amostra altitude).

Usado por `forest test hybrid-aerial-sitl`. Não depende de ROS — fala MAVLink direto.
Requer pymavlink (em ~/venv-ardupilot). Imprime métricas (§3.2 do plano) e PASS/FAIL.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("[hover-check] FAIL: pymavlink em falta (usa ~/venv-ardupilot/bin/python3)", file=sys.stderr)
    sys.exit(2)


def _drain_status(m) -> None:
    """Imprime quaisquer STATUSTEXT pendentes (mostra razões de PreArm)."""
    while True:
        msg = m.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            return
        txt = getattr(msg, "text", "")
        if isinstance(txt, bytes):
            txt = txt.decode(errors="ignore")
        print(f"[sitl] {txt}")


def _wait_gps_ekf(m, timeout: float) -> bool:
    """Espera fix GPS 3D — primeiro passo (rápido)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        _drain_status(m)
        if msg and msg.fix_type >= 3:
            return True
    return False


def _wait_armable(m, timeout: float) -> bool:
    """Espera o EKF convergir: atitude + velocidade horiz + posição horiz (prearm)."""
    need = (
        mavutil.mavlink.EKF_ATTITUDE
        | mavutil.mavlink.EKF_VELOCITY_HORIZ
        | mavutil.mavlink.EKF_POS_HORIZ_REL
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        _drain_status(m)
        if msg and (msg.flags & need) == need:
            return True
    return False


def _is_armed(m) -> bool:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


_ACK_RESULT = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS",
}


def _arm_with_retry(m, timeout: float) -> bool:
    # Drena PreArm pendentes (mensagens periódicas) antes de tentar.
    t0 = time.monotonic() + 3.0
    while time.monotonic() < t0:
        _drain_status(m)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            res = _ACK_RESULT.get(ack.result, str(ack.result))
            print(f"[arm] COMMAND_ACK = {res}")
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            _drain_status(m)
            if _is_armed(m):
                return True
    return False


def _rel_alt(m, timeout: float = 2.0):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    return None if msg is None else msg.relative_alt / 1000.0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    p.add_argument("--alt", type=float, default=5.0, help="altitude alvo (m)")
    p.add_argument("--hover-sec", type=float, default=20.0)
    p.add_argument("--tol", type=float, default=0.5, help="tolerância de altitude (m)")
    p.add_argument("--timeout", type=float, default=120.0, help="timeout global (s)")
    args = p.parse_args()

    print(f"[hover-check] a ligar a {args.connect} …")
    m = mavutil.mavlink_connection(args.connect)
    if not m.wait_heartbeat(timeout=args.timeout):
        print("[hover-check] FAIL: sem heartbeat do SITL")
        return 1
    # Sem MAVProxy, o autopiloto é sys 1 comp 1 (MAV_COMP_ID_AUTOPILOT1).
    if m.target_component == 0:
        m.target_component = 1
    print(f"[hover-check] heartbeat de sys {m.target_system} comp {m.target_component}")

    # Sem MAVProxy ninguém pede os streams → o SITL não envia GPS/POSITION.
    # Pedir explicitamente (ALL @ 5 Hz) para GPS_RAW_INT e GLOBAL_POSITION_INT fluírem.
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1,
    )

    print("[hover-check] à espera de fix GPS …")
    if not _wait_gps_ekf(m, timeout=min(60.0, args.timeout)):
        print("[hover-check] FAIL: GPS não ficou pronto")
        return 1

    print("[hover-check] à espera do EKF convergir (prearm) …")
    if not _wait_armable(m, timeout=min(90.0, args.timeout)):
        print("[hover-check] AVISO: EKF não reportou pronto; tento armar à mesma")

    print("[hover-check] modo GUIDED …")
    m.set_mode_apm("GUIDED")
    time.sleep(1.0)

    print("[hover-check] a armar …")
    if not _arm_with_retry(m, timeout=60.0):
        print("[hover-check] FAIL: não armou (ver mensagens PreArm acima)")
        return 1
    print("[hover-check] armado. takeoff →", args.alt, "m")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.alt,
    )

    # Subir até ~alt
    climb_deadline = time.monotonic() + 60.0
    while time.monotonic() < climb_deadline:
        a = _rel_alt(m)
        if a is not None and a >= args.alt - args.tol:
            print(f"[hover-check] atingiu {a:.2f} m")
            break

    # Amostrar hover — altitude + atitude (diagnóstico de acoplamento atitude→altitude)
    print(f"[hover-check] a amostrar hover {args.hover_sec:.0f}s …")
    import math as _math
    samples: list[float] = []
    roll: list[float] = []
    pitch: list[float] = []
    rollrate: list[float] = []
    pitchrate: list[float] = []
    end = time.monotonic() + args.hover_sec
    while time.monotonic() < end:
        msg = m.recv_match(
            type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=True, timeout=2
        )
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            samples.append(msg.relative_alt / 1000.0)
        else:  # ATTITUDE (rad)
            roll.append(_math.degrees(msg.roll))
            pitch.append(_math.degrees(msg.pitch))
            rollrate.append(_math.degrees(msg.rollspeed))
            pitchrate.append(_math.degrees(msg.pitchspeed))

    if not samples:
        print("[hover-check] FAIL: sem amostras de altitude")
        return 1

    def _stats(xs):
        k = len(xs) or 1
        mu = sum(xs) / k
        sd = (sum((x - mu) ** 2 for x in xs) / k) ** 0.5
        return mu, sd, (min(xs) if xs else 0.0), (max(xs) if xs else 0.0)

    def _rms(xs):
        return (sum(x * x for x in xs) / len(xs)) ** 0.5 if xs else 0.0

    n = len(samples)
    mean, std, zmin, zmax = _stats(samples)

    print("\n=== métricas de hover (§3.2) ===")
    print(f"  ALT  alvo={args.alt:.2f} m  média={mean:.3f}  std={std:.3f}  min={zmin:.3f}  max={zmax:.3f}  n={n}")
    if roll:
        print(
            f"  ATT  roll RMS={_rms(roll):.2f}° (max|{max(abs(min(roll)), abs(max(roll))):.1f}|)  "
            f"pitch RMS={_rms(pitch):.2f}° (max|{max(abs(min(pitch)), abs(max(pitch))):.1f}|)"
        )
        print(
            f"  RATE roll RMS={_rms(rollrate):.1f}°/s  pitch RMS={_rms(pitchrate):.1f}°/s  "
            f"→ {'ATITUDE INSTÁVEL (tomba)' if _rms(roll) > 8 or _rms(pitch) > 8 else 'atitude ok; instabilidade é vertical'}"
        )

    # Land + disarm (best-effort, não afeta o veredito)
    try:
        m.set_mode_apm("LAND")
    except Exception:
        pass

    ok = (abs(mean - args.alt) <= args.tol) and (std <= args.tol) and (zmin >= args.alt * 0.5)
    if ok:
        print("\n[hover-check] PASS: hover estável\n")
        return 0
    print("\n[hover-check] FAIL: hover fora de tolerância (deriva/queda)\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
