#!/usr/bin/env python3
"""Mata processos zombies do ForestGen / stack híbrido sem `pkill -f` (evita self-match).

Lê ``/proc/*/cmdline``, exclui o próprio PID, a cadeia de PIDs-pai e processos
do Cursor/sandbox/grep. Correspondência por caminhos de executáveis ROS/Gazebo.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

# Caminhos / tokens que identificam alvos reais (evita falsos positivos em policy files).
_SIM_PATTERNS: tuple[str, ...] = (
    "gz sim",
    "/ros_gz_bridge/",
    "/rviz2/rviz2",
    "/forest_sim_bridge/lib/forest_sim_bridge/",
    "/forest_gen_bringup/lib/forest_gen_bringup/",
    "ros2 launch forest_sim_bridge",
    "ros2 launch forest_gen_bringup",
    "ros2 launch forest_hybrid_conf",
    "parameter_bridge",
    "joy_node",
    "ps5_controller",
)

_HYBRID_PATTERNS: tuple[str, ...] = (
    "/forest_planner_ros2/lib/forest_planner_ros2/mission_manager",
    "/forest_navigation_ros2/lib/forest_navigation_ros2/navigation",
)

_EXCLUDE_SUBSTRINGS: tuple[str, ...] = (
    "cursorsandbox",
    "/cursor/resources/",
    "sandbox-policy",
    " grep ",
    "grep -",
    "COMMAND_EXIT_CODE",
    "dump_bash_state",
    "__CURSOR_SANDBOX",
)


def _read_cmdline(pid: int) -> str | None:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            raw = fh.read()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    return raw.decode("utf-8", "replace").replace("\x00", " ").strip()


def _read_ppid(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/stat", "rb") as fh:
            data = fh.read().decode("utf-8", "replace")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return -1
    end = data.rfind(")")
    if end < 0:
        return -1
    fields = data[end + 2 :].split()
    if len(fields) < 2:
        return -1
    try:
        return int(fields[1])
    except ValueError:
        return -1


def _is_excluded(cmdline: str) -> bool:
    return any(ex in cmdline for ex in _EXCLUDE_SUBSTRINGS)


def _matches(cmdline: str, include_hybrid: bool) -> bool:
    if not cmdline or _is_excluded(cmdline):
        return False
    patterns = _SIM_PATTERNS
    if include_hybrid:
        patterns = _SIM_PATTERNS + _HYBRID_PATTERNS
    return any(pat in cmdline for pat in patterns)


def _find_targets(verbose: bool, include_hybrid: bool) -> list[tuple[int, str]]:
    me = os.getpid()
    my_uid = os.getuid()
    parents: set[int] = set()
    p = _read_ppid(me)
    while p > 1 and p not in parents:
        parents.add(p)
        p = _read_ppid(p)

    out: list[tuple[int, str]] = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == me or pid in parents:
            continue
        try:
            st = os.stat(entry.path)
        except (FileNotFoundError, PermissionError):
            continue
        if st.st_uid != my_uid:
            continue
        cmdline = _read_cmdline(pid)
        if not cmdline or not _matches(cmdline, include_hybrid):
            continue
        out.append((pid, cmdline))
    if verbose and not out:
        print("[forest_gen_cleanup] sem processos para matar")
    return out


def _signal_one(pid: int, sig: int) -> None:
    """Signal process group when safe; fall back to single PID."""
    me_pgid = os.getpgid(os.getpid())
    try:
        pgid = os.getpgid(pid)
    except (ProcessLookupError, PermissionError):
        return
    try:
        if pgid != me_pgid and pgid > 0:
            os.killpg(pgid, sig)
        else:
            os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            return


def _send(targets: list[tuple[int, str]], sig: int) -> None:
    for pid, _ in targets:
        _signal_one(pid, sig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Mata zombies ForestGen / stack híbrido")
    parser.add_argument("--quiet", action="store_true", help="silenciar prints")
    parser.add_argument(
        "--hybrid",
        action="store_true",
        help="incluir mission_manager_node e navigation_node (shutdown stack completo)",
    )
    parser.add_argument(
        "--term-wait",
        type=float,
        default=0.6,
        help="segundos entre SIGTERM e SIGKILL",
    )
    args = parser.parse_args()

    verbose = not args.quiet
    targets = _find_targets(verbose=verbose, include_hybrid=args.hybrid)
    if not targets:
        return 0
    if verbose:
        print(f"[forest_gen_cleanup] a matar {len(targets)} processo(s):")
        for pid, cmd in targets:
            snippet = cmd[:100] + ("…" if len(cmd) > 100 else "")
            print(f"  pid={pid:>7}  {snippet}")
    _send(targets, signal.SIGINT)
    time.sleep(min(0.4, max(0.05, args.term_wait * 0.25)))
    _send(targets, signal.SIGTERM)
    time.sleep(max(0.05, args.term_wait))
    survivors = [(p, c) for p, c in targets if os.path.exists(f"/proc/{p}")]
    if survivors:
        if verbose:
            print(f"[forest_gen_cleanup] SIGKILL em {len(survivors)} sobrevivente(s)")
        _send(survivors, signal.SIGKILL)
        time.sleep(0.2)
        survivors = [(p, c) for p, c in targets if os.path.exists(f"/proc/{p}")]
    if survivors:
        if verbose:
            print(f"[forest_gen_cleanup] AVISO: {len(survivors)} processo(s) ainda em /proc")
            for pid, cmd in survivors:
                snippet = cmd[:100] + ("…" if len(cmd) > 100 else "")
                print(f"  pid={pid:>7}  {snippet}")
        return 1
    if verbose:
        print("[forest_gen_cleanup] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
