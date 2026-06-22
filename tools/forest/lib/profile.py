#!/usr/bin/env python3
"""Load forest YAML profiles and emit bash fragments for session.bash."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

API_VERSION = "forest.dev/v1"

_DEFAULTS_PATH = Path(__file__).resolve().parent.parent / "config" / "launch_defaults.yaml"


def _require_yaml() -> Any:
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PyYAML em falta. Instala: sudo apt install python3-yaml"
        ) from exc
    return yaml


def _load_yaml_file(path: Path) -> dict[str, Any]:
    yaml = _require_yaml()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: root must be a mapping")
    return data


def _is_sim_layer(layer: dict[str, Any], sim_launches: list[str]) -> bool:
    if layer.get("kind") == "sim":
        return True
    launch = str(layer.get("launch", ""))
    return any(pat in launch for pat in sim_launches)


_OVERRIDE_RE = re.compile(r"^([a-zA-Z_][a-zA-Z0-9_]*):=((?:true|false)|-?\d+(?:\.\d+)?|\S+)$")


def parse_launch_overrides(raw: str) -> dict[str, Any]:
    """Parse FOREST_LAUNCH_OVERRIDES: 'use_rviz:=false,paused:=true'."""
    out: dict[str, Any] = {}
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        m = _OVERRIDE_RE.match(part)
        if not m:
            raise ValueError(f"invalid launch override {part!r} (expected key:=value)")
        key, val = m.group(1), m.group(2)
        if val in ("true", "false"):
            out[key] = val == "true"
        else:
            try:
                out[key] = int(val)
            except ValueError:
                try:
                    out[key] = float(val)
                except ValueError:
                    out[key] = val
    return out


def merge_launch_overrides(profile: dict[str, Any]) -> dict[str, Any]:
    """Apply FOREST_LAUNCH_OVERRIDES env to sim layers (wins over profile + defaults)."""
    raw = os.environ.get("FOREST_LAUNCH_OVERRIDES", "").strip()
    if not raw:
        return profile
    overrides = parse_launch_overrides(raw)
    profile = dict(profile)
    defaults = _load_yaml_file(_DEFAULTS_PATH) if _DEFAULTS_PATH.is_file() else {}
    sim_launches = list(defaults.get("sim_layer_launches") or [])
    layers = []
    for layer in profile.get("layers") or []:
        layer = dict(layer)
        if _is_sim_layer(layer, sim_launches):
            args = dict(layer.get("args") or {})
            args.update(overrides)
            layer["args"] = args
        layers.append(layer)
    profile["layers"] = layers
    return profile


def apply_launch_defaults(profile: dict[str, Any]) -> dict[str, Any]:
    """Merge config/launch_defaults.yaml into sim layers (profile args win)."""
    if not _DEFAULTS_PATH.is_file():
        return profile
    defaults = _load_yaml_file(_DEFAULTS_PATH)
    modes = defaults.get("modes") or {}
    base = dict(defaults.get("sim_launch") or {})
    mode = profile.get("launch_mode")
    profile = dict(profile)
    profile["_launch_ui_hint"] = ""
    if mode:
        if mode not in modes:
            known = ", ".join(sorted(modes.keys())) or "(none)"
            raise ValueError(f"unknown launch_mode {mode!r} (known: {known})")
        mode_cfg = modes[mode]
        base.update(mode_cfg.get("sim_launch") or {})
        profile["_launch_ui_hint"] = mode_cfg.get("ui_hint", "") or ""
        profile["launch_mode"] = mode
    sim_launches = list(defaults.get("sim_layer_launches") or [])
    layers = []
    for layer in profile.get("layers") or []:
        layer = dict(layer)
        if layer.get("inherit_launch_defaults") is False:
            layers.append(layer)
            continue
        if _is_sim_layer(layer, sim_launches):
            args = dict(base)
            args.update(layer.get("args") or {})
            layer["args"] = args
        layers.append(layer)
    profile["layers"] = layers
    return profile


def load_profile(path: Path) -> dict[str, Any]:
    data = _load_yaml_file(path)
    version = data.get("api_version", API_VERSION)
    if version != API_VERSION:
        raise ValueError(f"{path}: unsupported api_version {version!r}")
    if "name" not in data:
        raise ValueError(f"{path}: missing 'name'")
    if "layers" not in data or not isinstance(data["layers"], list):
        raise ValueError(f"{path}: missing or invalid 'layers'")
    if data.get("status") == "legacy":
        allow = os.environ.get("FOREST_ALLOW_LEGACY", "").strip().lower() in (
            "1",
            "true",
            "yes",
        )
        if not allow:
            reason = data.get("legacy_reason", "ver docs/LEGACY_PATHS.md")
            raise ValueError(
                f"Perfil LEGACY '{data['name']}' recusado ({path.name}): {reason}. "
                "Para comparação histórica: export FOREST_ALLOW_LEGACY=1"
            )
    return merge_launch_overrides(apply_launch_defaults(data))


def _ros_arg(key: str, value: Any) -> str:
    if isinstance(value, bool):
        v = "true" if value else "false"
    else:
        v = str(value)
    return f"{key}:={v}"


def _layer_ros_args(layer: dict[str, Any], panel_only: bool) -> list[str]:
    args = dict(layer.get("args") or {})
    if panel_only:
        extra = layer.get("panel_only_args") or {}
        args.update(extra)
    return [_ros_arg(k, v) for k, v in args.items()]


def list_profiles(profiles_dir: Path, include_legacy: bool = False) -> list[str]:
    names: set[str] = set()
    for p in profiles_dir.glob("*.yaml"):
        if not include_legacy:
            try:
                data = _load_yaml_file(p)
                if data.get("status") == "legacy":
                    continue
            except ValueError:
                pass
        names.add(p.stem)
    legacy_dir = profiles_dir / "legacy"
    if legacy_dir.is_dir() and include_legacy:
        for p in legacy_dir.glob("*.yaml"):
            names.add(f"legacy/{p.stem}")
    for p in profiles_dir.glob("*.profile.bash"):
        names.add(p.name.replace(".profile.bash", ""))
    return sorted(names)


def emit_bash(profile: dict[str, Any], panel_only: bool) -> str:
    """Generate forest_profile_up + wait_nodes for sourcing in bash."""
    lines: list[str] = [
        f'forest_profile_name="{profile["name"]}"',
        f'forest_profile_description="{profile.get("description", "")}"',
        "forest_profile_up() {",
        "  local panel_only=\"${1:-false}\"",
    ]

    panel_cfg = profile.get("panel_only")
    if panel_cfg:
        nodes = panel_cfg.get("require_nodes") or []
        nodes_s = " ".join(nodes)
        lines.extend(
            [
                "  if [[ \"$panel_only\" == \"true\" ]]; then",
                f"    forest_profile_panel_only_mode {nodes_s}",
                "    return $?",
                "  fi",
                "",
            ]
        )

    for layer in profile["layers"]:
        lid = layer["id"]
        pkg = layer["package"]
        launch = layer["launch"]
        base_args = _layer_ros_args(layer, panel_only=False)
        panel_extra = layer.get("panel_only_args") or {}
        delay = int(layer.get("delay_after_sec") or 0)
        lines.append(f"  local -a _fargs_{lid}=(")
        for a in base_args:
            lines.append(f'    "{a}"')
        lines.append("  )")
        if panel_extra:
            lines.append('  if [[ "$panel_only" == "true" ]]; then')
            for k, v in panel_extra.items():
                lines.append(f'    _fargs_{lid}+=("{_ros_arg(k, v)}")')
            lines.append("  fi")
        lines.append(
            f"  forest_launch_layer {lid} {pkg} {launch} \"${{_fargs_{lid}[@]}}\" || return $?"
        )
        lines.append(
            '  if [[ -n "${FOREST_LAST_LAUNCH_PGID:-}" && -n "${FOREST_LAST_LAUNCH_PID:-}" ]]; then'
        )
        lines.append(
            f'    forest_session_register_layer {lid} "$FOREST_LAST_LAUNCH_PGID" '
            f'"$FOREST_LAST_LAUNCH_PID" "ros2 launch {pkg} {launch}"'
        )
        lines.append("  fi")
        if delay > 0:
            lines.append(f"  sleep {delay}")

    lines.append("  return 0")
    lines.append("}")

    wait = profile.get("wait_nodes") or []
    if wait:
        nodes = " ".join(wait)
        lines.append(f"forest_profile_wait_nodes=({nodes})")

    pre = (profile.get("lifecycle") or {}).get("pre_start", "cleanup_hybrid")
    lines.append(f'forest_profile_pre_start="{pre}"')

    ui = profile.get("ui") or {}
    if ui.get("mission_panel_on_foreground"):
        lines.append("forest_profile_ui_panel_foreground=true")

    hint = profile.get("_launch_ui_hint") or ui.get("launch_hint") or ""
    if hint:
        lines.append(f'forest_profile_launch_hint="{hint}"')
    lines.append(f'forest_profile_launch_mode="{profile.get("launch_mode", "interactive")}"')
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Forest profile loader")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_list = sub.add_parser("list", help="List profile names")
    p_list.add_argument(
        "--profiles-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "profiles",
    )
    p_list.add_argument(
        "--all",
        action="store_true",
        help="Incluir perfis legacy/ (requer FOREST_ALLOW_LEGACY para forest up)",
    )

    p_emit = sub.add_parser("emit-bash", help="Emit bash profile fragment")
    p_emit.add_argument("yaml", type=Path)
    p_emit.add_argument(
        "panel_only",
        nargs="?",
        default="false",
        choices=("true", "false"),
    )

    p_val = sub.add_parser("validate", help="Validate YAML profile")
    p_val.add_argument("yaml", type=Path)

    args = parser.parse_args()
    if args.cmd == "list":
        active = list_profiles(args.profiles_dir, include_legacy=False)
        for name in active:
            print(name)
        if args.all:
            legacy = list_profiles(args.profiles_dir, include_legacy=True)
            legacy_only = [n for n in legacy if n.startswith("legacy/") or n not in active]
            for name in legacy_only:
                print(f"[LEGACY] {name}")
        return 0
    if args.cmd == "validate":
        load_profile(args.yaml)
        print(f"OK: {args.yaml}")
        return 0
    if args.cmd == "emit-bash":
        prof = load_profile(args.yaml)
        sys.stdout.write(emit_bash(prof, panel_only=(args.panel_only == "true")))
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
