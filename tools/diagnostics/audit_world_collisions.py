#!/usr/bin/env python3
"""Auditoria estática: colisores nos modelos/mundos ForestGen (sem Gazebo)."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

FORESTGEN = Path.home() / "Projetos/Gazebo/ForestGen"


def _mesh_exists(uri: str, models_dir: Path) -> bool:
    if not uri.startswith("model://"):
        return True
    rel = uri.replace("model://", "").strip("/")
    p = models_dir / rel
    return p.is_file()


def audit_model(sdf: Path, models_dir: Path) -> dict:
    try:
        root = ET.parse(sdf).getroot()
    except ET.ParseError as exc:
        return {"error": str(exc)}
    collisions = []
    for col in root.iter("collision"):
        geom = col.find("geometry")
        if geom is None:
            continue
        for child in geom:
            tag = child.tag.split("}")[-1]
            uri = child.findtext("uri") if tag == "mesh" else None
            ok = True
            if uri:
                ok = _mesh_exists(uri, models_dir)
            collisions.append({"type": tag, "uri": uri, "mesh_ok": ok})
    visuals = len(list(root.iter("visual")))
    sensors = [
        (s.attrib.get("name", "?"), s.attrib.get("type", "?"))
        for s in root.iter("sensor")
    ]
    return {"collisions": collisions, "visuals": visuals, "sensors": sensors}


def audit_world(world: Path) -> dict:
    root = ET.parse(world).getroot()
    includes = []
    for inc in root.iter("include"):
        uri = inc.findtext("uri") or ""
        includes.append(uri)
    return {
        "trees": sum(1 for u in includes if "Tree" in u),
        "rocks": sum(1 for u in includes if "Rock" in u),
        "terrain": [u for u in includes if "terrain" in u or "flat_ground" in u],
        "robot": [u for u in includes if "forest_tracked" in u],
    }


def main() -> int:
    fg = FORESTGEN
    models = fg / "models"
    print(f"ForestGen: {fg}\n")

    print("=== Modelos (collision + visual + sensor) ===")
    for name in sorted(
        [
            "forest_tracked_robot",
            "custom_terrain_gentle",
            "custom_terrain_rugged",
            "flat_ground",
            "Tree1",
            "Tree3",
            "Rock1",
        ]
    ):
        sdf = models / name / "model.sdf"
        if not sdf.is_file():
            print(f"  {name}: MISSING")
            continue
        r = audit_model(sdf, models)
        if "error" in r:
            print(f"  {name}: XML ERROR — {r['error']}")
            continue
        ncol = len(r["collisions"])
        bad_mesh = [c for c in r["collisions"] if c.get("uri") and not c["mesh_ok"]]
        print(
            f"  {name}: collisions={ncol}, visuals={r['visuals']}, "
            f"sensors={r['sensors']}, bad_mesh={len(bad_mesh)}"
        )
        if bad_mesh:
            for c in bad_mesh:
                print(f"    MISSING {c['uri']}")

    print("\n=== Mundos (includes) ===")
    for w in sorted((fg / "worlds").glob("forest_*.sdf")):
        try:
            info = audit_world(w)
            print(
                f"  {w.name}: trees={info['trees']} rocks={info['rocks']} "
                f"terrain={info['terrain']}"
            )
        except Exception as exc:
            print(f"  {w.name}: ERROR {exc}")

    print(
        "\nNota: gpu_lidar no Gazebo usa ray casting na cena RENDERIZADA (visuals), "
        "não nos collision meshes da física. Árvores têm visual + collision."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
