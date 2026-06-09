"""Unit tests for hybrid_physics_audit (no Gazebo required)."""

from pathlib import Path

from forest_sim_bridge.hybrid_physics_audit import run_audit

SDF = Path.home() / "Projetos/Gazebo/ForestGen/models/forest_hybrid_robot/model.sdf"


def test_mass_under_7kg():
    assert SDF.is_file(), f"missing {SDF}"
    report = run_audit(SDF, max_mass_kg=7.0, min_twr=1.5, max_twr=6.0)
    assert report.total_mass_kg <= 7.0, report.total_mass_kg
    assert report.passed, [(c.name, c.detail) for c in report.checks if not c.ok]


def test_allocation_rank_four():
    report = run_audit(SDF, max_mass_kg=7.0, min_twr=1.5, max_twr=6.0)
    rank_check = next(c for c in report.checks if c.name == "allocation_rank")
    assert rank_check.ok, rank_check.detail
