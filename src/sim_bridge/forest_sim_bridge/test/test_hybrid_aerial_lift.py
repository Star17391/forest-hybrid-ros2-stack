"""Tests for hybrid aerial lift (offline physics / SDF semantics)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from forest_sim_bridge.hybrid_aerial_lift_diagnostic import (
    LEFT_AERIAL,
    RIGHT_AERIAL,
    _prop_frame_semantics_ok,
    run_offline,
    thrust_link_z_in_world,
)

SDF = Path.home() / "Projetos/Gazebo/ForestGen/models/forest_hybrid_robot/model.sdf"


@pytest.mark.skipif(not SDF.is_file(), reason="ForestGen SDF not found")
def test_prop_frames_anchored_on_joints():
    ok, detail = _prop_frame_semantics_ok(SDF)
    assert ok, detail


@pytest.mark.skipif(not SDF.is_file(), reason="ForestGen SDF not found")
def test_thrust_vertical_when_tracks_at_drone_pose():
    l = thrust_link_z_in_world(LEFT_AERIAL, "left")
    r = thrust_link_z_in_world(RIGHT_AERIAL, "right")
    assert l[2] > 0.9, l
    assert r[2] > 0.9, r


def test_thrust_horizontal_at_ground_pose():
    g = thrust_link_z_in_world(0.0, "left")
    assert abs(g[2]) < 0.15, g


@pytest.mark.skipif(not SDF.is_file(), reason="ForestGen SDF not found")
def test_offline_hover_margin():
    report = run_offline(SDF, mass_kg=5.84, motor_k=1.0e-4, hover_omega=385.0)
    by_name = {c.name: c for c in report.checks}
    assert by_name["hover_thrust_margin"].ok, by_name["hover_thrust_margin"].detail
    assert by_name["thrust_link_z_aerial"].ok, by_name["thrust_link_z_aerial"].detail


@pytest.mark.skipif(not SDF.is_file(), reason="ForestGen SDF not found")
def test_offline_report_passes_semantics_and_mass():
    report = run_offline(SDF, mass_kg=0.0, motor_k=0.0, hover_omega=0.0)
    assert report.checks, "expected checks"
    failed = [c.name for c in report.checks if not c.ok]
    assert "prop_frame_semantics" not in failed, failed
    assert "mass_budget" not in failed, failed
