#!/usr/bin/env python3
"""Teste sintético do modelo de linha Palacín (sem ROS)."""

from __future__ import annotations

import math
import random
import sys


def fit_ransac(xz, thresh=0.06, iters=80, min_inliers=5):
    """Réplica simplificada da lógica C++ para validação offline."""
    best = None
    best_n = 0
    for _ in range(iters):
        p0, p1 = random.sample(xz, 2)
        dx = p1[0] - p0[0]
        if abs(dx) < 1e-6:
            continue
        m = (p1[1] - p0[1]) / dx
        b = p0[1] - m * p0[0]
        n = sum(1 for x, z in xz if abs(z - (m * x + b)) <= thresh)
        if n > best_n:
            best_n = n
            best = (m, b)
    return best, best_n


def classify(xz, m, b, band=0.05, hole=0.05, obs=0.05):
    labels = []
    for x, z in xz:
        dz = z - (m * x + b)
        if dz < -hole:
            labels.append(3)
        elif dz > obs:
            labels.append(4)
        elif abs(dz) <= band:
            labels.append(1)
        else:
            labels.append(2)
    return labels


def test_flat_ground():
    random.seed(42)
    xz = [(0.5 + 0.1 * i, 0.02 * random.gauss(0, 1)) for i in range(40)]
    line, n = fit_ransac(xz)
    assert line is not None and n >= 30, f"flat ground inliers {n}"
    labels = classify(xz, *line)
    assert labels.count(1) >= 35, "expected mostly ground"
    print("OK test_flat_ground")


def test_hole():
    random.seed(1)
    xz = [(0.5 + 0.1 * i, 0.0) for i in range(30)]
    xz.append((2.0, -0.15))  # hole
    line, n = fit_ransac(xz)
    assert line is not None
    labels = classify(xz, *line)
    assert 3 in labels, "expected hole label"
    print("OK test_hole")


def test_obstacle():
    random.seed(2)
    xz = [(0.5 + 0.1 * i, 0.0) for i in range(30)]
    xz.append((2.0, 0.20))  # trunk/obstacle
    line, n = fit_ransac(xz)
    labels = classify(xz, *line)
    assert 4 in labels, "expected obstacle label"
    print("OK test_obstacle")


def main() -> int:
    test_flat_ground()
    test_hole()
    test_obstacle()
    print("test_palacin_ground_fit: PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
