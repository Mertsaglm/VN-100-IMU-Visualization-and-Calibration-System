"""
Sphere coverage tracking tests (calibration guide).

Equal-area binning, coverage threshold, full-sphere -> 100%, and gravity-face
detection, verified without hardware.
"""
import math

import numpy as np

from tools.coverage import SphereCoverage, gravity_face


def _fibonacci_sphere(n: int) -> np.ndarray:
    """n points ~uniformly distributed on a sphere (golden-angle spiral)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)          # polar angle
    theta = math.pi * (1.0 + 5.0 ** 0.5) * i    # golden angle
    return np.column_stack([
        np.sin(phi) * np.cos(theta),
        np.sin(phi) * np.sin(theta),
        np.cos(phi),
    ])


def test_bin_boundaries():
    cov = SphereCoverage(n_az=8, n_el=4)
    # +Z (uz=+1) -> top elevation band; -Z -> bottom
    assert cov.bin_of([0, 0, 1])[0] == cov.n_el - 1
    assert cov.bin_of([0, 0, -1])[0] == 0
    # zero vector -> None
    assert cov.bin_of([0, 0, 0]) is None


def test_equal_area_distribution():
    # Uniform sphere distribution -> all bins should get ~equal sample counts (equal-area proof)
    cov = SphereCoverage(n_az=12, n_el=6, min_samples=1)
    pts = _fibonacci_sphere(20000)
    for p in pts:
        cov.add(p)
    counts = cov.counts.astype(float)
    # coefficient of variation should be small (no pole clustering)
    cv = counts.std() / counts.mean()
    assert cv < 0.15, f"bins are not equal-area (cv={cv:.3f})"


def test_full_sphere_is_100_percent():
    cov = SphereCoverage(n_az=12, n_el=6, min_samples=5)
    for p in _fibonacci_sphere(20000):
        cov.add(p)
    assert cov.coverage() == 1.0
    assert cov.uncovered_bins() == []


def test_partial_coverage():
    # Northern hemisphere only -> coverage ~50%, lower bands empty
    cov = SphereCoverage(n_az=12, n_el=6, min_samples=3)
    for p in _fibonacci_sphere(20000):
        if p[2] > 0:
            cov.add(p)
    assert 0.35 < cov.coverage() < 0.6
    assert len(cov.uncovered_bins()) > 0


def test_add_reports_new_bin():
    cov = SphereCoverage(n_az=4, n_el=2, min_samples=3)
    v = [1, 0, 0.001]
    assert cov.add(v) is False   # 1st sample
    assert cov.add(v) is False   # 2nd
    assert cov.add(v) is True    # 3rd -> reached threshold (newly covered)
    assert cov.add(v) is False   # 4th -> already covered


def test_reset():
    cov = SphereCoverage(min_samples=1)
    cov.add([1, 0, 0])
    assert cov.total_samples() == 1
    cov.reset()
    assert cov.total_samples() == 0
    assert cov.coverage() == 0.0


def test_gravity_face_detection():
    assert gravity_face([0, 0, 9.81]) == "Z+"     # level
    assert gravity_face([0, 0, -9.81]) == "Z-"    # inverted
    assert gravity_face([9.81, 0, 0]) == "X+"
    assert gravity_face([0, -9.81, 0]) == "Y-"
    # corner (in-between) orientation -> no clear face
    d = 9.81 / math.sqrt(3)
    assert gravity_face([d, d, d]) is None


def test_project_center_and_edge():
    # +Z lands at the center (r=0), -Z at the edge (r=1)
    x, y = SphereCoverage.project([0, 0, 1])
    assert abs(x) < 1e-9 and abs(y) < 1e-9
    x, y = SphereCoverage.project([1, 0, -1])   # not pure -Z (uz != -1); pure -Z case follows
    px, py = SphereCoverage.project([0, 0, -1])
    assert abs(math.hypot(px, py) - 1.0) < 1e-9


def test_project_and_cell_geometry_agree():
    # The point a direction vector projects to via project() must lie inside the cell
    # the same vector maps to via bin_of()/cell_geometry() — i.e. the marker/scatter
    # point visually overlaps the cell it fills. Regression: a 180-degree azimuth shift
    # (missing -pi offset in cell_geometry) must not reappear.
    cov = SphereCoverage(n_az=12, n_el=6)
    for p in _fibonacci_sphere(3000):
        px, py = cov.project(p)
        pang = math.atan2(py, px)
        pr = math.hypot(px, py)
        el, az = cov.bin_of(p)
        r0, r1, a0, a1 = cov.cell_geometry(el, az)
        assert a0 - 1e-9 <= pang <= a1 + 1e-9, (
            f"azimuth shifted: point {math.degrees(pang):.1f}° cell "
            f"[{math.degrees(a0):.1f}°,{math.degrees(a1):.1f}°]")
        assert r0 - 1e-9 <= pr <= r1 + 1e-9, (
            f"radius shifted: point r={pr:.3f} cell [{r0:.3f},{r1:.3f}]")
