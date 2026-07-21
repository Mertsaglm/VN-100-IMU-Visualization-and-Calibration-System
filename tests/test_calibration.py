"""
Offline calibration tests.

A known distortion (soft + hard iron) is applied and then recovered via the
fit; sphericity and center are verified after calibration.
"""
import math

import numpy as np

from tools.calibration import (
    mag_calibration,
    mag_calibration_report,
    apply_calibration,
    sphericity,
    gyro_bias,
    build_mag_clouds,
)


def _corrupted_data(seed=0, n=2000):
    rng = np.random.default_rng(seed)
    true = rng.normal(size=(n, 3))
    true /= np.linalg.norm(true, axis=1, keepdims=True)
    true *= 0.5  # radius 0.5 Gauss
    S = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    h = np.array([0.3, -0.2, 0.1])
    raw = true @ S.T + h + rng.normal(scale=0.001, size=(n, 3))
    return raw, S, h


def test_mag_calibration_recovers_sphere():
    raw, S, h = _corrupted_data()
    assert sphericity(raw) > 0.05          # raw data is a clearly distorted ellipsoid
    center, gain = mag_calibration(raw)
    cal = apply_calibration(raw, center, gain)
    assert sphericity(cal) < 0.02          # ~sphere after calibration (<2%)


def test_hard_iron_center_is_found():
    raw, S, h = _corrupted_data()
    center, _ = mag_calibration(raw)
    assert np.allclose(center, h, atol=0.05)


def test_gyro_bias():
    rng = np.random.default_rng(1)
    true_bias = np.array([0.01, -0.02, 0.005])
    samples = true_bias + rng.normal(scale=0.002, size=(5000, 3))
    b = gyro_bias(samples)
    assert np.allclose(b, true_bias, atol=0.001)


def test_ideal_sphere_stays_unchanged():
    rng = np.random.default_rng(2)
    pts = rng.normal(size=(1500, 3))
    pts /= np.linalg.norm(pts, axis=1, keepdims=True)
    center, gain = mag_calibration(pts)
    cal = apply_calibration(pts, center, gain)
    assert sphericity(cal) < 0.02
    assert np.allclose(center, 0.0, atol=0.03)


def test_build_mag_clouds():
    # Visualization helper for dashboard.gl_view (raw -> calibrated cloud + reference
    # radius); a pure function with no Qt dependency, so testable here.
    raw_in, S, h = _corrupted_data(seed=4)
    center, gain = mag_calibration(raw_in)
    raw, cal, radius = build_mag_clouds(raw_in, center, gain)
    assert raw.shape == cal.shape == raw_in.shape
    assert np.allclose(raw, raw_in)
    # calibrated cloud ~sphere: norms settle at the reference radius, more spherical than raw
    assert np.isclose(np.linalg.norm(cal, axis=1).mean(), radius, rtol=1e-6)
    assert sphericity(cal) < sphericity(raw)
    # explicit target_radius is used as-is
    _, _, r2 = build_mag_clouds(raw_in, center, gain, target_radius=1.0)
    assert r2 == 1.0


def test_distant_center_fit_is_robust():
    # Large hard-iron -> center far from the origin, where the RHS=1 fit is
    # ill-conditioned. Pre-centering (fit_ellipsoid) should still recover a good
    # sphericity and the correct center (real mag data has an offset center too).
    rng = np.random.default_rng(3)
    true = rng.normal(size=(2000, 3))
    true /= np.linalg.norm(true, axis=1, keepdims=True)
    true *= 50.0                                     # radius ~50
    S = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    h = np.array([500.0, -300.0, 400.0])
    raw = true @ S.T + h + rng.normal(scale=0.05, size=(2000, 3))
    center, gain = mag_calibration(raw)
    cal = apply_calibration(raw, center, gain)
    assert sphericity(cal) < 0.025                   # ~sphere thanks to pre-centering
    assert np.allclose(center, h, atol=1.0)


# ── F4: fit reliability diagnostics (mag_calibration_report) ──────

def test_report_ok_on_good_fit():
    # Good (full-sphere) coverage -> diagnostic reports OK, and center/gain must
    # match mag_calibration exactly (report must not change existing behavior).
    raw, S, h = _corrupted_data()
    c0, g0 = mag_calibration(raw)
    c1, g1, info = mag_calibration_report(raw)
    assert info["ok"] is True
    assert info["positive_definite"] is True
    assert np.allclose(c0, c1) and np.allclose(g0, g1)


def test_report_no_false_alarm_on_distant_center():
    # Large hard-iron (distant center) but full-sphere coverage -> should still be reliable.
    rng = np.random.default_rng(3)
    true = rng.normal(size=(2000, 3))
    true /= np.linalg.norm(true, axis=1, keepdims=True)
    true *= 50.0
    S = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    h = np.array([500.0, -300.0, 400.0])
    raw = true @ S.T + h + rng.normal(scale=0.05, size=(2000, 3))
    _, _, info = mag_calibration_report(raw)
    assert info["ok"] is True


def test_report_rejects_planar_data():
    # Planar coverage (operator only rotated within one plane): the fit is finite
    # but not an ellipsoid, so the diagnostic must reject it — otherwise sphericity
    # would look fine on screen while a wrong hard-iron value gets written to Reg 23.
    rng = np.random.default_rng(7)
    ang = rng.uniform(0, 2 * np.pi, size=1500)
    # z ~ constant (a thin band) -> a planar ring instead of a sphere
    pts = np.column_stack([np.cos(ang), np.sin(ang), rng.normal(scale=0.01, size=ang.size)])
    pts = pts * 0.5 + np.array([0.3, -0.2, 0.1])
    _, _, info = mag_calibration_report(pts)
    assert info["ok"] is False
    assert not info["positive_definite"] or info["axis_ratio"] > 8.0


# ── NaN/Inf poisoning ──────────────────────────────────────────────
def test_single_nan_sample_does_not_poison_fit():
    """REGRESSION: a single NaN row used to turn `lstsq` entirely to NaN, silently
    making the fit meaningless while the diagnostic shown to the user said
    'insufficient coverage'. The fit input now drops non-finite rows and reports
    how many were dropped."""
    rng = np.random.default_rng(11)
    S = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    h = np.array([0.3, -0.2, 0.1])
    true = rng.normal(size=(2000, 3))
    true /= np.linalg.norm(true, axis=1, keepdims=True)
    raw = true * 0.5 @ S.T + h

    clean_c, _clean_g, clean_info = mag_calibration_report(raw)
    assert clean_info["ok"] is True and clean_info["dropped_nonfinite"] == 0

    # Same cloud + 3 poisoned rows (NaN, +Inf, -Inf)
    poisoned = np.vstack([raw,
                         [np.nan, 0.0, 0.0],
                         [0.0, np.inf, 0.0],
                         [0.0, 0.0, -np.inf]])
    c, g, info = mag_calibration_report(poisoned)

    assert info["dropped_nonfinite"] == 3, "poisoned rows were not dropped"
    assert info["ok"] is True, "clean data was rejected because of poisoned rows"
    assert np.all(np.isfinite(c)) and np.all(np.isfinite(g)), "result contains NaN/Inf"
    # Result should be practically IDENTICAL to the poison-free fit (dropped rows carried no information)
    assert np.linalg.norm(c - clean_c) < 1e-9


def test_dropped_nonfinite_is_zero_on_clean_data():
    """Counter-check: the filter should drop nothing on healthy data (over-strictness regression)."""
    rng = np.random.default_rng(3)
    p = rng.normal(size=(800, 3))
    p /= np.linalg.norm(p, axis=1, keepdims=True)
    center, gain, info = mag_calibration_report(p * 0.5 + np.array([0.1, 0.0, -0.05]))
    assert info["dropped_nonfinite"] == 0
    assert info["ok"] is True
    assert all(math.isfinite(x) for x in np.asarray(center).ravel())
    assert all(math.isfinite(x) for x in np.asarray(gain).ravel())
