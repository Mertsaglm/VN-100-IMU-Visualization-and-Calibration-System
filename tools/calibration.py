"""
tools.calibration — VN-100 offline calibration.

Ellipsoid-fit based hard-iron (offset) and soft-iron (matrix) calibration for
the magnetometer (and accelerometer); gyro bias estimation.

Method (magnetometer):
  Raw magnetic readings should ideally lie on a SPHERE. Hard/soft iron
  distortion turns this into an ELLIPSOID. We fit the ellipsoid algebraically
  and find the transform that maps it back onto a sphere:

      calibrated = gain @ (raw - center)

  center -> hard-iron offset,  gain -> soft-iron correction matrix.

Validation: after calibration, point norms should be constant (sphericity).
"""
from __future__ import annotations

import numpy as np


def fit_ellipsoid(points: np.ndarray, *, return_info: bool = False):
    """
    Fits a general ellipsoid to points (Nx3).

    Returns: (center[3], evecs[3x3], radii[3]).

    With return_info=True, also returns a diagnostics dict (F4). A real
    ellipsoid needs ALL eigenvalues of A3 positive; `radii` is still computed
    as `1/sqrt(|evals|)` for backward compatibility, but `abs()` would
    silently mask a negative/near-zero eigenvalue (planar/hyperboloid fit =
    insufficient coverage) as a "plausible radius". The sign is exposed here
    for diagnostics; return_info=False is unaffected.
    """
    p = np.asarray(points, dtype=float)
    # A single NaN/Inf row turns `lstsq` entirely into NaN (or raises a misleading
    # LinAlgError). Live ingest already filters this; here we recover poisoned samples
    # from a recording (--replay/hybrid) or historical accumulation. Dropped row count
    # is recorded in `info`.
    _finite = np.isfinite(p).all(axis=1)
    _dropped = int((~_finite).sum())
    if _dropped:
        p = p[_finite]
    _mean = p.mean(axis=0)          # Pre-center: RHS=1 fit scales poorly when far from origin
    p = p - _mean
    x, y, z = p[:, 0], p[:, 1], p[:, 2]

    # Algebraic form: ax^2+by^2+cz^2+2fyz+2gxz+2hxy+2px+2qy+2rz = 1
    D = np.column_stack([x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z])
    v, *_ = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)

    A4 = np.array([
        [v[0], v[5], v[4], v[6]],
        [v[5], v[1], v[3], v[7]],
        [v[4], v[3], v[2], v[8]],
        [v[6], v[7], v[8], -1.0],
    ])

    center = np.linalg.solve(-A4[:3, :3], v[6:9])   # center — in pre-centered space

    # Shift to center; evecs/radii come from the pre-centered A4 — do not add _mean yet.
    T = np.eye(4)
    T[3, :3] = center
    R = T @ A4 @ T.T
    A3 = R[:3, :3] / (-R[3, 3])

    evals, evecs = np.linalg.eigh(A3)
    radii = 1.0 / np.sqrt(np.abs(evals))
    # Shift center back to original coordinates only on return; shape math stays untouched.
    if not return_info:
        return center + _mean, evecs, radii
    with np.errstate(divide="ignore", invalid="ignore"):
        axis_ratio = float(np.max(radii) / np.min(radii)) if np.min(radii) > 0 else float("inf")
    info = {
        "evals": evals,
        "positive_definite": bool(np.all(evals > 0.0)),   # real ellipsoid <=> all positive
        "min_eval": float(np.min(evals)),
        "axis_ratio": axis_ratio,                          # extreme eccentricity = near-singular
        "dropped_nonfinite": _dropped,
    }
    return center + _mean, evecs, radii, info


def mag_calibration(points: np.ndarray, target_field: float | None = None):
    """
    Computes magnetometer calibration.

    Returns: (center[3], gain[3x3])  ->  calibrated = gain @ (raw - center)
    target_field: radius of the calibrated sphere (None -> mean of the radii).
    """
    center, evecs, radii = fit_ellipsoid(points)
    if target_field is None:
        target_field = float(np.mean(radii))
    gain = evecs @ np.diag(target_field / radii) @ evecs.T
    return center, gain


def mag_calibration_report(points: np.ndarray, target_field: float | None = None,
                           *, max_axis_ratio: float = 8.0):
    """
    mag_calibration + fit RELIABILITY diagnostics (F4). Same math as
    `mag_calibration`/`fit_ellipsoid`; only adds a check for whether the fit is
    solid enough to write to the sensor (Reg 23).

    Returns: (center[3], gain[3x3], info).

    WARNING: `info['ok']` is NOT permission to write by itself. This gate only
    catches algebraic degeneration: is the fit a positive-definite ellipsoid
    (not planar/hyperboloid), is the axis ratio reasonable (extreme
    eccentricity -> near-singular, condition number blows up), and are
    center/gain finite.

    It does NOT catch insufficient geometric coverage: points clustered in a
    single "cap"/narrow band can yield an algebraically solid fit (positive
    definite, normal axis ratio) with a hard-iron center still off by a large
    fraction of the field magnitude — sphericity looks perfect because it's
    computed over the same training set. Hence `covers_geometry` always
    returns None ("unknown").

    Real protection lives in the caller: the wizard's `cov_gate` (orientation
    coverage >= 60%) and `MIN_SAMPLES_FIT` gates. This function is defense in
    depth only — it has no knowledge of the true magnetic field magnitude.
    """
    center, evecs, radii, fi = fit_ellipsoid(points, return_info=True)
    if target_field is None:
        target_field = float(np.mean(radii))
    gain = evecs @ np.diag(target_field / radii) @ evecs.T
    finite = bool(np.all(np.isfinite(center)) and np.all(np.isfinite(gain)))
    if not finite:
        reason = "fit is not finite (NaN/inf) - degenerate coverage"
    elif not fi["positive_definite"]:
        reason = "fit is not an ellipsoid (planar/insufficient coverage)"
    elif fi["axis_ratio"] > max_axis_ratio:
        reason = f"fit too eccentric (axis ratio {fi['axis_ratio']:.1f}) - unbalanced coverage"
    else:
        reason = "ok"
    info = {
        "ok": reason == "ok",
        "reason": reason,
        "finite": finite,
        "positive_definite": fi["positive_definite"],
        "axis_ratio": fi["axis_ratio"],
        "dropped_nonfinite": fi.get("dropped_nonfinite", 0),
        # None: coverage isn't geometric here; caller enforces cov_gate separately.
        "covers_geometry": None,
    }
    return center, gain, info


def apply_calibration(points: np.ndarray, center: np.ndarray, gain: np.ndarray) -> np.ndarray:
    """calibrated = gain @ (raw - center) (row-vector form)."""
    p = np.asarray(points, dtype=float)
    return (p - np.asarray(center, dtype=float)) @ np.asarray(gain, dtype=float).T


def sphericity(points: np.ndarray) -> float:
    """Sphericity metric: std-dev of norms / mean of norms (0 = perfect sphere)."""
    norms = np.linalg.norm(np.asarray(points, dtype=float), axis=1)
    return float(norms.std() / norms.mean())


def build_mag_clouds(samples, center, gain, target_radius: float | None = None):
    """Builds raw and calibrated point clouds for before/after fit visualization.

    Returns: (raw[Nx3], cal[Nx3], target_radius); target_radius defaults to the
    mean calibrated norm when not given.

    Pure Qt-free helper (testable without a GUI); used by dashboard.gl_view.
    """
    raw = np.asarray(samples, dtype=float)
    cal = apply_calibration(raw, center, gain)
    if target_radius is None:
        target_radius = float(np.linalg.norm(cal, axis=1).mean())
    return raw, cal, target_radius


def gyro_bias(samples: np.ndarray) -> np.ndarray:
    """Mean of gyro samples collected while stationary = bias [3]."""
    return np.mean(np.asarray(samples, dtype=float), axis=0)


# -- Manual trial run --------------------------------------------------
def _demo():
    rng = np.random.default_rng(0)
    # True sphere (radius 0.5 Gauss)
    true = rng.normal(size=(2000, 3))
    true /= np.linalg.norm(true, axis=1, keepdims=True)
    true *= 0.5
    # Distortion: soft-iron S + hard-iron h + noise
    S = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    h = np.array([0.3, -0.2, 0.1])
    raw = true @ S.T + h + rng.normal(scale=0.002, size=(2000, 3))

    center, gain = mag_calibration(raw)
    cal = apply_calibration(raw, center, gain)
    print(f"Raw sphericity        : {sphericity(raw) * 100:.2f}%")
    print(f"Calibrated sphericity : {sphericity(cal) * 100:.2f}%")
    print(f"Found hard-iron        : {center}")
    print(f"True hard-iron         : {h}")


if __name__ == "__main__":
    _demo()
