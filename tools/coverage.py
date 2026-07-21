"""
tools.coverage — Orientation-sphere coverage tracking (for the calibration wizard).

Magnetometer calibration requires rotating the sensor through all orientations:
readings should trace out a sphere, and the ellipsoid fit is only stable if
points spread across the whole sphere (otherwise it's underdetermined).

This module bins incoming magnetometer direction vectors into equal-area cells
of the sphere and tracks the sample count per cell, used by the wizard to draw
a circular coverage visualization, flag "movements done/not done", and gate
the fit until enough coverage is collected.

Equal-area method: for a unit sphere, uz (the z component) is uniformly
distributed on [-1,1] (Archimedes' theorem), so dividing uz into equal bands
and azimuth (atan2(uy,ux)) into equal sectors yields cells of equal solid
angle — no pole clustering. (Reference: PX4/ArduPilot compass calibration,
VectorNav UM001 Reg 46 8-bin coverage counter.)
"""
from __future__ import annotations

import math

import numpy as np

# 6 cardinal directions (which body axis is pointing UP) — gravity face label.
FACE_LABELS = {
    "Z+": "Z up (level)",
    "Z-": "Z down (inverted)",
    "X+": "X up",
    "X-": "X down",
    "Y+": "Y up",
    "Y-": "Y down",
}


def _unit(vec) -> np.ndarray | None:
    """Reduces the vector to unit length; None if ~zero."""
    v = np.asarray(vec, dtype=float)
    n = float(np.linalg.norm(v))
    if not math.isfinite(n) or n < 1e-9:
        return None
    return v / n


def gravity_face(accel, dominance: float = 0.7) -> str | None:
    """
    Determines which face of the sensor is pointing up from the acceleration
    (gravity) vector.

    When level, accel ~= [0,0,+g] -> '+Z' (Z axis up). Returns None if no axis
    is clearly dominant (corner/intermediate orientation).
    """
    u = _unit(accel)
    if u is None:
        return None
    i = int(np.argmax(np.abs(u)))
    if abs(u[i]) < dominance:
        return None                       # no clear face (intermediate orientation)
    axis = "XYZ"[i]
    return f"{axis}{'+' if u[i] >= 0 else '-'}"


class SphereCoverage:
    """Equal-area sphere coverage tracker (uz-band x azimuth-sector)."""

    def __init__(self, n_az: int = 12, n_el: int = 6, min_samples: int = 8):
        if n_az < 1 or n_el < 1:
            raise ValueError("n_az and n_el must both be >= 1")
        self.n_az = n_az
        self.n_el = n_el
        self.min_samples = min_samples
        self.counts = np.zeros((n_el, n_az), dtype=int)

    # -- binning -----------------------------------------------------
    def bin_of(self, vec) -> tuple[int, int] | None:
        """Maps a direction vector to its (el_band, az_sector) cell; None if invalid."""
        u = _unit(vec)
        if u is None:
            return None
        el = int((u[2] + 1.0) * 0.5 * self.n_el)
        el = min(max(el, 0), self.n_el - 1)
        ang = math.atan2(u[1], u[0])                  # [-pi, pi]
        az = int((ang + math.pi) / (2.0 * math.pi) * self.n_az)
        az = min(max(az, 0), self.n_az - 1)
        return el, az

    def add(self, vec) -> bool:
        """
        Adds a sample. Returns True the first time this cell crosses the
        "covered" threshold (lets the wizard flag when a new region is done).
        """
        b = self.bin_of(vec)
        if b is None:
            return False
        before = self.counts[b] >= self.min_samples
        self.counts[b] += 1
        after = self.counts[b] >= self.min_samples
        return bool(after and not before)

    # -- queries -------------------------------------------------------
    def covered_mask(self) -> np.ndarray:
        """Covered cells (counts >= min_samples) — bool (n_el, n_az)."""
        return self.counts >= self.min_samples

    def coverage(self) -> float:
        """Fraction of covered cells [0,1]."""
        return float(self.covered_mask().sum()) / float(self.n_el * self.n_az)

    def total_samples(self) -> int:
        return int(self.counts.sum())

    def uncovered_bins(self) -> list[tuple[int, int]]:
        """List of cells not yet covered."""
        return [tuple(b) for b in np.argwhere(~self.covered_mask())]

    def reset(self) -> None:
        self.counts[:] = 0

    # -- visualization helpers -----------------------------------------
    def cell_geometry(self, el: int, az: int) -> tuple[float, float, float, float]:
        """
        Returns the (r0, r1, a0, a1) bounds of the cell for a circular
        (azimuthal) projection. Center = uz~+1 (Z up), edge = uz~-1 (Z down);
        angle = azimuth. r = (1 - uz)/2.
        """
        r0 = 1.0 - (el + 1) / self.n_el
        r1 = 1.0 - el / self.n_el
        # bin_of computes az sector from ang=atan2(uy,ux) in [-pi,pi] (az=0 <-> ang=-pi).
        # Must use the same -pi reference here, or filled cells shift 180 deg from the
        # marker/scatter points drawn by project().
        a0 = az / self.n_az * 2.0 * math.pi - math.pi
        a1 = (az + 1) / self.n_az * 2.0 * math.pi - math.pi
        return r0, r1, a0, a1

    @staticmethod
    def project(vec) -> tuple[float, float] | None:
        """Projects a direction vector onto the coverage disk (x,y) (r=(1-uz)/2)."""
        u = _unit(vec)
        if u is None:
            return None
        r = (1.0 - u[2]) * 0.5
        ang = math.atan2(u[1], u[0])
        return r * math.cos(ang), r * math.sin(ang)
