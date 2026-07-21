"""
3D orientation axes — VN-100 body frame (UM001 Rev 2.22 §2.6.1, Rugged axis diagram).

FACTS (what these tests pin down):
  • Vendor: VN-100 is right-handed; **+X points AWAY from the connector**, +Y is
    opposite the mounting ear, **+Z points INTO the top face (downward)** — the
    ⊕ mark on the device.
  • Mesh asset (dashboard/assets/vn100_rugged_mesh.npz): **connector is at mesh −Y**,
    **top face is at mesh +Z** (base flange at the −Z end).
  ⇒ sensor +X = mesh +Y, sensor +Y = mesh +X, sensor +Z = mesh −Z

The tests don't just pin a hardcoded constant; they measure the mesh ITSELF and
check its consistency with `_MOUNT`. That way, if the .npz asset is ever replaced
with a CAD model in a different orientation, the test BREAKS — whereas a bare
`assert np.allclose(M, diag(1,-1,-1))` would still pass with the new asset and
miss the regression.

Skipped if the GUI (PySide6) isn't available; verify.py runs it with PySide6 in .venv.
"""
import numpy as np
import pytest

pytest.importorskip("PySide6")


def _mesh():
    from dashboard.gl_view import _load_vn100_asset
    a = _load_vn100_asset()
    if a is None:
        pytest.skip("no CAD mesh asset available (falls back to procedural geometry)")
    return np.asarray(a[0], dtype=float)


def test_mount_is_proper_rotation_and_matches_on_both_paths():
    import dashboard.app as app
    import dashboard.gl_view as gl
    for M in (app._MOUNT, gl._MOUNT):
        assert np.allclose(M @ M.T, np.eye(3))          # orthonormal
        assert np.isclose(np.linalg.det(M), 1.0)        # PROPER rotation (not a reflection -> handedness preserved)
    assert np.allclose(app._MOUNT, gl._MOUNT)           # QPainter and GL paths apply the SAME correction


def test_mount_maps_sensor_axes_to_mesh_directions():
    import dashboard.gl_view as gl
    M = gl._MOUNT
    assert np.allclose(M @ [1.0, 0, 0], [0, 1.0, 0]), "sensor +X should map to mesh +Y (away from connector)"
    assert np.allclose(M @ [0, 1.0, 0], [1.0, 0, 0]), "sensor +Y should map to mesh +X"
    assert np.allclose(M @ [0, 0, 1.0], [0, 0, -1.0]), "sensor +Z should map to mesh -Z (into the top face)"


def test_mesh_connector_is_actually_at_negative_Y():
    """The MEASUREMENT that `_MOUNT` relies on. If the asset changes, this breaks
    and `_MOUNT` needs to be re-derived."""
    v = _mesh()
    y = v[:, 1]
    threshold = 0.55 * y.min()
    neg = int((y < threshold).sum())                 # connector side
    pos = int((y > -threshold).sum())                # opposite side
    assert neg > 3 * pos, (
        f"connector mass expected at mesh -Y ({neg} verts) vs +Y ({pos}) - "
        "re-derive dashboard/gl_view.py:_MOUNT if the CAD asset changed.")


def test_mesh_top_face_is_at_positive_Z():
    """The base flange sticks out from the body; whichever Z end it's at defines the 'top face'."""
    v = _mesh()
    z, x = v[:, 2], v[:, 0]
    overhang = np.abs(x) > 0.96 * np.abs(x).max()
    assert z[overhang].mean() < 0, "expected the base flange at mesh -Z -> top face at +Z"


def test_triad_comes_from_same_source_as_MOUNT():
    """The drawn triad's rotation must be derived from the same `_MOUNT` — a
    hand-coded, independent constant could silently drift out of sync when
    `_MOUNT` changes."""
    import dashboard.gl_view as gl
    v = _mesh()
    tips = gl.triad_endpoints(v)
    assert [t[0] for t in tips] == ["X", "Y", "Z"]
    for k, (lab, org, tip) in enumerate(tips):
        direction = (tip - org) / np.linalg.norm(tip - org)
        assert np.allclose(direction, gl._MOUNT[:, k], atol=1e-9), f"{lab} arrow deviates from the _MOUNT column"
    # Handedness must be preserved: X x Y = Z
    d = [(tip - org) / np.linalg.norm(tip - org) for _l, org, tip in tips]
    assert np.allclose(np.cross(d[0], d[1]), d[2], atol=1e-9)


def test_Z_arrow_extends_below_the_body():
    """+Z points down: the arrow must start at the top face, pass through the body,
    and extend BELOW it (⊕ = inward)."""
    import dashboard.gl_view as gl
    v = _mesh()
    _lab, org, tip = gl.triad_endpoints(v)[2]
    assert tip[2] < float(v[:, 2].min()), "Z arrow doesn't extend below the body"
    assert tip[2] < org[2]


def test_set_orientation_runs_without_crashing():
    from PySide6 import QtWidgets
    import dashboard.app as app
    _ = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = app._Orientation3D()
    for ypr in [(0, 0, 0), (90, 0, 0), (0, 45, 0), (0, 0, 45), (-30, 60, -90)]:
        w.set_orientation(*ypr)     # paintEvent's compute path must not crash (grab triggers it)
        w.grab()


def test_conjugation_preserves_rotation_about_sensor_axis():
    """Physical correctness: if the sensor rotates about its own +Z (pure yaw), the
    on-screen model must also rotate about the MESH counterpart of sensor +Z —
    not about some other axis."""
    import dashboard.app as app
    R_yaw = app._rot_z(np.deg2rad(30.0))                 # pure yaw in sensor frame
    R_disp = app._MOUNT @ R_yaw @ app._MOUNT.T
    axis_mesh = app._MOUNT @ np.array([0, 0, 1.0])       # mesh direction of sensor +Z
    assert np.allclose(R_disp @ axis_mesh, axis_mesh, atol=1e-9), \
        "yaw axis did not stay fixed after conjugation"
    assert np.isclose(np.linalg.det(R_disp), 1.0)
