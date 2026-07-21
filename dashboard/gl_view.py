"""
dashboard.gl_view — OpenGL 3D visuals + a mandatory CPU fallback.

Two visuals live here:
  * Orientation3DGL       — a lit/shaded 3D sensor model shaped like the VN-100 (mesh-based; GL).
  * CalibrationResultDialog — calibration point cloud before/after fit (2D XY/XZ/YZ projections).

Design principles:
  * OpenGL is used ONLY for the mesh-based 3D sensor model (Orientation3DGL); the
    time-series plots are untouched.
  * Orientation3DGL is guarded: if GL is unavailable/broken, app.py falls back to the
    pure-QPainter `_Orientation3D`.
  * CalibrationResultDialog is PURE 2D (raster pyqtgraph, reliable on every GPU/driver).
    A GL point cloud (GLScatterPlotItem) fails to render on some Windows/Intel drivers,
    so it is deliberately NOT used for the calibration visual — three plane projections
    already show the ellipsoid-vs-sphere difference most clearly.
  * This module does NOT import `dashboard.app` (import cycle: app <- calibration_dialog
    <- gl_view). Color constants are passed as parameters; a local palette for the dialog
    is defined below (same hex values as app.py).

To test the sensor-model fallback path on a machine that DOES have GL: set the
environment variable `VN100_NO_GL=1`.
"""
from __future__ import annotations

import os

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from tools.calibration import build_mag_clouds, sphericity

# ── GL import gate (falls through here if PyOpenGL is missing; the module still imports safely) ──
try:
    import pyqtgraph.opengl as gl
    _IMPORT_OK = True
except Exception:                      # ImportError + any other load failure
    gl = None
    _IMPORT_OK = False

_GL_CACHE: bool | None = None

# Same palette as app.py (repeated here to avoid an import cycle)
_BG = "#0e141b"        # C_PANEL
_TEXT = "#e6edf3"      # C_TEXT
_MUTED = "#8b949e"     # C_MUTED


# ── Availability probe ────────────────────────────────────────
def _gl_context_ok() -> bool:
    """Can a real GL context be created without showing a widget? (offscreen probe.)"""
    try:
        ctx = QtGui.QOpenGLContext()
        if not ctx.create():
            return False
        surf = QtGui.QOffscreenSurface()
        surf.create()
        if not surf.isValid():
            return False
        ok = ctx.makeCurrent(surf)
        if ok:
            ctx.doneCurrent()
        return bool(ok)
    except Exception:
        return False


def gl_available() -> bool:
    """Is OpenGL usable? (cached). VN100_NO_GL=1 -> False (force the fallback)."""
    global _GL_CACHE
    if _GL_CACHE is not None:
        return _GL_CACHE
    try:
        if os.environ.get("VN100_NO_GL"):
            _GL_CACHE = False
        elif not _IMPORT_OK:
            _GL_CACHE = False
        else:
            _GL_CACHE = _gl_context_ok()
    except Exception:
        _GL_CACHE = False
    return _GL_CACHE


# ── Small helpers ────────────────────────────────────────
def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ── Sensor body frame <-> mesh frame — SINGLE SOURCE OF TRUTH ──────────────────
# The COLUMNS of _MOUNT give each sensor axis's direction in MESH coordinates:
#     _MOUNT[:,0] = sensor +X,   _MOUNT[:,1] = sensor +Y,   _MOUNT[:,2] = sensor +Z
# Both the axis-triad DRAWING and the body ROTATION are derived from this single
# matrix, so they can't drift apart — if one were edited without the other, the
# axis mapping would silently become inconsistent.
#
# MEASURED mesh facts (dashboard/assets/vn100_rugged_mesh.npz, 22818 vertices):
#   * connector -> mesh -Y   (vertex density 10146 at -Y vs 1710 at +Y; protrusion in the XY footprint)
#   * top face  -> mesh +Z   (the base flange overhang sits at the mesh -Z end)
# VENDOR ground truth (UM001 Rev 2.22 §2.6.1, VN-100 **Rugged** axis diagram, p.15):
#   * +X points AWAY from the connector (connector is on the sensor's -X face)
#   * +Y is opposite the mounting ear
#   * +Z points INTO the top face = down (the on-case ⊕ marking)
# => sensor +X = mesh +Y,  sensor +Y = mesh +X,  sensor +Z = mesh -Z
_MOUNT = np.array([[0.0, 1.0,  0.0],
                   [1.0, 0.0,  0.0],
                   [0.0, 0.0, -1.0]])


def triad_endpoints(verts) -> list:
    """Derives the axis triad's endpoints in mesh coordinates from `_MOUNT`.

    Returns: [(label, origin[3], tip[3]), ...] — sensor X, Y, Z in order.
    Needs no GL, so a test can call it directly to verify the triad and the
    rotation share one source (see `_MOUNT` above)."""
    vv = np.asarray(verts, dtype=float)
    ztop, zbot = float(vv[:, 2].max()), float(vv[:, 2].min())
    ext = np.array([float(np.abs(vv[:, 0]).max()),
                    float(np.abs(vv[:, 1]).max()),
                    float(max(ztop, -zbot))])
    origin = np.array([0.0, 0.0, ztop + 0.03])      # top-face center (where the etched triad sits)
    out = []
    for k, lab in enumerate(("X", "Y", "Z")):
        direction = _MOUNT[:, k]                     # mesh direction of sensor axis k
        length = (ext[2] + origin[2] + 1.15) if k == 2 else (float(np.abs(direction) @ ext) + 0.5)
        out.append((lab, origin, origin + direction * length))
    return out


def _rgba(color) -> tuple:
    """Hex string / QColor -> (r,g,b,a) float 0-1."""
    q = QtGui.QColor(color)
    return (q.redF(), q.greenF(), q.blueF(), 1.0)


# ── VN-100 RUGGED body geometry (procedural mesh) ─────────────
# The red ALUMINUM enclosure seen in photos: one chamfered corner (pentagonal top
# outline), a mounting ear on the side, connector on the front edge. Real Rugged
# dimensions are ~36x33x9.5 mm; proportions preserved with half-sizes scaled down
# (exact mm isn't needed for recognizability at dashboard scale).
_HX, _HY, _HZ = 1.55, 1.45, 0.46      # body half-dimensions
_CHAMFER = 0.85                        # length of the chamfer at the +X+Y corner

# Colors (VectorNav red + metal ear + black connector + gold pins)
_C_RED     = (0.80, 0.14, 0.11, 1.0)   # anodized red body (VectorNav red)
_C_RED_TOP = (0.90, 0.22, 0.17, 1.0)   # top face (catches slightly more light)
_C_SILVER  = (0.68, 0.71, 0.75, 1.0)   # mounting ear (machined aluminum)
_C_BLACK   = (0.10, 0.10, 0.12, 1.0)   # connector housing
_C_GOLD    = (0.85, 0.68, 0.30, 1.0)   # connector pins


def _prism(poly_xy, z0, z1, side_col, top_col=None):
    """Extrudes a 2D polygon (CCW vertices) into a vertical prism from z0 to z1.
    Returns: (verts, faces[Nx3], faceColors[Nx4]). If top_col is omitted, top=side_col."""
    n = len(poly_xy)
    verts = [[x, y, z0] for (x, y) in poly_xy] + [[x, y, z1] for (x, y) in poly_xy]
    verts = np.array(verts, float)
    faces, fcol = [], []
    for i in range(n):                                   # side faces (each edge is a quad = 2 triangles)
        j = (i + 1) % n
        faces += [[i, j, j + n], [i, j + n, i + n]]
        fcol += [side_col, side_col]
    tc = top_col if top_col is not None else side_col
    for i in range(1, n - 1):                            # top cap (fan triangulation)
        faces.append([n, n + i, n + i + 1]); fcol.append(tc)
    for i in range(1, n - 1):                            # bottom cap (reversed winding)
        faces.append([0, i + 1, i]); fcol.append(side_col)
    return verts, np.array(faces), np.array(fcol)


def _box(cx, cy, cz, hx, hy, hz, col):
    """Axis-aligned box (center + half-size) -> prism."""
    poly = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
    return _prism(poly, cz - hz, cz + hz, col)


def _merge(parts):
    """Merges multiple (verts, faces, fcol) parts into a single mesh (with index offsets)."""
    vs, fs, cs, off = [], [], [], 0
    for v, f, c in parts:
        vs.append(v); fs.append(np.asarray(f) + off); cs.append(np.asarray(c)); off += len(v)
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


def _vn100_mesh():
    """VN-100 Rugged body: chamfered-corner red box + mounting ear + connector + pins.
    This is the PROCEDURAL fallback model used when the real CAD asset can't be found."""
    # Pentagonal top outline: +X+Y corner is chamfered (the cut corner seen in photos)
    body_poly = [(-_HX, -_HY), (_HX, -_HY), (_HX, _HY - _CHAMFER),
                 (_HX - _CHAMFER, _HY), (-_HX, _HY)]
    body = _prism(body_poly, -_HZ, _HZ, _C_RED, _C_RED_TOP)
    ear = _box(cx=-_HX - 0.42, cy=-0.35, cz=-_HZ + 0.11, hx=0.42, hy=0.5, hz=0.11, col=_C_SILVER)
    conn = _box(cx=0.15, cy=-_HY - 0.24, cz=-_HZ + 0.30, hx=0.62, hy=0.24, hz=0.24, col=_C_BLACK)
    pins = [_box(cx=0.15 + k * 0.17, cy=-_HY - 0.46, cz=-_HZ + 0.30,
                 hx=0.045, hy=0.05, hz=0.12, col=_C_GOLD) for k in (-2, -1, 0, 1, 2)]
    return _merge([body, ear, conn, *pins])


# Mesh asset derived from the actual manufacturer CAD (VectorNav VN-100 Rugged STEP).
# The STEP was tessellated and normalized (centered, thin axis vertical/flat lying down,
# ~3.1 units) and written to a .npz; at runtime it's loaded with numpy ONLY (no gmsh or
# other conversion tooling needed at runtime). 22818 vertices / 23188 triangles, normals
# precomputed. Falls back to the procedural model if missing/corrupt.
_ASSET_PATH = os.path.join(os.path.dirname(__file__), "assets", "vn100_rugged_mesh.npz")


def _load_vn100_asset():
    """Returns the mesh asset (verts, faces); None if missing/corrupt (-> procedural model)."""
    try:
        d = np.load(_ASSET_PATH)
        v = d["vertices"].astype(float)
        f = d["faces"].astype(int)
        if v.ndim == 2 and v.shape[1] == 3 and f.ndim == 2 and f.shape[1] == 3 and len(f) >= 4:
            return v, f
    except Exception:
        pass
    return None


def _vn100_geometry():
    """Returns the real CAD asset if available (smooth, single-color anodized red);
    otherwise the procedural model (faceted, multi-color). Returns: (verts, faces, faceColors, smooth)."""
    real = _load_vn100_asset()
    if real is not None:
        v, f = real
        fcol = np.tile(np.array(_C_RED, dtype=float), (len(f), 1))
        return v, f, fcol, True
    v, f, fcol = _vn100_mesh()
    return v, f, fcol, False


# ── Feature 1: 3D orientation model shaped like the VN-100 ─────────────
class Orientation3DGL(QtWidgets.QWidget):
    """Draws the VN-100 body as a lit 3D mesh; same API as the QPainter `_Orientation3D`."""

    def __init__(self, bg, col_x, col_y, col_z, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 170)
        self._live = False
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(bg)
        self._view.setCameraPosition(distance=7.5, elevation=26, azimuth=42)
        self._view.setMinimumHeight(168)
        lay.addWidget(self._view)
        self._body: list = []           # all items that rotate together with the body
        self._build_model(col_x, col_y, col_z)

    def _build_model(self, col_x, col_y, col_z) -> None:
        verts, faces, fcol, smooth = _vn100_geometry()   # real CAD if available, else procedural
        md = gl.MeshData(vertexes=verts, faces=faces, faceColors=fcol)
        # Real CAD (thousands of triangles) -> smooth, no edges; procedural -> faceted + thin edges.
        mesh = gl.GLMeshItem(meshdata=md, smooth=smooth, shader="shaded",
                             drawEdges=not smooth, edgeColor=(0.0, 0.0, 0.0, 0.35))
        self._add_body(mesh)

        # Axis triad mimics the real sensor body frame (UM001 §2.6.1): VN-100 is
        # RIGHT-HANDED, +X forward, +Y right (toward the connector), +Z DOWN (into
        # the top face). The mesh asset is rotated 180° about X relative to the sensor
        # frame (connector at mesh -Y, UM001 says +Y; mesh +Z out of the top face,
        # sensor +Z in) -> in mesh coords the sensor axes are [+X, -Y, -Z]; the
        # rotation in set_orientation is conjugated with _MOUNT accordingly. On-screen
        # axes match the arrows etched on the device — the calibration reference.
        endpoints, colors, labels = [], [], []
        for (lab, origin, uc), col in zip(triad_endpoints(verts), (col_x, col_y, col_z)):
            endpoints += [origin, uc]
            colors += [_rgba(col), _rgba(col)]
            labels.append((lab, tuple(uc), col))

        triad = gl.GLLinePlotItem(pos=np.array(endpoints, float), color=np.array(colors),
                                  width=2.5, mode="lines", antialias=True)
        self._add_body(triad)
        for txt, pos, col in labels:
            try:
                t = gl.GLTextItem(pos=np.array(pos, float), text=txt, color=QtGui.QColor(col))
                self._add_body(t)
            except Exception:
                pass                    # skip the label if GLTextItem isn't available in this pyqtgraph version

    def _add_body(self, item) -> None:
        self._view.addItem(item)
        self._body.append(item)

    def set_orientation(self, yaw, pitch, roll, live=True) -> None:
        """Same convention as the QPainter version: R = Rz(yaw)*Ry(pitch)*Rx(roll),
        conjugated into the VN-100 body frame via _MOUNT (UM001 §2.6.1; +Z down, right-handed)."""
        self._live = live
        y, p, r = np.deg2rad([float(yaw), float(pitch), float(roll)])
        # Since p_mesh = _MOUNT . p_sensor, the sensor rotation R is conjugated for display:
        # R_disp = _MOUNT . R . _MOUNT^-1.  _MOUNT is orthonormal -> its inverse is its transpose.
        # (Older code wrote `_MOUNT @ R @ _MOUNT`; correct only because _MOUNT happens to be
        #  its own inverse — written with .T here so it doesn't silently break if _MOUNT changes.)
        R = _MOUNT @ (_rot_z(y) @ _rot_y(p) @ _rot_x(r)) @ _MOUNT.T
        m = QtGui.QMatrix4x4(
            R[0, 0], R[0, 1], R[0, 2], 0.0,
            R[1, 0], R[1, 1], R[1, 2], 0.0,
            R[2, 0], R[2, 1], R[2, 2], 0.0,
            0.0, 0.0, 0.0, 1.0)
        for it in self._body:           # rotate the MESH, not the camera
            it.setTransform(m)


# ── Feature 2: calibration fit before/after window (2D) ─────
class CalibrationResultDialog(QtWidgets.QDialog):
    """Shows raw (ellipsoid) vs. calibrated (sphere) magnetometer points across 3 planes (XY/XZ/YZ).

    Pure 2D pyqtgraph — reliable on every GPU/driver, unlike a GL point cloud
    (see module docstring). Only reads a snapshot of `samples/center/gain`;
    does not touch the calling dialog's flow.
    """

    def __init__(self, samples, center, gain, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration Result — Before / After Fit")
        self.resize(880, 560)
        self.setStyleSheet(f"QDialog {{ background: {_BG}; }}")

        raw, cal, radius = build_mag_clouds(samples, center, gain)
        sb = sphericity(raw) * 100.0
        sa = sphericity(cal) * 100.0
        verdict = "Excellent" if sa < 1.0 else "Good" if sa < 2.5 else "Poor"

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)

        info = QtWidgets.QLabel(
            f"Sphericity: {sb:.2f}% -> {sa:.2f}%   [{verdict}]      "
            f"Points: {len(raw)}      Reference radius: {radius:.4f} Gauss")
        info.setStyleSheet(f"color:{_TEXT}; font-size:13px; font-family:Consolas,monospace; font-weight:700;")
        lay.addWidget(info)

        legend = QtWidgets.QLabel(
            "<span style='color:#ffa500;'>&#9679;</span> raw (ellipsoid, before correction)   "
            "<span style='color:#3cd870;'>&#9679;</span> calibrated (sphere, after correction)   "
            "<span style='color:#c8c8c8;'>&#9412;</span> reference circle (calibrated radius)")
        legend.setStyleSheet(f"color:{_MUTED}; font-size:11px;")
        lay.addWidget(legend)

        lay.addWidget(self._build_2d(raw, cal, radius), 1)

        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        btn = QtWidgets.QPushButton("Close")
        btn.clicked.connect(self.accept)
        row.addWidget(btn)
        lay.addLayout(row)

    def _build_2d(self, raw, cal, radius):
        glw = pg.GraphicsLayoutWidget()
        glw.setBackground(_BG)
        theta = np.linspace(0, 2 * np.pi, 120)
        cx, cy = radius * np.cos(theta), radius * np.sin(theta)
        for title, i, j in (("XY  (top view)", 0, 1), ("XZ  (side view)", 0, 2), ("YZ  (front view)", 1, 2)):
            p = glw.addPlot(title=title)
            p.setAspectLocked(True)
            p.showGrid(x=True, y=True, alpha=0.18)
            p.setMenuEnabled(False)
            p.addItem(pg.ScatterPlotItem(x=raw[:, i], y=raw[:, j], size=4, pen=None,
                                         brush=pg.mkBrush(255, 168, 0, 110)))
            p.addItem(pg.ScatterPlotItem(x=cal[:, i], y=cal[:, j], size=4, pen=None,
                                         brush=pg.mkBrush(60, 216, 112, 150)))
            p.plot(cx, cy, pen=pg.mkPen(210, 210, 210, 150, width=1.4, style=QtCore.Qt.DashLine))
        return glw
