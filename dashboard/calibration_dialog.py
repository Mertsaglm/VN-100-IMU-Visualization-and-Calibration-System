"""
dashboard.calibration_dialog — Guided magnetometer calibration wizard.

TWO WORKFLOWS (per UM001 §7.44-7.46 — see docs/calibration.md):

  1) THE SENSOR'S OWN HSI (recommended, default):
     The VN-100 ships from the factory with real-time HSI calibration ON. The
     correct approach is to RESET it (Reg 44 RESET) for the current environment
     and let it converge, watch convergence via Reg 46 (NumMeas/AvgResidual/bins —
     7 OR 8 bins depending on hardware, most likely 7; the code tolerates both via
     len(bins)), then TURN IT OFF (Reg 44 OFF) once done and save to flash ($VNWNV).
     -> This is the root fix for "the longer I calibrate, the worse it gets": an
        offline fit was fighting the sensor's CONTINUOUSLY changing correction.

  2) OFFLINE ELLIPSOID FIT (advanced):
     First set Reg 23 to identity + TURN OFF onboard HSI (output goes RAW), then
     run an ellipsoid fit and write it to Reg 23, keep onboard OFF (NO_ONBOARD), save.

The circular coverage visual guides motion in both modes (green = collected).

HYBRID MODE (measurements from a recording, commands to the real sensor —
`--replay CSV --port COMx`): the wizard detects this via `self.hybrid`
(`transport.data_is_recorded and transport.writable`) and **locks the method to
offline fit**: onboard HSI converges using the sensor's OWN live magnetometer,
and replaying a recording feeds it no data -> Reg 46 would never fill up. The
stillness gate there also measures the real sensor, not the recording
(`still_reference`). The recording MUST have been captured in RAW mode —
rationale and procedure: `docs/calibration.md` §4b.

HSIOutput (Reg 44 field 1) has two values (UM001 Rev 2.22): {1 NO_ONBOARD, 3 USE_ONBOARD}.
NO_ONBOARD -> output is only the Reg 23 user solution; USE_ONBOARD -> onboard real-time HSI.

Test without hardware: `python vn100_dashboard.py --sim --sim-motion calibration`.
"""
from __future__ import annotations

import math
import time
from contextlib import contextmanager

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from pyvn100 import selfcheck
from pyvn100.registers import (HSI_STABLE_TOL, HSIMode, HSIOutput, IDENTITY_MAG_CAL, Reg,
                               decode_hsi_status, decode_mag_cal, hsi_solution_converged,
                               mag_cal_max_delta)
from tools.calibration import (apply_calibration, mag_calibration_report,
                               sphericity)
from tools.coverage import FACE_LABELS, SphereCoverage, gravity_face
from dashboard.app import (QSS, C_PAGE, C_PANEL, C_PANEL2, C_BORDER,
                           C_TEXT, C_MUTED, C_ACCENT, C_GREEN, C_RED)
from dashboard.gl_view import CalibrationResultDialog
# F5: pre-WNV stillness gate constants — single source shared with the gyro tool (no
# circular import: gyro_bias_dialog does not import calibration_dialog).
from dashboard.gyro_bias_dialog import (STILL_GYRO, ACCEL_LO, ACCEL_HI, STALE_MAX_AGE_S,
                                        still_reference)

# ── Settings ──────────────────────────────────────────────────────
N_AZ, N_EL = 12, 6
MIN_SAMPLES_BIN = 6
MOVE_GATE_GYRO = 0.10          # rad/s — below this, consider it "still"
FACE_BINS_NEEDED = 6
SCATTER_KEEP = 400

# ── Onboard-mode thresholds ────────────────────────────────────────
#
# ⚠ This firmware's ICD does NOT have **Reg 46** (it's absent from the register
# index; HSI = Reg 44 + Reg 47) — a bin/AvgResidual-based convergence gate would
# never open on this hardware (bins stay 0 -> `all(b >= 3)` stays False forever ->
# the wizard would say "converging" forever). Instead, two ICD-supported metrics
# are used:
#   * Progress    -> PC-side orientation coverage (tools/coverage.py; already used in offline mode)
#   * Convergence -> the Reg 47 solution SETTLING (registers.hsi_solution_converged) + leaving identity
# Reg 46 is still read but ONLY as INFORMATION (populated on v2.x hardware); no decision depends on it.
ONBOARD_RESID_OK = 0.020       # (v2.x info panel only) AvgResidual threshold
ONBOARD_BIN_MIN = 3            # (v2.x info panel only) samples per bin
ONBOARD_MIN_COVERAGE = 0.55    # orientation coverage (PC side) required for onboard convergence
ONBOARD_TIMEOUT_S = 60         # if it hasn't converged after being rotated this long, redirect the user
                               # (in a genuinely noisy environment onboard HSI may never converge)

# Offline-mode thresholds
MIN_COVERAGE_FIT = 0.60
MIN_SAMPLES_FIT = 300
# UPPER bound on the fit sample buffer (unbounded growth = a memory leak on a long session).
# The ellipsoid fit already saturates at 300 samples; 20,000 is a generous ceiling.
MAX_FIT_SAMPLES = 20000
# M-12: tolerance for "is the sensor's solution the one we're PREVIEWING?" before writing to
# flash. Looser than HSI_STABLE_TOL (0.002): the sensor stores float32 and the readback is
# parsed from %.6f text -> a few ULPs of difference is NORMAL. 0.01 both covers that noise and
# reliably catches a genuinely DIFFERENT calibration (a typical term difference is >>0.01).
MAG_CAL_SAVE_TOL = 0.01

STATUS_POLL_EVERY = 16         # ~0.5 s (30 ms timer x 16)
SNAPSHOT_TIMEOUT_S = 1.0       # wait for the Reg 23/44 response for the 'Discard' snapshot (real
                               # hardware responds in ~250 ms; if it doesn't arrive, the snapshot
                               # stays empty and Discard warns honestly)

# Coverage-wheel colors (consistent with app.py's palette; C_TEXT/C_MUTED/C_ACCENT are imported)
C_BG = C_PANEL          # CoverageWheel/plot background = card background
C_CELL = "#161b22"      # empty coverage cell
C_PART = "#9e6a03"      # partial coverage (amber)
C_DONE = C_GREEN
C_WARN = C_RED

# Calibration moves. The text references the X/Y/Z arrows etched ON the sensor
# (same as the triad in the on-screen 3D model) — no vague "one side/the other end" wording.
# Each step: (1) orient the sensor as specified, (2) rotate a full turn in ~5 s while watching that axis's arrow.
MOVES: list[tuple[str, str]] = [
    ("Z+", "Set it flat on the table — top face (arrows/logo) facing UP. Slowly rotate a full turn (360°) over ~5 s."),
    ("Z-", "Flip it upside down — top face facing the table. Rotate a full 360° the same way."),
    ("X+", "Stand it on edge so the tip of the top X arrow points UP. Rotate 360°."),
    ("X-", "Flip it over: the X arrow points DOWN (opposite of the previous step). Rotate 360°."),
    ("Y+", "Stand it on the adjacent edge so the tip of the top Y arrow points UP. Rotate 360°."),
    ("Y-", "Flip it over: the Y arrow points DOWN (opposite). Rotate 360°."),
]

# Orientation-icon axis colors (consistent with the 3D model's triad: X red, Y green, Z blue)
_AXIS_ICON_COL = {"X": "#ff6b6b", "Y": "#51cf66", "Z": "#4dabf7"}


class _MoveIcon(QtWidgets.QWidget):
    """Small glyph showing one calibration step's orientation: the sensor box + an arrow
    showing which axis should point which way (up/down) + the axis letter. code e.g. 'Z+' / 'X-'."""

    def __init__(self, code: str, parent=None):
        super().__init__(parent)
        self.code = code
        self.setFixedSize(30, 28)
        self.setStyleSheet("background: transparent;")   # let the card background show through (no box halo)

    def paintEvent(self, _ev) -> None:
        axis, sign = self.code[0], self.code[1]
        up = sign == "+"
        col = QtGui.QColor(_AXIS_ICON_COL.get(axis, C_MUTED))
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        body = QtCore.QRectF(w * 0.30, h * 0.32, w * 0.40, h * 0.40)
        p.setPen(QtGui.QPen(QtGui.QColor(C_MUTED), 1.2))
        p.setBrush(QtGui.QColor("#20272e"))
        p.drawRoundedRect(body, 3, 3)                      # sensor body
        cx = w * 0.5
        p.setPen(QtGui.QPen(col, 2.0))
        if up:                                             # up arrow
            p.drawLine(QtCore.QPointF(cx, h * 0.28), QtCore.QPointF(cx, h * 0.05))
            p.drawLine(QtCore.QPointF(cx, h * 0.05), QtCore.QPointF(cx - 4, h * 0.15))
            p.drawLine(QtCore.QPointF(cx, h * 0.05), QtCore.QPointF(cx + 4, h * 0.15))
        else:                                              # down arrow
            p.drawLine(QtCore.QPointF(cx, h * 0.72), QtCore.QPointF(cx, h * 0.95))
            p.drawLine(QtCore.QPointF(cx, h * 0.95), QtCore.QPointF(cx - 4, h * 0.85))
            p.drawLine(QtCore.QPointF(cx, h * 0.95), QtCore.QPointF(cx + 4, h * 0.85))
        p.setPen(col)                                      # axis letter (inside the body)
        f = p.font(); f.setPointSize(8); f.setBold(True); p.setFont(f)
        p.drawText(body, QtCore.Qt.AlignCenter, axis)
        p.end()


class CoverageWheel(QtWidgets.QWidget):
    """Circular (azimuthal) projection of the sphere coverage — filled/empty cells."""

    def __init__(self, cov: SphereCoverage, parent=None):
        super().__init__(parent)
        self.cov = cov
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.plot = pg.PlotWidget()
        self.plot.setBackground(C_BG)
        self.plot.setAspectLocked(True)
        self.plot.hideAxis("bottom")
        self.plot.hideAxis("left")
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(False, False)
        self.plot.setRange(xRange=(-1.25, 1.25), yRange=(-1.25, 1.25), padding=0)
        lay.addWidget(self.plot)
        self._vb = self.plot.getPlotItem().getViewBox()

        self._cells: dict[tuple[int, int], QtWidgets.QGraphicsPolygonItem] = {}
        self._build_cells()

        self._scatter = pg.ScatterPlotItem(size=3, pen=None,
                                            brush=pg.mkBrush(230, 237, 243, 70))
        self.plot.addItem(self._scatter)
        self._marker = pg.ScatterPlotItem(size=15, brush=pg.mkBrush(0, 0, 0, 0),
                                          pen=pg.mkPen(C_ACCENT, width=2))
        self.plot.addItem(self._marker)
        self._add_labels()

    def _arc_polygon(self, r0, r1, a0, a1, steps=8) -> QtGui.QPolygonF:
        pts = []
        for k in range(steps + 1):
            a = a0 + (a1 - a0) * k / steps
            pts.append(QtCore.QPointF(r1 * math.cos(a), r1 * math.sin(a)))
        for k in range(steps + 1):
            a = a1 - (a1 - a0) * k / steps
            pts.append(QtCore.QPointF(r0 * math.cos(a), r0 * math.sin(a)))
        return QtGui.QPolygonF(pts)

    def _build_cells(self) -> None:
        pen = pg.mkPen(C_BG, width=1)
        for el in range(self.cov.n_el):
            for az in range(self.cov.n_az):
                r0, r1, a0, a1 = self.cov.cell_geometry(el, az)
                item = QtWidgets.QGraphicsPolygonItem(self._arc_polygon(r0, r1, a0, a1))
                item.setPen(pen)
                item.setBrush(pg.mkBrush(C_CELL))
                self._vb.addItem(item)
                self._cells[(el, az)] = item

    def _add_labels(self) -> None:
        for text, (x, y) in [("Z↑", (0.0, 0.0)), ("Z↓", (0.0, -1.14)),
                             ("N", (1.14, 0.0)), ("S", (0.0, 1.14))]:
            t = pg.TextItem(text, color=C_MUTED, anchor=(0.5, 0.5))
            t.setPos(x, y)
            self.plot.addItem(t)

    def refresh(self, scatter_xy=None, marker_vec=None) -> None:
        counts = self.cov.counts
        m = self.cov.min_samples
        for (el, az), item in self._cells.items():
            c = counts[el, az]
            item.setBrush(pg.mkBrush(C_DONE if c >= m else C_PART if c > 0 else C_CELL))
        if scatter_xy is not None and len(scatter_xy):
            arr = np.asarray(scatter_xy)
            self._scatter.setData(arr[:, 0], arr[:, 1])
        if marker_vec is not None:
            pr = self.cov.project(marker_vec)
            self._marker.setData([pr[0]] if pr else [], [pr[1]] if pr else [])


class CalibrationDialog(QtWidgets.QDialog):
    """Guided magnetometer calibration wizard (onboard HSI + offline fit)."""

    def __init__(self, vn, parent=None):
        super().__init__(parent)
        self.vn = vn
        # HYBRID: measurements from a recording, commands to the real sensor (see module
        # docstring) — onboard HSI can't converge on replayed data, so only offline fit
        # is offered here.
        self.hybrid = bool(getattr(vn.transport, "data_is_recorded", False)
                           and getattr(vn.transport, "writable", True))
        self.mode = "offline" if self.hybrid else "onboard"   # "onboard" | "offline"
        self.samples: list[tuple[float, float, float]] = []
        self._decimate_idx = 0          # thinning cursor once MAX_FIT_SAMPLES is reached
        self._nonfinite_skipped = 0     # count of samples dropped at the finiteness gate (diagnostics)
        self._scatter_xy: list[tuple[float, float]] = []
        self.cov = SphereCoverage(n_az=N_AZ, n_el=N_EL, min_samples=MIN_SAMPLES_BIN)
        # Separate, accel(gravity)-based coverage for the FIT GATE: mag coverage can stay
        # below 60% under strong hard-iron (|offset|/field >= 1) even with a perfect sweep,
        # so it never unlocked 'Fit' (C-M2). Gravity direction is independent of hard-iron ->
        # a correct gate. The visual mag scatter/wheel is UNCHANGED (still driven by self.cov).
        self.cov_gate = SphereCoverage(n_az=N_AZ, n_el=N_EL, min_samples=MIN_SAMPLES_BIN)
        self.face_bins: dict[str, set] = {code: set() for code, _ in MOVES}
        self._center = self._gain = None
        self._poll_ctr = 0
        self._hsi_status = None
        self._converged = False
        # Reg 47 solution history — the REAL metric for onboard convergence (Reg 46 doesn't exist on this FW).
        self._r47_history: list = []
        self._r47_ts = 0.0
        # The sensor's firmware capabilities (does Reg 46 exist, does $VNTAR exist, ...). If
        # Reg 4 hasn't been read, the baseline (v3) profile is assumed and known=False -> the UI
        # doesn't silently hide this fact.
        self._caps = selfcheck.capabilities(self.vn)
        self._prev_sim_motion = None   # temporary 'calibration' motion while the wizard is open in sim mode
        # Two-stage apply/save flow: "collect" -> "preview" (try in RAM) -> "saved" (flash)
        self._stage = "collect"
        self._snapshot: dict[int, list] | None = None   # Reg 23+44 snapshot for Discard
        self._session_t0 = 0.0         # to verify snapshot/register reads belong to this session
        self._session_started = False  # _start_session should run only once (Pause->Resume shouldn't RESET it again)
        self._io_error: str | None = None   # last failed transport.write error (F1: no silent success)
        self._cur_face: str | None = None    # the sensor's current orientation (for the live hint)

        self.setWindowTitle("Magnetometer Calibration — Wizard")
        self.resize(1000, 700)
        self.setStyleSheet(QSS + f"QDialog {{ background: {C_PAGE}; }}")
        self._build()
        self._apply_mode()

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._collect)

    # ── UI ───────────────────────────────────────────────────────
    def _card(self, title: str, subtitle: str = ""):
        """Returns a titled card -> (frame, content-layout). Same language as app.py."""
        card = QtWidgets.QFrame()
        card.setObjectName("card")
        v = QtWidgets.QVBoxLayout(card)
        v.setContentsMargins(12, 10, 12, 12)
        v.setSpacing(8)
        h = QtWidgets.QHBoxLayout()
        t = QtWidgets.QLabel(title)
        t.setObjectName("h")
        h.addWidget(t)
        h.addStretch(1)
        if subtitle:
            s = QtWidgets.QLabel(subtitle)
            s.setObjectName("sub")
            h.addWidget(s)
        v.addLayout(h)
        return card, v

    def _build(self) -> None:
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 14)
        outer.setSpacing(11)

        # Header
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(10)
        logo = QtWidgets.QLabel("◈")
        logo.setStyleSheet(f"color:{C_ACCENT}; font-size:22px;")
        head.addWidget(logo)
        tb = QtWidgets.QVBoxLayout()
        tb.setSpacing(0)
        ttl = QtWidgets.QLabel("MAGNETOMETER CALIBRATION")
        ttl.setStyleSheet(f"color:{C_TEXT}; font-size:15px; font-weight:800; letter-spacing:1px;")
        subt = QtWidgets.QLabel("onboard HSI · offline ellipsoid fit — guided wizard")
        subt.setObjectName("sub")
        tb.addWidget(ttl)
        tb.addWidget(subt)
        head.addLayout(tb)
        head.addStretch(1)
        outer.addLayout(head)

        # Method card
        mcard, mlay = self._card("METHOD")
        mrow = QtWidgets.QHBoxLayout()
        mrow.setSpacing(8)
        self.cmb_mode = QtWidgets.QComboBox()
        self.cmb_mode.addItem("Sensor's own calibration (recommended)", "onboard")
        self.cmb_mode.addItem("Offline ellipsoid fit (advanced)", "offline")
        if self.hybrid:
            # Make onboard UNSELECTABLE (not hidden: the user shouldn't have to wonder why it's
            # gone) and lock to offline. Only onboard is disabled; fit + apply + save all work.
            self.cmb_mode.model().item(0).setEnabled(False)
            self.cmb_mode.setCurrentIndex(1)
            self.cmb_mode.setToolTip("The sensor's own (onboard) calibration is unavailable in "
                                     "hybrid mode: the sensor converges its solution using its OWN "
                                     "live magnetometer, and replaying a recording feeds it no data.")
        self.cmb_mode.currentIndexChanged.connect(self._on_mode_changed)
        mrow.addWidget(self.cmb_mode, 1)
        rlbl = QtWidgets.QLabel("CONVERGENCE RATE")
        rlbl.setObjectName("sub")
        mrow.addWidget(rlbl)
        self.spin_rate = QtWidgets.QSpinBox()
        self.spin_rate.setRange(1, 5)
        self.spin_rate.setValue(3)
        self.spin_rate.setToolTip("Reg 44 ConvergeRate: 1=slow/accurate (~60-90s), 5=fast (~15-20s)")
        mrow.addWidget(self.spin_rate)
        mlay.addLayout(mrow)
        outer.addWidget(mcard)

        # Simulation info (only visible in sim): the wizard rotates the sensor automatically.
        self.lbl_sim = QtWidgets.QLabel("")
        self.lbl_sim.setObjectName("sub")
        self.lbl_sim.setWordWrap(True)
        outer.addWidget(self.lbl_sim)

        # Hybrid warning banner — the user should always know the on-screen data is not LIVE.
        # (This distinction is critical: 'Apply/Save' writes to the REAL sensor, but the chart shows the recording.)
        if self.hybrid:
            self.lbl_hybrid = QtWidgets.QLabel(
                "◈ HYBRID MODE — measurements are coming FROM A RECORDING (the sensor is not "
                "being rotated), commands are being written to the REAL sensor. Method is locked "
                "to 'offline fit'. For the result to be valid, the recording MUST have been "
                "captured in RAW mode (Reg 23 identity + onboard HSI off); otherwise you'd be "
                "writing a correction ON TOP OF an existing correction.")
            self.lbl_hybrid.setWordWrap(True)
            self.lbl_hybrid.setStyleSheet(
                f"color:{C_TEXT}; font-size:11px; font-weight:600; padding:8px 10px; "
                f"background:{C_PANEL2}; border:1px solid {C_PART}; border-radius:8px;")
            outer.addWidget(self.lbl_hybrid)

        root = QtWidgets.QHBoxLayout()
        root.setSpacing(11)
        outer.addLayout(root, 1)

        # Left card: coverage wheel
        lcard, llay = self._card("ORIENTATION COVERAGE", "azimuthal projection")
        hint = QtWidgets.QLabel("Center = Z up, edge = Z down. Green = collected, dark = missing. "
                                "Goal: paint the whole circle green.")
        hint.setObjectName("sub")
        hint.setWordWrap(True)
        llay.addWidget(hint)
        self.wheel = CoverageWheel(self.cov)
        llay.addWidget(self.wheel, 1)
        self.lbl_metrics = QtWidgets.QLabel("Coverage: 0.0%   Samples: 0")
        self.lbl_metrics.setStyleSheet(f"color:{C_TEXT}; font-size:13px; font-family:Consolas,monospace;")
        llay.addWidget(self.lbl_metrics)
        self.bar_cov = QtWidgets.QProgressBar()
        self.bar_cov.setRange(0, 100)
        llay.addWidget(self.bar_cov)
        root.addWidget(lcard, 3)

        # Right column
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(10)
        root.addLayout(right, 2)

        # Live instruction (callout — highlighted box)
        self.lbl_now = QtWidgets.QLabel("Press 'Start' to begin.")
        self.lbl_now.setWordWrap(True)
        self.lbl_now.setStyleSheet(
            f"color:{C_ACCENT}; font-size:12px; font-weight:bold; padding:9px 11px; "
            f"background:{C_PANEL2}; border:1px solid {C_ACCENT}; border-radius:8px;")
        right.addWidget(self.lbl_now)

        # HSI status card (onboard)
        self.grp_hsi, hlay = self._card("SENSOR HSI STATUS", "REG 46")
        self.lbl_resid = QtWidgets.QLabel("AvgResidual: —   NumMeas: 0")
        self.lbl_resid.setStyleSheet(f"color:{C_TEXT}; font-size:11px; font-family:Consolas,monospace;")
        hlay.addWidget(self.lbl_resid)
        self.bar_resid = QtWidgets.QProgressBar()
        self.bar_resid.setRange(0, 100)
        self.bar_resid.setTextVisible(False)
        hlay.addWidget(self.bar_resid)
        binrow = QtWidgets.QHBoxLayout()
        self.lbl_bin_count = QtWidgets.QLabel("BIN")   # the real count is written on the first Reg 46 read (7/8)
        self.lbl_bin_count.setObjectName("sub")
        binrow.addWidget(self.lbl_bin_count)
        self._bin_lbls = []
        for i in range(8):                              # up to 8 boxes; extras are hidden if the sensor reports fewer bins
            b = QtWidgets.QLabel("■")
            b.setStyleSheet(f"color:{C_CELL}; font-size:16px;")
            binrow.addWidget(b)
            self._bin_lbls.append(b)
        binrow.addStretch(1)
        hlay.addLayout(binrow)
        right.addWidget(self.grp_hsi)

        # Moves card — COMPLETED counter + only remaining moves are shown (completed rows are
        # hidden -> the list "refreshes"), the first one is highlighted as "▶ NOW" (mentor request).
        movcard, movlay = self._card("MOVES TO DO")
        # Layer 1 — goal + reference heading: the user should know WHAT they're doing and WHAT to look at.
        lbl_goal = QtWidgets.QLabel(
            "Goal: point the magnetometer in every direction across all 3 axes. The reference is "
            "the X/Y/Z arrows etched ON the sensor (same ones shown in the 3D model on the left). "
            "For each step, orient the sensor as shown, then rotate a full turn (360°) over ~5 s. "
            "The coverage wheel below turns green as you progress.")
        lbl_goal.setWordWrap(True)
        lbl_goal.setStyleSheet(f"color:{C_MUTED}; font-size:11px; padding-bottom:4px;")
        movlay.addWidget(lbl_goal)
        self.lbl_move_count = QtWidgets.QLabel(f"Completed: 0 / {len(MOVES)} orientations")
        self.lbl_move_count.setStyleSheet(f"color:{C_ACCENT}; font-size:12px; font-weight:800;")
        movlay.addWidget(self.lbl_move_count)
        self.bar_moves = QtWidgets.QProgressBar()
        self.bar_moves.setRange(0, len(MOVES))
        self.bar_moves.setTextVisible(False)
        self.bar_moves.setFixedHeight(6)
        movlay.addWidget(self.bar_moves)
        self._move_rows: list[dict] = []
        for code, desc in MOVES:
            row = self._make_move_row(code, desc)
            self._move_rows.append(row)
            movlay.addWidget(row["widget"])
        # Layer 3a — LIVE hint for the active step: is the sensor currently in the right orientation?
        self.lbl_active_hint = QtWidgets.QLabel("")
        self.lbl_active_hint.setWordWrap(True)
        self.lbl_active_hint.setStyleSheet(f"color:{C_MUTED}; font-size:11px; padding-top:2px;")
        movlay.addWidget(self.lbl_active_hint)
        self.lbl_remaining = QtWidgets.QLabel("")
        self.lbl_remaining.setStyleSheet(f"color:{C_DONE}; font-size:11px; font-weight:600;")
        self.lbl_remaining.setWordWrap(True)
        movlay.addWidget(self.lbl_remaining)
        right.addWidget(movcard)

        right.addStretch(1)

        # Result (callout)
        self.lbl_result = QtWidgets.QLabel("—")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setStyleSheet(
            f"color:{C_TEXT}; font-size:11px; font-family:Consolas,monospace; padding:8px 10px; "
            f"background:{C_PANEL2}; border:1px solid {C_BORDER}; border-radius:8px;")
        right.addWidget(self.lbl_result)

        b1 = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_start.clicked.connect(self._toggle_record)
        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_reset.setToolTip("Clears the wizard's screen (samples/coverage) and, if this "
                                  "session already wrote to the sensor, writes the PRE-SESSION "
                                  "Reg 23/44 values back to RAM (from the snapshot). Never touches "
                                  "the saved calibration in flash.")
        self.btn_reset.clicked.connect(self._reset)
        b1.addWidget(self.btn_start)
        b1.addWidget(self.btn_reset)
        right.addLayout(b1)

        # Stage 1: compute (offline) + apply to RAM (preview). No flash write yet.
        b2 = QtWidgets.QHBoxLayout()
        self.btn_fit = QtWidgets.QPushButton("Fit")
        self.btn_fit.setEnabled(False)
        self.btn_fit.clicked.connect(self._fit)
        self.btn_apply = QtWidgets.QPushButton("Apply ▸ Preview")
        self.btn_apply.setToolTip("Writes the calibration to the sensor's RAM (temporary). You'll "
                                  "see its effect live on the chart; NOT written to flash — lost on power loss.")
        self.btn_apply.setEnabled(False)
        self.btn_apply.clicked.connect(self._apply)
        b2.addWidget(self.btn_fit)
        b2.addWidget(self.btn_apply)
        right.addLayout(b2)

        # Fit result visualization (raw ellipsoid -> calibrated sphere) — separate window.
        # Only active after a successful offline fit; doesn't touch the main flow.
        self.btn_viz = QtWidgets.QPushButton("Visualize Result")
        self.btn_viz.setToolTip("Shows the before/after magnetometer point cloud in a separate "
                                "window: raw (shifted ellipsoid) vs. calibrated (centered sphere).")
        self.btn_viz.setEnabled(False)
        self.btn_viz.clicked.connect(self._show_result_viz)
        right.addWidget(self.btn_viz)

        # Stage 2: make permanent ($VNWNV) or discard (write the snapshot back). Only active in preview.
        b3 = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("Save (permanent)")
        self.btn_save.setObjectName("apply")
        self.btn_save.setToolTip("Writes the previewed calibration to flash ($VNWNV) — survives power loss.")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._save)
        self.btn_cancel = QtWidgets.QPushButton("Discard")
        self.btn_cancel.setToolTip("Reverts the preview: returns the sensor to its state before "
                                   "the apply. Does not touch flash.")
        self.btn_cancel.setEnabled(False)
        self.btn_cancel.clicked.connect(self._cancel)
        b3.addWidget(self.btn_save)
        b3.addWidget(self.btn_cancel)
        right.addLayout(b3)

        # Dangerous: restores the sensor's stored calibration to factory state + writes to flash.
        self.btn_clear = QtWidgets.QPushButton("Clear Calibration From Sensor")
        self.btn_clear.setObjectName("danger")
        self.btn_clear.setToolTip("Reg 23 -> identity + Reg 44 -> onboard HSI (on) + $VNWNV. "
                                  "Permanently erases the STORED calibration on the sensor.")
        self.btn_clear.clicked.connect(self._clear_sensor)
        right.addWidget(self.btn_clear)

        note = QtWidgets.QLabel("ApplyCompensation (ICD FW3 §3.5.1): 1=Disable (Reg 23 only), "
                                "3=Enable (onboard). Values are the same as FW 2.1; only the name "
                                "changed (HSIOutput -> ApplyCompensation).")
        note.setObjectName("sub")
        note.setWordWrap(True)
        right.addWidget(note)

    def _make_move_row(self, code: str, desc: str) -> dict:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(2, 1, 2, 1)
        icon = _MoveIcon(code)                          # Layer 3b — orientation glyph
        dot = QtWidgets.QLabel("○")
        dot.setFixedWidth(16)
        dot.setStyleSheet(f"color:{C_MUTED}; font-size:14px;")
        text = QtWidgets.QLabel(desc)
        text.setWordWrap(True)
        text.setStyleSheet(f"color:{C_MUTED}; font-size:11px;")
        bar = QtWidgets.QProgressBar()
        bar.setRange(0, 100)
        bar.setFixedWidth(70)
        bar.setTextVisible(False)
        lay.addWidget(icon)
        lay.addWidget(dot)
        lay.addWidget(text, stretch=1)
        lay.addWidget(bar)
        return {"widget": w, "icon": icon, "dot": dot, "text": text, "bar": bar, "done": False}

    # ── Mode management ─────────────────────────────────────────────
    def _on_mode_changed(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
            self.btn_start.setText("Start")
        self.mode = self.cmb_mode.currentData()
        self._reset()
        self._apply_mode()

    def _apply_mode(self) -> None:
        onboard = self.mode == "onboard"
        self.grp_hsi.setVisible(onboard)
        self.btn_fit.setVisible(not onboard)
        self.btn_apply.setText("Apply ▸ Preview")
        self.btn_start.setText("Start")
        self._set_stage("collect")
        if self.hybrid:
            self.lbl_now.setText("Hybrid: 'Start' -> the sensor is switched to RAW mode and the "
                                 "RECORDING is played back (no manual rotation). Once coverage fills, press 'Fit'.")
        elif onboard:
            self.lbl_now.setText("Sensor's own calibration: 'Start' -> the sensor resets for this "
                                 "environment, converge by rotating it.")
        else:
            self.lbl_now.setText("Offline fit: 'Start' -> the sensor is switched to RAW mode, "
                                 "rotate to collect data, then press 'Fit'.")

    # ── Recording ────────────────────────────────────────────────────
    def _toggle_record(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
            self.btn_start.setText("Resume")
            self.lbl_now.setText("Paused.")
            return
        if not self._session_started:
            self._start_session()
        self._timer.start()
        self.btn_start.setText("Pause")

    def _start_session(self) -> None:
        """Sends the mode-appropriate startup commands to the sensor (ONCE per session)."""
        rate = self.spin_rate.value()
        self._session_started = True
        self._session_t0 = time.time()
        self._snapshot = None
        self._r47_history.clear()
        self._r47_ts = 0.0
        # Refresh capabilities: if Reg 4 arrived via the identity read, we now know the real
        # profile (does Reg 46 exist, which register does $VNSGB write to, ...). Assume baseline if not read yet.
        self._caps = selfcheck.capabilities(self.vn)
        self._capture_snapshot()          # <- the basis for 'Discard': BEFORE any writes, SYNCHRONOUS
        self._enter_sim_calibration_motion()
        # Calibration needs mag data ($VNYMR, ASCII only); the sensor might be in binary
        # mode -> switch to ASCII first (otherwise mag would always read 0).
        if self.mode == "onboard":
            # RESET onboard HSI for this environment + apply the onboard solution
            ok = self._send_all(self.vn.link.set_output_mode("ascii"), self.vn.link.hsi_reset(rate=rate))
        else:
            # Offline: set Reg 23 to identity + TURN OFF onboard HSI -> output is RAW (uncalibrated).
            # These writes are VERIFIED: if the sensor didn't actually switch to raw mode, the
            # samples we collect would be pre-corrected and the fit would silently come out WRONG
            # (the sneakiest kind of bug).
            ok = (self._send(self.vn.link.set_output_mode("ascii"))
                  and self._write_verified(Reg.MAG_CALIBRATION, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)
                  and self._write_verified(Reg.HSI_CONTROL, HSIMode.OFF, HSIOutput.DISABLE, rate))
        if not ok:
            # F1: session startup commands could not be verified -> don't silently say 'started'; warn the user.
            self.lbl_now.setText("⚠ Session startup commands COULD NOT BE VERIFIED — data may not "
                                 f"be raw [{self._io_error}]. Check the connection, press 'Reset', "
                                 "then start again.")

    def _send(self, cmds) -> bool:
        """Writes one or more commands to the sensor using the active link (BRIDGE/DIRECT) framing. Returns: SUCCESS.

        `cmds` can be a `str` or `list[str]` — `vn.link.*` methods return `list[str]`
        (including multi-register commands like `set_output_mode` in DIRECT mode). The list is
        sent in order; STOPS on the first error (limits the risk of a partial/inconsistent write).

        F1: if transport.write raises (port half-open, USB re-enumeration, disconnect), it is
        NOT swallowed — returns False, and the reason is stored in `_io_error` -> the caller
        won't WRONGLY report 'applied'. Stays fire-and-forget otherwise (VNACK != acceptance —
        definitive confirmation comes from the sensor's echo)."""
        if not self.vn.transport.writable:
            self._io_error = "replay mode — commands never reach the sensor"
            return False
        if isinstance(cmds, str):
            cmds = [cmds]
        for text in cmds:
            try:
                self.vn.send(text)          # on_tx -> logged to the main console as 'tx' (calibration commands are visible)
            except Exception as exc:        # noqa: BLE001 — keep the GUI from crashing; report the error visibly
                self._io_error = str(exc)
                return False
        return True

    def _send_all(self, *groups) -> bool:
        """Sends multiple command GROUPS (each a str or list[str]) in order; True only if ALL
        succeed. STOPS on the first failure (to avoid leaving things in a partial/inconsistent state).

        ⚠ This only measures "bytes left the PC" — it does NOT prove the sensor ACCEPTED them.
        Use `_write_verified()` for writes where the outcome matters, like calibration."""
        for g in groups:
            if not self._send(g):
                return False
        return True

    @contextmanager
    def _busy(self):
        """While waiting for a sensor response: locks the window, shows a busy cursor, and
        keeps the GUI responsive via processEvents.

        A verified write waits for the response — ~250 ms on real hardware, up to 1 s x 3
        retries = 3 s on an unresponsive sensor (e.g. wrong BRIDGE/DIRECT). Without a hook
        the GUI thread would block and the window would say "Not Responding".

        `setEnabled(False)` also guards against processEvents' re-entrancy trap: without it
        the user could click 'Apply' twice and start overlapping writes.

        M-8: the dialog is MODELESS, so processEvents also drains the MAIN WINDOW's queue —
        the parent is disabled too, or the user could click 'Factory Reset'/'Save' mid-write.
        Charts keep drawing (setEnabled only blocks input).

        The collection timer is also stopped: a `_collect` $VNRRG poll landing mid-write adds
        wire traffic that produces `$VNERR,03` (docs/protocol.md §8.1) — `write_register_verified`
        already mutes telemetry (`$VNASY,0`) for this reason, so our own command traffic must be
        muted too. Everything is restored in `finally`.
        """
        app = QtWidgets.QApplication.instance()
        prev = self.vn.on_wait
        parent = self.parent()
        parent_prev = parent.isEnabled() if parent is not None else None
        timer_was_running = self._timer.isActive()
        self.setEnabled(False)
        if parent is not None:
            parent.setEnabled(False)
        self._timer.stop()                      # QTimer.stop() is idempotent -> a double-stop is harmless
        if app is not None:
            app.setOverrideCursor(QtGui.QCursor(QtCore.Qt.WaitCursor))
            self.vn.on_wait = app.processEvents
        try:
            yield
        finally:
            self.vn.on_wait = prev
            if app is not None:
                app.restoreOverrideCursor()
            if timer_was_running:
                self._timer.start()
            if parent is not None and parent_prev is not None:
                parent.setEnabled(parent_prev)
            self.setEnabled(True)

    def _write_verified(self, reg: int, *values) -> bool:
        """Write a register and verify the sensor ACCEPTED it by reading it back (with retries).

        Why: the sensor can reject a write with `$VNERR`, and transport/VNACK success does NOT
        reflect that (it only means "bytes were sent"). So an unverified write is treated as
        FAILED and the reason is stored in `_io_error` (see docs/protocol.md §8.2).
        """
        with self._busy():
            res = self.vn.write_register_verified(reg, *values)
        if not res["ok"]:
            self._io_error = res["reason"]
        return bool(res["ok"])

    # ── Simulated motion (sim only; no-op on real hardware) ──
    def _sim_transport(self):
        """Returns the active transport if it's a simulator; None on real hardware."""
        t = getattr(self.vn, "transport", None)
        if t is not None and hasattr(t, "set_motion") and hasattr(t, "sim"):
            return t
        return None

    def _enter_sim_calibration_motion(self) -> None:
        """In sim, the default 'gentle' mode barely moves the sensor (can't pass the gyro-motion
        gate) -> the wizard would never fill up. While the wizard is active, switch the sim to
        full-sphere 'calibration' tumbling. Does NOT affect real hardware (there you rotate it by hand)."""
        sim = self._sim_transport()
        if sim is None or self._prev_sim_motion is not None:
            return
        self._prev_sim_motion = sim.set_motion("calibration") or "gentle"
        self.lbl_sim.setText("● Simulation: the sensor is being automatically rotated through a "
                             "full-sphere calibration motion (on real hardware you'd do this by hand).")

    def _restore_sim_motion(self) -> None:
        """When the wizard closes, restores the sim's motion to what it was before (usually 'gentle')."""
        sim = self._sim_transport()
        if sim is not None and self._prev_sim_motion is not None:
            sim.set_motion(self._prev_sim_motion)
        self._prev_sim_motion = None
        self.lbl_sim.setText("")

    def _reset(self) -> None:
        # F2/F3: if a SESSION changed the sensor's RAM (offline: Reg 23=identity + HSI OFF ->
        # output RAW; onboard: HSI RESET), write the snapshot back BEFORE clearing the screen.
        # Otherwise 'Reset' would silently leave the sensor uncalibrated, and the next session
        # would mistake that identity for the 'snapshot' and break the basis for 'Discard'. Red
        # lines preserved: does NOT touch anything once 'saved' (the user wrote to flash), and
        # never touches flash at all if there's no snapshot.
        # (_clear_sensor sets _snapshot to None before calling this, to opt out of this restore.)
        restored = False
        if self._session_started and self._stage != "saved" and self._snapshot is not None:
            restored = self._restore_snapshot()
        self._timer.stop()
        self.btn_start.setText("Start")
        self.samples.clear()
        self._scatter_xy.clear()
        self.cov.reset()
        self.cov_gate.reset()
        for code in self.face_bins:
            self.face_bins[code] = set()
        self._center = self._gain = None
        self._poll_ctr = 0
        self._hsi_status = None
        self._converged = False
        self._r47_history.clear()
        self._r47_ts = 0.0
        self._snapshot = None
        self._session_started = False
        self.btn_fit.setEnabled(False)
        self.btn_apply.setEnabled(False)
        self.btn_viz.setEnabled(False)
        self.lbl_result.setText("—")
        for row in self._move_rows:
            self._set_row(row, 0.0, False)
            row["widget"].setVisible(True)            # bring back rows hidden (completed) before the reset
        self.lbl_move_count.setText(f"Completed: 0 / {len(MOVES)} orientations")
        self.bar_moves.setValue(0)
        self._cur_face = None
        self.lbl_active_hint.setText("")
        self.lbl_remaining.setText("")
        self.wheel.refresh([], None)
        self._refresh_hsi_panel()
        self._refresh_metrics()
        self._set_stage("collect")
        if restored:
            self.lbl_now.setText("↩ Reset — the sensor's pre-session calibration was written back "
                                 "(RAM; flash untouched).")

    def _collect(self) -> None:
        d = self.vn.get_data()
        if d is None:
            return
        # If the link drops, get_data() keeps returning the LAST packet and the dedup gate
        # (below) silently returns -> the wizard FREEZES and the user rotates the sensor for
        # nothing. Extend the same freshness contract already used on the write paths (_still_ok)
        # to collection too.
        st = self.vn.stats()
        son = st.get("last_update")
        if (not st.get("connected", True)) or son is None or (time.time() - son) > STALE_MAX_AGE_S:
            self.lbl_now.setText("⚠ Data stream STOPPED (link dropped?) — collection paused. "
                                 "It will resume where it left off once the stream returns.")
            return
        if getattr(self, "_last_ts", None) == d.timestamp:
            return                      # don't count the same packet twice (if the timer outpaces the sensor)
        self._last_ts = d.timestamp
        # NOTE: the snapshot is NOT taken here. `_start_session` captures it synchronously
        # before any writes; leaving it to the tick let a verified write's readback overwrite
        # the cache and poison the snapshot with identity (see the _capture_snapshot docstring).
        # The sensor's CURRENT orientation (from gravity) — for the live hint (independent of motion).
        self._cur_face = gravity_face((d.accel_x, d.accel_y, d.accel_z))
        mag = (d.mag_x, d.mag_y, d.mag_z)
        # FINITENESS GATE: NaN/Inf slip through both parsers (strtof accepts "nan"; in binary, a
        # corrupt 4 bytes can decode to a valid NaN float). A single NaN sample entering
        # `self.samples` PERMANENTLY poisons the fit, and LinAlgError's message would show the
        # user the WRONG diagnosis ("insufficient coverage") — rotating the sensor more fixes
        # nothing. Keep the poison out at the gate.
        if not all(map(math.isfinite, mag)):
            self._nonfinite_skipped += 1
            self.lbl_now.setText(
                f"Invalid mag data (NaN/Inf) — sample skipped "
                f"(total {self._nonfinite_skipped}). Could be line noise or a corrupt frame.")
            return
        if abs(mag[0]) + abs(mag[1]) + abs(mag[2]) < 1e-9:
            self.lbl_now.setText("Mag data is 0 — make sure you're in ASCII ($VNYMR) mode.")
            return

        gyro_mag = math.sqrt(d.gyro_x ** 2 + d.gyro_y ** 2 + d.gyro_z ** 2)
        moving = gyro_mag > MOVE_GATE_GYRO
        if moving:
            # Upper bound: this used to be the dashboard's ONE unbounded buffer. A long
            # calibration session (or a forgotten, still-open wizard) would grow memory
            # indefinitely. Once MAX_FIT_SAMPLES is reached, keep going by THINNING: coverage
            # is preserved and fit quality doesn't degrade (the fit already saturates at >=300
            # samples; measured: center error <=0.002 at 2000 samples), but memory stays flat.
            if len(self.samples) < MAX_FIT_SAMPLES:
                self.samples.append(mag)
            elif len(self.samples) % 2 == 0:        # thinning: replace every other one
                self.samples[self._decimate_idx % MAX_FIT_SAMPLES] = mag
                self._decimate_idx += 1
            self.cov.add(mag)
            self.cov_gate.add((d.accel_x, d.accel_y, d.accel_z))   # gravity direction -> a fit gate independent of hard-iron
            pr = self.cov.project(mag)
            if pr:
                self._scatter_xy.append(pr)
                if len(self._scatter_xy) > SCATTER_KEEP:
                    del self._scatter_xy[0]
            face = gravity_face((d.accel_x, d.accel_y, d.accel_z))
            if face in self.face_bins:
                b = self.cov.bin_of(mag)
                if b is not None:
                    self.face_bins[face].add(b)

        # Periodic polling in onboard mode. Reg 47 (the solution) is the REAL metric — read every
        # round and its history kept; its stability determines convergence. Reg 46 is only
        # requested on v2.x hardware and only for the info panel (absent from this FW's ICD ->
        # don't generate $VNERR,08 noise).
        if self.mode == "onboard":
            self._poll_ctr += 1
            if self._poll_ctr % STATUS_POLL_EVERY == 0:
                self._send(self.vn.link.read_register(Reg.HSI_CALCULATED))
                if self._caps.has_hsi_status_reg:
                    self._send(self.vn.link.hsi_status())
                r47 = self._fresh_register(Reg.HSI_CALCULATED)
                if r47 is not None:
                    sol = decode_mag_cal(r47[0])
                    if sol is not None and (not self._r47_history or r47[1] > self._r47_ts):
                        self._r47_history.append(sol)
                        self._r47_ts = r47[1]
                        del self._r47_history[:-8]       # the last 8 readings are enough (stability window)
            if self._caps.has_hsi_status_reg:
                r = self._fresh_register(Reg.HSI_STATUS)  # only if it arrived THIS session (not stale)
                if r is not None:
                    self._hsi_status = decode_hsi_status(r[0])

        self._refresh_status(moving)
        self.wheel.refresh(self._scatter_xy, mag)

    # ── Status ────────────────────────────────────────────────────
    def _set_row(self, row, progress, done) -> None:
        row["done"] = done
        row["bar"].setValue(int(progress * 100))
        row["dot"].setText("✓" if done else "○")
        row["dot"].setStyleSheet(f"color:{C_DONE if done else C_MUTED}; font-size:14px;")
        row["text"].setStyleSheet(f"color:{C_TEXT if done else C_MUTED}; font-size:11px;")

    def _mark_active(self, row, active: bool) -> None:
        """Highlights the FIRST of the remaining moves (do this now) with '▶'; other remaining
        rows stay dim. Called AFTER _set_row (overrides the dot/text style of an incomplete row)."""
        if active:
            row["dot"].setText("▶")
            row["dot"].setStyleSheet(f"color:{C_ACCENT}; font-size:15px; font-weight:bold;")
            row["text"].setStyleSheet(f"color:{C_TEXT}; font-size:12px; font-weight:700;")
        else:
            row["dot"].setText("○")
            row["dot"].setStyleSheet(f"color:{C_MUTED}; font-size:14px;")
            row["text"].setStyleSheet(f"color:{C_MUTED}; font-size:11px;")

    def _update_active_hint(self, active_code) -> None:
        """Layer 3a — live hint for the active step: compares the sensor's CURRENT orientation
        (_cur_face) against the target. If it's already correct, says 'rotate'; otherwise says 'move it there'."""
        if active_code is None:                          # all done
            self.lbl_active_hint.setText("")
            return
        target = FACE_LABELS.get(active_code, active_code)
        cur = getattr(self, "_cur_face", None)
        if cur == active_code:
            self.lbl_active_hint.setText("✓ Correct orientation — now rotate slowly, a full turn (360°) over ~5 s.")
            self.lbl_active_hint.setStyleSheet(f"color:{C_DONE}; font-size:11px; font-weight:700; padding-top:2px;")
        elif cur in FACE_LABELS:
            self.lbl_active_hint.setText(f"Currently: {FACE_LABELS[cur]} -> target: {target}. Move the sensor there.")
            self.lbl_active_hint.setStyleSheet(f"color:{C_MUTED}; font-size:11px; padding-top:2px;")
        else:
            self.lbl_active_hint.setText(f"Move the sensor to the '{target}' position (currently in-between).")
            self.lbl_active_hint.setStyleSheet(f"color:{C_MUTED}; font-size:11px; padding-top:2px;")

    def _refresh_metrics(self) -> None:
        cov = self.cov_gate.coverage()          # coverage % + bar = orientation coverage (matches the fit gate)
        self.bar_cov.setValue(int(cov * 100))
        self.lbl_metrics.setText(
            f"Coverage: {cov * 100:4.1f}%   Samples: {len(self.samples)}   "
            f"Cells: {int(self.cov.covered_mask().sum())}/{self.cov.n_az * self.cov.n_el}   "
            f"(% = orientation coverage; wheel/cells = mag cloud)")

    def _refresh_hsi_panel(self) -> None:
        st = self._hsi_status
        if not self._caps.has_hsi_status_reg:
            # Reg 46 isn't in this firmware's ICD -> the bin boxes never fill in. Showing empty
            # boxes would be a "hang on, it's filling in" lie; show the real metric instead.
            n = len(self._r47_history)
            sol = self._r47_history[-1] if self._r47_history else None
            d = mag_cal_max_delta(sol, IDENTITY_MAG_CAL) if sol is not None else 0.0
            self.lbl_resid.setText(f"Reg 47 solution: {n} reading(s)   |Δidentity|={d:.4f}   "
                                   f"(Reg 46 doesn't exist on this firmware — convergence measured via Reg 47)")
            self.bar_resid.setValue(int(100 * min(1.0, len(self._r47_history) / 4.0)))
            self.lbl_bin_count.setText("REG 47")
            for b in self._bin_lbls:
                b.setVisible(False)
            return
        if st is None:
            self.lbl_resid.setText("AvgResidual: —   NumMeas: 0")
            self.bar_resid.setValue(0)
            self.lbl_bin_count.setText("BIN")
            for b in self._bin_lbls:
                b.setVisible(True)
                b.setStyleSheet(f"color:{C_CELL}; font-size:16px;")
            return
        res = st["avg_residual"]
        self.lbl_resid.setText(f"AvgResidual: {res:.4f}   NumMeas: {st['num_meas']}   "
                               f"(target < {ONBOARD_RESID_OK})")
        # bar from residual 0.12->0 (low is good -> a full bar is good)
        pct = max(0.0, min(1.0, 1.0 - res / 0.12))
        self.bar_resid.setValue(int(pct * 100))
        # Reg 46's bin count can vary by firmware (7/8; see the note in registers.decode_hsi_status).
        # Label reflects the REAL count and shows ONLY that many boxes (hiding the rest) -> if the
        # sensor reports 7 bins, an empty 8th box doesn't cause confusion, and there's no fixed indexing.
        bins = st["bins"]
        n = min(len(bins), len(self._bin_lbls))
        self.lbl_bin_count.setText(f"{len(bins)} BIN")
        for i, b in enumerate(self._bin_lbls):
            b.setVisible(i < n)
            if i < n:
                filled = bins[i] >= ONBOARD_BIN_MIN
                b.setStyleSheet(f"color:{C_DONE if filled else C_CELL}; font-size:16px;")

    def _refresh_status(self, moving: bool) -> None:
        remaining = []
        done_count = 0
        active_marked = False
        active_code = None
        for (code, _), row in zip(MOVES, self._move_rows):
            n = len(self.face_bins[code])
            done = n >= FACE_BINS_NEEDED
            self._set_row(row, min(n / FACE_BINS_NEEDED, 1.0), done)
            if done:
                done_count += 1
                row["widget"].setVisible(False)          # HIDE completed rows -> the list shows only what's left
            else:
                row["widget"].setVisible(True)
                remaining.append(FACE_LABELS.get(code, code))
                if not active_marked:
                    active_code = code
                self._mark_active(row, active=not active_marked)   # first remaining = "▶ do this NOW"
                active_marked = True
        self._update_active_hint(active_code)
        # Counter + progress bar (mentor request: 'moves done X / Y')
        self.lbl_move_count.setText(f"Completed: {done_count} / {len(MOVES)} orientations")
        self.bar_moves.setValue(done_count)
        if remaining:
            self.lbl_remaining.setStyleSheet(f"color:{C_MUTED}; font-size:11px;")
            self.lbl_remaining.setText(f"{len(remaining)} orientations remaining: " + ", ".join(remaining))
        else:
            self.lbl_remaining.setStyleSheet(f"color:{C_DONE}; font-size:11px; font-weight:600;")
            self.lbl_remaining.setText("✓ All orientations done — once coverage is sufficient, 'Fit' / 'Apply ▸ Preview'.")

        if self.mode == "onboard":
            self._refresh_hsi_panel()
            self._eval_onboard(moving, remaining)
        else:
            self._eval_offline(moving, remaining)
        self._refresh_metrics()

    def _eval_onboard(self, moving, remaining) -> None:
        """Onboard convergence — Reg 47 STABILITY + PC coverage (NOT dependent on Reg 46).

        Three conditions are checked together:
          1. Orientation coverage is sufficient (have we shown the sensor every direction — PC side, no ICD needed),
          2. The Reg 47 solution has MOVED OFF identity (onboard actually computed something),
          3. Reg 47 has SETTLED across consecutive readings (no longer changing -> converged).
        (3) alone without (2) is misleading: an HSI that never ran also shows a perfectly "stable" identity solution.
        """
        cov = self.cov_gate.coverage()
        cov_ok = cov >= ONBOARD_MIN_COVERAGE
        sol = self._r47_history[-1] if self._r47_history else None
        moved_off_identity = (sol is not None
                              and mag_cal_max_delta(sol, IDENTITY_MAG_CAL) > HSI_STABLE_TOL)
        stable = hsi_solution_converged(self._r47_history)
        self._converged = cov_ok and moved_off_identity and stable
        self.btn_apply.setEnabled(self._converged and self._stage == "collect")
        elapsed = time.time() - self._session_t0 if self._session_t0 else 0.0
        if not moving:
            self.lbl_now.setText("Rotate the sensor slowly… (it's converging its own solution)")
        elif self._converged:
            self.lbl_now.setText("✓ Sensor converged (Reg 47 solution has settled) — "
                                 "you can press 'Apply ▸ Preview'.")
        elif elapsed > ONBOARD_TIMEOUT_S:
            self.lbl_now.setText(f"⚠ Not converging after {int(elapsed)} s. There may be magnetic "
                                 "noise — move the sensor away from metal/magnets, or try the "
                                 "'Offline ellipsoid fit' method instead.")
        elif not cov_ok:
            self.lbl_now.setText(f"Keep rotating — coverage {cov * 100:.0f}% "
                                 f"(target >= {ONBOARD_MIN_COVERAGE * 100:.0f}%)"
                                 + (f", remaining: {remaining[0]}" if remaining else ""))
        elif not moved_off_identity:
            self.lbl_now.setText("Coverage is done but the sensor's solution (Reg 47) is still the "
                                 "identity matrix — onboard HSI may not be running; check the Reg 44 mode.")
        else:
            self.lbl_now.setText("Converging… keep rotating until the sensor's solution (Reg 47) settles.")

    def _eval_offline(self, moving, remaining) -> None:
        cov = self.cov_gate.coverage()          # fit GATE = orientation (accel) coverage (C-M2)
        ready = cov >= MIN_COVERAGE_FIT and len(self.samples) >= MIN_SAMPLES_FIT
        self.btn_fit.setEnabled(ready and self._stage == "collect")
        if ready:
            self.lbl_now.setText("✓ Coverage is sufficient — you can press 'Fit'.")
        elif self.hybrid:
            # In hybrid mode the user isn't rotating anything -> telling them to "rotate the
            # sensor" would be misleading; progress depends on the recording playing. If the
            # recording finishes and coverage is still low, the recording itself is insufficient.
            done = getattr(self.vn.transport, "finished", False)
            if done:
                self.lbl_now.setText(f"Recording finished but coverage is insufficient ({cov * 100:.0f}% < "
                                     f"{MIN_COVERAGE_FIT * 100:.0f}%, samples {len(self.samples)}/"
                                     f"{MIN_SAMPLES_FIT}) — this recording isn't suitable for a fit. "
                                     "Lower --replay-speed or capture a more thorough recording.")
            else:
                self.lbl_now.setText(f"Playing back the recording… coverage {cov * 100:.0f}% "
                                     f"(target >= {MIN_COVERAGE_FIT * 100:.0f}%)")
        elif not moving:
            self.lbl_now.setText("Rotate the sensor slowly… (collecting raw data)")
        elif remaining:
            self.lbl_now.setText(f"Keep rotating — remaining: {remaining[0]}")
        else:
            self.lbl_now.setText("Fill in the gaps with a figure-8 motion, then press 'Fit'.")

    # ── Offline fit ──────────────────────────────────────────────
    def _fit(self) -> None:
        self.btn_viz.setEnabled(False)      # keep visualization disabled on a failed fit
        if len(self.samples) < MIN_SAMPLES_FIT:
            self.lbl_result.setText(f"Not enough samples ({len(self.samples)} < {MIN_SAMPLES_FIT}).")
            return
        pts = np.asarray(self.samples, dtype=float)
        try:
            self._center, self._gain, fit_info = mag_calibration_report(pts)
        except np.linalg.LinAlgError:
            # Coverage isn't the only cause: invalid (NaN/Inf) data also produces a singular
            # system. If the message unconditionally said "insufficient coverage", the user would
            # rotate the sensor more and it would never fix anything, since the problem is in the
            # data — so the two are reported separately.
            self.lbl_result.setText("Fit failed — data is invalid or coverage is insufficient. "
                                    "Check the console for corrupt frames/line errors.")
            return
        # F4: a fit can be FINITE but WRONG (planar/unbalanced coverage -> not really an
        # ellipsoid, or near-singular). Since the on-screen sphericity is computed on the
        # training set, this kind of fit can look good yet still be off by a field-radius's worth
        # of hard-iron center if written to Reg 23. If it's not trustworthy, lock APPLY and explain why.
        if not fit_info["ok"]:
            self._center = self._gain = None
            self.btn_apply.setEnabled(False)
            self.btn_viz.setEnabled(False)
            self.lbl_result.setText(f"Fit is unreliable ({fit_info['reason']}) — NOT written to "
                                    "Reg 23. Rotate more evenly across all axes with a figure-8 motion.")
            return
        before = sphericity(pts) * 100
        after = sphericity(apply_calibration(pts, self._center, self._gain)) * 100
        c = self._center
        verdict = "Excellent" if after < 1.0 else "Good" if after < 2.5 else "Poor (needs more data)"
        self.lbl_result.setText(
            f"Sphericity: {before:.2f}% -> {after:.2f}%  [{verdict}]\n"
            f"Hard-iron: [{c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f}] Gauss")
        self.btn_apply.setEnabled(after < 5.0 and self._stage == "collect")
        self.btn_viz.setEnabled(True)       # successful fit -> the result can be visualized

    def _show_result_viz(self) -> None:
        """Shows the before/after fit point cloud in a separate window (isolated; doesn't touch the flow)."""
        if self._center is None or self._gain is None or len(self.samples) < 3:
            return
        try:
            CalibrationResultDialog(self.samples, self._center, self._gain, parent=self).exec()
        except Exception as e:              # the visualization must never break the wizard under any condition
            self.lbl_result.setText(self.lbl_result.text() + f"\n(Could not open the visualization: {e})")

    # ── Stage 1: apply to RAM (preview) ────────────────────────
    def _apply(self) -> None:
        """Writes the calibration to the sensor's RAM (temporary) — does NOT touch flash.

        The user sees its effect on live data; then chooses 'Save' (permanent) or
        'Discard' (write the snapshot back). VN-100 model: $VNWRG = RAM, $VNWNV = flash.
        These two stages are a direct reflection of that distinction.
        """
        ok = self._apply_onboard() if self.mode == "onboard" else self._apply_offline()
        if ok:
            self._set_stage("preview")

    def _apply_onboard(self) -> bool:
        """FREEZES the converged onboard solution in RAM (Reg 47 -> Reg 23, onboard OFF).

        The DEFINITIVE path: copies the computed solution (Reg 47) into the permanent Reg 23
        and switches output to USER. If Reg 47 hasn't been READ yet this session (async
        response lag), it doesn't fall back to some uncertain weak alternative — it requests
        the solution and tells the user to try again. This keeps the button's permanent
        behavior independent of timing and REPEATABLE. No save() — this is a preview only.
        """
        r47 = self._fresh_register(Reg.HSI_CALCULATED)
        if r47 is None:
            # Solution hasn't arrived yet -> request it, keep collecting; the user retries in ~1 s.
            self._send(self.vn.link.read_register(Reg.HSI_CALCULATED))
            self.lbl_now.setText("The computed solution (Reg 47) hasn't been read yet — wait a "
                                 "second and press 'Apply ▸ Preview' again.")
            return False

        self._timer.stop()
        self.btn_start.setText("Start")
        cal = decode_mag_cal(r47[0])
        if cal is not None:
            C, B = cal
            flat = [x for row in C for x in row] + list(B)
            # VERIFIED write: if the sensor doesn't accept it (e.g. $VNERR,03), we do NOT move to preview.
            sent = (self._write_verified(Reg.MAG_CALIBRATION, *flat)            # Reg 47 -> Reg 23
                    and self._write_verified(Reg.HSI_CONTROL, HSIMode.OFF,
                                             HSIOutput.DISABLE, self.spin_rate.value()))
            how = "Reg 47 copied to Reg 23, onboard OFF"
        else:
            # Reg 47 arrived but its format couldn't be decoded -> at least freeze the solution (apply onboard).
            sent = self._write_verified(Reg.HSI_CONTROL, HSIMode.OFF, HSIOutput.ENABLE, 5)
            how = "HSI frozen (Reg 47 could not be decoded)"

        if not sent:
            # The sensor did NOT accept the write (or the readback didn't match) -> don't say 'applied'.
            self.lbl_result.setText(f"⚠ Calibration NOT APPLIED — the sensor did not verify it [{self._io_error}]. "
                                    "Try again; if it persists, check for magnetic noise/the connection.")
            self.lbl_now.setText("⚠ Could not apply — the sensor's acceptance could not be verified. "
                                 "You can press 'Start' to keep collecting.")
            return False

        self.lbl_result.setText(
            f"◐ Preview (RAM): calibration applied ({how}) — verified by READING IT BACK from the sensor.\n"
            "Watch the live data — if you like it, press 'Save (permanent)'; if not, 'Discard'. "
            "Not yet written to flash; lost on power loss.")
        self.lbl_now.setText("◐ Preview: the sensor is applying the new solution live (not yet saved).")
        return True

    def _apply_offline(self) -> bool:
        """Writes to Reg 23 (C*(m-B)) + turns onboard OFF. No save() — this is a preview only.

        The write is verified by reading it back: the sensor can reject this write with
        `$VNERR`, and that would NOT show up in the UI as anything else — the wizard doesn't
        move to preview unless the sensor confirms (see docs/protocol.md §8.2).
        """
        if self._gain is None or self._center is None:
            return False
        # ICD §3.4.1 Reg 23: m_cal = C*(m_raw - B). Fit: cal = gain*(raw - center) -> C=gain, B=center.
        C = np.asarray(self._gain, dtype=float).flatten().tolist()
        B = np.asarray(self._center, dtype=float).tolist()
        # Switch output to the user's Reg 23 solution + onboard OFF (ApplyCompensation=Disable)
        sent = (self._write_verified(Reg.MAG_CALIBRATION, *(C + B))
                and self._write_verified(Reg.HSI_CONTROL, HSIMode.OFF, HSIOutput.DISABLE,
                                         self.spin_rate.value()))
        if not sent:
            self.lbl_result.setText(f"⚠ Reg 23 NOT WRITTEN — the sensor did not verify it [{self._io_error}]. "
                                    "Press 'Apply ▸ Preview' again; if it persists, check the connection.")
            self.lbl_now.setText("⚠ Could not apply — the sensor's acceptance could not be verified.")
            return False
        self._timer.stop()
        self.btn_start.setText("Start")
        self.lbl_result.setText(self.lbl_result.text() +
                                "\n◐ Preview (RAM): written to Reg 23 and verified by READING IT "
                                "BACK, onboard OFF. If you like it, press 'Save (permanent)'; if "
                                "not, 'Discard'. NOT written to flash.")
        self.lbl_now.setText("◐ Preview: the correction is applied live (not yet saved).")
        return True

    # ── Stage 2: make permanent / discard ──────────────────────
    def _still_ok(self) -> bool:
        """Fail-closed stillness/freshness gate before WNV (UM001 §5.1.3): the sensor must be
        STILL during $VNWNV (~500 ms), otherwise the Kalman filter drifts. Same thresholds as the
        gyro tool (gyro_bias_dialog); if stillness can't be verified, the write is cancelled.

        HYBRID: `vn.get_data()` returns the RECORDING, which always looks "moving" — the
        gate would measure the wrong thing and stay closed forever ('Save' would never
        work). Measurement instead comes from `transport.live_data` (what HybridTransport
        extracts from the sensor's own $VNYMR/binary stream). If the sensor isn't
        broadcasting, stillness can't be verified -> fail-closed (a sensor sitting still on
        the desk is expected to pass)."""
        d, age = still_reference(self.vn)         # the REAL sensor in hybrid mode, the live stream otherwise
        src = "Live data from the real sensor" if self.hybrid else "Live data"
        hint = " (in hybrid mode the sensor's own $VNYMR broadcast must be on)" if self.hybrid else ""
        if d is None:
            self.lbl_now.setText(f"⚠ No {src.lower()}{hint} — stillness could not be verified, write cancelled.")
            return False
        if age is None or age > STALE_MAX_AGE_S:
            self.lbl_now.setText(f"⚠ {src} is STALE — the stream may have stopped, write cancelled.")
            return False
        gmag = math.sqrt(d.gyro_x ** 2 + d.gyro_y ** 2 + d.gyro_z ** 2)
        amag = math.sqrt(d.accel_x ** 2 + d.accel_y ** 2 + d.accel_z ** 2)
        if gmag > STILL_GYRO or not (ACCEL_LO < amag < ACCEL_HI):
            self.lbl_now.setText("⚠ Sensor is NOT STILL — hold it still and press Save again. "
                                 "($VNWNV ~500 ms; the Kalman filter drifts if it's moving.)")
            return False
        return True

    def _save(self) -> None:
        """Commits the previewed calibration to flash ($VNWNV) — permanent.

        $VNWNV can't be read back (it's a command, not a register) -> verification has two layers:
          1. BEFORE saving, confirm Reg 23 in RAM really holds the value we intend to save
             (otherwise WNV could be sent on top of a rejected write, and whatever happens to
             be in RAM at that moment — e.g. identity — gets written to flash),
          2. After sending, check whether the sensor returned a $VNERR.
        """
        if not self._still_ok():           # F5: ICD §1.3.3 — pre-WNV stillness/freshness gate
            return

        # Layer 1: is what's about to be written to flash REALLY the solution we previewed?
        t0 = time.time()
        if not self._send(self.vn.link.read_register(Reg.MAG_CALIBRATION)):
            self.lbl_now.setText(f"⚠ Could not save — Reg 23 could not be read [{self._io_error}].")
            return
        with self._busy():          # keep the GUI from freezing for up to 1 s + lock against double-clicks
            r23 = self.vn._wait_fresh_register(Reg.MAG_CALIBRATION, t0, 1.0)
        cur = decode_mag_cal(r23[0]) if r23 is not None else None
        if cur is None:
            self.lbl_result.setText(self.lbl_result.text() +
                                    "\n⚠ NOT SAVED — could not read the sensor's Reg 23, so what "
                                    "would be written to flash CANNOT BE VERIFIED. Press 'Save' again.")
            self.lbl_now.setText("⚠ Not saved — Reg 23 could not be verified.")
            return
        if mag_cal_max_delta(cur, IDENTITY_MAG_CAL) <= HSI_STABLE_TOL:
            # The sensor still has identity in Reg 23 -> the previewed solution never actually reached RAM.
            self.lbl_result.setText(self.lbl_result.text() +
                                    "\n⚠ NOT SAVED — the sensor's Reg 23 is still the IDENTITY "
                                    "MATRIX: the previewed calibration never reached the sensor. "
                                    "Saving would PERMANENTLY ERASE it. Press 'Discard' -> then "
                                    "'Apply ▸ Preview' again.")
            self.lbl_now.setText("⚠ Not saved — the sensor has NO calibration (identity matrix).")
            return

        # M-12: the identity check alone is NOT ENOUGH. The docstring says "is what's about to be
        # written to flash REALLY the solution we previewed?" but the check above only asked "is
        # it NOT identity?" — if the sensor had SOME OTHER calibration sitting there (e.g. left
        # over from a previous session, or written by onboard HSI in the meantime), that check
        # would still pass and the WRONG solution would be made permanent. Now it's compared
        # DIRECTLY against the previewed solution.
        if self._gain is not None and self._center is not None:
            previewed = ([[float(x) for x in row] for row in np.asarray(self._gain)],
                         [float(x) for x in np.asarray(self._center).ravel()])
            diff = mag_cal_max_delta(cur, previewed)
            if diff > MAG_CAL_SAVE_TOL:
                self.lbl_result.setText(
                    self.lbl_result.text() +
                    f"\n⚠ NOT SAVED — the sensor's Reg 23 is NOT the PREVIEWED solution "
                    f"(max diff {diff:.4g} > {MAG_CAL_SAVE_TOL:g}). Another write may have "
                    "happened in between (onboard HSI / another window). Repeat 'Apply ▸ Preview'.")
                self.lbl_now.setText("⚠ Not saved — the sensor's solution doesn't match the preview.")
                return

        if not self._send(self.vn.link.save()):
            # F1: $VNWNV didn't go out -> don't say 'saved'; stay in preview, let the user retry.
            self.lbl_result.setText(self.lbl_result.text() +
                                    f"\n⚠ FLASH NOT WRITTEN — $VNWNV could not be sent [{self._io_error}]. "
                                    "The preview is still in RAM; check the connection and press 'Save' again.")
            self.lbl_now.setText("⚠ Could not save (no connection?) — NOT written to flash.")
            return

        # Layer 2: did the sensor reject $VNWNV? (a flash write takes ~1 s -> wait a bit)
        # A flash write takes ~1 s; we wait 1.5 s (SHORTENING this would miss the error — same
        # source as the gyro tool). `_busy` + processEvents keeps the GUI from freezing (the
        # gyro tool already did this; it was missing here — two different standards otherwise).
        deadline = time.time() + 1.5
        errs: list[str] = []
        with self._busy():
            while time.time() < deadline:
                errs = self.vn.errors_since(t0)
                if errs:
                    break
                QtWidgets.QApplication.processEvents()
                time.sleep(0.05)
        if errs:
            self.lbl_result.setText(self.lbl_result.text() +
                                    f"\n⚠ COULD NOT WRITE TO FLASH — the sensor returned an error: {errs[-1]}. "
                                    "The preview is still in RAM; press 'Save' again.")
            self.lbl_now.setText(f"⚠ Could not save — {errs[-1]}")
            return

        self._set_stage("saved")
        # F5 Layer 2: $VNRST after WNV — applies the new calibration cleanly and, if it was
        # written while moving, clears any Kalman drift (ICD §1.3.3). The stream drops for ~1-2
        # s; the reader thread reconnects automatically (vn100.py). Save is a terminal step -> acceptable.
        self._send(self.vn.link.reset())
        self.lbl_result.setText(self.lbl_result.text() +
                                "\n✓ Saved: Reg 23 was verified BEFORE being written to flash, "
                                "$VNWNV was accepted with no error."
                                "\n(↻ $VNRST sent — the stream will briefly drop and reconnect.)")
        self.lbl_now.setText("✓ Calibration saved to flash (verified).")

    def _cancel(self) -> None:
        """Reverts the preview: returns the sensor to its pre-apply state IF POSSIBLE.
        If there's no snapshot, the sensor is NOT touched — stays on the safe side rather
        than erasing a stored calibration with a destructive identity write (warns honestly)."""
        restored = self._restore_snapshot()
        self.lbl_result.setText("—")
        self._set_stage("collect")
        if restored:
            # CRITICAL (sample-poisoning guard): the restore just wrote the OLD calibration back
            # to the sensor. The offline session assumes "all samples are RAW" — if collection
            # continued like this, new samples would carry the old calibration while the
            # existing ones are raw, and a refit would be fitted to a MIXED distribution
            # (plausible-looking but WRONG Reg 23). So before continuing, the raw-mode writes
            # (identity + HSI OFF) are sent AGAIN; the snapshot still stands, so the next
            # Discard/close still reverts to the same old state.
            if self.mode == "offline":
                raw_ok = (self._write_verified(Reg.MAG_CALIBRATION, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)
                          and self._write_verified(Reg.HSI_CONTROL, HSIMode.OFF, HSIOutput.DISABLE,
                                                   self.spin_rate.value()))
                if not raw_ok:
                    self._timer.stop()
                    self.btn_start.setText("Start")
                    self.lbl_now.setText("↩ Discarded (previous state restored) — but the raw-mode "
                                         f"write COULD NOT BE VERIFIED [{self._io_error}]. NOT "
                                         "continuing collection: a mixed (raw+corrected) sample set "
                                         "would produce a wrong fit. Check the connection, then press 'Start'.")
                    return
            self.lbl_now.setText("↩ Discarded — the sensor is back to its previous state. "
                                 "You can keep rotating and try again.")
            self._timer.start()               # back to State A: keep collecting
            self.btn_start.setText("Pause")
        else:
            self._timer.stop()
            self.btn_start.setText("Start")
            self.lbl_now.setText("↩ Discarded — the previous state COULD NOT BE WRITTEN back to "
                                 "the sensor (no snapshot, or the connection dropped; the stored "
                                 "flash calibration was preserved). The previewed solution only "
                                 "existed in RAM; the sensor will revert to its flash setting on "
                                 "power cycle. Press 'Start' to try again.")

    def _restore_snapshot(self) -> bool:
        """If a snapshot exists, writes Reg 23+44 back (True). If NOT, does NOT touch the sensor
        (False): stays on the safe side rather than erasing the user's current calibration with a
        destructive 'identity + onboard-on' write. (The snapshot can be empty if the register-read
        response never arrived.) Never touches flash under any circumstance."""
        snap = self._snapshot
        if snap and 23 in snap and 44 in snap:
            # The write-back is VERIFIED: saying 'the sensor is back to its previous state'
            # requires knowing that write was also accepted (otherwise Discard would be lying too).
            return (self._write_verified(Reg.MAG_CALIBRATION, *snap[23])
                    and self._write_verified(Reg.HSI_CONTROL, *snap[44]))
        return False

    # ── Restore the sensor's calibration to factory state (permanent) ────
    def _clear_sensor(self) -> None:
        """Reg 23 -> identity + Reg 44 -> FACTORY default + $VNWNV. Erases the STORED calibration.

        ⚠ Reg 44's factory default is VERSION-DEPENDENT — hardcoding a value while claiming
        "restored to factory state" would actually put the sensor into a non-factory state:
          FW 3.1.0.0 (ICD §3.5.1): 0,1,5 -> HSI OFF
          FW 2.1     (UM001 §8.3): 1,3,5 -> HSI ON
        So the value is taken from `capabilities`.
        """
        mode, out, _rate = self._caps.hsi_control_default
        hsi_txt = ("onboard real-time HSI OFF (factory state)" if mode == HSIMode.OFF
                   else "onboard real-time HSI ON (factory state)")
        ok = QtWidgets.QMessageBox.warning(
            self, "Clear Calibration From Sensor",
            "The sensor's magnetometer calibration will be restored to FACTORY STATE and "
            "PERMANENTLY written to flash:\n\n"
            "• Reg 23 -> identity (removes the user hard/soft-iron correction)\n"
            f"• Reg 44 -> {mode},{out},{self.spin_rate.value()} — {hsi_txt}\n"
            "• $VNWNV -> permanent write\n\n"
            "This cannot be undone. Continue?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if ok != QtWidgets.QMessageBox.Yes:
            return
        if not self._still_ok():           # F5: ICD §1.3.3 — pre-WNV stillness/freshness gate
            return
        sent = (self._write_verified(Reg.MAG_CALIBRATION, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)
                and self._write_verified(Reg.HSI_CONTROL, mode, out, self.spin_rate.value())
                and self._send(self.vn.link.save()))    # $VNWNV — make the clear permanent
        # F7: only drop the snapshot on a SUCCESSFUL clear — dropping it unconditionally would
        # mean that if the clear failed (e.g. link dropped), the pre-session snapshot is gone and
        # _reset couldn't write RAM back. On failure the snapshot SURVIVES -> _reset restores the pre-session calibration.
        if sent:
            self._send(self.vn.link.reset())   # F5 Layer 2: cleanly apply the factory state after WNV (ICD §1.3.3)
            self._snapshot = None          # so _reset's 'restore snapshot' step doesn't UNDO this
        self._reset()                      # clear the screen + stage=collect
        if sent:
            self.lbl_result.setText(f"✓ Sensor cleared (verified by reading it back): Reg 23 identity + "
                                    f"Reg 44 factory ({hsi_txt}) + $VNWNV + $VNRST.")
            self.lbl_now.setText("Cleared. Press 'Start' for a new calibration.")
        else:
            # The write wasn't verified -> don't say 'cleared' (the sensor may be UNCHANGED).
            self.lbl_result.setText(f"⚠ Clear COULD NOT BE VERIFIED [{self._io_error}] — the sensor "
                                    "may be UNCHANGED. Check the connection and try again.")
            self.lbl_now.setText("⚠ Could not clear — the sensor's acceptance could not be verified.")

    # ── Helpers: snapshot capture + stage management ──────────
    def _fresh_register(self, reg: int):
        """Returns the register response only if it arrived THIS session (ts >= _session_t0);
        None if stale/absent. Since vn._registers persists, this stops a reopened dialog from
        reading a PREVIOUS session's stale 'converged' (Reg 46) or stale solution (Reg 47)."""
        r = self.vn.get_register(reg)
        if r is not None and r[1] >= self._session_t0:
            return r
        return None

    def _capture_snapshot(self) -> None:
        """The basis for 'Discard': captures the sensor's PRE-session Reg 23+44 state, SYNCHRONOUSLY.

        ⚠ WHY SYNCHRONOUS: this reads and waits for the pre-session state right here, BEFORE any
        writes begin — if it were instead called asynchronously from a `_collect` timer tick,
        `_write_verified`'s own `$VNRRG` readback would OVERWRITE `vn._registers`, and if the tick
        ran late, the snapshot would end up holding the identity value that was just written ->
        'Discard' would ERASE the user's calibration instead of restoring it.

        The response is awaited here, before any writes start -> a cache race is structurally
        impossible. If the response never arrives, the snapshot stays EMPTY; `_restore_snapshot`
        then does NOT touch the sensor (honestly saying "couldn't undo it" beats a destructive
        identity write — the existing contract).
        """
        t0 = time.time()
        # Reads are idempotent; sent redundantly (on a noisy line, a single dropped response -> loss probability p -> ~p^2).
        for reg in (Reg.MAG_CALIBRATION, Reg.HSI_CONTROL, Reg.MAG_CALIBRATION, Reg.HSI_CONTROL):
            if not self._send(self.vn.link.read_register(reg)):
                return                    # link is down -> no snapshot (Discard will warn honestly)
        with self._busy():
            r23 = self.vn._wait_fresh_register(Reg.MAG_CALIBRATION, t0, SNAPSHOT_TIMEOUT_S)
            r44 = self.vn._wait_fresh_register(Reg.HSI_CONTROL, t0, SNAPSHOT_TIMEOUT_S)
        if r23 is not None and r44 is not None:
            self._snapshot = {23: list(r23[0]), 44: list(r44[0])}

    def _set_stage(self, stage: str) -> None:
        """Sets button visibility/enabled state based on the flow stage.
        collect=collecting/trying, preview=applied in RAM (save/discard), saved=in flash."""
        self._stage = stage
        collect = stage == "collect"
        preview = stage == "preview"
        self.btn_start.setEnabled(collect)
        self.btn_save.setEnabled(preview)
        self.btn_cancel.setEnabled(preview)
        self.btn_clear.setEnabled(not preview)   # lock out clearing while a preview is active
        if not collect:                           # in collect, _eval manages fit/apply
            self.btn_fit.setEnabled(False)
            self.btn_apply.setEnabled(False)

    def closeEvent(self, event) -> None:  # noqa: N802
        # If a preview is active (applied to RAM but NOT written to flash), silently closing the
        # window would neither save nor revert it -> a 'ghost' correction would linger on the
        # sensor. Ask the user.
        if self._stage == "preview":
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle("Preview is active")
            box.setText("The calibration was applied to the sensor's RAM but NOT saved to flash.\n\n"
                        "* Save: make it permanent ($VNWNV)\n"
                        "* Revert: return to the pre-preview state (if possible)\n"
                        "* Cancel: keep the window open")
            b_save = box.addButton("Save", QtWidgets.QMessageBox.AcceptRole)
            b_revert = box.addButton("Revert", QtWidgets.QMessageBox.DestructiveRole)
            box.addButton("Cancel", QtWidgets.QMessageBox.RejectRole)
            box.exec()
            clicked = box.clickedButton()
            if clicked is b_save:
                self._save()
                if self._stage != "saved":
                    # _save() FAILED due to stillness/freshness or a connection issue (stage
                    # stayed 'preview'). Don't close the window -> the user shouldn't think they
                    # saved and silently lose the calibration (which only exists in RAM); let the warning show.
                    event.ignore()
                    return
            elif clicked is b_revert:
                # Don't IGNORE the revert's outcome (F4): if there's no snapshot, the sensor
                # wasn't touched -> warn the user so they know the 'ghost correction' is still in RAM.
                if not self._restore_snapshot():
                    QtWidgets.QMessageBox.information(
                        self, "Revert",
                        "The previous state (Reg 23/44) couldn't be read, so nothing was written "
                        "back to the sensor; the stored calibration was preserved. The previewed "
                        "solution only exists in RAM — the sensor will revert to its flash setting "
                        "on power cycle.")
            else:                                   # Cancel -> stop the close
                event.ignore()
                return
        elif self._session_started and self._stage != "saved" and self._snapshot is not None:
            # F2: if the window is closed OUTSIDE of preview (e.g. during offline 'collect'), the
            # session may have pulled the sensor's RAM into RAW mode (Reg 23=identity); if a
            # snapshot exists, silently write the pre-session calibration back (without touching
            # flash). If there's none/it fails, don't touch anything.
            self._restore_snapshot()
        self._timer.stop()
        self._restore_sim_motion()
        super().closeEvent(event)
