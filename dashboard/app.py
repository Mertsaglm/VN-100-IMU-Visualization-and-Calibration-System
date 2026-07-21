"""
dashboard.app — VN-100 IMU Visualization and Calibration System (pyqtgraph + PySide6).

Architecture:
  data source (Transport)  ->  VN100 (reader thread)  ->  on_packet  ->  ring buffer
                                                                          | (QTimer)
                                                                          v
                                                          pyqtgraph curves + indicators

Source selection happens in run() (SimTransport / SerialTransport / ReplayTransport),
so the UI stays independent of the hardware.

Layout (ground-station panel):
  +------------------------------------------------------------------------------+
  | HEADER: logo . title        [status pill] [SOURCE][RATE][MODE][CHART]        |
  +-----------------------------------------------+----------------------------+
  |  EULER/GYRO/ACCEL/MAG time-series cards        |  3D ORIENTATION            |
  |  (each card: title + unit + legend + range)    |  LIVE CHANNELS (12)        |
  |                                                 |  CONFIGURATION (rate/mode/..)|
  |                                                 |  COMMANDS                  |
  |                                                 |  SENSOR CONSOLE (cmd/ack)  |
  +-----------------------------------------------+----------------------------+
"""
from __future__ import annotations

import csv
import math
import os
import threading
import time
from collections import deque
from html import escape as _escape

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from pyvn100 import VN100, Vn100Data, link, selfcheck
from pyvn100.transport import Transport

# ── Visual constants (dark theme — GitHub-dark derivative) ───────────
BUF_LEN = 600            # curve length (points / most recent N samples)
MAG_FLOOR_G = 0.6        # mag Y-axis minimum half-span [Gauss] (prevents flat-zero collapse)
MAG_PAD = 1.15           # mag auto-range padding factor

C_PAGE = "#0b0e13"       # page background (darkest surface beneath the cards)
C_PANEL = "#0e141b"      # card background
C_PANEL2 = "#11181f"     # embedded input/button background
C_BORDER = "#1c242d"     # card border
C_BORDER2 = "#2a333d"    # input/button border
C_TEXT = "#e6edf3"
C_MUTED = "#8b949e"
C_DIM = "#6e7681"
C_ACCENT = "#58a6ff"     # accent (blue)
C_ACCENTBTN = "#1f6feb"  # selected segment/dropdown
C_GREEN = "#3fb950"
C_GREENBTN = "#238636"
C_RED = "#f85149"

# Channel colors (shared between charts and indicators)
C_YAW, C_PITCH, C_ROLL = "#58a6ff", "#3fb950", "#f85149"
C_GX, C_GY, C_GZ = "#d2a8ff", "#ffa657", "#79c0ff"
C_AX, C_AY, C_AZ = "#ff7b72", "#7ee787", "#e3b341"
C_MX, C_MY, C_MZ = "#39c5cf", "#db61a2", "#bc8cff"   # magnetometer (doesn't clash with the 9 colors above)

# ── Output-rate presets — the user PICKS from a list, never types a value (UM001) ──
#   The list is limited to rates the STM32->PC ST-Link VCP link (fixed 115200 baud, 8N1,
#   ~11.5 KB/s) can ACTUALLY carry: ASCII $VNYMR ~126 B/frame -> ~90 Hz ceiling; binary
#   42 B/frame -> ~270 Hz ceiling. Above that ceiling the relay drops frames (even if the
#   sensor produces them, they don't fully reach the PC) -> the displayed rate would be a
#   lie, so those values are NOT kept in the list.
#   (The sensor itself accepts 100/200 as ADOF, and 400/800 via the binary divisor; the
#    bottleneck is the LINK — for a higher rate, raise the USART6+VCP baud instead; see
#    docs/protocol.md §8.)
#   ASCII (ADOF / reg 7): the link-carryable subset of the values the device accepts.
ASCII_HZ = [1, 2, 4, 5, 10, 20, 25, 40, 50]
#   Binary (reg 75 RateDivisor): output Hz = 800 / integer divisor; link-carryable values.
BINARY_HZ = [10, 20, 40, 50, 80, 100, 200]

pg.setConfigOption("background", C_PANEL)
pg.setConfigOption("foreground", C_DIM)
pg.setConfigOptions(antialias=True)

# ── App-wide stylesheet (cards, buttons, dropdown, console, dialogs) ──
QSS = """
QWidget#central { background: %(page)s; }
QDialog { background: %(panel)s; }
QLabel { color: %(text)s; font-size: 12px; }
QLabel#h { color: %(text)s; font-weight: 700; font-size: 11px; letter-spacing: 1.5px; }
QLabel#sub { color: %(dim)s; font-size: 10px; letter-spacing: 1px; }
QLabel#dim { color: %(dim)s; font-size: 10px; }
QFrame#card { background: %(panel)s; border: 1px solid %(border)s; border-radius: 10px; }
QFrame#chip { background: %(panel)s; border: 1px solid %(border)s; border-radius: 8px; }
QFrame#pill { background: %(panel)s; border: 1px solid %(border)s; border-radius: 16px; }
QFrame#inset { background: %(page)s; border: 1px solid %(border)s; border-radius: 8px; }
QToolTip { background: %(panel2)s; color: %(text)s; border: 1px solid %(border2)s; padding: 4px; }
QPushButton { background: %(panel2)s; color: %(text)s; border: 1px solid %(border2)s;
              border-radius: 6px; padding: 6px 10px; font-size: 11px; }
QPushButton:hover { border-color: %(accent)s; }
QPushButton:pressed { background: %(page)s; }
QPushButton#apply { background: %(greenbtn)s; color: #ffffff; border: 1px solid %(green)s; font-weight: 600; }
QPushButton#apply:hover { background: %(green)s; }
QPushButton#danger { color: %(red)s; border-color: #5c2b2b; }
QPushButton#danger:hover { background: %(red)s; color: %(page)s; }
QFrame#seg { background: %(page)s; border: 1px solid %(border2)s; border-radius: 7px; }
QPushButton#segbtn { background: transparent; color: %(dim)s; border: none; border-radius: 5px;
                     padding: 5px 6px; font-size: 10px; font-weight: 700; letter-spacing: 0.5px; }
QPushButton#segbtn:checked { background: %(accentbtn)s; color: #ffffff; }
QPushButton#segbtn:hover:!checked { color: %(text)s; }
QComboBox { background: %(page)s; color: %(text)s; border: 1px solid %(border2)s;
            border-radius: 6px; padding: 5px 8px; font-size: 11px; }
QComboBox:hover { border-color: %(accent)s; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView { background: %(panel)s; color: %(text)s; border: 1px solid %(border2)s;
                              selection-background-color: %(accentbtn)s; outline: none; }
QPlainTextEdit#console { background: %(page)s; color: %(text)s; border: 1px solid %(border)s;
                         border-radius: 8px; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: transparent; width: 9px; margin: 2px; }
QScrollBar::handle:vertical { background: %(border2)s; border-radius: 4px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: %(dim)s; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
QGroupBox { border: 1px solid %(border)s; border-radius: 8px; margin-top: 8px;
            padding-top: 8px; color: %(text)s; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: %(muted)s; }
QProgressBar { background: %(page)s; border: 1px solid %(border2)s; border-radius: 5px;
               text-align: center; color: %(text)s; height: 14px; }
QProgressBar::chunk { background: %(accentbtn)s; border-radius: 4px; }
QSpinBox { background: %(page)s; color: %(text)s; border: 1px solid %(border2)s;
           border-radius: 6px; padding: 3px 6px; }
""" % {
    "page": C_PAGE, "panel": C_PANEL, "panel2": C_PANEL2, "border": C_BORDER,
    "border2": C_BORDER2, "text": C_TEXT, "muted": C_MUTED, "dim": C_DIM,
    "accent": C_ACCENT, "accentbtn": C_ACCENTBTN, "green": C_GREEN,
    "greenbtn": C_GREENBTN, "red": C_RED,
}


def _maxabs(arrs) -> float:
    """|max| of the finite values across the concatenated arrays; NaN if all are NaN (no warning)."""
    v = np.concatenate(arrs)
    v = v[np.isfinite(v)]
    return float(np.max(np.abs(v))) if v.size else float("nan")


def _fmt_range(m: float) -> str:
    return f"±{m:.1f}" if np.isfinite(m) and m > 0 else "—"


class _RingBuffers:
    """Thread-safe sliding-window buffers (reader thread writes, GUI reads)."""

    KEYS = ("yaw", "pitch", "roll", "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz")

    def __init__(self, n: int = BUF_LEN):
        self._lock = threading.Lock()
        # Start filled with NaN: the portion before data arrives isn't drawn (no warm-up spike)
        self._d = {k: deque([float("nan")] * n, maxlen=n) for k in self.KEYS}

    def push(self, d: Vn100Data) -> None:
        with self._lock:
            self._d["yaw"].append(d.yaw)
            self._d["pitch"].append(d.pitch)
            self._d["roll"].append(d.roll)
            self._d["gx"].append(d.gyro_x)
            self._d["gy"].append(d.gyro_y)
            self._d["gz"].append(d.gyro_z)
            self._d["ax"].append(d.accel_x)
            self._d["ay"].append(d.accel_y)
            self._d["az"].append(d.accel_z)
            self._d["mx"].append(d.mag_x)
            self._d["my"].append(d.mag_y)
            self._d["mz"].append(d.mag_z)

    def snapshot(self) -> dict:
        with self._lock:
            return {k: np.fromiter(v, dtype=float) for k, v in self._d.items()}


# ── 3D orientation visualization (dependency-free — pure QPainter, NO OpenGL) ──
def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Body-box corners — a thin PCB-like rectangle (symmetric; frame direction lives in the triad)
_BOX = np.array([[sx * 1.3, sy * 0.9, sz * 0.28]
                 for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7),      # z edges
          (0, 2), (1, 3), (4, 6), (5, 7),      # y edges
          (0, 4), (1, 5), (2, 6), (3, 7)]      # x edges
_VIEW = _rot_x(np.deg2rad(-24)) @ _rot_y(np.deg2rad(32))   # fixed 3/4 camera
# VN-100 body frame <-> mesh frame. MUST match gl_view._MOUNT exactly
# (tests/test_orientation_axes.py checks this). Columns = mesh direction of each sensor axis;
# rationale and measurements are at the top of the definition in gl_view.py (connector at
# mesh -Y, top face at mesh +Z, UM001 §2.6.1 Rugged diagram: +X away from connector, +Z down).
_MOUNT = np.array([[0.0, 1.0,  0.0],
                   [1.0, 0.0,  0.0],
                   [0.0, 0.0, -1.0]])


class _Orientation3D(QtWidgets.QWidget):
    """3D body box + axes that rotates with yaw/pitch/roll (pure QPainter)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 170)
        self._ypr = (0.0, 0.0, 0.0)
        self._live = False

    def set_orientation(self, yaw, pitch, roll, live=True):
        self._ypr = (float(yaw), float(pitch), float(roll))
        self._live = live
        self.update()

    def paintEvent(self, ev):  # noqa: N802 (Qt override)
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        qp.fillRect(0, 0, w, h, QtGui.QColor(C_PANEL))
        cx, cy = w / 2.0, h / 2.0
        scale = min(w, h) * 0.32

        yaw, pitch, roll = (np.deg2rad(a) for a in self._ypr)
        R = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
        M = _VIEW @ (_MOUNT @ R @ _MOUNT.T)        # correct into the VN-100 body frame (+Z down)

        def scr(p):
            return QtCore.QPointF(cx + scale * p[0], cy - scale * p[1])

        # Ground grid (horizontal reference) — a slight sense of depth
        grid_pen = QtGui.QPen(QtGui.QColor(C_BORDER2))
        grid_pen.setWidth(1)
        qp.setPen(grid_pen)
        for g in (-1.0, -0.5, 0.0, 0.5, 1.0):
            p0 = _VIEW @ np.array([g, -1.0, -0.32])
            p1 = _VIEW @ np.array([g, 1.0, -0.32])
            qp.drawLine(scr(p0), scr(p1))
            p2 = _VIEW @ np.array([-1.0, g, -0.32])
            p3 = _VIEW @ np.array([1.0, g, -0.32])
            qp.drawLine(scr(p2), scr(p3))

        verts = (M @ _BOX.T).T
        pen = QtGui.QPen(QtGui.QColor(C_MUTED if self._live else C_BORDER2))
        pen.setWidth(2)
        qp.setPen(pen)
        for a, b in _EDGES:
            qp.drawLine(scr(verts[a]), scr(verts[b]))

        # Body axes (VN-100, UM001 §2.6.1): X forward (red), Y right/toward connector (green),
        # Z DOWN/into the device (blue). In mesh coordinates the sensor axes are
        # _MOUNT . [unit vector] = [+X, -Y, -Z].
        axes = (M @ (_MOUNT @ (np.eye(3) * 1.7)).T).T
        for vec, col in zip(axes, (C_ROLL, C_PITCH, C_YAW)):
            pen = QtGui.QPen(QtGui.QColor(col))
            pen.setWidth(3)
            qp.setPen(pen)
            qp.drawLine(QtCore.QPointF(cx, cy), scr(vec))
        qp.end()


class _Segmented(QtWidgets.QFrame):
    """Two/three-option segmented control (a clear on/off toggle instead of a dropdown)."""

    changed = QtCore.Signal(int)

    def __init__(self, options, parent=None):
        super().__init__(parent)
        self.setObjectName("seg")
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(2)
        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)
        for i, text in enumerate(options):
            b = QtWidgets.QPushButton(text)
            b.setObjectName("segbtn")
            b.setCheckable(True)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            self._group.addButton(b, i)
            lay.addWidget(b, 1)
        self._group.button(0).setChecked(True)
        self._group.idClicked.connect(self.changed.emit)

    def current(self) -> int:
        return self._group.checkedId()

    def set_current(self, i: int) -> None:
        b = self._group.button(i)
        if b is not None:
            b.setChecked(True)


class DashboardWindow(QtWidgets.QMainWindow):
    """Main window: header + 4 time-series charts + 3D orientation + controls + console."""

    def __init__(self, vn: VN100, source_label: str = "", link_mode: str | None = None):
        super().__init__()
        self.vn = vn
        self.buffers = _RingBuffers()
        self._x = np.arange(BUF_LEN)
        self._source_label = source_label
        self._link_mode = link_mode     # None=auto-detect; 'bridge'/'direct'=forced manually

        # data-rate measurement
        self._last_pkt = 0
        self._last_t = time.perf_counter()
        self._rate = 0.0

        # CSV logging state
        self._csv_file = None
        self._csv_writer = None
        self._log_lock = threading.Lock()
        self._log_rows = 0
        self._log_t0 = 0.0
        self._log_path = None
        self._log_size = 0
        self._log_error = None          # set if the reader thread hits a CSV write error (disk full, etc.)
        self._was_connected = True      # so a connection-state change is only printed to the console once
        self._last_tx_line = None       # _log_tx: don't re-log the same TX line back-to-back (HSI poll)

        self.setWindowTitle("VN-100 IMU Visualization and Calibration System")
        self.resize(1400, 900)
        self.setMinimumSize(1120, 700)
        self.setStyleSheet(QSS)
        self._build_ui(source_label)

        # write packets from the reader thread into the buffer (+ optional CSV logging)
        self.vn.on_packet = self._on_packet
        # Log EVERY command sent (main window + dialogs + selfcheck) through one central hook —
        # without it, dialog writes (gyro bias/calibration) would never show up in the console.
        self.vn.on_tx = self._log_tx

        # Chart refresh mode — user-selectable: "data" (matches VN-100 rate) | "fps60" (fixed 60).
        # Data is always collected in full regardless (reader thread is separate); this only
        # controls the DRAWING rate.
        self._graph_mode = "data" if self.seg_graph.current() == 0 else "fps60"
        self._graph_hz = 60.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update_plots)
        self._refresh_graph_rate()
        self._timer.start()

        # Automatically read the sensor's IDENTITY (Reg 1/2/4) on startup: until Reg 4 is read,
        # `capabilities` returns known=False and version-gated buttons (e.g. Tare) stay fail-open
        # ($VNTAR doesn't exist on FW 3.1.0.0 -> a silent $VNERR,04). Not tied only to 'Bring-up
        # Check' — without this, the capability map would never fill in during normal use.
        # 600 ms delay: let the reader thread + link detection settle first.
        QtCore.QTimer.singleShot(600, self._probe_identity)

    def _probe_identity(self) -> None:
        """Silent identity read at startup: Reg 1/2/4 -> let the capability map reflect reality."""
        if not getattr(self.vn.transport, "writable", True):
            return                       # replay: commands never reach the sensor, don't bother
        self._identity_since = time.time()
        selfcheck.request_reads(self.vn)
        # Let the responses land in the reader thread's cache, then refresh firmware-dependent buttons.
        QtCore.QTimer.singleShot(1200, self._on_identity_ready)

    def _on_identity_ready(self) -> None:
        caps = selfcheck.capabilities(self.vn)
        if caps.known:
            self._log_console(f"sensor identity read — {caps.note()}", "note")
        else:
            self._log_console("could not read sensor identity (no Reg 4 response) — version-dependent "
                              "gates are running on assumptions; confirm with 'Bring-up Check'", "note")
        self._sync_tare_button()

    # ════════════════════════════════════════════════════════════
    #   UI setup
    # ════════════════════════════════════════════════════════════
    def _build_ui(self, source_label: str) -> None:
        central = QtWidgets.QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(12, 10, 12, 12)
        outer.setSpacing(10)

        outer.addWidget(self._build_header(source_label))

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        outer.addLayout(body, 1)

        body.addWidget(self._build_charts(), 1)
        body.addWidget(self._build_sidebar(), 0)

    # ── Header ───────────────────────────────────────────────────
    def _build_header(self, source_label: str) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(4, 0, 2, 0)
        h.setSpacing(12)

        logo = QtWidgets.QLabel("◈")
        logo.setStyleSheet(f"color:{C_ACCENT}; font-size:26px;")
        h.addWidget(logo)

        tblock = QtWidgets.QVBoxLayout()
        tblock.setSpacing(0)
        title = QtWidgets.QLabel("VN-100 VISUALIZATION AND CALIBRATION SYSTEM")
        title.setStyleSheet(f"color:{C_TEXT}; font-size:14px; font-weight:800; letter-spacing:1.0px;")
        # The sub-label reflects the selected link topology (not static; it's the auto-detect result).
        topo = ("STM32 NUCLEO-F722ZE   ·   STM BRIDGE" if self.vn.link.mode == link.BRIDGE
                else "VN-100   ·   DIRECT USB-TTL")
        sub = QtWidgets.QLabel(f"{topo}   ·   IMU BRING-UP")
        sub.setObjectName("sub")
        tblock.addWidget(title)
        tblock.addWidget(sub)
        h.addLayout(tblock)

        h.addStretch(1)

        h.addWidget(self._build_pill())
        self.chip_source = self._chip("DATA SOURCE", self._short_source(source_label), width=168)
        self.chip_source.setToolTip(source_label or "—")
        self.chip_rate = self._chip("RATE", "—", width=76)
        self.chip_mode = self._chip("MODE", "—", width=94)
        self.chip_chart = self._chip("CHART", "—", width=84)
        for c in (self.chip_source, self.chip_rate, self.chip_mode, self.chip_chart):
            h.addWidget(c)
        return bar

    def _build_pill(self) -> QtWidgets.QWidget:
        f = QtWidgets.QFrame()
        f.setObjectName("pill")
        h = QtWidgets.QHBoxLayout(f)
        h.setContentsMargins(13, 4, 15, 4)
        h.setSpacing(9)
        self.pill_dot = QtWidgets.QLabel("●")
        self.pill_dot.setStyleSheet(f"color:{C_DIM}; font-size:12px;")
        tb = QtWidgets.QVBoxLayout()
        tb.setSpacing(0)
        self.pill_state = QtWidgets.QLabel("STARTING")
        self.pill_state.setStyleSheet(f"color:{C_MUTED}; font-size:11px; font-weight:800; letter-spacing:1px;")
        self.pill_link = QtWidgets.QLabel("connecting…")
        self.pill_link.setObjectName("sub")
        tb.addWidget(self.pill_state)
        tb.addWidget(self.pill_link)
        h.addWidget(self.pill_dot)
        h.addLayout(tb)
        return f

    def _chip(self, caption: str, value: str = "—", width: int = 90) -> QtWidgets.QFrame:
        f = QtWidgets.QFrame()
        f.setObjectName("chip")
        f.setFixedWidth(width)
        v = QtWidgets.QVBoxLayout(f)
        v.setContentsMargins(10, 5, 10, 6)
        v.setSpacing(1)
        cap = QtWidgets.QLabel(caption)
        cap.setObjectName("sub")
        val = QtWidgets.QLabel(value)
        val.setStyleSheet(f"color:{C_TEXT}; font-size:13px; font-weight:700;")
        v.addWidget(cap)
        v.addWidget(val)
        f.value = val   # for updating from outside
        return f

    # ── Left: chart cards ─────────────────────────────────────
    def _build_charts(self) -> QtWidgets.QWidget:
        host = QtWidgets.QWidget()
        col = QtWidgets.QVBoxLayout(host)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(10)

        card_e, self.p_euler, self.rng_euler = self._chart_card(
            "EULER ANGLES", "deg · YPR",
            [("YAW", C_YAW), ("PITCH", C_PITCH), ("ROLL", C_ROLL)])
        card_g, self.p_gyro, self.rng_gyro = self._chart_card(
            "GYROSCOPE", "rad/s · body rates",
            [("Gx", C_GX), ("Gy", C_GY), ("Gz", C_GZ)])
        card_a, self.p_accel, self.rng_accel = self._chart_card(
            "ACCELEROMETER", "m/s² · specific force",
            [("Ax", C_AX), ("Ay", C_AY), ("Az", C_AZ)])
        card_m, self.p_mag, self.rng_mag = self._chart_card(
            "MAGNETOMETER", "Gauss · body field",
            [("Mx", C_MX), ("My", C_MY), ("Mz", C_MZ)])
        for c in (card_e, card_g, card_a, card_m):
            col.addWidget(c, 1)

        # Curves
        self.c_yaw = self.p_euler.plot(pen=pg.mkPen(C_YAW, width=2))
        self.c_pitch = self.p_euler.plot(pen=pg.mkPen(C_PITCH, width=2))
        self.c_roll = self.p_euler.plot(pen=pg.mkPen(C_ROLL, width=2))
        self.p_euler.setYRange(-180, 180)
        self.p_euler.getAxis("left").setTicks([[(v, str(v)) for v in (-180, -90, 0, 90, 180)]])
        self.rng_euler.setText("±180°")

        self.c_gx = self.p_gyro.plot(pen=pg.mkPen(C_GX, width=1.6))
        self.c_gy = self.p_gyro.plot(pen=pg.mkPen(C_GY, width=1.6))
        self.c_gz = self.p_gyro.plot(pen=pg.mkPen(C_GZ, width=1.6))
        self.p_gyro.enableAutoRange(axis="y")

        self.c_ax = self.p_accel.plot(pen=pg.mkPen(C_AX, width=1.6))
        self.c_ay = self.p_accel.plot(pen=pg.mkPen(C_AY, width=1.6))
        self.c_az = self.p_accel.plot(pen=pg.mkPen(C_AZ, width=1.6))
        self.p_accel.enableAutoRange(axis="y")

        self.c_mx = self.p_mag.plot(pen=pg.mkPen(C_MX, width=1.6))
        self.c_my = self.p_mag.plot(pen=pg.mkPen(C_MY, width=1.6))
        self.c_mz = self.p_mag.plot(pen=pg.mkPen(C_MZ, width=1.6))
        self.p_mag.setYRange(-MAG_FLOOR_G, MAG_FLOOR_G)   # NOT autorange — update_plots drives it with a floor clamp
        return host

    def _chart_card(self, title: str, unit: str, series):
        card = QtWidgets.QFrame()
        card.setObjectName("card")
        v = QtWidgets.QVBoxLayout(card)
        v.setContentsMargins(12, 9, 12, 10)
        v.setSpacing(6)

        head = QtWidgets.QHBoxLayout()
        head.setSpacing(8)
        t = QtWidgets.QLabel(title)
        t.setObjectName("h")
        u = QtWidgets.QLabel(unit)
        u.setObjectName("sub")
        head.addWidget(t)
        head.addWidget(u)
        head.addStretch(1)
        for name, col in series:
            head.addWidget(self._legend_chip(name, col))
        rng = QtWidgets.QLabel("")
        rng.setStyleSheet(f"color:{C_DIM}; font-size:10px; font-family:Consolas,monospace; margin-left:6px;")
        head.addWidget(rng)
        v.addLayout(head)

        p = pg.PlotWidget()
        p.setBackground(C_PANEL)
        p.showGrid(x=True, y=True, alpha=0.10)
        p.setMenuEnabled(False)
        p.setMouseEnabled(False, False)
        p.hideButtons()
        p.setXRange(0, BUF_LEN - 1, padding=0)
        ax, ay = p.getAxis("bottom"), p.getAxis("left")
        for a in (ax, ay):
            a.setPen(pg.mkPen(C_BORDER2))
            a.setTextPen(pg.mkPen(C_DIM))
        ay.setWidth(46)
        ax.setHeight(20)
        v.addWidget(p, 1)
        return card, p, rng

    def _legend_chip(self, name: str, col: str) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        l = QtWidgets.QHBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(5)
        dash = QtWidgets.QLabel()
        dash.setFixedSize(14, 3)
        dash.setStyleSheet(f"background:{col}; border-radius:1px;")
        txt = QtWidgets.QLabel(name)
        txt.setStyleSheet(f"color:{C_MUTED}; font-size:10px; font-weight:700; letter-spacing:1px;")
        l.addWidget(dash)
        l.addWidget(txt)
        return w

    # ── Right: scrollable side panel ────────────────────────────
    def _build_sidebar(self) -> QtWidgets.QWidget:
        host = QtWidgets.QWidget()
        side = QtWidgets.QVBoxLayout(host)
        side.setContentsMargins(0, 0, 4, 0)
        side.setSpacing(10)
        side.addWidget(self._build_orient_card())
        side.addWidget(self._build_channels_card())
        side.addWidget(self._build_config_card())
        side.addWidget(self._build_commands_card())
        side.addWidget(self._build_console_card(), 1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(host)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(380)
        return scroll

    def _card(self, title: str, subtitle: str = ""):
        """Returns an empty titled card -> (frame, content-layout)."""
        card = QtWidgets.QFrame()
        card.setObjectName("card")
        v = QtWidgets.QVBoxLayout(card)
        v.setContentsMargins(12, 10, 12, 12)
        v.setSpacing(8)
        head = QtWidgets.QHBoxLayout()
        t = QtWidgets.QLabel(title)
        t.setObjectName("h")
        head.addWidget(t)
        head.addStretch(1)
        if subtitle:
            s = QtWidgets.QLabel(subtitle)
            s.setObjectName("sub")
            head.addWidget(s)
        v.addLayout(head)
        return card, v

    def _build_orient_card(self) -> QtWidgets.QWidget:
        card, v = self._card("ORIENTATION", "3D body frame")
        # If OpenGL is available, a lit 3D model shaped like the VN-100; otherwise the pure-QPainter fallback.
        # Both classes offer set_orientation(yaw,pitch,roll) -> update_plots doesn't change either way.
        self.orient3d = None
        try:
            from dashboard.gl_view import Orientation3DGL, gl_available
            if gl_available():
                self.orient3d = Orientation3DGL(C_PANEL, C_ROLL, C_PITCH, C_YAW)
        except Exception:
            self.orient3d = None
        if self.orient3d is None:
            self.orient3d = _Orientation3D()          # dependency-free CPU fallback
        self.orient3d.setMinimumHeight(168)
        v.addWidget(self.orient3d)
        self.lbl_ypr = QtWidgets.QLabel("Y —   P —   R —")
        self.lbl_ypr.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ypr.setStyleSheet(f"color:{C_TEXT}; font-family:Consolas,monospace; font-size:13px;")
        v.addWidget(self.lbl_ypr)
        legend = QtWidgets.QHBoxLayout()
        legend.setSpacing(12)
        legend.addStretch(1)
        for name, col in (("X FWD", C_ROLL), ("Y RIGHT", C_PITCH), ("Z DOWN", C_YAW)):
            legend.addWidget(self._legend_chip(name, col))
        legend.addStretch(1)
        v.addLayout(legend)
        return card

    def _build_channels_card(self) -> QtWidgets.QWidget:
        card, v = self._card("LIVE CHANNELS", "instantaneous · 12 ch")
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(5)
        cols = [
            ("EULER °", [("YAW", "yaw", C_YAW), ("PIT", "pitch", C_PITCH), ("ROL", "roll", C_ROLL)]),
            ("GYRO rad/s", [("Gx", "gx", C_GX), ("Gy", "gy", C_GY), ("Gz", "gz", C_GZ)]),
            ("ACCEL m/s²", [("Ax", "ax", C_AX), ("Ay", "ay", C_AY), ("Az", "az", C_AZ)]),
            ("MAG G", [("Mx", "mx", C_MX), ("My", "my", C_MY), ("Mz", "mz", C_MZ)]),
        ]
        self._val_labels: dict[str, QtWidgets.QLabel] = {}
        for c, (cap, items) in enumerate(cols):
            capl = QtWidgets.QLabel(cap)
            capl.setObjectName("sub")
            grid.addWidget(capl, 0, c)
            for r, (nm, key, col) in enumerate(items, start=1):
                cell, val = self._chan_cell(nm, col)
                grid.addWidget(cell, r, c)
                self._val_labels[key] = val
            grid.setColumnStretch(c, 1)
        v.addLayout(grid)
        return card

    def _chan_cell(self, name: str, color: str):
        w = QtWidgets.QWidget()
        l = QtWidgets.QHBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(4)
        nm = QtWidgets.QLabel(name)
        nm.setStyleSheet(f"color:{C_MUTED}; font-size:11px;")
        val = QtWidgets.QLabel("—")
        val.setStyleSheet(f"color:{color}; font-family:Consolas,monospace; font-size:14px; font-weight:600;")
        val.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        l.addWidget(nm)
        l.addStretch(1)
        l.addWidget(val)
        return w, val

    def _build_config_card(self) -> QtWidgets.QWidget:
        card, v = self._card("CONFIGURATION")

        # CONNECTION — command framing mode: STM bridge (VN RAW $VN...) / direct USB-TTL ($VN...).
        # On connect, detect_link (VN PING->VNPONG) picks it automatically; the user can override here.
        # NOTE: this only changes the software framing — it must match the physical topology (is the STM in between?).
        self.lbl_link_active = QtWidgets.QLabel("—")
        v.addLayout(self._field_caption("CONNECTION", QtWidgets.QLabel("AUTO-DETECT"),
                                        self.lbl_link_active))
        self.seg_link = _Segmented(["STM BRIDGE", "DIRECT USB-TTL"])
        self.seg_link.setToolTip(
            "Command framing: STM BRIDGE = 'VN RAW $VN...' (via the STM32 bridge). "
            "DIRECT = raw '$VN...' (USB-TTL straight to the sensor). Chosen automatically on "
            "connect (VN PING->VNPONG); you can override it here. Must MATCH your physical wiring.")
        self.seg_link.set_current(0 if self.vn.link.mode == link.BRIDGE else 1)
        self.lbl_link_active.setText("STM" if self.vn.link.mode == link.BRIDGE else "DIRECT")
        self.seg_link.changed.connect(self._on_set_link)
        v.addWidget(self.seg_link)

        # OUTPUT MODE (presentation=ASCII / operation=Binary). Applied IMMEDIATELY on click (no
        # separate APPLY, per mentor request) — a direct action, not "select+confirm"; the Hz
        # list only updates for the new mode AFTER it's actually applied (no inconsistent state
        # in between). ORDER MATTERS: select the mode first, then pick a rate valid for it.
        self.lbl_mode_active = QtWidgets.QLabel("active —")
        v.addLayout(self._field_caption("OUTPUT MODE", QtWidgets.QLabel("REG 06/75"),
                                        self.lbl_mode_active))
        self.seg_mode = _Segmented(["ASCII", "BINARY"])
        self.seg_mode.setToolTip("ASCII: human-readable, for demos. Binary: compact, for >=200 Hz operation. "
                                 "Applied IMMEDIATELY on click (no separate APPLY).")
        self.seg_mode.changed.connect(self._on_set_mode)   # click -> apply immediately + refresh the Hz list
        v.addWidget(self.seg_mode)

        # OUTPUT DATA RATE (ASCII=Reg 07 / Binary=Reg 75) — lists valid Hz values for the selected mode
        self.lbl_rate_reg = QtWidgets.QLabel("REG 07")
        self.lbl_rate_active = QtWidgets.QLabel("active —")
        v.addLayout(self._field_caption("OUTPUT DATA RATE", self.lbl_rate_reg, self.lbl_rate_active))
        rrow = QtWidgets.QHBoxLayout()
        rrow.setSpacing(6)
        self.freq_combo = QtWidgets.QComboBox()
        self.freq_combo.setToolTip("The sensor's data output rate (not the chart refresh rate). "
                                   "Only values the device actually supports are listed.")
        btn_freq = QtWidgets.QPushButton("APPLY")
        btn_freq.setObjectName("apply")
        btn_freq.setFixedWidth(76)
        btn_freq.clicked.connect(self._on_set_freq)
        rrow.addWidget(self.freq_combo, 1)
        rrow.addWidget(btn_freq)
        v.addLayout(rrow)
        self._populate_freqs()

        # CHART REFRESH (applied instantly)
        v.addLayout(self._field_caption("CHART REFRESH", None, None))
        self.seg_graph = _Segmented(["SYNC TO DATA RATE", "FIXED 60 FPS"])
        self.seg_graph.setToolTip("How many times per second the chart is REDRAWN (not the data rate). "
                                  "At high data rates, 'Fixed 60 FPS' eases CPU load; data is still collected in full.")
        self.seg_graph.changed.connect(lambda *_: self._on_graph_mode())
        v.addWidget(self.seg_graph)

        # SESSION LOG
        self.lbl_log_file = QtWidgets.QLabel("—")
        v.addLayout(self._field_caption("SESSION LOG", None, self.lbl_log_file))
        self.btn_log = QtWidgets.QPushButton("● START LOGGING")
        self.btn_log.setObjectName("apply")
        self.btn_log.clicked.connect(self._on_toggle_log)
        v.addWidget(self.btn_log)

        stats = QtWidgets.QHBoxLayout()
        stats.setSpacing(6)
        self.stat_samples = self._mini_stat("SAMPLES")
        self.stat_size = self._mini_stat("SIZE")
        self.stat_elapsed = self._mini_stat("ELAPSED")
        for s in (self.stat_samples, self.stat_size, self.stat_elapsed):
            stats.addWidget(s, 1)
        v.addLayout(stats)
        return card

    def _field_caption(self, left: str, reg, active) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(left)
        lbl.setObjectName("sub")
        row.addWidget(lbl)
        if reg is not None:
            reg.setObjectName("dim")
            row.addWidget(reg)
        row.addStretch(1)
        if active is not None:
            active.setStyleSheet(f"color:{C_GREEN}; font-size:10px; font-weight:600;")
            row.addWidget(active)
        return row

    def _mini_stat(self, caption: str) -> QtWidgets.QFrame:
        f = QtWidgets.QFrame()
        f.setObjectName("inset")
        v = QtWidgets.QVBoxLayout(f)
        v.setContentsMargins(8, 5, 8, 6)
        v.setSpacing(1)
        cap = QtWidgets.QLabel(caption)
        cap.setObjectName("sub")
        val = QtWidgets.QLabel("—")
        val.setStyleSheet(f"color:{C_TEXT}; font-family:Consolas,monospace; font-size:13px; font-weight:700;")
        v.addWidget(cap)
        v.addWidget(val)
        f.value = val
        return f

    def _build_commands_card(self) -> QtWidgets.QWidget:
        card, v = self._card("COMMANDS")
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        btns = [
            ("Tare (zero YPR)", self._on_tare, None),
            ("Save Settings", self._on_save, None),
            ("Mag Calibration…", self._on_calibrate, None),
            ("Gyro Bias…", self._on_gyro_bias, None),
        ]
        for i, (label, handler, obj) in enumerate(btns):
            b = QtWidgets.QPushButton(label)
            if obj:
                b.setObjectName(obj)
            b.clicked.connect(handler)
            grid.addWidget(b, i // 2, i % 2)
            if label.startswith("Tare"):
                self.btn_tare = b
        self._sync_tare_button()
        v.addLayout(grid)
        # Bring-up check: one button verifies identity (Reg 1/2/4) + streaming + float-printf
        # sanity (|accel|≈9.81) -> ✓/⚠/✗ (pyvn100.selfcheck), no reasoning required from a
        # less-capable model during a bring-up session.
        self.btn_bringup = QtWidgets.QPushButton("Bring-up Check ✓")
        self.btn_bringup.setObjectName("apply")
        self.btn_bringup.setToolTip("While the sensor is connected: automatically probes model/hardware/firmware "
                                    "identity, the live stream, and float-printf sanity (|accel|≈9.81); reports "
                                    "✓/⚠/✗. Once the firmware profile is known, buttons like Tare update too.")
        self.btn_bringup.clicked.connect(self._on_bringup_check)
        v.addWidget(self.btn_bringup)
        btn_fac = QtWidgets.QPushButton("Factory Reset ($VNRFS)")
        btn_fac.setObjectName("danger")
        btn_fac.clicked.connect(self._on_factory)
        v.addWidget(btn_fac)
        return card

    def _build_console_card(self) -> QtWidgets.QWidget:
        card, v = self._card("SENSOR CONSOLE", "last cmd / ack")
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setObjectName("console")
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(300)
        self.console.setMinimumHeight(120)
        f = QtGui.QFont("Consolas")
        f.setStyleHint(QtGui.QFont.Monospace)
        f.setPointSize(9)
        self.console.setFont(f)
        v.addWidget(self.console, 1)
        self._log_console("dashboard started", "note")
        msg, kind = self._link_startup_line()      # announce the auto-detect/manual-selection result to the operator
        self._log_console(msg, kind)
        return card

    def _link_startup_line(self) -> tuple[str, str]:
        """Link-mode line printed to the console at startup (so auto-detect isn't silent).
        FINDING-1/2: manual switches logged noisily while the auto-detect result only showed up in a tiny label."""
        name = "STM BRIDGE" if self.vn.link.mode == link.BRIDGE else "DIRECT USB-TTL"
        if self._link_mode is not None:
            return f"link mode: {name} (manual: --{self._link_mode})", "note"
        if self.vn.link.mode == link.BRIDGE:
            return f"link mode: {name} (auto — VNPONG received)", "note"
        return ("link mode: DIRECT USB-TTL (auto — no VNPONG received). If you expected an STM "
                "bridge, check power/flash/cabling; you can switch manually from the CONNECTION segment.",
                "note")

    # ════════════════════════════════════════════════════════════
    #   Control callbacks (host command to sensor)
    # ════════════════════════════════════════════════════════════
    def _log_tx(self, text: str) -> None:
        """on_tx hook: logs the sent command to the console as 'tx'. Doesn't re-log the SAME
        command back-to-back (e.g. the ~0.5 s repeating HSI status poll $VNRRG,46), so the
        console isn't flooded by background polling — every distinct/changed line still shows."""
        t = text.strip()
        if not t or t == self._last_tx_line:
            return
        self._last_tx_line = t
        self._log_console(t, "tx")

    def _send(self, text: str) -> bool:
        """Sends a single command. Returns: SUCCESS (same contract as calibration_dialog._send)."""
        if not self.vn.transport.writable:
            # Replay mode: commands never reach any sensor — show honest information instead of
            # the LIE 'sent' (otherwise Tare/Save/Factory would silently appear to 'succeed' during replay).
            self._log_console("REPLAY mode — command NOT SENT: " + text.strip(), "rx-err")
            return False
        try:
            self.vn.send(text)          # on_tx (_log_tx) logs it to the console as 'tx'
            return True
        except Exception as exc:        # noqa: BLE001 — keep the GUI from crashing; report the error visibly
            self._log_console(f"WRITE FAILED: {text.strip()} [{exc}]", "rx-err")
            return False

    def _emit(self, cmds: list[str]) -> bool:
        """Sends the wire-ready command list produced by the active link (BRIDGE/DIRECT) in order.

        STOPS ON THE FIRST ERROR — same contract as calibration_dialog._send. In DIRECT mode
        `set_output_mode` is a SEQUENCE OF REGISTER writes (reg 6/7/75); continuing to send the
        rest after the first one fails would leave the sensor in a HALF-APPLIED, inconsistent
        output mode (worst case: the stream goes completely silent and the user doesn't know why).
        A partial failure is reported to the user EXPLICITLY."""
        for c in cmds:
            if not self._send(c):
                if len(cmds) > 1:
                    self._log_console(
                        "⚠ Command sequence left HALF-APPLIED — the sensor may be in an inconsistent "
                        "output mode; reselect the mode.", "rx-err")
                return False
        return True

    def _on_set_link(self, idx: int) -> None:
        """Manually switch the link mode (STM bridge <-> direct USB-TTL). Only changes the command
        framing; port/baud/transport stay the same (no reconnect needed). WARNING: if the software
        mode doesn't match the physical topology, commands will silently go unanswered (e.g.
        selecting 'STM BRIDGE' when there's no STM in between)."""
        self.vn.link = link.BridgeLink() if idx == 0 else link.DirectLink()
        name = "STM" if idx == 0 else "DIRECT"
        frame = "VN RAW $VN..." if idx == 0 else "$VN..."
        self.lbl_link_active.setText(name)
        self._log_console(f"link mode -> {name} (command framing: {frame}) — must match your physical wiring",
                          "note")

    def _populate_freqs(self) -> None:
        """Lists the VALID Hz values for the current output mode (no manual entry, per mentor request)."""
        binary = self.seg_mode.current() == 1
        values = BINARY_HZ if binary else ASCII_HZ
        default = 200 if binary else 40
        self.lbl_rate_reg.setText("REG 75" if binary else "REG 07")
        self.freq_combo.blockSignals(True)
        self.freq_combo.clear()
        for hz in values:
            self.freq_combo.addItem(f"{hz} Hz", hz)
        self.freq_combo.setCurrentIndex(values.index(default) if default in values else len(values) - 1)
        self.freq_combo.blockSignals(False)

    def _on_set_freq(self) -> None:
        hz = self.freq_combo.currentData()
        if hz is not None:
            binary = self.seg_mode.current() == 1
            self._emit(self.vn.link.set_freq(int(hz), binary=binary))
            self._refresh_graph_rate()          # if in 'data' mode, switch the chart to the same Hz too

    def _on_set_mode(self, *args) -> None:
        """Selects the output mode: ASCII (presentation) / Binary (operation). docs/protocol.md §4.3.
        Called from the segment CLICK (the changed signal passes an int -> *args). ORDER MATTERS:
        rebuild the Hz list for the new mode first (so the combo lands on a valid value), THEN apply to the sensor."""
        mode = "binary" if self.seg_mode.current() == 1 else "ascii"
        self._populate_freqs()                  # the Hz list is only updated once the mode is ACTUALLY applied
        # In DIRECT mode the output rate is also written -> the (post-populate) valid Hz for that
        # mode is passed; in BRIDGE mode the STM keeps its last freq and rate_hz is ignored (inside the link).
        self._emit(self.vn.link.set_output_mode(mode, self.freq_combo.currentData()))
        self._refresh_graph_rate()              # if in 'data' mode, refresh the chart Hz when the mode changes too

    def _sync_tare_button(self) -> None:
        """Enables/disables the Tare button based on the sensor's firmware CAPABILITY.

        $VNTAR is NOT in the FW 3.1.0.0 ICD §1.3 command list -> on that hardware the sensor
        returns `$VNERR,04` (Invalid Command) and the button does nothing. Rather than leave a
        dead button clickable, we disable it and explain WHY.
        If Reg 4 hasn't been read yet, the profile is unknown (known=False) -> the button stays
        enabled but its tooltip flags the uncertainty; it becomes definitive after 'Bring-up Check'.
        """
        btn = getattr(self, "btn_tare", None)
        if btn is None:
            return
        caps = selfcheck.capabilities(self.vn)
        if caps.known and not caps.has_tare:
            btn.setEnabled(False)
            btn.setToolTip(f"Not supported on this firmware (no $VNTAR, FW {caps.fw}) — "
                           "the Tare command isn't in the ICD §1.3 command list.")
        else:
            btn.setEnabled(True)
            btn.setToolTip("Zero the current orientation as the reference ($VNTAR)."
                           + ("" if caps.known else
                              "  ⚠ Firmware not read yet — confirm with 'Bring-up Check'; "
                              "this command does NOT exist on FW 3.x."))

    def _on_tare(self) -> None:
        caps = selfcheck.capabilities(self.vn)
        if caps.known and not caps.has_tare:
            self._log_console(f"Tare not sent: $VNTAR does NOT exist on this firmware (FW {caps.fw}) "
                              "(ICD §1.3) — the sensor would return $VNERR,04.", "rx-err")
            return
        self._emit(self.vn.link.tare())

    def _on_save(self) -> None:
        # UM001 §5.1.3: the sensor must be STILL during $VNWNV, otherwise the Kalman filter
        # drifts. Same fail-closed stillness/freshness gate + confirmation as the gyro tool —
        # without a gate, a fire-and-forget save would silently write while moving too.
        # If stillness can't be verified, the write is cancelled.
        from .gyro_bias_dialog import (STILL_GYRO, ACCEL_LO, ACCEL_HI, STALE_MAX_AGE_S,
                                       still_reference)
        # In hybrid mode (measurements from a recording) the gate must measure the REAL sensor's
        # live telemetry — the recording always looks 'moving' and would measure the wrong thing (still_reference).
        d, age = still_reference(self.vn)
        if d is None or age is None or age > STALE_MAX_AGE_S:
            self._log_console("Save cancelled: no live data / data is stale — stillness cannot be verified.", "rx-err")
            return
        gmag = math.sqrt(d.gyro_x ** 2 + d.gyro_y ** 2 + d.gyro_z ** 2)
        amag = math.sqrt(d.accel_x ** 2 + d.accel_y ** 2 + d.accel_z ** 2)
        if gmag > STILL_GYRO or not (ACCEL_LO < amag < ACCEL_HI):
            self._log_console("Save cancelled: sensor is NOT STILL — $VNWNV could throw off the "
                              "Kalman filter (UM001 §5.1.3). Hold it still and try again.", "rx-err")
            return
        ok = QtWidgets.QMessageBox.warning(
            self, "Save Settings (permanent)",
            "The current settings will be PERMANENTLY written to the sensor's flash ($VNWNV). "
            "Make sure the sensor is COMPLETELY STILL — writing while it's moving can throw off the "
            "Kalman filter.\n\nContinue?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ok != QtWidgets.QMessageBox.Yes:
            return
        self._emit(self.vn.link.save())

    def _on_factory(self) -> None:
        ok = QtWidgets.QMessageBox.warning(
            self, "Restore Factory Settings",
            "ALL sensor settings will be restored to factory defaults ($VNRFS): baud, output rate, "
            "mode, calibration — everything resets, and the current session may break.\n\nContinue?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ok == QtWidgets.QMessageBox.Yes:
            self._emit(self.vn.link.factory())

    def _on_bringup_check(self) -> None:
        """Bring-up check: triggers an identity read (Reg 1/2/4), shows the report ~1.2 s later.
        Doesn't block the GUI thread (single-shot QTimer); the reader thread caches the responses."""
        self.btn_bringup.setEnabled(False)
        self._log_console("running bring-up check… (identity Reg 1/2/4 + stream + float-printf)", "note")
        self._bringup_since = time.time()   # freshness gate: any cache entry BEFORE this moment is STALE
        if not selfcheck.request_reads(self.vn):
            self._log_console("bring-up: could NOT send read commands (no connection?)", "rx-err")
        QtCore.QTimer.singleShot(1200, self._show_bringup_report)

    def _show_bringup_report(self) -> None:
        try:
            # since: if the sensor went silent between two clicks, don't show the PREVIOUS
            # session's Reg 4 response as '✓' (stale cache) — only this request's response counts.
            results = selfcheck.run_checks(self.vn, since=getattr(self, "_bringup_since", None))
            for r in results:
                kind = {"ok": "rx-ok", "warn": "note", "fail": "rx-err", "unknown": "note"}.get(
                    r.status, "note")
                self._log_console(str(r), kind)
            # Firmware is now KNOWN -> set version-dependent buttons to match reality
            # (e.g. no $VNTAR on FW 3.x -> the Tare button disables and explains why).
            self._sync_tare_button()
            caps = selfcheck.capabilities(self.vn)
            if caps.known:
                self._log_console(f"firmware profile: {caps.note()}", "note")
            QtWidgets.QMessageBox.information(self, "Bring-up Check",
                                              selfcheck.format_report(results))
        finally:
            self.btn_bringup.setEnabled(True)

    def _on_calibrate(self) -> None:
        # If a wizard is already open, bring it to the front (opening a second one would let
        # two wizards write conflicting HSI/Reg 23/44 to the same sensor, and the old dialog's
        # 30 ms timer would be orphaned but keep polling). Guarantee a single window.
        dlg = getattr(self, "_cal_dialog", None)
        try:
            alive = dlg is not None and dlg.isVisible()
        except RuntimeError:                       # the C++ object was deleted (WA_DeleteOnClose)
            alive = False
        if alive:
            dlg.raise_(); dlg.activateWindow(); return
        from .calibration_dialog import CalibrationDialog
        self._cal_dialog = CalibrationDialog(self.vn, self)
        self._cal_dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self._cal_dialog.destroyed.connect(lambda *_: setattr(self, "_cal_dialog", None))
        self._cal_dialog.show()

    def _on_gyro_bias(self) -> None:
        dlg = getattr(self, "_gyro_dialog", None)
        try:
            alive = dlg is not None and dlg.isVisible()
        except RuntimeError:
            alive = False
        if alive:
            dlg.raise_(); dlg.activateWindow(); return
        from .gyro_bias_dialog import GyroBiasDialog
        self._gyro_dialog = GyroBiasDialog(self.vn, self)
        self._gyro_dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self._gyro_dialog.destroyed.connect(lambda *_: setattr(self, "_gyro_dialog", None))
        self._gyro_dialog.show()

    def _on_packet(self, d: Vn100Data) -> None:
        """From the reader thread: write into the buffer + (if logging is on) append a CSV row."""
        self.buffers.push(d)
        with self._log_lock:
            if self._csv_writer is not None:
                try:
                    self._csv_writer.writerow([
                        f"{d.timestamp:.6f}" if d.timestamp else "",
                        d.yaw, d.pitch, d.roll,
                        d.gyro_x, d.gyro_y, d.gyro_z,
                        d.accel_x, d.accel_y, d.accel_z,
                        d.mag_x, d.mag_y, d.mag_z,
                    ])
                    self._log_rows += 1
                except OSError as exc:
                    # Disk full / I/O error: CLOSE the log but do NOT let the exception
                    # propagate — if it did, the whole data stream (reader thread) would stop.
                    # The GUI picks up the flag instead.
                    self._log_error = str(exc)
                    try:
                        self._csv_file.close()
                    except Exception:
                        pass
                    self._csv_file = None
                    self._csv_writer = None

    def _on_toggle_log(self) -> None:
        with self._log_lock:
            if self._csv_file is not None:
                try:
                    self._csv_file.close()   # don't let a full disk / removed drive drop the GUI's slot
                except OSError as exc:
                    self._log_console(f"LOG CLOSE ERROR: {exc}", "rx-err")
                path = self._log_path
                self._csv_file = None
                self._csv_writer = None
                self.btn_log.setText("● START LOGGING")
                self.btn_log.setObjectName("apply")
                self._repolish(self.btn_log)
                self._log_console("REC stop → " + os.path.basename(path or "") + f"  ({self._log_rows})", "rec")
                return
            # The log directory is pinned to the PROJECT ROOT (if it were relative to CWD, running
            # the dashboard from another directory would drop recordings outside the project and
            # `--replay logs/...` couldn't find them — the user would think nothing was recorded).
            log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                   "logs")
            os.makedirs(log_dir, exist_ok=True)
            fname = os.path.join(log_dir, "vn100_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
            try:
                self._csv_file = open(fname, "w", newline="", encoding="utf-8")
            except OSError as exc:
                self._log_console(f"COULD NOT OPEN LOG FILE: {exc}", "rx-err")
                return
            self._log_error = None
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "yaw", "pitch", "roll",
                "gyro_x", "gyro_y", "gyro_z",
                "accel_x", "accel_y", "accel_z",
                "mag_x", "mag_y", "mag_z",
            ])
            self._log_rows = 0
            self._log_t0 = time.perf_counter()
            self._log_path = fname
            self.lbl_log_file.setText(os.path.basename(fname))
            self.btn_log.setText("● STOP LOGGING")
            self.btn_log.setObjectName("danger")
            self._repolish(self.btn_log)
            self._log_console("REC start → " + os.path.basename(fname), "rec")

    def _repolish(self, w: QtWidgets.QWidget) -> None:
        w.style().unpolish(w)
        w.style().polish(w)

    # ── Console ───────────────────────────────────────────────────
    def _log_console(self, msg: str, kind: str = "tx") -> None:
        stamp = time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
        colors = {"tx": C_ACCENT, "rx-ok": C_GREEN, "rx-err": C_RED,
                  "rec": C_ACCENT, "note": C_MUTED}
        arrows = {"tx": "→", "rx-ok": "←", "rx-err": "←", "rec": "●", "note": "·"}
        c = colors.get(kind, C_MUTED)
        a = arrows.get(kind, "·")
        tag = ""
        if kind == "rx-ok":
            tag = ' <span style="color:%s">[ACK]</span>' % C_GREEN
        elif kind == "rx-err":
            tag = ' <span style="color:%s">[ERR]</span>' % C_RED
        self.console.appendHtml(
            f'<span style="color:{C_DIM}">{stamp}</span> '
            f'<span style="color:{c}">{a}</span> '
            f'<span style="color:{C_TEXT}">{_escape(msg)}</span>{tag}')

    # ── Chart refresh rate + time axis ──────────────────────
    def _current_source_hz(self) -> float:
        """Best estimate of the current data (output) rate:
        1) sim -> transport.rate_hz (exact);
        2) real hardware -> the MEASURED packet rate (self._rate) — reflects the actual stream
           instead of the combo's SELECTED-but-not-yet-applied value (keeps the time axis accurate);
        3) no measurement yet (no data has arrived) -> the combo's selected Hz."""
        r = getattr(self.vn.transport, "rate_hz", None)
        if r:
            return float(r)
        if self._rate > 1.0:
            return float(self._rate)
        hz = self.freq_combo.currentData()
        return float(hz) if hz else 40.0

    def _refresh_graph_rate(self) -> None:
        """Sets the chart refresh period based on the selected mode:
        'data' -> the VN-100 data rate; 'fps60' -> fixed 60 FPS. (Drawing rate only; data is always collected in full.)"""
        hz = 60.0 if self._graph_mode == "fps60" else max(1.0, self._current_source_hz())
        self._graph_hz = hz
        self._timer.setInterval(max(1, round(1000.0 / hz)))
        self._apply_time_axis()

    def _apply_time_axis(self) -> None:
        """Converts the X axis into 'seconds ago' labels (based on the buffer and source Hz)."""
        hz = max(1.0, self._current_source_hz())
        n = BUF_LEN
        span = n / hz
        step = max(1, int(round(span / 6.0)))
        majors, k = [], 0
        while True:
            idx = (n - 1) - k * hz
            if idx < 0:
                break
            majors.append((idx, "0s" if k == 0 else f"-{k}s"))
            k += step
        for p in (self.p_euler, self.p_gyro, self.p_accel, self.p_mag):
            p.getAxis("bottom").setTicks([majors])

    def _on_graph_mode(self) -> None:
        self._graph_mode = "data" if self.seg_graph.current() == 0 else "fps60"
        self._refresh_graph_rate()

    # ════════════════════════════════════════════════════════════
    #   Update loop
    # ════════════════════════════════════════════════════════════
    def update_plots(self) -> None:
        d = self.buffers.snapshot()
        x = self._x

        yv = d["yaw"][-1]
        if np.isfinite(yv):
            pv, rv = d["pitch"][-1], d["roll"][-1]
            # The `live` flag was DEAD CODE: the one call site never passed it (default True)
            # -> when the connection dropped, the 3D model kept showing a FROZEN value as if it were LIVE.
            # Compute freshness here instead (same 500 ms rule as the pill; `_update_status` runs
            # AFTER this, so its value can't be relied on yet).
            son = self.vn.last_update
            fresh = (son is not None) and ((time.time() - son) * 1000.0 < 500)
            self.orient3d.set_orientation(yv, pv, rv, live=fresh)
            self.lbl_ypr.setText(
                f'<span style="color:{C_YAW}">Y</span> {yv:+.1f}°   '
                f'<span style="color:{C_PITCH}">P</span> {pv:+.1f}°   '
                f'<span style="color:{C_ROLL}">R</span> {rv:+.1f}°')

        # connect="finite": NaN (not-yet-filled) points aren't drawn
        self.c_yaw.setData(x, d["yaw"], connect="finite")
        self.c_pitch.setData(x, d["pitch"], connect="finite")
        self.c_roll.setData(x, d["roll"], connect="finite")
        self.c_gx.setData(x, d["gx"], connect="finite")
        self.c_gy.setData(x, d["gy"], connect="finite")
        self.c_gz.setData(x, d["gz"], connect="finite")
        self.c_ax.setData(x, d["ax"], connect="finite")
        self.c_ay.setData(x, d["ay"], connect="finite")
        self.c_az.setData(x, d["az"], connect="finite")

        self.rng_gyro.setText(_fmt_range(_maxabs([d["gx"], d["gy"], d["gz"]])))
        self.rng_accel.setText(_fmt_range(_maxabs([d["ax"], d["ay"], d["az"]])))

        def val(v, spec):
            return spec.format(v) if np.isfinite(v) else "—"
        for k in ("yaw", "pitch", "roll"):
            self._val_labels[k].setText(val(d[k][-1], "{:+.2f}"))
        for k in ("gx", "gy", "gz"):
            self._val_labels[k].setText(val(d[k][-1], "{:+.3f}"))
        for k in ("ax", "ay", "az"):
            self._val_labels[k].setText(val(d[k][-1], "{:+.2f}"))

        # ── Magnetometer — the binary frame carries NO mag data (binary.py decodes it to 0.0).
        #    In binary mode (or when the whole window is ~0 -> a binary-recorded replay), inform
        #    the user instead of showing a MISLEADING flat zero; in ASCII, plot the real field +
        #    a floor-clamped symmetric range.
        mmax = _maxabs([d["mx"], d["my"], d["mz"]])
        binary = (self.vn.last_fmt == "binary")
        flat = bool(np.isfinite(mmax) and mmax < 1e-9)        # ASCII but all-zero -> the source was binary
        if binary or flat:
            self.c_mx.setData([], [])
            self.c_my.setData([], [])
            self.c_mz.setData([], [])
            self.p_mag.setYRange(-MAG_FLOOR_G, MAG_FLOOR_G)
            self.rng_mag.setText("BINARY · mag needs ASCII" if binary else "mag ≈ 0 · binary recording?")
            for k in ("mx", "my", "mz"):
                self._val_labels[k].setText("—")
        else:
            self.c_mx.setData(x, d["mx"], connect="finite")
            self.c_my.setData(x, d["my"], connect="finite")
            self.c_mz.setData(x, d["mz"], connect="finite")
            hi = max(mmax * MAG_PAD, MAG_FLOOR_G) if np.isfinite(mmax) else MAG_FLOOR_G
            self.p_mag.setYRange(-hi, hi)
            # Label = instantaneous field MAGNITUDE |M| = sqrt(Mx^2+My^2+Mz^2). Health check: as
            # the sensor rotates, the components change but |M| stays roughly constant in Earth's
            # field (~0.45 G).
            mlast = np.array([d["mx"][-1], d["my"][-1], d["mz"][-1]], dtype=float)
            if np.all(np.isfinite(mlast)):
                self.rng_mag.setText(f"|M| {float(np.sqrt(mlast @ mlast)):.2f} G")
            else:
                self.rng_mag.setText("—")
            for k in ("mx", "my", "mz"):
                self._val_labels[k].setText(val(d[k][-1], "{:+.3f}"))

        self._update_status()

    def _update_status(self) -> None:
        st = self.vn.stats()
        now = time.perf_counter()
        dt = now - self._last_t
        slow_tick = dt >= 0.5
        if slow_tick:
            self._rate = (st["packets"] - self._last_pkt) / dt
            self._last_pkt = st["packets"]
            self._last_t = now

        # last_update is wall-clock based -> measure its age with the wall clock too
        age_ms = (time.time() - st["last_update"]) * 1000.0 if st["last_update"] else 9999.0
        live = age_ms < 500
        fmt = self.vn.last_fmt
        fmt_txt = {"ascii": "ASCII", "binary": "BINARY"}.get(fmt, "—")

        # Status pill — priority: serial disconnect > live > no data
        connected = st.get("connected", True)
        if connected != self._was_connected:      # log to console once per state change
            self._was_connected = connected
            self._log_console("serial link dropped — reconnecting…" if not connected
                              else "serial link re-established",
                              "rx-err" if not connected else "rx-ok")
        if not connected:
            self.pill_dot.setStyleSheet(f"color:{C_RED}; font-size:12px;")
            self.pill_state.setStyleSheet(f"color:{C_RED}; font-size:11px; font-weight:800; letter-spacing:1px;")
            self.pill_state.setText("LINK LOST")
            self.pill_link.setText("reconnecting…")
        elif live:
            self.pill_dot.setStyleSheet(f"color:{C_GREEN}; font-size:12px;")
            self.pill_state.setStyleSheet(f"color:{C_GREEN}; font-size:11px; font-weight:800; letter-spacing:1px;")
            # If measurements are coming FROM A RECORDING, saying 'STREAMING' would mislead the
            # user: the charts and the 3D model show the recording's history, not the sensor's
            # CURRENT state (the transport.data_is_recorded contract). The calibration dialog
            # already flagged this; the main window used to stay silent.
            self.pill_state.setText(
                "REPLAY" if getattr(self.vn.transport, "data_is_recorded", False) else "STREAMING")
            self.pill_link.setText(f"LINK OK · CRC {st['errors']} ERR")
        else:
            self.pill_dot.setStyleSheet(f"color:{C_RED}; font-size:12px;")
            self.pill_state.setStyleSheet(f"color:{C_RED}; font-size:11px; font-weight:800; letter-spacing:1px;")
            self.pill_state.setText("NO DATA")
            self.pill_link.setText("LINK DOWN · check the connection")

        # Header chips
        self.chip_rate.value.setText(f"{self._rate:.0f} Hz")
        self.chip_mode.value.setText(fmt_txt)
        self.chip_chart.value.setText(f"{self._graph_hz:.0f} fps")

        # Configuration 'active' indicators
        self.lbl_rate_active.setText(f"active {self._current_source_hz():.0f} Hz")
        self.lbl_mode_active.setText(f"active {fmt_txt}")

        # Session-log stats
        if self._csv_file is not None:
            self.stat_samples.value.setText(str(self._log_rows))
            if slow_tick:
                # Python's CSV buffer lags behind the disk -> flush periodically (every 0.5 s) so the size shown is accurate
                with self._log_lock:
                    if self._csv_file is not None:
                        try:
                            self._csv_file.flush()   # disk full -> OSError; don't let it drop the Qt slot
                        except OSError as exc:
                            # SAME contract as the reader-thread error path: close+reset the file,
                            # record the reason in _log_error — the block below closes REC and reports it.
                            try:
                                self._csv_file.close()
                            except OSError:
                                pass
                            self._csv_file = None
                            self._csv_writer = None
                            self._log_error = f"flush: {exc}"
                        try:
                            self._log_size = os.path.getsize(self._log_path) if self._log_path else 0
                        except OSError:
                            pass
                self.stat_size.value.setText(self._human_size(self._log_size))
                self.stat_elapsed.value.setText(f"{now - self._log_t0:.1f} s")

        # Command responses -> console: ALL of them if several arrive in one tick (no $VNERR gets overwritten)
        for text, err, _ts in self.vn.drain_responses():
            self._log_console(text, "rx-err" if err else "rx-ok")

        # If the reader thread hit a CSV write error (disk full, etc.): reset the button + notify
        if self._log_error is not None:
            werr = self._log_error
            self._log_error = None
            self.btn_log.setText("● START LOGGING")
            self.btn_log.setObjectName("apply")
            self._repolish(self.btn_log)
            self.lbl_log_file.setText("—")
            self._log_console(f"LOGGING STOPPED (write error): {werr}", "rx-err")

        if slow_tick:
            self._apply_time_axis()   # the source Hz may have changed -> refresh axis labels

    @staticmethod
    def _human_size(n: int) -> str:
        if n < 1024:
            return f"{n} B"
        if n < 1024 * 1024:
            return f"{n / 1024:.1f} KB"
        return f"{n / (1024 * 1024):.1f} MB"

    def _short_source(self, label: str) -> str:
        """Short label for the header 'DATA SOURCE' chip."""
        s = (label.split("Source:")[-1] if label and "Source:" in label else label or "—").strip()
        up = s.upper()
        if "SIM" in up:
            return "SIM · virtual VN-100"
        # HYBRID must be checked BEFORE PLAYBACK: the hybrid label can also mention "recording",
        # and showing 'PLAYBACK · CSV' would hide that commands are going to the REAL sensor.
        if "HYBRID" in up:
            return "HYBRID · log→sensor"
        if "PLAYBACK" in up:
            return "PLAYBACK · CSV"
        return (s.split("@")[0].strip() or s)   # "COM5 @ 115200" -> "COM5"

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        try:
            self.vn.stop_reader()
            with self._log_lock:
                if self._csv_file is not None:
                    self._csv_file.close()
        finally:
            super().closeEvent(event)


def run(transport: Transport, source_label: str = "", fmt: str = "ascii",
        link_mode: str | None = None) -> int:
    """Runs the dashboard with the given transport (blocks).

    link_mode: None -> auto-detect (STM bridge if VN PING->VNPONG comes back, else direct
    USB-TTL); 'bridge'/'direct' -> force manually. Detection runs synchronously BEFORE the
    reader starts -> the probe reads raw bytes directly from the transport, so the RX/parse
    path (VN100._scan) is never touched by it."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyleSheet(QSS)          # keep dialogs/QMessageBox visually consistent too
    vn = VN100(transport, fmt=fmt)
    # Determine the link mode before the reader starts (otherwise the reader would consume bytes and break the probe).
    vn.link = link.detect_link(transport, forced=link_mode)
    _how = "auto" if link_mode is None else f"manual (--{link_mode})"
    _name = "STM BRIDGE" if vn.link.mode == link.BRIDGE else "DIRECT USB-TTL"
    print(f"[DASHBOARD] Link mode: {_name} ({_how})")
    # Build the window first (wires up on_packet), THEN start the reader —
    # otherwise the first packets would arrive and be lost before the chart could receive them.
    win = DashboardWindow(vn, source_label, link_mode=link_mode)
    vn.start_reader()
    win.show()
    return app.exec()
