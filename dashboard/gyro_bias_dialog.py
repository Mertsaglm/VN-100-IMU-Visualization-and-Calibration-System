"""
dashboard.gyro_bias_dialog — gyroscope bias (static) measurement tool.

While the sensor sits completely STILL, the average gyro output = the bias
estimate (tools.calibration.gyro_bias). Samples are discarded if motion is
detected.

The VN-100 continuously compensates gyro bias with its onboard VPE filter,
but a small stationary drift can still exist at startup. This tool does two
things:
  1) Verification/characterization (no write): is the bias small, how much
     noise is there.
  2) Optional "Write to Sensor (SetGyroBias)": while the sensor is STILL,
     captures the bias via $VNSGB and makes it permanent via $VNWNV
     (docs/calibration.md §7).
"""
from __future__ import annotations

import math
import time

import numpy as np
from PySide6 import QtCore, QtWidgets

from pyvn100 import selfcheck
from tools.calibration import gyro_bias
from dashboard.app import (QSS, C_PAGE, C_PANEL2, C_BORDER,
                           C_TEXT, C_ACCENT, C_GREEN, C_RED)

STILL_GYRO = 0.15      # rad/s — above this, "not still" (bias is far below this)
MOVED_GYRO = 0.35      # rad/s — clear motion -> discard collected samples
ACCEL_LO, ACCEL_HI = 9.4, 10.2   # m/s^2 — if |accel| is outside this range, the sensor is being moved
TARGET = 500           # sample count; ~15 s (30 ms timer, while ODR >= ~33 Hz —
                       # takes longer at lower ODR due to duplicate-packet dedup)
STALE_MAX_AGE_S = 1.0  # F7: if the last packet at write time is older than this, data is STALE -> abort write
SGB_TIMEOUT_S = 1.5    # wait for the Filter Startup Gyro Bias readback after $VNSGB
WNV_WAIT_S = 1.5       # error wait after $VNWNV (flash write takes ~1 s; asking without
                       # waiting would MISS the error -> falsely report '✓' every time)
_RAD2DEG = 180.0 / math.pi

# consistent with app.py's palette (semantic names via C_GREEN/C_RED)
C_OK = C_GREEN
C_WARN = C_RED


def still_reference(vn) -> tuple:
    """Returns the (data, age[s]) pair the stillness gate must check before $VNWNV/$VNSGB.

    Kept together with the thresholds (STILL_GYRO/ACCEL_*/STALE_MAX_AGE_S) so what the
    gate checks and which thresholds it uses stay in one place. Three callers: app._on_save,
    calibration_dialog, and this tool's own write path.

    Normally the source is the live stream. In HYBRID mode (measurements from a recording,
    commands to the real sensor — pyvn100.replay.HybridTransport), `vn.get_data()` returns
    the RECORDING: a calibration recording always looks like it's "moving", so the gate
    would measure the wrong thing and stay closed forever. There, the real sensor's
    separately-tracked live telemetry is used instead.

    Treat the return as fail-closed: if data is None or age is None/stale, stillness has
    NOT been verified -> abort the write (never silently assume 'still')."""
    tp = getattr(vn, "transport", None)
    if getattr(tp, "data_is_recorded", False) and getattr(tp, "writable", False):
        return getattr(tp, "live_data", None), getattr(tp, "live_age", None)
    last = vn.last_update
    return vn.get_data(), (None if last is None else time.time() - last)


class GyroBiasDialog(QtWidgets.QDialog):
    def __init__(self, vn, parent=None):
        super().__init__(parent)
        self.vn = vn
        self.samples: list[tuple[float, float, float]] = []
        self._t_samples: list[float] = []   # arrival time [s] of each sample — BACKUP f_s calculation
        self._last_ts = None                # get_data() duplicate-packet dedup (keeps sigma/f_s accurate)
        # REAL f_s for ARW: total packets decoded by the reader thread / elapsed time (the ODR).
        # The 30 ms timer thins out sampling but NOT packet_count -> gives the true ODR (see _finalize).
        self._rate_n0: int | None = None    # packet_count at window start
        self._rate_t0: float | None = None  # monotonic time at window start
        self._prev_sim_motion = None      # temporary 'still' motion while the tool is open in sim mode
        self.setWindowTitle("Gyroscope Bias — Static Measurement")
        self.resize(460, 360)
        self.setStyleSheet(QSS + f"QDialog {{ background: {C_PAGE}; }}")
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._collect)
        self._enter_sim_still()           # keep the sensor STILL in sim mode (no-op on real hardware)

    # ── Simulated motion (sim only; no-op on real hardware) ──
    def _sim_transport(self):
        """Returns the active transport if it's a simulator; None on real hardware."""
        t = getattr(self.vn, "transport", None)
        if t is not None and hasattr(t, "set_motion") and hasattr(t, "sim"):
            return t
        return None

    def _enter_sim_still(self) -> None:
        """The sim's default 'gentle' oscillation would inflate the gyro reading, making a
        static measurement unrealistic. Switch the sim to 'still' while the tool is open
        (does NOT affect real hardware, where you hold the sensor still by hand)."""
        sim = self._sim_transport()
        if sim is None or self._prev_sim_motion is not None:
            return
        self._prev_sim_motion = sim.set_motion("still") or "gentle"

    def _restore_sim_motion(self) -> None:
        sim = self._sim_transport()
        if sim is not None and self._prev_sim_motion is not None:
            sim.set_motion(self._prev_sim_motion)
        self._prev_sim_motion = None

    def _card(self, title: str, subtitle: str = ""):
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
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 12, 14, 14)
        lay.setSpacing(11)

        # Header
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(10)
        logo = QtWidgets.QLabel("◈")
        logo.setStyleSheet(f"color:{C_ACCENT}; font-size:22px;")
        head.addWidget(logo)
        tb = QtWidgets.QVBoxLayout()
        tb.setSpacing(0)
        ttl = QtWidgets.QLabel("GYROSCOPE BIAS")
        ttl.setStyleSheet(f"color:{C_TEXT}; font-size:15px; font-weight:800; letter-spacing:1px;")
        subt = QtWidgets.QLabel("static measurement — stationary drift characterization")
        subt.setObjectName("sub")
        tb.addWidget(ttl)
        tb.addWidget(subt)
        head.addLayout(tb)
        head.addStretch(1)
        lay.addLayout(head)

        # Instructions card
        icard, ilay = self._card("HOW IT WORKS")
        info = QtWidgets.QLabel(
            "Place the sensor on a solid surface and keep it COMPLETELY STILL. 'Start' -> "
            "bias is computed once ~500 samples are collected. Samples reset automatically "
            "if the sensor moves.")
        info.setObjectName("sub")
        info.setWordWrap(True)
        ilay.addWidget(info)
        lay.addWidget(icard)

        # Measurement card
        scard, slay = self._card("MEASUREMENT")
        self.lbl_state = QtWidgets.QLabel("Ready.")
        self.lbl_state.setStyleSheet(f"color:{C_TEXT}; font-size:12px; font-weight:bold;")
        slay.addWidget(self.lbl_state)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, TARGET)
        slay.addWidget(self.bar)
        self.lbl_result = QtWidgets.QLabel("—")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setStyleSheet(
            f"color:{C_TEXT}; font-family:Consolas,monospace; font-size:12px; padding:8px 10px; "
            f"background:{C_PANEL2}; border:1px solid {C_BORDER}; border-radius:8px;")
        slay.addWidget(self.lbl_result)
        lay.addWidget(scard)

        note = QtWidgets.QLabel("The VN-100 compensates gyro bias with its own filter. This measurement is "
                                "for verification; if you want, it can be written and made permanent with "
                                "SetGyroBias while the sensor is STILL.")
        note.setObjectName("sub")
        note.setWordWrap(True)
        lay.addWidget(note)

        lay.addStretch(1)
        row = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_start.clicked.connect(self._toggle)
        btn_reset = QtWidgets.QPushButton("Reset")
        btn_reset.clicked.connect(self._reset)
        self.btn_write = QtWidgets.QPushButton("Write to Sensor (SetGyroBias)")
        self.btn_write.setObjectName("apply")
        self.btn_write.setEnabled(False)                 # only enabled after a successful static measurement
        self.btn_write.clicked.connect(self._write_to_sensor)
        row.addWidget(self.btn_start)
        row.addWidget(btn_reset)
        row.addWidget(self.btn_write)
        lay.addLayout(row)

    def _toggle(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
            self.btn_start.setText("Resume")
        else:
            if len(self.samples) >= TARGET:
                self._reset()           # start over after a completed measurement
            self._timer.start()
            self.btn_start.setText("Pause")

    def _reset(self) -> None:
        self.samples.clear()
        self._t_samples.clear()
        self._rate_n0 = self._rate_t0 = None   # reset the f_s window too
        self.bar.setValue(0)
        self.lbl_result.setText("—")
        self.lbl_state.setText("Reset.")
        self.btn_write.setEnabled(False)

    def _collect(self) -> None:
        # In HYBRID mode the measurement must also come from the REAL sensor: same source
        # as the write gate (still_reference), otherwise bias would be computed from the
        # recording's gyro and written to the real sensor.
        d, _age = still_reference(self.vn)
        if d is None:
            return
        # If the link drops, get_data() keeps returning the LAST packet and the dedup gate
        # silently returns -> the tool would appear FROZEN. Give a visible pause using the
        # same staleness threshold.
        st = self.vn.stats()
        son = st.get("last_update")
        if (not st.get("connected", True)) or son is None or (time.time() - son) > STALE_MAX_AGE_S:
            self.lbl_state.setText("⚠ Data stream STOPPED (link dropped?) — collection paused.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return
        if self._last_ts is not None and d.timestamp == self._last_ts:
            return          # don't count the same packet twice (if the timer outpaces the sensor) -> keeps sigma/f_s accurate
        self._last_ts = d.timestamp
        gmag = math.sqrt(d.gyro_x ** 2 + d.gyro_y ** 2 + d.gyro_z ** 2)
        amag = math.sqrt(d.accel_x ** 2 + d.accel_y ** 2 + d.accel_z ** 2)

        if gmag > MOVED_GYRO or not (ACCEL_LO < amag < ACCEL_HI):
            if self.samples:
                self.samples.clear()
                self._t_samples.clear()      # reset timestamps too (keeps fs/ARW consistent)
                self._rate_n0 = self._rate_t0 = None   # reset the f_s window too (motion -> start over)
                self.bar.setValue(0)
            self.lbl_state.setText("⚠ Sensor is moving — keep it still.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return
        if gmag > STILL_GYRO:
            self.lbl_state.setText("Settling…")
            return

        # If a new collection window is starting, begin the REAL f_s measurement: from this
        # point, measure ALL packets decoded by the reader (packet_count) and elapsed time -> true output rate (ODR).
        if self._rate_t0 is None:
            self._rate_n0 = int(getattr(self.vn, "packet_count", 0))
            self._rate_t0 = time.monotonic()
        self.samples.append((d.gyro_x, d.gyro_y, d.gyro_z))
        # Sample arrival time — for the BACKUP f_s (when packet_count is unavailable). Note:
        # the 30 ms timer thins this out; the primary f_s comes from packet_count (see _finalize).
        self._t_samples.append(d.timestamp if d.timestamp is not None else time.monotonic())
        self.bar.setValue(len(self.samples))
        self.lbl_state.setText(f"Collecting… {len(self.samples)}/{TARGET}")
        self.lbl_state.setStyleSheet(f"color:{C_TEXT}; font-size:12px; font-weight:bold;")
        # Live convergence: show the running bias +/- sigma as it settles (lets the user write with confidence)
        if len(self.samples) >= 20:
            arr = np.asarray(self.samples, dtype=float)
            b = arr.mean(axis=0) * _RAD2DEG      # running bias so far [deg/s]
            s = arr.std(axis=0) * _RAD2DEG       # running sigma so far [deg/s]
            self.lbl_result.setText(
                f"Running bias [deg/s]: X {b[0]:+.4f}  Y {b[1]:+.4f}  Z {b[2]:+.4f}\n"
                f"Running sigma [deg/s]: X {s[0]:.4f}  Y {s[1]:.4f}  Z {s[2]:.4f}   (write once it settles)")
        if len(self.samples) >= TARGET:
            self._finalize()

    def _finalize(self) -> None:
        self._timer.stop()
        self.btn_start.setText("Start")
        arr = np.asarray(self.samples, dtype=float)
        bias = gyro_bias(arr)                      # rad/s
        noise = arr.std(axis=0)                    # rad/s
        bd = bias * _RAD2DEG
        nd = noise * _RAD2DEG
        mag = float(np.linalg.norm(bd))
        verdict = "Good (small)" if mag < 1.0 else "High — sensor may not be warmed up"
        # Noise density (ARW) — convert sigma to noise density: sigma_density = sigma / sqrt(f_s).
        # f_s = REAL output rate (ODR): total packets decoded by the reader thread during the
        # collection window / elapsed time. CRITICAL: _collect samples on a 30 ms timer and only
        # picks up the LATEST packet (intermediate packets are thinned out above ~33 Hz ODR), BUT
        # packet_count is NOT thinned -> it gives the true ODR. (Computing f_s from arrival
        # timestamps instead would collapse to the ~33 Hz timer rate -> ARW would be INFLATED by
        # sqrt(ODR/33).) Sigma itself is unaffected by the thinning (it's correct as-is).
        # NOTE: this is not a true "Allan variance" (that requires hours of static logging); it's
        # just an honest sigma-to-density conversion.
        fs = None
        if self._rate_t0 is not None and self._rate_n0 is not None:
            dn = int(getattr(self.vn, "packet_count", 0)) - self._rate_n0
            dt = time.monotonic() - self._rate_t0
            if dn >= 2 and dt > 0:
                fs = dn / dt
        if fs is None and len(self._t_samples) >= 2:   # BACKUP: arrival timestamps if packet_count is unavailable
            dt = self._t_samples[-1] - self._t_samples[0]
            if dt > 0:
                fs = (len(self._t_samples) - 1) / dt
        # M-20: f_s measured on data replayed FROM A RECORDING is not the REAL ODR — it's a
        # PLAYBACK rate scaled by `--replay-speed` (replay.py `_speed`). At 8x playback speed,
        # f_s is measured 8x too large -> noise density appears sqrt(8) ~= 2.83x too SMALL. The
        # number has lost its physical meaning, so it's not shown at all — showing a wrong
        # number is worse than showing none (the user would compare it against the datasheet).
        is_replay = getattr(getattr(self.vn, "transport", None), "data_is_recorded", False)
        if is_replay:
            dens_txt = ("\nNoise density / ARW: — (RECORDING is being replayed; sample rate "
                        "is not the real ODR, it's scaled by --replay-speed -> calculation is meaningless)")
        elif fs and fs > 0:
            # NOTE: this quantity is NOISE DENSITY [deg/s/sqrt(Hz)], NOT ARW.
            # ARW (Angle Random Walk) has units of deg/sqrt(hr): deg/s/sqrt(Hz) * sqrt(3600 s/hr) = x60.
            # If the label combined both under one heading like "Noise density (ARW)", two
            # different quantities would be attached to the same number and comparison against
            # datasheet ARW values would be wrong — so they're shown on separate lines with
            # separate labels.
            dens = nd / (fs ** 0.5)                 # [deg/s/sqrt(Hz)]
            arw = dens * 60.0                       # [deg/sqrt(hr)]
            dens_txt = (f"\nNoise density [deg/s/sqrt(Hz)]: "
                        f"X {dens[0]:.4f}  Y {dens[1]:.4f}  Z {dens[2]:.4f}   (f_s≈{fs:.0f} Hz)"
                        f"\nARW [deg/sqrt(hr)]: X {arw[0]:.3f}  Y {arw[1]:.3f}  Z {arw[2]:.3f}")
        else:
            dens_txt = "\nNoise density: could not compute f_s (insufficient timing data)."
        self.lbl_state.setText("✓ Bias computed.")
        self.lbl_state.setStyleSheet(f"color:{C_OK}; font-size:12px; font-weight:bold;")
        self.lbl_result.setText(
            f"Bias [deg/s]  : X {bd[0]:+.4f}  Y {bd[1]:+.4f}  Z {bd[2]:+.4f}\n"
            f"Noise sigma   : X {nd[0]:.4f}  Y {nd[1]:.4f}  Z {nd[2]:.4f}  [deg/s]\n"
            f"|Bias|        : {mag:.4f} deg/s  [{verdict}]" + dens_txt)
        self.btn_write.setEnabled(True)     # static measurement succeeded -> enable writing to the sensor

    def _write_to_sensor(self) -> None:
        """Sends $VNSGB (capture bias) + $VNWNV (make permanent) while the sensor is STILL.

        $VNSGB treats the sensor's CURRENT gyro output as the bias, so stillness is
        re-verified AT WRITE TIME, fail-closed (no data, or data too stale, cancels the
        write rather than risk etching a moving/stale reading in as permanent), and the
        user is asked to confirm before the write."""
        # In HYBRID mode vn.get_data() returns the RECORDING; the gate must measure the
        # REAL sensor instead, so still_reference() is the single source of truth here too.
        d, age = still_reference(self.vn)
        if d is None:
            self.lbl_state.setText("⚠ No live data — stillness could not be verified, write cancelled. "
                                   "Check the connection and the data stream, then repeat the measurement.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return
        # F7: get_data() keeps returning the LAST packet even if the connection has stalled.
        # If that packet merely looks still but is STALE, the stillness gate could pass
        # incorrectly -> freshness is verified too.
        if age is None or age > STALE_MAX_AGE_S:
            self.lbl_state.setText("⚠ Live data is STALE (stream may have stopped) — stillness is not "
                                   "current, write cancelled. Confirm the stream is live and try again.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return
        gmag = math.sqrt(d.gyro_x ** 2 + d.gyro_y ** 2 + d.gyro_z ** 2)
        amag = math.sqrt(d.accel_x ** 2 + d.accel_y ** 2 + d.accel_z ** 2)
        if gmag > STILL_GYRO or not (ACCEL_LO < amag < ACCEL_HI):
            self.lbl_state.setText("⚠ Sensor is currently NOT STILL — write cancelled. "
                                   "Hold the sensor still and repeat the measurement.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        ok = QtWidgets.QMessageBox.question(
            self, "Write to Sensor (SetGyroBias)",
            "The measured gyro bias will be written to the sensor and PERMANENTLY saved to "
            "flash ($VNSGB + $VNWNV). Make sure the sensor is COMPLETELY STILL.\n\nContinue?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ok != QtWidgets.QMessageBox.Yes:
            return

        if not self.vn.transport.writable:
            self.lbl_state.setText("⚠ REPLAY mode — cannot write to the sensor (a recording is being replayed).")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        # $VNSGB itself can't be read back (it's a command, not a register), but the register
        # it WRITES can be: the sensor copies its current Kalman bias estimate into the Filter
        # Startup Gyro Bias register (FW3 ICD §3.3.5 -> Reg 43; FW2.1 UM001 §7.1.3 -> Reg 74 —
        # the ID depends on firmware version). Reading it back PROVES the write actually
        # happened. Critical: this dialog writes straight to FLASH in a single click (no
        # preview) — an unverified 'success' message could silently etch in a wrong bias.
        caps = selfcheck.capabilities(self.vn)
        bias_reg = caps.gyro_bias_reg
        t0 = time.time()
        try:
            for c in self.vn.link.gyro_bias():
                self.vn.send(c)
            for c in self.vn.link.read_register(bias_reg):
                self.vn.send(c)
        except Exception as exc:            # noqa: BLE001 — keep the GUI from crashing, report honestly
            self.lbl_state.setText(f"⚠ Could not send $VNSGB ({exc}) — NOT written to flash.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        r = self.vn._wait_fresh_register(bias_reg, t0, SGB_TIMEOUT_S)
        errs = self.vn.errors_since(t0)

        # ⚠ ORDER MATTERS — the error check comes BEFORE writing to flash. If $VNSGB is
        # rejected, Reg 43 is still readable (with its OLD value) -> `r` would be non-empty;
        # skipping the error check here would send $VNWNV on top of a rejected bias and make
        # the wrong value PERMANENT. This dialog writes straight to flash in a single click
        # (no preview) — there's no undo.
        if errs:
            self.lbl_state.setText(f"⚠ $VNSGB REJECTED — sensor: {errs[-1]}. "
                                   "NOT written to flash (bias unchanged); try again.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return
        if r is None:
            self.lbl_state.setText(f"⚠ $VNSGB COULD NOT BE VERIFIED — Reg {bias_reg} readback failed "
                                   f"(no response within {SGB_TIMEOUT_S:.1f} s). "
                                   "NOT written to flash; try again.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        # The bias actually stored by the sensor — show it so the user can compare against what we measured.
        try:
            stored = [float(x) for x in r[0][:3]]
        except (ValueError, IndexError):
            stored = []
        stored_txt = ("[" + ", ".join(f"{v:+.5f}" for v in stored) + "] rad/s") if stored else "?"

        t_save = time.time()
        try:
            for c in self.vn.link.save():
                self.vn.send(c)
        except Exception as exc:            # noqa: BLE001
            self.lbl_state.setText(f"⚠ Bias written to sensor but $VNWNV could not be sent ({exc}) — "
                                   "NOT PERMANENT; it will be lost on power cycle.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        # $VNWNV can't be read back (it's a command, not a register) -> the only evidence is
        # the sensor NOT returning an error. Flash writes take ~1 s; asking WITHOUT waiting
        # would miss the error and falsely report '✓' every time (same standard as
        # calibration_dialog._save — no reason to handle the same thing two different ways).
        deadline = time.time() + WNV_WAIT_S
        errs = []
        while time.time() < deadline:
            errs = self.vn.errors_since(t_save)
            if errs:
                break
            QtWidgets.QApplication.processEvents()   # keep the window from freezing
            time.sleep(0.05)
        if errs:
            self.lbl_state.setText(f"⚠ $VNWNV REJECTED — sensor: {errs[-1]}. Bias is in RAM only "
                                   "(will be lost on power cycle); try 'Write to Sensor' again.")
            self.lbl_state.setStyleSheet(f"color:{C_WARN}; font-size:12px; font-weight:bold;")
            return

        self.lbl_state.setText(f"✓ Written and verified — sensor Reg {bias_reg}: {stored_txt} "
                               f"($VNWNV error-free for {WNV_WAIT_S:.1f} s -> permanent).")
        self.lbl_state.setStyleSheet(f"color:{C_OK}; font-size:12px; font-weight:bold;")
        self.btn_write.setEnabled(False)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._timer.stop()
        self._restore_sim_motion()
        super().closeEvent(event)
