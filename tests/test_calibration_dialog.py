"""
Calibration wizard dialog-layer tests (no hardware, Qt offscreen).

`tests/test_verified_write.py` only exercises the `VN100.write_register_verified`
primitive — not enough, since the dialog can still misuse a correct primitive (e.g.
`_send_all` returning True while `_apply_offline` still advances to preview on a
rejected write). These tests verify the dialog layer uses the primitive correctly:

  1. A rejected sensor write must not advance the dialog to preview.
  2. The 'Discard' snapshot must not be poisoned by the verified write's readback
     (verification must not force `_capture_snapshot` to store the identity matrix).
  3. Onboard convergence works without Reg 46, which doesn't exist in FW 3.1.0.0.

Skipped if PySide6 isn't available; verify.py runs it with PySide6 in .venv.
"""
import time

import pytest

pytest.importorskip("PySide6")

from pyvn100 import VN100, SimTransport                      # noqa: E402
from pyvn100.registers import Reg                            # noqa: E402

_GOOD_CAL = (1.05, 0.01, -0.02, 0.01, 0.97, 0.03, -0.02, 0.03, 1.01, 0.12, -0.08, 0.03)
_IDENTITY = (1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)


@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _live_vn(transport=None, motion="calibration"):
    tp = transport or SimTransport(rate_hz=50.0, motion=motion, noise=False)
    vn = VN100(tp)
    vn.start_reader()
    time.sleep(0.2)
    return vn


def _dialog(vn, mode="offline"):
    from dashboard.calibration_dialog import CalibrationDialog
    dlg = CalibrationDialog(vn)
    dlg.mode = mode
    return dlg


class _RejectingTransport(SimTransport):
    """Rejects Reg 23 writes (simulated checksum failure); reads still work.

    Mimics a field failure mode: the PC's checksum is correct but the wire receives
    corrupted data (an STM32 ISR conflict drops a byte), so the sensor returns $VNERR,03.
    """

    def write(self, text):
        if "VNWRG,23" in text:
            self._buf.extend(b"$VNERR,03*72\r\n")
            return len(text)
        return super().write(text)


def test_rejected_write_does_not_advance_to_preview(qapp):
    """If the sensor rejects Reg 23, the dialog must not report success.

    If `_send_all` returned True and `_apply` called `_set_stage("preview")`, 'Save' would
    enable and the user could write a rejected calibration to flash. Verifies that path
    stays closed."""
    vn = _live_vn(_RejectingTransport(rate_hz=50.0, motion="calibration", noise=False))
    dlg = _dialog(vn)
    dlg._gain = [[1.02, 0, 0], [0, 0.98, 0], [0, 0, 1.01]]
    dlg._center = [0.03, -0.01, 0.02]

    ok = dlg._apply_offline()

    assert ok is False, "sensor rejected the write but the dialog reported success"
    assert dlg._stage != "preview", "must not advance to preview on a rejected write"
    assert not dlg.btn_save.isEnabled(), "'Save' must not be enabled — there's nothing to save"
    assert "NOT APPLIED" in dlg.lbl_result.text() or "NOT WRITTEN" in dlg.lbl_result.text()
    vn.stop_reader()


def test_accepted_write_advances_to_preview(qapp):
    """Sanity check: preview must still work on the happy path (guards against an overly strict rejection check)."""
    vn = _live_vn()
    dlg = _dialog(vn)
    dlg._gain = [[1.02, 0, 0], [0, 0.98, 0], [0, 0, 1.01]]
    dlg._center = [0.03, -0.01, 0.02]

    assert dlg._apply_offline() is True
    dlg._set_stage("preview")
    assert dlg.btn_save.isEnabled()
    vn.stop_reader()


def test_snapshot_is_not_poisoned(qapp):
    """REGRESSION: a verified write's readback must not overwrite the snapshot.

    `_write_verified` sends its own `$VNRRG,23`, overwriting `vn._registers[23]` with the
    identity matrix. If `_capture_snapshot` ran from the timer tick (old behavior), it
    would store that identity matrix, and 'Discard' would erase the user's calibration
    instead of restoring it."""
    vn = _live_vn()
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_GOOD_CAL)   # pre-existing on the sensor
    vn.write_register_verified(Reg.HSI_CONTROL, 1, 3, 5)

    dlg = _dialog(vn)
    dlg._start_session()          # snapshot must be captured here, BEFORE the writes

    assert dlg._snapshot is not None, "snapshot capture failed -> 'Discard' won't work"
    snap23 = [float(x) for x in dlg._snapshot[23]]
    assert abs(snap23[0] - 1.05) < 1e-3, f"snapshot POISONED (identity matrix stored): {snap23[:3]}"
    assert [int(x) for x in dlg._snapshot[44][:3]] == [1, 3, 5]
    vn.stop_reader()


def test_discard_restores_previous_calibration(qapp):
    """End to end: start session -> switch to raw mode -> Discard -> sensor must revert to the previous calibration."""
    vn = _live_vn()
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_GOOD_CAL)
    vn.write_register_verified(Reg.HSI_CONTROL, 1, 3, 5)

    dlg = _dialog(vn)
    dlg._start_session()                                   # Reg 23 -> identity (raw mode)
    mid = [float(x) for x in vn.get_register(Reg.MAG_CALIBRATION)[0][:3]]
    assert abs(mid[0] - 1.0) < 1e-6, "session should have switched to raw mode"

    assert dlg._restore_snapshot() is True
    time.sleep(0.15)
    restored = [float(x) for x in vn.get_register(Reg.MAG_CALIBRATION)[0][:3]]
    assert abs(restored[0] - 1.05) < 1e-3, f"Discard did not restore the calibration: {restored}"
    vn.stop_reader()


def test_discard_without_snapshot_does_not_touch_sensor(qapp):
    """Fail-safe: if no snapshot was captured, do nothing rather than write a destructive identity matrix."""
    vn = _live_vn()
    dlg = _dialog(vn)
    dlg._snapshot = None
    sent = []
    vn.on_tx = sent.append
    assert dlg._restore_snapshot() is False
    assert not any("VNWRG" in c for c in sent), "must NOT write to the sensor when there's no snapshot"
    vn.stop_reader()


def test_onboard_converges_without_reg46(qapp):
    """Reg 46 doesn't exist in FW 3.1.0.0, so the wizard must converge using Reg 47 stability +
    coverage instead. The old code checked `all(b >= 3 for b in bins)`; bins were always 0,
    so the gate never opened."""
    from pyvn100.registers import decode_mag_cal
    vn = _live_vn()
    dlg = _dialog(vn, mode="onboard")
    assert dlg._caps.has_hsi_status_reg is False           # this FW doesn't have Reg 46

    # Run onboard HSI + fill coverage and Reg 47 history (in place of the wizard's own tick)
    for c in vn.link.hsi_reset(rate=5):
        vn.send(c)
    time.sleep(1.2)                                        # sim tumble -> let the solution converge
    for _ in range(5):
        for c in vn.link.read_register(Reg.HSI_CALCULATED):
            vn.send(c)
        time.sleep(0.12)
        solution = decode_mag_cal(vn.get_register(Reg.HSI_CALCULATED)[0])
        dlg._r47_history.append(solution)

    class _FullCoverage:
        @staticmethod
        def coverage():
            return 1.0
    dlg.cov_gate = _FullCoverage()
    dlg._stage = "collect"
    dlg._eval_onboard(moving=True, remaining=[])

    assert dlg._converged is True, "convergence not detected without Reg 46"
    assert dlg.btn_apply.isEnabled()
    vn.stop_reader()


# ── FLASH-WRITE CHAIN (_save) and ONBOARD apply path ──────────────────────
# The guard primitives (write_register_verified, mag_cal_max_delta, still_reference) are
# tested individually, but without testing their composition/order inside _save /
# _apply_onboard, a save-to-flash command could still follow a rejected write.

def _tx_log(vn):
    sent = []
    vn.on_tx = sent.append
    return sent


def _save_cmds(vn):
    """Flash-save command text for the current link mode: 'VN SAVE' in BRIDGE mode, '$VNWNV'
    in DIRECT mode. Derived from the link itself — hardcoding '$VNWNV' would never match in
    BRIDGE mode, so the test would pass tautologically."""
    return [c.strip() for c in vn.link.save()]


def _flash_was_written(vn, sent):
    expected = _save_cmds(vn)
    return any(any(e in c for e in expected) for c in sent)


def test_onboard_rejected_write_does_not_advance_to_preview(qapp):
    """Same contract as the offline branch, which had this test while the onboard branch didn't.

    If the sensor rejects the Reg 23 write (_RejectingTransport -> $VNERR,03),
    `_apply_onboard` must return False, not advance to preview, and leave 'Save' disabled."""
    from pyvn100.registers import decode_mag_cal
    vn = _live_vn(_RejectingTransport(rate_hz=50.0, motion="calibration", noise=False))
    dlg = _dialog(vn, mode="onboard")

    # Ensure Reg 47 (computed solution) has been read this session — otherwise `_apply_onboard`
    # takes the "solution not ready yet" branch and the rejection path is never exercised.
    for c in vn.link.hsi_reset(rate=5):
        vn.send(c)
    time.sleep(1.2)
    for _ in range(5):
        for c in vn.link.read_register(Reg.HSI_CALCULATED):
            vn.send(c)
        time.sleep(0.12)
    assert dlg._fresh_register(Reg.HSI_CALCULATED) is not None, "precondition: Reg 47 was not read"
    assert decode_mag_cal(vn.get_register(Reg.HSI_CALCULATED)[0]) is not None

    ok = dlg._apply_onboard()

    assert ok is False, "sensor rejected Reg 23 but the onboard branch reported success"
    assert dlg._stage != "preview", "must not advance to preview on a rejected write"
    assert not dlg.btn_save.isEnabled(), "'Save' must not be enabled"
    assert "NOT APPLIED" in dlg.lbl_result.text()
    vn.stop_reader()


def test_save_does_not_write_flash_on_identity(qapp):
    """If Reg 23 on the sensor is identity, $VNWNV must not be sent — otherwise whatever is
    in RAM (identity) gets written to flash, permanently erasing the user's previous
    calibration."""
    vn = _live_vn(SimTransport(rate_hz=50.0, motion="still", noise=False))
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_IDENTITY)   # identity is sitting on the sensor
    dlg = _dialog(vn)
    dlg._stage = "preview"
    sent = _tx_log(vn)

    dlg._save()

    assert not _flash_was_written(vn, sent), \
        "flash was written despite seeing identity — the user's calibration would be ERASED"
    assert "NOT SAVED" in dlg.lbl_result.text()
    assert dlg._stage != "saved"
    vn.stop_reader()


def test_save_does_not_report_saved_if_VNWNV_send_fails(qapp):
    """If $VNWNV can't be sent on the wire, must NOT report 'saved' (fail-closed)."""
    class _WnvDropper(SimTransport):
        # In BRIDGE mode the flash command is 'VN SAVE', in DIRECT mode it's '$VNWNV' — cover both.
        def write(self, text):
            if "VN SAVE" in text or "VNWNV" in text:
                raise OSError("port disconnected")     # _send must catch this and return False
            return super().write(text)

    vn = _live_vn(_WnvDropper(rate_hz=50.0, motion="still", noise=False))
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_GOOD_CAL)    # not identity -> layer-1 check passes
    dlg = _dialog(vn)
    dlg._stage = "preview"

    dlg._save()

    assert dlg._stage != "saved", "an unsent $VNWNV was still counted as 'saved'"
    assert "NOT WRITTEN" in dlg.lbl_result.text()
    vn.stop_reader()


def test_save_does_not_send_VNWNV_while_sensor_is_moving(qapp):
    """`_still_ok` fail-closed gate: $VNWNV takes ~500 ms; the Kalman filter drifts while moving."""
    vn = _live_vn(SimTransport(rate_hz=50.0, motion="calibration", noise=False))  # rotating
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_GOOD_CAL)
    dlg = _dialog(vn)
    dlg._stage = "preview"
    sent = _tx_log(vn)

    dlg._save()

    assert not _flash_was_written(vn, sent), "flash was written while the sensor was moving"
    assert "NOT STILL" in dlg.lbl_now.text()
    vn.stop_reader()


def test_onboard_identity_solution_not_counted_as_converged(qapp):
    """If HSI never ran, Reg 47 is identity and looks perfectly 'stable' -> must not show a false checkmark."""
    from pyvn100.registers import IDENTITY_MAG_CAL
    vn = _live_vn()
    dlg = _dialog(vn, mode="onboard")
    dlg._r47_history = [IDENTITY_MAG_CAL] * 5              # stable BUT no real solution

    class _FullCoverage:
        @staticmethod
        def coverage():
            return 1.0
    dlg.cov_gate = _FullCoverage()
    dlg._stage = "collect"
    dlg._eval_onboard(moving=True, remaining=[])

    assert dlg._converged is False, "identity solution was counted as 'converged' — false positive"
    vn.stop_reader()


def test_NaN_mag_sample_does_not_enter_samples(qapp):
    """Ingest layer: a single NaN sample entering `self.samples` permanently poisons the
    fit — rotating more won't fix it. The gate must drop the sample, increment the
    counter, and show the user an accurate diagnosis."""
    import math
    from pyvn100.types import Vn100Data

    vn = _live_vn(SimTransport(rate_hz=50.0, motion="calibration", noise=False))
    dlg = _dialog(vn)
    dlg._stage = "collect"

    def _feed(mag, ts):
        d = Vn100Data(yaw=0.0, pitch=0.0, roll=0.0,
                      mag_x=mag[0], mag_y=mag[1], mag_z=mag[2],
                      accel_x=0.0, accel_y=0.0, accel_z=9.81,
                      gyro_x=1.0, gyro_y=0.0, gyro_z=0.0)   # "moving" -> goes down the collection branch
        d.timestamp = ts
        dlg.vn.get_data = lambda: d          # noqa: B023 — intentional, injecting a single sample
        dlg._collect()

    before = len(dlg.samples)
    _feed((float("nan"), 0.1, 0.2), 1001.0)
    _feed((0.1, float("inf"), 0.2), 1002.0)
    _feed((0.1, 0.2, float("-inf")), 1003.0)

    assert len(dlg.samples) == before, "a NaN/Inf sample entered samples — poisons the fit"
    assert dlg._nonfinite_skipped == 3
    assert "NaN" in dlg.lbl_now.text()
    assert all(math.isfinite(v) for s in dlg.samples for v in s)

    # Sanity check: a valid sample should still be collected (no overly-strict regression)
    _feed((0.21, -0.13, 0.35), 1004.0)
    assert len(dlg.samples) == before + 1, "a valid sample was dropped too"
    vn.stop_reader()


def test_busy_locks_main_window_and_stops_timer(qapp):
    """Since the dialog is modeless, `_busy` must lock the main window too, not just
    itself — otherwise `processEvents` also processes the main window's events and the
    user could click 'Factory Reset'/'Save' mid-write. The collection timer must stop
    too, or extra $VNRRG traffic piles up (docs/protocol.md §8.1: exactly the condition
    that produces $VNERR,03)."""
    from PySide6 import QtWidgets

    vn = _live_vn(SimTransport(rate_hz=50.0, motion="still", noise=False))
    main_win = QtWidgets.QWidget()               # stand-in for the main window
    main_win.show()
    from dashboard.calibration_dialog import CalibrationDialog
    dlg = CalibrationDialog(vn, main_win)
    dlg._timer.start(30)

    assert main_win.isEnabled() and dlg._timer.isActive(), "precondition"

    with dlg._busy():
        assert not main_win.isEnabled(), "main window was not locked — clickable mid-write"
        assert not dlg.isEnabled(), "dialog was not locked"
        assert not dlg._timer.isActive(), "collection timer did not stop — extra traffic piles up"

    # finally: everything must be restored, or the UI stays permanently locked
    assert main_win.isEnabled(), "main window was not re-enabled"
    assert dlg.isEnabled(), "dialog was not re-enabled"
    assert dlg._timer.isActive(), "timer was not restarted"
    vn.stop_reader()


def test_busy_does_not_auto_start_a_stopped_timer(qapp):
    """`_busy` must restore the timer to its PREVIOUS state — not start it if it was stopped."""
    vn = _live_vn(SimTransport(rate_hz=50.0, motion="still", noise=False))
    dlg = _dialog(vn)
    dlg._timer.stop()
    with dlg._busy():
        pass
    assert not dlg._timer.isActive(), "a stopped timer was started on its own after _busy"
    vn.stop_reader()


def test_save_does_not_write_flash_if_sensor_solution_differs_from_preview(qapp):
    """Before writing to flash, must verify the sensor's solution actually matches what's
    being previewed — checking 'is it not identity?' isn't enough: a different calibration
    left over from a previous session, or from an interleaved onboard HSI write, would
    pass that check and get made permanent."""
    vn = _live_vn(SimTransport(rate_hz=50.0, motion="still", noise=False))
    # Sensor holds a calibration that is NOT identity, but DIFFERENT from what's previewed
    vn.write_register_verified(Reg.MAG_CALIBRATION, *_GOOD_CAL)
    dlg = _dialog(vn)
    dlg._stage = "preview"
    # The solution the user is previewing is completely different
    dlg._gain = [[2.0, 0, 0], [0, 2.0, 0], [0, 0, 2.0]]
    dlg._center = [9.0, 9.0, 9.0]
    sent = _tx_log(vn)

    dlg._save()

    assert not _flash_was_written(vn, sent), \
        "flash was written while the sensor's solution differed from the preview"
    assert "PREVIEWED" in dlg.lbl_result.text()
    assert dlg._stage != "saved"
    vn.stop_reader()


def test_save_writes_when_solution_matches_preview(qapp):
    """Sanity check: saving must work when the sensor's solution matches the preview (the
    match gate tolerates float32 rounding and shouldn't block the legitimate flow)."""
    vn = _live_vn(SimTransport(rate_hz=50.0, motion="still", noise=False))
    dlg = _dialog(vn)
    dlg._gain = [[1.02, 0, 0], [0, 0.98, 0], [0, 0, 1.01]]
    dlg._center = [0.03, -0.01, 0.02]
    assert dlg._apply_offline() is True          # actually writes to the sensor
    dlg._set_stage("preview")
    sent = _tx_log(vn)

    dlg._save()

    assert _flash_was_written(vn, sent), "save was blocked despite a matching solution (overly strict)"
    vn.stop_reader()
