"""
Hybrid mode (HybridTransport) tests - measurements come from a RECORDING, commands go
to the REAL sensor. Purpose: replay calibration against a "golden recording" taken once,
without physically rotating the sensor, then write the result to the real sensor
(docs/calibration.md Sec.4b).

Critical contract: the sensor's OWN telemetry must NEVER surface (a sensor sitting still
on the desk would mix its stationary samples with the recording's rotating cloud and
silently poison the fit), but command RESPONSES must pass through (Reg 23/44 snapshot
for 'Undo', Reg 46/47 readings, the accept/reject echo).
"""
import csv

import pytest

from pyvn100 import (VN100, HybridTransport, LoopbackTransport, ReplayTransport,
                     Vn100Simulator, hostlink, binary, protocol)
from pyvn100.types import Vn100Data


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


_HEADER = ["timestamp", "yaw", "pitch", "roll",
           "gyro_x", "gyro_y", "gyro_z",
           "accel_x", "accel_y", "accel_z",
           "mag_x", "mag_y", "mag_z"]


def _rec(tmp_path, n=3):
    """Writes a recording with yaw=10,11,12... so recording-sourced yaw is unmistakable."""
    p = tmp_path / "golden.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(_HEADER)
        for i in range(n):
            w.writerow([100.0 + i * 0.01, 10 + i, -5, 3, 0.5, 0.5, 0.5,
                        0.1, 0.2, 9.81, 0.2, -0.1, 0.3])
    return str(p)


def _resp(body: str) -> str:
    """Builds a sensor response with a valid checksum - VN100._handle_line treats a bad
    checksum as an ERROR, not a response, so a hand-written one would get dropped."""
    return protocol.build_command(body)


def _sensor_ymr(yaw=999.0, gyro_z=0.0):
    """The real sensor's OWN telemetry, with a yaw distinguishable from the recording;
    gyro_z lets a test simulate the sensor actively ROTATING ($VNSGB gate tests)."""
    return Vn100Simulator.encode_ascii(Vn100Data(
        yaw=yaw, pitch=0.0, roll=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0,
        accel_x=0.0, accel_y=0.0, accel_z=9.81, gyro_x=0.0, gyro_y=0.0, gyro_z=gyro_z))


def _rec_still(tmp_path, n=3):
    """A recording that LOOKS still (gyro=0, |accel|=9.81) - the input meant to fool the
    hybrid stillness gate."""
    p = tmp_path / "still.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(_HEADER)
        for i in range(n):
            w.writerow([100.0 + i * 0.01, 10 + i, -5, 3, 0.0, 0.0, 0.0,
                        0.0, 0.0, 9.81, 0.2, -0.1, 0.3])
    return str(p)


def _hybrid(tmp_path, n=3):
    clk = Clk()
    src = ReplayTransport(_rec(tmp_path, n), clock=clk)
    sensor = LoopbackTransport()
    return HybridTransport(src, sensor), sensor, clk


# -- Flags: the mode is identified by these two axes ----------------
def test_hybrid_flags(tmp_path):
    """writable=True (commands go to the real sensor) + data_is_recorded=True (measurements
    come from the recording). writable is what distinguishes it from pure replay;
    data_is_recorded is what distinguishes it from a live port."""
    tp, _, _ = _hybrid(tmp_path)
    assert tp.writable is True
    assert tp.data_is_recorded is True

    src = ReplayTransport(_rec(tmp_path), clock=Clk())
    assert src.writable is False and src.data_is_recorded is True
    assert LoopbackTransport().data_is_recorded is False      # live transport -> False


# -- RX separation: the whole reason this class exists --------------
def test_measurements_come_from_recording(tmp_path):
    tp, _, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    clk.adv(1.0)
    assert vn.poll() == 3
    assert vn.get_data().yaw == pytest.approx(12.0, abs=1e-2)   # last row of the recording


def test_sensors_own_telemetry_does_not_surface(tmp_path):
    """CRITICAL: a sensor on the desk also broadcasts $VNYMR - if it leaked through, its
    stationary samples would silently corrupt the fit."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    sensor.feed(_sensor_ymr(yaw=999.0))     # the sensor's own telemetry
    clk.adv(1.0)
    n = vn.poll()
    assert n == 3                            # only the recording's 3 rows - sensor's not counted
    assert vn.get_data().yaw == pytest.approx(12.0, abs=1e-2)   # NOT 999
    assert vn.packet_count == 3


def test_sensors_binary_frames_are_also_filtered(tmp_path):
    """The sensor may have been left in binary mode -> 0xFA frames are telemetry too and
    must be dropped. (An ASCII-only filter isn't enough: VN100 used to commit binary
    frames as data.)"""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    sensor.feed(binary.encode(Vn100Data(
        yaw=999.0, pitch=0.0, roll=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0,
        accel_x=0.0, accel_y=0.0, accel_z=9.81, gyro_x=0.0, gyro_y=0.0, gyro_z=0.0)))
    clk.adv(1.0)
    assert vn.poll() == 3                    # binary frame NOT counted as a packet
    assert vn.get_data().yaw == pytest.approx(12.0, abs=1e-2)
    assert vn.last_fmt == "ascii"            # binary was never decoded


def test_command_responses_pass_through(tmp_path):
    """Reg-read responses must pass through: the Reg 23/44 snapshot 'Undo' relies on, and
    Reg 46/47, come from here. If they didn't, the wizard would become silently
    non-reversible."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    sensor.feed(_resp("VNRRG,23,+1.0,0,0,0,+1.0,0,0,0,+1.0,0,0,0"))
    clk.adv(1.0)
    vn.poll()
    r = vn.get_register(23)
    assert r is not None and len(r[0]) == 12


def test_error_and_echo_lines_pass_through(tmp_path):
    """Both $VNERR (sensor rejected) and the bridge's dollar-less VNERR (host-side) must
    stay visible - VNACK != acceptance; real accept/reject can ONLY be read from these
    lines."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    sensor.feed(_resp("VNERR,03"))          # sensor rejected
    sensor.feed("VNERR fail\n")             # bridge's dollar-less host-level error
    clk.adv(1.0)
    vn.poll()
    texts = [t for t, _err, _ts in vn.drain_responses()]
    assert any("$VNERR" in t for t in texts)
    assert any(t.strip() == "VNERR fail" for t in texts)


def test_telemetry_and_response_interleaved_in_one_read(tmp_path):
    """On a real link, telemetry and a response can arrive interleaved in the same chunk.
    The response must not be lost, and telemetry must not leak through (even split
    across a line boundary)."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    blob = _sensor_ymr(999.0) + _resp("VNWRG,23,+1.0") + _sensor_ymr(998.0)
    sensor.feed(blob[:len(blob) // 2])       # split mid-line
    clk.adv(1.0)
    vn.poll()
    sensor.feed(blob[len(blob) // 2:])
    vn.poll()
    texts = [t for t, _e, _ts in vn.drain_responses()]
    assert any("$VNWRG,23" in t for t in texts)          # response passed through
    assert vn.get_data().yaw == pytest.approx(12.0, abs=1e-2)   # telemetry did not leak


# -- TX: commands go to the real sensor ------------------------------
def test_commands_go_to_the_real_sensor(tmp_path):
    tp, sensor, _ = _hybrid(tmp_path)
    tp.write(hostlink.read_reg(23))
    assert b"VN RAW $VNRRG,23" in bytes(sensor.tx_log)


# -- live_data: the real source for the stillness gate ---------------
def test_live_data_tracks_real_sensor_not_recording(tmp_path):
    """The pre-$VNWNV gate (UM001 Sec.5.1.3) must measure the sensor's CURRENT state, not
    the recording's rotating data. That's why extracted telemetry isn't discarded - it's
    kept in live_data."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    assert tp.live_data is None and tp.live_age is None     # sensor hasn't spoken yet -> fail-closed
    sensor.feed(_sensor_ymr(yaw=999.0))
    clk.adv(1.0)
    vn.poll()
    live = tp.live_data
    assert live is not None
    assert live.yaw == pytest.approx(999.0, abs=1e-2)       # the SENSOR's data (recording: 12.0)
    assert live.gyro_z == pytest.approx(0.0, abs=1e-3)      # sensor is still
    assert tp.live_age is not None and tp.live_age >= 0.0
    assert vn.get_data().yaw == pytest.approx(12.0, abs=1e-2)   # display still shows the recording


def test_still_reference_picks_real_sensor_in_hybrid(tmp_path):
    """If the gate looked at the RECORDING in hybrid mode: the recording always says
    'rotating' -> it measures the wrong thing AND 'Save' never becomes available.
    still_reference must pick the real live stream as the single source of truth."""
    pytest.importorskip("PySide6")             # dialog module requires Qt
    from dashboard.gyro_bias_dialog import still_reference

    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    sensor.feed(_sensor_ymr(yaw=999.0))
    clk.adv(1.0)
    vn.poll()
    d, age = still_reference(vn)
    assert d.yaw == pytest.approx(999.0, abs=1e-2)          # NOT the recording (12.0)
    assert age is not None and age < 1.0

    # On a live (non-hybrid) path, behavior is UNCHANGED: the live stream is read.
    lo = LoopbackTransport()
    vn2 = VN100(lo)
    lo.feed(_sensor_ymr(yaw=42.0))
    vn2.poll()
    d2, age2 = still_reference(vn2)
    assert d2.yaw == pytest.approx(42.0, abs=1e-2) and age2 is not None


def test_gyro_write_gate_measures_real_sensor_in_hybrid(tmp_path):
    """H-1 REGRESSION: if the recording LOOKS STILL while the real sensor is ROTATING, $VNSGB
    must not be written. Reading `vn.get_data()` (= the recording) would call it 'still' and
    burn the rotating sensor's instantaneous gyro output into FLASH as the Filter Startup
    Gyro Bias, with the UI reporting 'verified'. The gate must read still_reference()
    (= the real sensor) and reject the write."""
    pytest.importorskip("PySide6")
    from PySide6 import QtWidgets
    from dashboard.gyro_bias_dialog import GyroBiasDialog, STILL_GYRO
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    # A regression would reach the modal confirmation, which BLOCKS FOREVER offscreen.
    # Short-circuit it with "No" so a regression shows up as an assertion failure, not a hang.
    _orig_question = QtWidgets.QMessageBox.question
    QtWidgets.QMessageBox.question = staticmethod(
        lambda *a, **k: QtWidgets.QMessageBox.No)

    clk = Clk()
    src = ReplayTransport(_rec_still(tmp_path), clock=clk)   # recording: gyro=0 -> "still"
    sensor = LoopbackTransport()
    tp = HybridTransport(src, sensor)
    vn = VN100(tp)

    sensor.feed(_sensor_ymr(yaw=999.0, gyro_z=5.0))          # REAL sensor: rotating at 5 rad/s
    clk.adv(1.0)
    vn.poll()

    # Precondition - proves why the old code passed: the recording really does look "still".
    assert abs(vn.get_data().gyro_z) < STILL_GYRO

    dlg = GyroBiasDialog(vn)
    n_tx = len(bytes(sensor.tx_log))
    try:
        dlg._write_to_sensor()      # the stillness gate must run BEFORE the QMessageBox
    finally:
        QtWidgets.QMessageBox.question = _orig_question

    assert b"VNSGB" not in bytes(sensor.tx_log), "wrote a permanent bias to a rotating sensor"
    assert b"VNWNV" not in bytes(sensor.tx_log), "a write that should have been rejected reached flash"
    assert len(bytes(sensor.tx_log)) == n_tx, "gate was bypassed - a command was sent to the sensor"
    assert "NOT STILL" in dlg.lbl_state.text()


# -- Resource management ----------------------------------------------
def test_reopen_and_port_delegate_to_sensor(tmp_path):
    """What can disconnect is the sensor link (the CSV can't) -> reopen/port_name/is_open
    all delegate to the sensor."""
    class FakeSerial(LoopbackTransport):
        def __init__(self):
            super().__init__()
            self.reopened = 0

        def reopen(self):
            self.reopened += 1
            return True

        @property
        def port_name(self):
            return "COM7"

    src = ReplayTransport(_rec(tmp_path), clock=Clk())
    sensor = FakeSerial()
    tp = HybridTransport(src, sensor)
    assert tp.reopen() is True and sensor.reopened == 1
    assert tp.port_name == "COM7"
    assert tp.n_rows == 3


def test_stream_ends_when_recording_finishes_but_sensor_link_stays_alive(tmp_path):
    """Once the recording is exhausted, finished=True but the transport doesn't die: the
    command/response path keeps working (so the user can apply the fit and save it via
    $VNWNV)."""
    tp, sensor, clk = _hybrid(tmp_path)
    vn = VN100(tp)
    clk.adv(5.0)
    vn.poll()
    assert tp.finished is True
    sensor.feed(_resp("VNWRG,23,+1.0"))
    vn.poll()
    assert any("$VNWRG" in t for t, _e, _ts in vn.drain_responses())
    tp.write("VN RAW $VNWNV*57\n")
    assert b"VNWNV" in bytes(sensor.tx_log)
