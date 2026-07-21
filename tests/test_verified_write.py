"""
Verified write (VN100.write_register_verified) tests -- no hardware required.

Contract under test: a successful VNACK/transport.write() does not mean the
sensor accepted the write -- it can reject with e.g. `$VNERR,03` (Invalid
Checksum). So writes must be verified by READING BACK: a rejected write
returns `ok=False`, and its REASON ($VNERR) reaches the caller.
"""
import re
import time

from pyvn100 import VN100, SimTransport
from pyvn100.registers import Reg


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


_CAL_12 = (1.002110, -0.004610, 0.003751, -0.004610, 0.988125, -0.004520,
           0.003751, -0.004520, 1.010125, 0.035443, 0.003326, 0.007365)


def test_nan_readback_is_not_counted_as_a_match():
    """FAIL-CLOSED: `abs(nan - x) > tol` is always False, so a naive comparison
    would count NaN as a match. This function's only job is preventing that
    false positive -- a non-finite readback is never verified."""
    assert VN100._fields_match([1.0, 2.0], ["nan", "nan"], 1e-4) is False
    assert VN100._fields_match([1.0, 2.0], ["inf", "-inf"], 1e-4) is False
    assert VN100._fields_match([1.0], [""], 1e-4) is False            # float-printf breakage
    assert VN100._fields_match([1.0, 2.0], ["1.00001", "2.0"], 1e-4) is True   # real match


def test_on_wait_hook_is_called_while_waiting():
    """The wait loop calls `on_wait` (dashboard -> processEvents) so the GUI
    doesn't freeze; without it, the GUI thread blocks 3s on an unresponsive
    sensor and the window shows 'Not Responding'."""
    class Deaf(SimTransport):
        def write(self, text):
            return len(text)              # swallow the command, produce no reply

    vn = VN100(Deaf(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    counter = {"n": 0}
    vn.on_wait = lambda: counter.__setitem__("n", counter["n"] + 1)
    vn.write_register_verified(Reg.HSI_CONTROL, 0, 1, 5, retries=0, timeout=0.05)
    assert counter["n"] > 0, "on_wait was never called -> GUI freezes while waiting"


def test_on_wait_exception_does_not_break_the_write_path():
    """Even if the UI hook throws, the write path must survive (same rule as the log hook)."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))

    def blow_up():
        raise RuntimeError("GUI paint error")

    vn.on_wait = blow_up
    r = vn.write_register_verified(Reg.HSI_CONTROL, 0, 1, 5)
    assert r["ok"] is True


def test_valid_write_is_verified_by_readback():
    """Happy path: sensor accepts -> readback matches -> ok=True."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, *_CAL_12)
    assert r["ok"] is True
    assert r["attempts"] == 1
    assert r["readback"] is not None and len(r["readback"]) >= 12
    assert not r["errors"]


def test_float32_rounding_does_not_trigger_a_false_alarm():
    """Sensor stores float32 (1.002110 -> 1.00211), so exact string equality
    would be wrong; a numeric tolerance is used instead, or every valid
    calibration would look 'rejected'."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, *_CAL_12)
    assert r["ok"] is True
    # readback values need not be byte-identical STRINGS to what was written
    assert any(a != b for a, b in zip([f"{v}" for v in _CAL_12], r["readback"]))


def test_rejected_write_returns_ok_false_with_visible_reason():
    """If the sensor rejects with $VNERR, ok=False + the reason is reported --
    guards against `_send()` returning True here, which would let the UI show
    a rejected write as 'applied'."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    # 2 fields instead of 12 -> sim rejects with $VNERR like real hardware, and Reg 23 is UNCHANGED
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, 1.0, 2.0, retries=0)
    assert r["ok"] is False
    assert "MISMATCH" in r["reason"] or "could not be read" in r["reason"]
    assert any("$VNERR" in e for e in r["errors"])          # the REASON for rejection reached the caller


def test_error_does_not_race_the_console_drain():
    """Verification must see the $VNERR even if the dashboard's
    drain_responses() consumed it first. app.py drains the response queue
    every tick; if verification read that same queue, it could miss the
    error by microseconds -- a separate error log (errors_since) eliminates
    that race."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    t0 = time.time()
    vn.write_register_verified(Reg.MAG_CALIBRATION, 1.0, 2.0, retries=0)
    vn.drain_responses()                                     # console queue DRAINED
    assert any("$VNERR" in e for e in vn.errors_since(t0))   # the error log still knows


class _ReadOnlyTransport(SimTransport):
    """Playback stand-in: data streams but commands never reach any sensor."""

    @property
    def writable(self) -> bool:
        return False


def test_write_in_replay_mode_is_never_counted_as_successful():
    """During playback a command never reaches a sensor -> it must NOT be called 'applied'."""
    vn = VN100(_ReadOnlyTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, *_CAL_12)
    assert r["ok"] is False and "replay" in r["reason"]


def test_verified_write_mutes_and_restores_the_stream():
    """$VNASY,0 ... $VNASY,1 wrapper mutes telemetry during the write window,
    then restores it (ICD §1.3.9) -- a firmware-safe antidote to the STM32
    ISR collision that causes $VNERR,03. Must always restore at the end, or
    the dashboard freezes."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk())
    vn = VN100(tp)
    sent: list[str] = []
    vn.on_tx = sent.append
    vn.write_register_verified(Reg.HSI_CONTROL, 0, 1, 5)
    assert any("VNASY,0" in s for s in sent), "stream was not muted"
    assert any("VNASY,1" in s for s in sent), "stream was NOT restored (dashboard would freeze)"
    assert sent.index(next(s for s in sent if "VNASY,1" in s)) == len(sent) - 1   # last in the list
    assert tp._async_paused is False                          # sensor's stream resumed too


def test_stream_muting_can_be_disabled():
    """quiet_stream=False -> $VNASY is never sent (for a caller that doesn't want the stream interrupted)."""
    vn = VN100(SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk()))
    sent: list[str] = []
    vn.on_tx = sent.append
    r = vn.write_register_verified(Reg.HSI_CONTROL, 0, 1, 5, quiet_stream=False)
    assert r["ok"] is True
    assert not any("VNASY" in s for s in sent)


def test_stream_is_restored_even_after_a_failed_write():
    """Even if the write FAILS, $VNASY,1 is still sent (finally) -- the stream is never left muted."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk())
    vn = VN100(tp)
    sent: list[str] = []
    vn.on_tx = sent.append
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, 1.0, 2.0, retries=0)
    assert r["ok"] is False
    assert any("VNASY,1" in s for s in sent) and tp._async_paused is False


def test_retry_tries_a_rejected_write_again():
    """Intermittent byte-loss: first attempt rejected, second sticks -> ok=True.
    On real hardware $VNERR,03 showed up ~57% of the time (long command x
    50 Hz stream) -- without retry, the demo would fail about half the time."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False, clock=Clk())
    vn = VN100(tp)
    real_write = tp.write
    state = {"n": 0}

    def corrupt_once(text):
        # Corrupt the checksum of the first Reg 23 write -> sensor rejects with
        # $VNERR,03, mimicking the field bug: PC's checksum is correct but
        # bytes arrive corrupted on the wire (STM32 ISR collision drops a byte).
        if "VNWRG,23" in text and state["n"] == 0:
            state["n"] += 1
            text = re.sub(r"\*[0-9A-Fa-f]{2}", "*00", text, count=1)
        return real_write(text)

    tp.write = corrupt_once
    r = vn.write_register_verified(Reg.MAG_CALIBRATION, *_CAL_12, retries=2)
    assert r["ok"] is True
    assert r["attempts"] == 2                      # first was rejected, second was verified
    assert any("$VNERR,03" in e for e in r["errors"])   # the rejection's trace must not disappear
