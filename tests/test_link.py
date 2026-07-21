"""
Link-mode abstraction (pyvn100.link) tests — BRIDGE (STM bridge) vs DIRECT (USB-TTL).

Scope:
  1. Unit string equality — for each logical command, does BridgeLink (`VN RAW`/`VN <verb>`)
     and DirectLink (raw `$VN...*CS`) produce the correct wire frame. Especially for DIRECT,
     set_output_mode (multi-register ORDER) and set_freq (reg7/reg75 distinction).
  2. Structural invariants — BRIDGE lines start with 'VN ' and end with '\n'; DIRECT lines
     start with '$VN' and end with '\r\n' and have a valid checksum.
  3. End to end (sim) — DirectLink commands drive SimTransport's raw `$VN` branch; the Reg 46
     reply lands in the VN100 cache (hardware-free DIRECT validation).
  4. Auto-detection — VNPONG -> BridgeLink; no reply -> DirectLink; writable=False -> BridgeLink;
     forced override; real SimTransport -> BridgeLink (no regression).
  5. selfcheck parity — bring-up validation also works in DIRECT mode (raw $VNRRG).
"""
import pytest

from pyvn100 import VN100, SimTransport, selfcheck
from pyvn100 import protocol
from pyvn100.link import BRIDGE, DIRECT, BridgeLink, DirectLink, detect_link
from pyvn100.registers import Reg, decode_hsi_status, decode_mag_cal


# -- 1. Unit: string equality (validated wire frames) ----------------------

def test_bridge_read_write_reg():
    assert BridgeLink().read_register(46) == ["VN RAW $VNRRG,46*71\n"]
    assert BridgeLink().write_register(6, 14) == ["VN RAW $VNWRG,6,14*69\n"]


def test_direct_read_write_reg():
    assert DirectLink().read_register(46) == ["$VNRRG,46*71\r\n"]
    assert DirectLink().write_register(6, 14) == ["$VNWRG,6,14*69\r\n"]


def test_direct_scalar_commands():
    d = DirectLink()
    assert d.tare() == ["$VNTAR*5F\r\n"]
    assert d.save() == ["$VNWNV*57\r\n"]
    assert d.factory() == ["$VNRFS*5F\r\n"]
    assert d.reset() == ["$VNRST*4D\r\n"]
    assert d.gyro_bias() == ["$VNSGB*4E\r\n"]
    assert d.set_type(14) == ["$VNWRG,6,14*69\r\n"]


def test_direct_hsi_commands():
    d = DirectLink()
    assert d.hsi_reset(rate=5) == ["$VNWRG,44,2,3,5*6E\r\n"]     # RESET, USE_ONBOARD, rate
    assert d.hsi_off() == ["$VNWRG,44,0,3,5*6C\r\n"]            # OFF, USE_ONBOARD, 5
    assert d.hsi_status() == ["$VNRRG,46*71\r\n"]


def test_bridge_verbs_single_line():
    b = BridgeLink()
    assert b.set_output_mode("ascii") == ["VN MODE ASCII\n"]
    assert b.set_output_mode("binary") == ["VN MODE BINARY\n"]
    assert b.set_freq(50, binary=False) == ["VN FREQ 50\n"]
    assert b.set_freq(50, binary=True) == ["VN FREQ 50\n"]     # over the bridge the binary flag is resolved on the STM side
    assert b.tare() == ["VN TARE\n"] and b.save() == ["VN SAVE\n"]


def test_direct_set_output_mode_ascii_order():
    # ASCII: [disable reg75, reg6=VNYMR, reg7=hz] — ORDER matches VN100.set_output_mode exactly.
    assert DirectLink().set_output_mode("ascii", 40) == [
        "$VNWRG,75,0,4,01,0128*7A\r\n",
        "$VNWRG,6,14*69\r\n",
        "$VNWRG,7,40*69\r\n",
    ]


def test_direct_set_output_mode_binary_order():
    # BINARY: [reg6=0, reg75 divisor] — divisor = 800//200 = 4.
    assert DirectLink().set_output_mode("binary", 200) == [
        "$VNWRG,6,0*5C\r\n",
        "$VNWRG,75,2,4,01,0128*78\r\n",
    ]


def test_direct_set_freq_reg7_vs_reg75():
    d = DirectLink()
    assert d.set_freq(50, binary=False) == ["$VNWRG,7,50*68\r\n"]        # ASCII -> Reg 7 (ADOF)
    assert d.set_freq(50, binary=True) == ["$VNWRG,75,2,16,01,0128*4B\r\n"]  # binary -> Reg 75, div=800//50=16


def test_direct_set_output_mode_invalid_mode_rejected():
    with pytest.raises(ValueError):
        DirectLink().set_output_mode("binry")   # a typo must not be silently treated as ASCII
    with pytest.raises(ValueError):
        BridgeLink().set_output_mode("xyz")


class _RecTransport:
    """Minimal transport that records the raw commands VN100 writes (for the drift guard)."""
    writable = True

    def __init__(self):
        self.writes: list[str] = []

    def write(self, text) -> int:
        self.writes.append(text)
        return len(text)

    def read(self, max_bytes: int = 4096) -> bytes:
        return b""


def test_direct_set_output_mode_matches_vn100_byte_for_byte():
    """DRIFT GUARD: DirectLink.set_output_mode must match the bytes VN100.set_output_mode
    ACTUALLY writes. If the VN100 side (order/divisor/port) changes later, DirectLink must not drift."""
    for mode, hz in [("ascii", 40), ("binary", 200), ("binary", 100), ("ascii", 50)]:
        rec = _RecTransport()
        VN100(rec).set_output_mode(mode, hz)      # the $VN sequence the high-level API actually writes
        assert DirectLink().set_output_mode(mode, hz) == rec.writes, (mode, hz)


# -- 2. Structural invariants -----------------------------------------------

_ALL_OPS = [
    lambda k: k.read_register(1),
    lambda k: k.write_register(6, 0),
    lambda k: k.set_output_mode("ascii", 40),
    lambda k: k.set_output_mode("binary", 200),
    lambda k: k.set_freq(50, binary=False),
    lambda k: k.set_freq(50, binary=True),
    lambda k: k.set_type(14),
    lambda k: k.tare(), lambda k: k.save(), lambda k: k.factory(),
    lambda k: k.reset(), lambda k: k.gyro_bias(),
    lambda k: k.hsi_reset(), lambda k: k.hsi_off(), lambda k: k.hsi_status(),
]


def test_bridge_structural_invariants():
    for op in _ALL_OPS:
        for cmd in op(BridgeLink()):
            assert cmd.startswith("VN ") and cmd.endswith("\n") and not cmd.startswith("$")


def test_direct_structural_invariants_and_checksum():
    for op in _ALL_OPS:
        for cmd in op(DirectLink()):
            assert cmd.startswith("$VN") and cmd.endswith("\r\n")
            assert protocol.verify_ascii(cmd)     # raw frame's XOR checksum is valid


# -- 3. End to end (sim): DIRECT raw $VN -> Reg 46 reply --------------------

class _Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


def test_direct_reg47_end_to_end_sim():
    """Raw '$VNRRG,47' -> the sim's direct branch -> the cache. (Reg 47 is used rather than
    Reg 46; Reg 46 does not exist in the FW 3.1.0.0 ICD — the point here is DIRECT framing, not the register.)"""
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=_Clk(), noise=False)
    vn = VN100(tp, link=DirectLink())
    for c in vn.link.read_register(Reg.HSI_CALCULATED):
        tp.write(c)
    vn.poll()
    r = vn.get_register(Reg.HSI_CALCULATED)
    assert r is not None
    assert decode_mag_cal(r[0]) is not None


def test_bridge_and_direct_get_same_register_reply():
    # Parity: both modes (VN RAW $VNRRG vs raw $VNRRG) fill the same register cache.
    for lk in (BridgeLink(), DirectLink()):
        tp = SimTransport(rate_hz=50.0, motion="calibration", clock=_Clk(), noise=False)
        vn = VN100(tp, link=lk)
        for c in vn.link.read_register(Reg.HSI_CALCULATED):
            tp.write(c)
        vn.poll()
        assert vn.get_register(Reg.HSI_CALCULATED) is not None, f"{lk.mode} Reg 47 reply did not arrive"


def test_bridge_and_direct_get_same_reg46_reply_fw2():
    """On older hardware (FW 2.1), Reg 46 parity is also preserved — no backward-compat regression."""
    for lk in (BridgeLink(), DirectLink()):
        tp = SimTransport(rate_hz=50.0, motion="calibration", clock=_Clk(), noise=False,
                          fw_version="2.1.0.0")
        vn = VN100(tp, link=lk)
        for c in vn.link.read_register(Reg.HSI_STATUS):
            tp.write(c)
        vn.poll()
        r = vn.get_register(Reg.HSI_STATUS)
        assert r is not None and decode_hsi_status(r[0]) is not None, f"{lk.mode}"


# -- 4. Auto-detection --------------------------------------------------------

class _FakeTransport:
    """Minimal transport for detect_link: can reply VNPONG to VN PING, or stay silent."""

    def __init__(self, respond_pong: bool, stream: bytes = b"", writable: bool = True):
        self.writable = writable
        self._respond = respond_pong
        self._out = bytearray(stream)

    def write(self, data) -> int:
        text = data if isinstance(data, str) else data.decode("ascii", "ignore")
        if self._respond and "VN PING" in text:
            self._out.extend(b"VNPONG\r\n")
        return len(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        out = bytes(self._out)
        self._out.clear()
        return out


def test_detect_vnpong_bridge():
    lk = detect_link(_FakeTransport(respond_pong=True), timeout=0.3)
    assert lk.mode == BRIDGE


def test_detect_no_reply_direct():
    # A transport that only streams $VNYMR and never produces VNPONG -> timeout -> DIRECT (no false positive).
    tp = _FakeTransport(respond_pong=False, stream=b"$VNYMR,1,2,3*00\r\n")
    lk = detect_link(tp, timeout=0.05)
    assert lk.mode == DIRECT


def test_detect_not_writable_defaults_to_bridge():
    lk = detect_link(_FakeTransport(respond_pong=False, writable=False), timeout=0.05)
    assert lk.mode == BRIDGE          # not writable -> no probe -> bridge (commands are already no-ops)


def test_detect_forced_override():
    assert detect_link(_FakeTransport(respond_pong=True), forced="direct").mode == DIRECT
    assert detect_link(_FakeTransport(respond_pong=False), forced="bridge").mode == BRIDGE


def test_detect_real_sim_bridge():
    # SimTransport mimics the STM bridge (VN PING->VNPONG) -> BRIDGE (existing behavior, no regression).
    tp = SimTransport(rate_hz=50.0, motion="still", clock=_Clk(), noise=False)
    assert detect_link(tp, timeout=0.3).mode == BRIDGE


# -- 5. selfcheck parity: bring-up also works in DIRECT mode -----------------

def test_selfcheck_request_reads_works_direct():
    """Bring-up identity read also works in DIRECT mode (raw $VNRRG,1/2/4)."""
    tp = SimTransport(rate_hz=50.0, motion="still", clock=_Clk(), noise=False)
    vn = VN100(tp, link=DirectLink())
    assert selfcheck.request_reads(vn) is True
    for _ in range(5):
        vn.poll()
    # The identity triple must also land in the cache in DIRECT mode (Reg 46 doesn't exist on this FW — expected)
    assert vn.get_register(Reg.MODEL_NUMBER) is not None
    assert vn.get_register(Reg.HARDWARE_REVISION) is not None
    assert vn.get_register(Reg.FIRMWARE_VERSION) is not None
