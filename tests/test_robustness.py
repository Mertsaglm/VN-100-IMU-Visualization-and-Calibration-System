"""
tests.test_robustness -- sim-to-real-hardware transition robustness tests.

Covers:
  - reader survives a disconnect + reconnects
  - command response queue (drain_responses -- no $VNERR gets clobbered)
  - find_stlink_port finds the port by VID (0x0483) even with a generic driver
  - a $VN line with a bad checksum is counted as an error + not cached/surfaced
  - sim register writes produce $VNWRG echo / $VNERR (write is verifiable)
"""
from __future__ import annotations

from pyvn100 import VN100, SimTransport, hostlink, protocol
from pyvn100 import transport as tmod
from pyvn100.transport import LoopbackTransport, Transport


# ── Reader survives a disconnect + reconnects ──────────────────
class _FlakyTransport(Transport):
    """First reopen fails, second succeeds -- mimics a real USB drop-and-return."""

    def __init__(self):
        self.reopened = 0

    def read(self, max_bytes: int = 4096) -> bytes:
        raise OSError("device disconnected")    # every read fails

    def write(self, data) -> int:
        return len(data)

    def reopen(self) -> bool:
        self.reopened += 1
        if self.reopened < 2:
            raise OSError("still gone")          # first attempt fails
        return True                              # second attempt connects


def test_reader_reconnect_logic():
    vn = VN100(_FlakyTransport())
    assert vn.connected is True

    vn._mark_disconnected(OSError("x"))          # simulate a read error
    assert vn.connected is False
    assert vn.stats()["errors"] == 1

    vn._try_reconnect()                          # 1st reopen raises -> still disconnected
    assert vn.connected is False

    vn._try_reconnect()                          # 2nd reopen succeeds -> connected
    assert vn.connected is True
    assert vn.stats()["last_error"] is None


def test_sim_transport_reopen_false():
    """Sim/loopback never disconnect, so reopen()=False -> reader never routes them to reconnect."""
    assert LoopbackTransport().reopen() is False
    assert SimTransport(rate_hz=50).reopen() is False


# ── Command response queue ──────────────────────────────────
def test_drain_responses_captures_all():
    vn = VN100(LoopbackTransport())
    vn.transport.feed(protocol.build_command("VNERR,03"))        # one $VNERR
    vn.transport.feed(protocol.build_command("VNWRG,06,14"))     # one $VNWRG echo
    vn.poll()
    resps = vn.drain_responses()
    assert len(resps) == 2                                       # both captured (neither clobbered)
    assert resps[0][1] is True                                   # VNERR -> err=True
    assert resps[1][1] is False
    assert vn.drain_responses() == []                            # queue drained


# ── find_stlink_port by VID ──────────────────────────────────
def test_find_stlink_by_vid(monkeypatch):
    # Windows generic driver: description does NOT contain "STM" but VID=0x0483 (ST)
    monkeypatch.setattr(tmod, "list_ports", lambda: [
        ("COM1", "USB Serial Device (COM1)", 0x1234, 0x0001),
        ("COM5", "USB Serial Device (COM5)", 0x0483, 0x5740),
    ])
    assert tmod.find_stlink_port() == "COM5"


def test_find_stlink_text_fallback(monkeypatch):
    monkeypatch.setattr(tmod, "list_ports", lambda: [
        ("COM3", "STMicroelectronics STLink Virtual COM Port", None, None),
    ])
    assert tmod.find_stlink_port() == "COM3"


# ── Bad checksum is counted as an error, not cached/surfaced ─────
def test_bad_checksum_rrg_counts_error():
    vn = VN100(LoopbackTransport())
    vn.transport.feed("$VNRRG,46,1,2,3*00\r\n")                  # corrupt checksum
    vn.poll()
    assert vn.stats()["errors"] == 1                            # link error counted
    assert vn.get_register(46) is None                          # stale/corrupt -> did NOT get cached
    assert vn.drain_responses() == []                           # not surfaced as a response either


def test_valid_rrg_still_cached():
    vn = VN100(LoopbackTransport())
    vn.transport.feed(protocol.build_command("VNRRG,46,0,1,0.005,0,0,0,3,3,3,3,3,3,3,3"))
    vn.poll()
    assert vn.get_register(46) is not None                      # valid response got cached
    assert vn.stats()["errors"] == 0


# ── Sim register write produces an echo ────────────────────────
def _drain_after_poll(vn, cmd):
    vn.send_raw(cmd)
    vn.poll()
    return vn.drain_responses()


def test_sim_write_echo_and_error():
    vn = VN100(SimTransport(rate_hz=50, motion="gentle"))
    ok = _drain_after_poll(vn, hostlink.write_reg(23, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0))
    assert any(r[0].startswith("$VNWRG,23") for r in ok)        # accepted -> echoed
    bad = _drain_after_poll(vn, hostlink.write_reg(23, 1, 2))   # 2 fields instead of 12 -> rejected
    assert any("$VNERR" in r[0] for r in bad)                   # format error surfaces in sim too


# ── Host-level ('$'-less) VNERR is surfaced on PC; VNACK/VNPONG are dropped ──
def test_host_level_vnerr_surfaced():
    vn = VN100(LoopbackTransport())
    vn.transport.feed("VNERR baud-disabled\r\n")                # host_link.c's '$'-less response
    vn.poll()
    resps = vn.drain_responses()
    assert len(resps) == 1
    assert resps[0][0] == "VNERR baud-disabled"
    assert resps[0][1] is True                                 # error -> dashboard shows red


def test_host_level_ack_pong_dropped_mode_surfaced():
    # VNACK ('wrote to STM UART', repeats on every command+poll) and VNPONG
    # (startup probe only) are deliberately DROPPED. VNMODE (mode-change ack,
    # rare) is SURFACED so it stays visible in the console.
    vn = VN100(LoopbackTransport())
    vn.transport.feed("VNACK\r\nVNPONG\r\n")
    vn.poll()
    assert vn.drain_responses() == []                          # ACK/PONG dropped
    vn.transport.feed("VNMODE BINARY\r\n")
    vn.poll()
    resps = vn.drain_responses()
    assert len(resps) == 1 and resps[0][0] == "VNMODE BINARY" and resps[0][1] is False


def test_sensor_vnerr_and_host_vnerr_both_surfaced():
    vn = VN100(LoopbackTransport())
    vn.transport.feed(protocol.build_command("VNERR,03"))       # sensor's $VNERR (ASCII path)
    vn.transport.feed("VNERR fail\r\n")                          # STM32 host_link's '$'-less VNERR
    vn.poll()
    texts = [r[0] for r in vn.drain_responses()]
    assert any(t.startswith("$VNERR,03") for t in texts)        # sensor error
    assert "VNERR fail" in texts                                # host error -- no longer dropped


# ── Stray 0xFA + short $VNRRG + quiet stream -> recovers like C (no stall) ──
def test_stray_fa_before_short_rrg_does_not_stall():
    vn = VN100(LoopbackTransport())
    reply = protocol.build_command("VNRRG,5,115200")            # short (<42 B) register reply
    vn.transport.feed(b"\xFA" + reply.encode("ascii"))          # preceded by a stray 0xFA
    vn.poll()
    reg = vn.get_register(5)
    assert reg is not None                                      # must not stall after a stray 0xFA
    assert reg[0] == ["115200"]
