"""
Unit tests for the VN100 high-level API + LoopbackTransport.

End-to-end path, no hardware: feed(bytes) -> poll() -> parse -> data/stats.
"""
import math

from pyvn100 import VN100, LoopbackTransport, protocol


def _ymr(yaw, pitch, roll, extra=None):
    vals = [yaw, pitch, roll] + (extra or [0.1, -0.1, -0.4, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0])
    body = "VNYMR," + ",".join(f"{x:+.4f}" for x in vals)
    return protocol.build_command(body)


def test_single_packet_is_decoded():
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(_ymr(12.0, -3.0, 1.5))
    assert vn.poll() == 1
    d = vn.get_data()
    assert d is not None
    assert math.isclose(d.yaw, 12.0, abs_tol=1e-3)
    assert math.isclose(d.pitch, -3.0, abs_tol=1e-3)
    assert math.isclose(d.roll, 1.5, abs_tol=1e-3)
    assert d.timestamp is not None
    assert vn.stats()["packets"] == 1
    assert vn.stats()["errors"] == 0


def test_multiple_packets():
    tp = LoopbackTransport()
    vn = VN100(tp)
    for i in range(5):
        tp.feed(_ymr(float(i), 0.0, 0.0))
    assert vn.poll() == 5
    assert vn.stats()["packets"] == 5
    assert math.isclose(vn.get_data().yaw, 4.0, abs_tol=1e-3)


def test_split_line_is_reassembled():
    tp = LoopbackTransport()
    vn = VN100(tp)
    line = _ymr(7.0, 0.0, 0.0)
    half = len(line) // 2
    tp.feed(line[:half])
    assert vn.poll() == 0          # line not complete yet
    assert vn.get_data() is None
    tp.feed(line[half:])
    assert vn.poll() == 1          # decoded once complete
    assert math.isclose(vn.get_data().yaw, 7.0, abs_tol=1e-3)


def test_corrupt_checksum_counts_as_error():
    tp = LoopbackTransport()
    vn = VN100(tp)
    line = _ymr(1.0, 2.0, 3.0)
    corrupt = line[:-4] + "00\r\n"
    tp.feed(corrupt)
    assert vn.poll() == 0
    assert vn.stats()["errors"] == 1
    assert vn.get_data() is None


def test_different_message_type_is_ignored():
    tp = LoopbackTransport()
    vn = VN100(tp)
    # Valid checksum but an unhandled message type -> silently ignored, not
    # an error (a bad checksum would count as a 'link error' -- see above).
    tp.feed("$VNQTN,0,0,0,1*52\r\n")
    assert vn.poll() == 0
    assert vn.stats()["errors"] == 0
    assert vn.get_data() is None


def test_callback_fires():
    tp = LoopbackTransport()
    received = []
    vn = VN100(tp, on_packet=lambda d: received.append(d))
    tp.feed(_ymr(9.0, 0.0, 0.0))
    vn.poll()
    assert len(received) == 1
    assert math.isclose(received[0].yaw, 9.0, abs_tol=1e-3)


# ── Command (TX) path ──────────────────────────────────────────────

def test_command_send_tx_log():
    tp = LoopbackTransport()
    vn = VN100(tp)
    vn.read_register(6)
    vn.set_async_output_freq(40)
    vn.set_async_output_type(14)
    vn.write_settings()
    sent = tp.tx_log.decode("ascii")
    assert "$VNRRG,6*" in sent
    assert "$VNWRG,7,40*" in sent
    assert "$VNWRG,6,14*" in sent
    assert "$VNWNV*" in sent
    # every line sent must carry a valid checksum
    for line in sent.strip().split("\r\n"):
        assert protocol.verify_ascii(line), f"invalid: {line!r}"


def test_baud_write_is_NOT_in_the_HIGH_level_API():
    """`VN100.set_baudrate` deliberately does not exist: the system blocks baud
    changes everywhere (`host_link.c`'s `VN BAUD` + `$VNWRG,5` leak, the
    simulator) because a one-sided change severs the sensor<->STM32 link. If
    the method existed, it could bypass the link layer and write straight to
    the transport. This test anchors the gate as closed -- adding the method
    back breaks it."""
    assert not hasattr(VN100, "set_baudrate")
    assert not hasattr(VN100, "tare")      # unguarded $VNTAR twin -- the link path must be used instead


def test_binary_divisor_rule_matches_c_exactly():
    """The divisor rule must be IDENTICAL to the C core (800/hz INTEGER DIVISION,
    hz<=0 -> 4) -- using round() would make the same request produce a
    DIFFERENT output rate on each side (300 Hz: C gives 400 Hz, Py gives 266.7 Hz)."""
    for hz, expected_div in ((300, 2), (200, 4), (170, 4), (90, 8), (0, 4), (801, 1)):
        tp = LoopbackTransport()
        vn = VN100(tp)
        vn.set_output_mode("binary", hz)
        sent = tp.tx_log.decode("ascii")
        assert f"VNWRG,75,2,{expected_div},01,0128" in sent, (
            f"hz={hz}: expected divisor {expected_div}, sent: {sent!r}")
