"""
Sensor console visibility: sent commands (TX) and incoming replies (RX) are
visible from a single place.

Contract: (a) all sent commands, including dialog commands (gyro bias/calibration),
must go through `VN100.send()` and land in the console — writing directly to the
transport bypasses it. (b) register-read replies ($VNRRG) must be both cached and
queued in the response queue so they show up in the console; `VN100.send()` triggers
`on_tx`, and both $VNRRG and VNMODE land in the response queue.
"""
from pyvn100 import VN100, protocol
from pyvn100.transport import LoopbackTransport


def _drain_texts(vn):
    return [t for t, _e, _ts in vn.drain_responses()]


def test_on_tx_captures_every_sent_command():
    tx = []
    vn = VN100(LoopbackTransport(), on_tx=lambda t: tx.append(t.strip()))
    vn.send("VN RAW $VNSGB*4E\n")      # capture gyro bias (dialog path)
    vn.send("VN SAVE\n")               # persist write
    assert tx == ["VN RAW $VNSGB*4E", "VN SAVE"]


def test_send_writes_even_without_on_tx():
    # send() must still write when on_tx is None — the hook is optional.
    tp = LoopbackTransport()
    vn = VN100(tp)
    n = vn.send("VN PING\n")
    assert n == len("VN PING\n") and b"VN PING" in bytes(tp.tx_log)


def test_vnrrg_register_reply_reaches_console():
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(protocol.build_command("VNRRG,46,3,120,0.01,0.1,0.2,0.3,10,12,9,11,8,10,13").encode())
    for _ in range(3):
        vn.poll()
    texts = _drain_texts(vn)
    assert any(t.startswith("$VNRRG,46,") for t in texts)
    assert vn.get_register(46) is not None                    # both paths preserved: cache also populated


def test_stm_vnmode_info_vnerr_error_surface_vnack_vnpong_dropped():
    tp = LoopbackTransport()
    vn = VN100(tp)

    def feed_poll(b):
        tp.feed(b)
        for _ in range(2):
            vn.poll()
        return vn.drain_responses()

    # VNMODE (mode confirmation) -> info (err=False)
    r = feed_poll(b"VNMODE ASCII\r\n")
    assert r and r[0][0] == "VNMODE ASCII" and r[0][1] is False
    # VNERR -> error (err=True)
    r = feed_poll(b"VNERR fail\r\n")
    assert r and r[0][0] == "VNERR fail" and r[0][1] is True
    # VNACK / VNPONG -> deliberately dropped (poll noise; TX log + sensor echo are enough)
    assert feed_poll(b"VNACK\r\n") == []
    assert feed_poll(b"VNPONG\r\n") == []


def test_bridge_vnerr_is_seen_by_errors_since():
    """'VNERR fail' from the bridge must also land in the non-destructive error log.

    $VNWNV/$VNSGB can't be read back, so errors_since() is the only channel for "did the
    sensor reject it?". If the host branch only wrote to _responses, a red line would
    flash by in the console while errors_since() stayed empty, and the UI could show a
    "saved" confirmation for a write that never happened."""
    import time
    tp = LoopbackTransport()
    vn = VN100(tp)
    t0 = time.time()

    tp.feed(b"VNERR fail\r\n")
    for _ in range(2):
        vn.poll()

    assert vn.errors_since(t0) == ["VNERR fail"], "bridge error did not reach errors_since()"
    # console path must stay intact: same line should still appear in the response queue too
    assert [t for t, _e, _ts in vn.drain_responses()] == ["VNERR fail"]
    # drain is destructive but _error_log isn't -> it still holds the error after the console reads it
    assert vn.errors_since(t0) == ["VNERR fail"]


def test_vnmode_and_vnack_do_not_enter_error_log():
    """Only VNERR should count as an error. The info line (VNMODE) and the
    deliberately-dropped VNACK/VNPONG must not count as errors — otherwise
    every mode change would look like a "write rejected"."""
    import time
    tp = LoopbackTransport()
    vn = VN100(tp)
    t0 = time.time()
    for b in (b"VNMODE ASCII\r\n", b"VNACK\r\n", b"VNPONG\r\n"):
        tp.feed(b)
        for _ in range(2):
            vn.poll()
    assert vn.errors_since(t0) == []


def test_host_lines_surface_even_while_streaming():
    """The bridge's VNERR/VNMODE line must stay visible even when it lands in the same
    buffer as a telemetry frame.

    If host lines were only handled when the buffer contains neither a '$' nor a 0xFA,
    bytes before a frame's start would be treated as garbage and discarded unprocessed —
    losing error lines exactly while streaming is on, i.e. during every real session."""
    ymr = protocol.build_command("VNYMR,1,2,3,0,0,0,0,0,9.8,0,0,0").encode()
    bin_frame = bytes([0xFA, 0x01, 0x28, 0x01]) + b"\x00" * 38

    for name, incoming, expected in [
        ("VNERR + ASCII frame", b"VNERR fail\r\n" + ymr, "VNERR fail"),
        ("VNERR + binary frame", b"VNERR baud-disabled\r\n" + bin_frame, "VNERR baud-disabled"),
        ("VNMODE + ASCII frame", b"VNMODE ASCII\r\n" + ymr, "VNMODE ASCII"),
    ]:
        tp = LoopbackTransport()
        vn = VN100(tp)
        tp.feed(incoming)
        for _ in range(4):
            vn.poll()
        assert expected in _drain_texts(vn), f"{name}: host line was swallowed along with the frame"

    # Frame decoding must stay intact: telemetry in the same stream should also be processed.
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(b"VNERR fail\r\n" + ymr)
    for _ in range(4):
        vn.poll()
    assert vn.get_data() is not None, "$VNYMR decoding broke while processing a host line"


def test_split_host_line_is_not_reported_twice():
    """A half line left in the prefix must not be retained, or the same VNERR
    ends up queued twice — once as the fragment and once complete."""
    ymr = protocol.build_command("VNYMR,1,2,3,0,0,0,0,0,9.8,0,0,0").encode()
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(b"VNERR fail\r\nVNERR mo")     # second line is a half line, then the frame arrives
    tp.feed(ymr)
    for _ in range(4):
        vn.poll()
    texts = _drain_texts(vn)
    assert texts.count("VNERR fail") == 1, f"complete line was reported more than once: {texts}"
    assert not any(t.startswith("VNERR mo") and t != "VNERR mo" for t in texts)


def test_vnymr_stream_does_not_reach_console():
    # The telemetry stream ($VNYMR) must not enter the response queue (otherwise 40-200 lines/s would flood the console).
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(protocol.build_command("VNYMR,1,2,3,0,0,0,0,0,9.8,0,0,0").encode())
    for _ in range(3):
        vn.poll()
    assert _drain_texts(vn) == []
    assert vn.get_data() is not None      # but it is still processed as data
