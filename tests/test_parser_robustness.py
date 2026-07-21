"""
Dual-mode (ASCII+binary) parser ROBUSTNESS tests - verify the "auto-detect each frame
from its header" rule (docs/protocol.md §4.3) holds under edge cases: '$'(0x24)/'\n'(0x0A)
bytes inside a binary payload, byte-by-byte feeding, half frames, garbage between frames,
and recovering from a corrupt CRC while still counting the error. All of these occur on a
real serial link, so the driver must handle them.
"""
from pyvn100 import VN100, LoopbackTransport, binary, protocol
from pyvn100.types import Vn100Data


def _good_frame(yaw=1.0):
    return binary.encode(Vn100Data(yaw=yaw, accel_z=9.81))


def _frame_with_payload(payload36: bytes) -> bytes:
    """Build a VALID header+CRC frame around the given 36-byte payload."""
    assert len(payload36) == 36
    header = bytes([binary.SYNC, 0x01, 0x28, 0x01])
    crc = protocol.crc16_ccitt(header[1:] + payload36)
    return header + payload36 + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def test_binary_payload_with_dollar_and_newline_is_not_corrupted():
    # Even with 0x24('$') and 0x0A('\n') in the payload, the parser must not
    # mangle the binary frame, emit a fake ASCII line, or count an error.
    payload = bytes([0x24, 0x0A, 0x24, 0x0A]) * 9      # 36 bytes, full of $ and \n
    frame = _frame_with_payload(payload)
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(frame)
    assert vn.poll() == 1                              # exactly 1 binary packet
    assert vn.stats()["errors"] == 0                   # no fake ASCII / no error
    assert vn.last_fmt == "binary"


def test_binary_frame_fed_byte_by_byte():
    # Even fed one byte at a time, the frame is only decoded once complete.
    frame = _good_frame(7.0)
    tp = LoopbackTransport()
    vn = VN100(tp)
    total = 0
    for b in frame:
        tp.feed(bytes([b]))
        total += vn.poll()
    assert total == 1
    assert abs(vn.get_data().yaw - 7.0) < 1e-3


def test_half_binary_frame_is_awaited():
    frame = _good_frame(2.0)
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(frame[:41])
    assert vn.poll() == 0                              # 41 bytes -> not complete yet
    assert vn.stats()["errors"] == 0                   # half frame is NOT an error
    tp.feed(frame[41:])
    assert vn.poll() == 1                              # last byte -> decoded


def test_garbage_between_frames_is_skipped():
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(b"\x00\x11\x22")
    tp.feed(_good_frame(1.0))
    tp.feed(b"\x99\x99")
    tp.feed(_good_frame(2.0))
    assert vn.poll() == 2
    assert abs(vn.get_data().yaw - 2.0) < 1e-3


def test_corrupt_crc_binary_counts_error_and_recovers():
    # Header correct but CRC corrupt -> counted as an error; next frame still decodes.
    frame = bytearray(_good_frame(1.0))
    frame[20] ^= 0xFF                                  # corrupt payload -> CRC no longer matches
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(bytes(frame))
    tp.feed(_good_frame(2.0))
    n = vn.poll()
    assert n == 1                                      # the good frame decoded
    assert vn.stats()["errors"] >= 1                   # corrupt frame counted as an error
    assert abs(vn.get_data().yaw - 2.0) < 1e-3


def test_ascii_and_binary_in_same_poll_are_ordered_correctly():
    # Mixed stream: ASCII line + binary frame -> both decode, format is detected.
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(protocol.build_command("VNYMR,1,2,3,0.1,0.1,0.1,0,0,9.81,0,0,0"))
    tp.feed(_good_frame(5.0))
    assert vn.poll() == 2
    assert vn.last_fmt == "binary"                     # last decoded was binary


def test_missing_newline_then_embedded_dollar_resyncs():
    """C-core parity: when a '\\n' is lost, two frames merge into one line. The C side
    resyncs on seeing '$' and decodes the LAST frame; Python must match, or it treats the
    block as one line, fails the checksum, and drops the sound response too - losing not
    just a frame but a COMMAND REPLY (the calibration 'read back to verify' proof disappears)."""
    from pyvn100 import VN100, protocol
    from pyvn100.transport import LoopbackTransport

    ymr = protocol.build_command("VNYMR,1,2,3,0,0,0,0,0,9.8,0,0,0")
    rrg = protocol.build_command("VNRRG,23,1.5,0,0,0,1,0,0,0,1,0.1,0.2,0.3")

    tp = LoopbackTransport()
    vn = VN100(tp)
    # SIMULATED '\n' LOSS: a sound $VNRRG is glued right after a truncated $VNYMR
    half = ymr.rstrip("\r\n")[:-4]                    # checksum truncated -> undecodable
    tp.feed((half + rrg).encode("ascii"))
    for _ in range(4):
        vn.poll()

    texts = [t for t, _e, _ts in vn.drain_responses()]
    assert any(t.startswith("$VNRRG,23") for t in texts), \
        "no resync on embedded '$' -- the sound register reply was swallowed with the half frame"
    assert vn.error_count >= 1, "the discarded half frame was not counted"


def test_valid_single_frame_line_resync_behavior_is_unchanged():
    """Counter-check: on a normal line, `rfind('$')` finds the start itself (j==0) ->
    behavior must stay identical, error_count must NOT increase."""
    from pyvn100 import VN100, protocol
    from pyvn100.transport import LoopbackTransport

    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(protocol.build_command("VNYMR,1,2,3,0,0,0,0,0,9.8,0,0,0").encode("ascii"))
    for _ in range(3):
        vn.poll()
    assert vn.get_data() is not None
    assert vn.error_count == 0, "a false error was counted on a sound frame"
