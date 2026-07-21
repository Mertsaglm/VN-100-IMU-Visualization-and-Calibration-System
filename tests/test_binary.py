"""
Binary protocol tests — encode/decode round-trip, CRC, end-to-end.
"""
import math

from pyvn100 import binary, VN100, SimTransport, Vn100Data


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def test_frame_length():
    assert binary.FRAME_LEN == 42


def test_encode_decode_roundtrip():
    d = Vn100Data(
        yaw=12.5, pitch=-3.25, roll=1.0,
        gyro_x=0.01, gyro_y=-0.02, gyro_z=0.03,
        accel_x=0.1, accel_y=0.2, accel_z=9.81,
    )
    frame = binary.encode(d)
    assert len(frame) == binary.FRAME_LEN
    assert frame[0] == binary.SYNC
    out = binary.decode(frame)
    assert out is not None
    assert math.isclose(out.yaw, 12.5, abs_tol=1e-4)
    assert math.isclose(out.accel_z, 9.81, abs_tol=1e-4)
    assert math.isclose(out.gyro_z, 0.03, abs_tol=1e-5)


def test_corrupt_crc_rejected():
    d = Vn100Data(yaw=1.0, accel_z=9.81)
    frame = bytearray(binary.encode(d))
    frame[10] ^= 0xFF
    assert binary.decode(bytes(frame)) is None


def test_wrong_sync_rejected():
    d = Vn100Data(yaw=1.0)
    frame = bytearray(binary.encode(d))
    frame[0] = 0x00
    assert binary.decode(bytes(frame)) is None


def test_wrong_output_group_rejected():
    # groups != 0x01 -> wrong output group, must be rejected (spec §3.3)
    d = Vn100Data(yaw=1.0)
    frame = bytearray(binary.encode(d))
    frame[1] = 0x02
    assert binary.decode(bytes(frame)) is None


def test_wrong_field_mask_rejected():
    # fieldMask != 0x0128 -> wrong field set, must be rejected
    d = Vn100Data(yaw=1.0)
    frame = bytearray(binary.encode(d))
    frame[2] = 0x29                # 0x0128 -> 0x0129
    assert binary.decode(bytes(frame)) is None


def test_truncated_frame_rejected():
    d = Vn100Data(yaw=1.0)
    frame = binary.encode(d)
    assert binary.decode(frame[:41]) is None
    assert binary.decode(b"") is None


def test_extract_frames_skips_garbage():
    d1 = Vn100Data(yaw=1.0, accel_z=9.81)
    d2 = Vn100Data(yaw=2.0, accel_z=9.80)
    buf = bytearray(b"\x00\x11\x22")
    buf += binary.encode(d1)
    buf += b"\xAB"
    buf += binary.encode(d2)
    frames = binary.extract_frames(buf)
    assert len(frames) == 2
    assert math.isclose(frames[0].yaw, 1.0, abs_tol=1e-4)
    assert math.isclose(frames[1].yaw, 2.0, abs_tol=1e-4)


def test_binary_end_to_end_simtransport():
    clk = FakeClock()
    tp = SimTransport(rate_hz=100.0, noise=False, clock=clk, fmt="binary")
    vn = VN100(tp, fmt="binary")

    clk.advance(1.0)
    total = 0
    while True:
        n = vn.poll()
        if n == 0:
            break
        total += n

    assert 99 <= total <= 102, f"expected ~100, got {total}"
    d = vn.get_data()
    assert d is not None
    assert 9.0 < d.accel_z < 10.5
