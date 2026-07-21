"""
Binary WIRE-FORMAT spec tests.

Separate from test_binary.py, which only checks encode<->decode CONSISTENCY
(self-verification) -- if `_FIELDS_G1` were changed to a wrong value, that
round-trip would still pass even though the sensor would never produce that
frame. These tests check the raw on-wire bytes directly against the
docs/protocol.md §3.3 CONTRACT (UM001 Rev 2.22 Common group: bit3 YPR,
bit5 AngularRate, bit8 Accel), independently of `binary.decode`.
"""
import math
import struct

from pyvn100 import binary, protocol
from pyvn100.types import Vn100Data


def _golden():
    d = Vn100Data(yaw=10.0, pitch=-5.0, roll=3.0,
                  gyro_x=0.5, gyro_y=-0.25, gyro_z=0.125,
                  accel_x=1.0, accel_y=-2.0, accel_z=9.81)
    return d, binary.encode(d)


def test_header_bytes_match_spec():
    # docs/protocol.md §3.3: 0xFA | OutputGroup=0x01 (Common) | OutputField=0x0128 (LE -> 28 01)
    _, f = _golden()
    assert len(f) == 42
    assert f[0] == 0xFA, "sync byte must be 0xFA"
    assert f[1] == 0x01, "OutputGroup must be Common (0x01)"
    assert f[2] == 0x28 and f[3] == 0x01, "OutputField must be 0x0128 little-endian (28 01)"


def test_payload_field_order_matches_spec():
    # Fields in ascending bit order: YawPitchRoll -> AngularRate(gyro) -> Accel, 9x float32 LE.
    # Read independently with struct, WITHOUT using binary.decode.
    d, f = _golden()
    vals = struct.unpack("<9f", f[4:40])
    expected = [d.yaw, d.pitch, d.roll,
                d.gyro_x, d.gyro_y, d.gyro_z,
                d.accel_x, d.accel_y, d.accel_z]
    for got, want in zip(vals, expected):
        assert math.isclose(got, want, abs_tol=1e-4), f"field order does not match spec: {got} != {want}"


def test_crc_is_big_endian_and_zeros_over_the_whole_packet():
    # docs/protocol.md §3.2: CRC-16 is appended big-endian; the receiver must
    # get 0 over the whole packet (everything after 0xFA).
    _, f = _golden()
    crc_wire = (f[40] << 8) | f[41]                  # big-endian read
    assert crc_wire == protocol.crc16_ccitt(f[1:40]), "CRC must be computed over the body"
    assert protocol.crc16_ccitt(f[1:42]) == 0, "result must be 0 over the body including the CRC"


def test_decode_returns_the_same_values():
    # Consistency: an independent struct read must match decode (two-way assurance)
    d, f = _golden()
    out = binary.decode(f)
    assert out is not None
    assert math.isclose(out.yaw, d.yaw, abs_tol=1e-4)
    assert math.isclose(out.gyro_z, d.gyro_z, abs_tol=1e-5)
    assert math.isclose(out.accel_z, d.accel_z, abs_tol=1e-4)


def test_ascii_vnymr_field_order_matches_spec():
    # ASCII $VNYMR: yaw,pitch,roll, MAG(xyz), ACCEL(xyz), GYRO(xyz) -- a DIFFERENT
    # order than binary (mag included). Spec §2.3. Verify independently by
    # counting fields in the raw text.
    d = Vn100Data(yaw=1.0, pitch=2.0, roll=3.0,
                  mag_x=0.1, mag_y=0.2, mag_z=0.3,
                  accel_x=4.0, accel_y=5.0, accel_z=6.0,
                  gyro_x=0.01, gyro_y=0.02, gyro_z=0.03)
    from pyvn100.simulator import Vn100Simulator
    body = Vn100Simulator.encode_ascii(d).split("*")[0]      # "$VNYMR,...."
    parts = body.split(",")
    assert parts[0] == "$VNYMR"
    # order: [yaw,pitch,roll, magx,magy,magz, accx,accy,accz, gyrox,gyroy,gyroz]
    assert math.isclose(float(parts[4]), 0.1, abs_tol=1e-3)   # magX field 4
    assert math.isclose(float(parts[7]), 4.0, abs_tol=1e-3)   # accX field 7
    assert math.isclose(float(parts[10]), 0.01, abs_tol=1e-3)  # gyroX field 10


# ── ASCII WIRE-FORMAT GOLDEN ANCHORS ────────────────────────────
# Shared contract between the C and Python implementations. Counterparts:
# pc/host_selftest.c -- "encode body matches Python ascii_frame exactly" and
# "Reg 23 float format (%.6f) matches Python protocol.write_register exactly".
# Do NOT derive the golden value from the producer (circular); it's a literal
# so a drift on either side breaks the test, forcing both to be updated together.

_YMR_GOLDEN = ("$VNYMR,+12.500,-3.250,+90.000,"
               "+0.2280,-0.0150,-0.3870,"
               "+0.100,-0.200,+9.810,"
               "+0.0010,-0.0020,+0.0030")


def test_ascii_encode_matches_C_core_EXACTLY():
    """Without a Python-side golden, sim<->C `$VNYMR` parity would be anchored
    only on the C side -- field order/precision could drift here and the
    suite would still pass."""
    from pyvn100.simulator import Vn100Simulator
    d = Vn100Data(yaw=12.5, pitch=-3.25, roll=90.0,
                  mag_x=0.228, mag_y=-0.015, mag_z=-0.387,
                  accel_x=0.1, accel_y=-0.2, accel_z=9.81,
                  gyro_x=0.001, gyro_y=-0.002, gyro_z=0.003)
    line = Vn100Simulator.encode_ascii(d)
    assert line.startswith(_YMR_GOLDEN), f"drifted from the C golden value:\n{line}"
    # Field order: ypr, MAG, accel, gyro (DIFFERENT from binary -- docs/protocol.md)
    fields = line.split("*")[0].split(",")
    assert fields[0] == "$VNYMR" and len(fields) == 13


def test_reg23_float_format_is_fixed_no_scientific_notation():
    """Reg 23 calibration write's `%.6f` format needs its own test: drifting to
    `%g` produces things like `5e-05` for a 12-float write -- the sensor
    REJECTS that ($VNERR) or misparses it, and no other test would catch this regression."""
    s = protocol.write_register(23, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                                0.00005, -0.08, 0.03)
    expected = ("$VNWRG,23,1.000000,0.000000,0.000000,"
                "0.000000,1.000000,0.000000,"
                "0.000000,0.000000,1.000000,"
                "0.000050,-0.080000,0.030000")
    assert s.startswith(expected), f"Reg 23 float format drifted:\n{s}"
    body = s.split("*")[0]
    assert "e" not in body and "E" not in body, "scientific notation leaked in (%g drift)"
    # A small term must not round to zero -- the hard-iron component would be lost
    assert "0.000050" in body
