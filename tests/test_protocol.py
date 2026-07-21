"""
Unit tests for pyvn100.protocol.

Run (from repo root):
    .venv\\Scripts\\python.exe -m pytest -q
"""
import math

from pyvn100 import protocol as p
from pyvn100.types import Vn100Data


# ── XOR checksum ─────────────────────────────────────────────────

def test_xor_checksum_known_vector():
    # docs/protocol.md §2.2 -- externally verifiable example
    assert p.xor_checksum("VNYMR") == 0x5E


def test_xor_checksum_empty():
    assert p.xor_checksum("") == 0


# ── Command construction ──────────────────────────────────────────

def test_build_command_frame():
    cmd = p.build_command("VNRRG,6")
    assert cmd.startswith("$VNRRG,6*")
    assert cmd.endswith("\r\n")


def test_commands_checksum_consistent():
    """Checksum-only verification would be circular: verify_ascii() reuses the
    same checksum code that built the command, so a wrong body (e.g. wrong
    register number) would still pass. Anchor the body against the expected
    literal first, then check the checksum on top.
    """
    EXPECTED = {
        p.read_register(6):        "$VNRRG,6*",
        p.write_register(7, 40):   "$VNWRG,7,40*",
        p.write_register(5, 921600): "$VNWRG,5,921600*",
        p.write_settings():        "$VNWNV*",
        p.restore_factory():       "$VNRFS*",
        p.tare():                  "$VNTAR*",
        p.reset():                 "$VNRST*",
    }
    for cmd, prefix in EXPECTED.items():
        assert cmd.startswith(prefix), f"body drifted: {cmd!r} (expected prefix {prefix!r})"
        assert p.verify_ascii(cmd), f"checksum mismatch: {cmd!r}"
        assert cmd.endswith("\r\n"), f"missing line ending: {cmd!r}"


def test_write_register_format():
    assert p.write_register(7, 40).startswith("$VNWRG,7,40*")


# ── $VNYMR parsing ─────────────────────────────────────────────────

def _make_vnymr(vals) -> str:
    body = "VNYMR," + ",".join(f"{x:+.4f}" for x in vals)
    return p.build_command(body)


def test_parse_vnymr_valid():
    vals = [10.5, -5.25, 3.0, 0.2, -0.1, -0.4, 0.01, 0.02, 9.81, 0.001, -0.002, 0.003]
    line = _make_vnymr(vals)
    d = p.parse_vnymr(line)
    assert d is not None
    assert isinstance(d, Vn100Data)
    assert math.isclose(d.yaw, 10.5, abs_tol=1e-3)
    assert math.isclose(d.pitch, -5.25, abs_tol=1e-3)
    assert math.isclose(d.accel_z, 9.81, abs_tol=1e-3)
    assert math.isclose(d.gyro_z, 0.003, abs_tol=1e-4)


def test_parse_vnymr_corrupt_checksum():
    line = _make_vnymr([0] * 12)
    corrupt = line[:-4] + "00\r\n"
    assert p.parse_vnymr(corrupt) is None


def test_parse_vnymr_wrong_header():
    assert p.parse_vnymr("$VNQTN,1,2,3*00\r\n") is None


def test_parse_vnymr_missing_field():
    line = p.build_command("VNYMR,1,2,3")   # 3 fields instead of 12
    assert p.parse_vnymr(line) is None


def test_parse_vnymr_garbage_data():
    assert p.parse_vnymr("random text\r\n") is None
    assert p.parse_vnymr("") is None


# ── Binary CRC-16 ────────────────────────────────────────────────

def test_crc16_known_vector():
    # CRC-16/XMODEM standard check value
    assert p.crc16_ccitt(b"123456789") == 0x31C3


def test_crc16_roundtrip_zero():
    # VectorNav rule: appending the CRC big-endian and recomputing must give 0
    payload = bytes(range(32))
    crc = p.crc16_ccitt(payload)
    full = payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    assert p.crc16_ccitt(full) == 0
    assert p.binary_crc_ok(full)
