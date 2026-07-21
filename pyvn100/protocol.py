"""
pyvn100.protocol — VN-100 protocol layer (MID, platform-independent).

This module is the Python side of the docs/protocol.md contract and must
behave **identically** to `vn100_protocol.c` (M7) on the C side.

Contents:
  - ASCII XOR checksum + `$VNYMR` decoding
  - Command builders ($VNRRG / $VNWRG / $VNTAR / $VNWNV / $VNRFS / $VNRST)
  - VectorNav binary CRC-16 (same algorithm as the device, bit for bit)
"""
from __future__ import annotations

import numbers

from .types import Vn100Data

# ── Constants ─────────────────────────────────────────────────────
# NOTE: the binary sync byte (0xFA) lives in `binary.SYNC`, NOT here — that's
# the single source of truth (also checked by `tests/test_spec_constants.py`).
HEADER_VNYMR = "$VNYMR"


# ════════════════════════════════════════════════════════════════
#   ASCII — checksum and command building
# ════════════════════════════════════════════════════════════════

def xor_checksum(payload: str | bytes) -> int:
    """8-bit XOR of the characters between '$' and '*' (docs/protocol.md §2.2)."""
    if isinstance(payload, str):
        payload = payload.encode("ascii")
    cs = 0
    for b in payload:
        cs ^= b
    return cs & 0xFF


def build_command(body: str) -> str:
    """Build a full command frame from a body (e.g. 'VNRRG,06') -> '$VNRRG,06*CS\\r\\n'."""
    return f"${body}*{xor_checksum(body):02X}\r\n"


def _fmt(v) -> str:
    """Format a command argument IDENTICALLY to the C core.

    Floats are written with `%.6f` (matches vn100_protocol.c's
    `vn100_cmd_write_register_floats`). `%g` is NOT used: for small values it
    produces scientific notation (5e-05), which the VN-100 ASCII $VNWRG
    parser rejects — this bites especially on the small terms of the Reg 23
    calibration matrix. numpy scalars are also caught via numbers.*.
    """
    if isinstance(v, bool):
        return str(int(v))
    if isinstance(v, numbers.Integral):
        return str(int(v))
    if isinstance(v, numbers.Real):
        return f"{float(v):.6f}"
    return str(v)


def read_register(reg: int) -> str:
    """$VNRRG,<reg> — read a register."""
    return build_command(f"VNRRG,{reg}")


def write_register(reg: int, *values) -> str:
    """$VNWRG,<reg>,<v1>,... — write a register."""
    body = ",".join(["VNWRG", str(reg), *[_fmt(v) for v in values]])
    return build_command(body)


def write_settings() -> str:
    """$VNWNV — write settings to non-volatile memory."""
    return build_command("VNWNV")


def restore_factory() -> str:
    """$VNRFS — restore factory settings."""
    return build_command("VNRFS")


def tare() -> str:
    """$VNTAR — tare (use the current orientation as the reference).

    Warning: FW v2.x ONLY. $VNTAR is NOT in the FW v3.1.0.0 ICD §1.3 command
    list -> the sensor returns `$VNERR,04` (Invalid Command). Check
    `capabilities.has_tare` before calling.
    """
    return build_command("VNTAR")


def reset() -> str:
    """$VNRST — software reset."""
    return build_command("VNRST")


def async_pause() -> str:
    """$VNASY,0 — TEMPORARILY stop asynchronous output (ICD §1.3.9 / UM001 §5.1.8).

    Does NOT change any registers (ADOR/ADOF/Reg 75), only silences the
    stream. ICD's own rationale: avoid filtering continuously streaming
    async messages while sending a configuration command.

    ALSO a safety measure here: on the STM32 bridge, the sensor RX ISR
    (priority 1) preempts the host RX ISR (priority 5), and since the F7 has
    no RX FIFO the tolerance is only ~87 us. When a long configuration
    command (e.g. a 12-float Reg 23 write, ~11.5 ms on the wire) overlaps
    streaming telemetry, a byte can silently drop mid-command, corrupting
    the checksum and causing the sensor to return `$VNERR,03`. Silencing the
    stream closes this window (docs/protocol.md §8).
    """
    return build_command("VNASY,0")


def async_resume() -> str:
    """$VNASY,1 — resume asynchronous output (ICD §1.3.9)."""
    return build_command("VNASY,1")


def binary_output(reg: int, async_mode: int, rate_divisor: int,
                  group: int = 0x01, fields: int = 0x0128) -> str:
    """
    Configure a Binary Output register (75/76/77) (docs/protocol.md §4.2).

    $VNWRG,<reg>,<AsyncMode>,<RateDivisor>,<OutputGroup:hex>,<OutputField:hex>
    OutputGroup/OutputField are hex-encoded (this project uses
    75,2,4,01,0128 — AsyncMode=2/TTL Serial Port 2; UM001's generic example
    uses AsyncMode=1/Port 1). Output Hz = 800 / RateDivisor.
    """
    body = f"VNWRG,{reg},{async_mode},{rate_divisor},{group:02X},{fields:04X}"
    return build_command(body)


def set_gyro_bias() -> str:
    """$VNSGB — capture/save the current gyro bias estimate while the sensor is STATIONARY (UM001)."""
    return build_command("VNSGB")


# ════════════════════════════════════════════════════════════════
#   ASCII — decoding
# ════════════════════════════════════════════════════════════════

def verify_ascii(line: str) -> bool:
    """Verify the checksum of an ASCII message."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return False
    star = line.rindex("*")
    if star + 3 > len(line):
        return False
    try:
        recv = int(line[star + 1:star + 3], 16)
    except ValueError:
        return False
    return xor_checksum(line[1:star]) == recv


def parse_vnymr(line: str) -> Vn100Data | None:
    """
    Decode a $VNYMR line. Returns None if invalid (bad header/checksum/fields).

    Format: $VNYMR,yaw,pitch,roll,magX,magY,magZ,accX,accY,accZ,gyroX,gyroY,gyroZ*CS
    """
    line = line.strip()
    if not line.startswith(HEADER_VNYMR):
        return None
    # Header must match EXACTLY: extended headers like '$VNYMRZ,...' are rejected
    # (the C side already rejects these via memcmp+offset — parity).
    if len(line) > len(HEADER_VNYMR) and line[len(HEADER_VNYMR)] not in (",", "*"):
        return None
    if "*" not in line:
        return None

    star = line.rindex("*")
    body = line[1:star]

    # Verify checksum — EXACTLY 2 hex digits required after '*' (parity with the
    # C side; a truncated line like '*5' must not verify by luck with one digit).
    if star + 3 > len(line):
        return None
    try:
        recv = int(line[star + 1:star + 3], 16)
    except (ValueError, IndexError):
        return None
    if xor_checksum(body) != recv:
        return None

    parts = body.split(",")
    # 'VNYMR' + 12 fields
    if len(parts) < 13:
        return None
    try:
        v = [float(x) for x in parts[1:13]]
    except ValueError:
        return None

    return Vn100Data(
        yaw=v[0], pitch=v[1], roll=v[2],
        mag_x=v[3], mag_y=v[4], mag_z=v[5],
        accel_x=v[6], accel_y=v[7], accel_z=v[8],
        gyro_x=v[9], gyro_y=v[10], gyro_z=v[11],
    )


def parse_vnrrg(line: str):
    """
    Decode a register-read response: '$VNRRG,<reg>,<f1>,<f2>,...*CS' -> (reg:int, [str,...]).

    Checksum is verified. Returns None if not $VNRRG or malformed. Fields are
    returned as raw strings; typed decoding is done by the registers.decode_* functions.
    """
    line = line.strip()
    if not line.startswith("$VNRRG,") or "*" not in line:
        return None
    star = line.rindex("*")
    body = line[1:star]
    if star + 3 > len(line):           # exactly 2 hex digits required after '*' (truncated line)
        return None
    try:
        recv = int(line[star + 1:star + 3], 16)
    except (ValueError, IndexError):
        return None
    if xor_checksum(body) != recv:
        return None
    parts = body.split(",")            # ['VNRRG', '<reg>', '<f1>', ...]
    if len(parts) < 2:
        return None
    try:
        reg = int(parts[1])
    except ValueError:
        return None
    return reg, parts[2:]


# ════════════════════════════════════════════════════════════════
#   Binary — CRC-16 (VectorNav CCITT, init=0)
# ════════════════════════════════════════════════════════════════

def crc16_ccitt(data: bytes) -> int:
    """
    VectorNav binary protocol CRC-16 (docs/protocol.md §3.2).

    Matches the C core's function of the same name exactly. Mimics unsigned
    short (16-bit) arithmetic. Equivalent to CRC-16/XMODEM.
    """
    crc = 0
    for b in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= b
        crc ^= (crc & 0xFF) >> 4
        crc = (crc ^ ((crc << 12) & 0xFFFF)) & 0xFFFF
        crc = (crc ^ ((crc & 0x00FF) << 5)) & 0xFFFF
    return crc & 0xFFFF


def binary_crc_ok(packet_without_sync: bytes) -> bool:
    """
    Validate a full binary packet (after 0xFA). VectorNav's rule: valid iff
    the CRC computed over the entire body, CRC field included, equals 0.
    """
    return crc16_ccitt(packet_without_sync) == 0
