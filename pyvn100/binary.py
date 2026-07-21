"""
pyvn100.binary — VN-100 binary protocol (MID, docs/protocol.md §3).

Selected output configuration (for high rate):
  Group 1 (Common): YawPitchRoll (bit3) + AngularRate (bit5) + Accel (bit8)
  -> 9 x float32 (little-endian) = 36-byte payload

Frame (42 bytes):
  0xFA | groups(0x01) | fieldMask(LE,2B=0x0128) | payload(36B) | CRC16(BE,2B)

CRC: vn100 CRC-16 over all bytes after 0xFA (CRC included) must equal 0.

Matches vn100_binary.c exactly; bit positions (3/5/8) and field order were
verified against the UM001 Rev 2.22 Common-group table. To stream this frame,
the sensor's Binary Output register is written: $VNWRG,75,2,4,01,0128
(docs/protocol.md §4.2).
"""
from __future__ import annotations

import struct

from .protocol import crc16_ccitt
from .types import Vn100Data

SYNC = 0xFA
_GROUPS = 0x01
_FIELDS_G1 = (1 << 3) | (1 << 5) | (1 << 8)   # YPR, AngularRate, Accel = 0x0128
_PAYLOAD_FMT = "<9f"
_PAYLOAD_LEN = 36
FRAME_LEN = 1 + 1 + 2 + _PAYLOAD_LEN + 2       # 42


def encode(d: Vn100Data) -> bytes:
    """Vn100Data -> 42-byte binary frame."""
    header = struct.pack("<BBH", SYNC, _GROUPS, _FIELDS_G1)
    payload = struct.pack(
        _PAYLOAD_FMT,
        d.yaw, d.pitch, d.roll,
        d.gyro_x, d.gyro_y, d.gyro_z,
        d.accel_x, d.accel_y, d.accel_z,
    )
    crc = crc16_ccitt(header[1:] + payload)     # excludes sync byte
    return header + payload + struct.pack(">H", crc)


def decode(frame: bytes) -> Vn100Data | None:
    """42-byte frame -> Vn100Data, or None if invalid."""
    if len(frame) < FRAME_LEN or frame[0] != SYNC:
        return None
    if frame[1] != _GROUPS:
        return None
    if struct.unpack_from("<H", frame, 2)[0] != _FIELDS_G1:
        return None
    if crc16_ccitt(frame[1:FRAME_LEN]) != 0:    # must be 0 including CRC
        return None
    v = struct.unpack_from(_PAYLOAD_FMT, frame, 4)
    return Vn100Data(
        yaw=v[0], pitch=v[1], roll=v[2],
        gyro_x=v[3], gyro_y=v[4], gyro_z=v[5],
        accel_x=v[6], accel_y=v[7], accel_z=v[8],
    )


def extract_frames(buf: bytearray) -> list[Vn100Data]:
    """Extract complete frames from a byte buffer (consuming them) and return decoded ones."""
    out: list[Vn100Data] = []
    while True:
        i = buf.find(bytes([SYNC]))
        if i < 0:
            buf.clear()
            break
        if i > 0:
            del buf[:i]                 # drop garbage before sync
        if len(buf) < FRAME_LEN:
            break                       # full frame not yet available
        d = decode(bytes(buf[:FRAME_LEN]))
        if d is not None:
            del buf[:FRAME_LEN]
            out.append(d)
        else:
            del buf[:1]                 # bad sync — advance one byte
    return out
