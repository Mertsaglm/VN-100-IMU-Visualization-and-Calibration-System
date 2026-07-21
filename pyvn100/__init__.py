"""
pyvn100 — Portable, layered Python library for the VN-100 IMU.

Mirrors the same 3-layer architecture as the C core (Core/Src/vn100*.c):
  - transport (LOW)  : byte transport      -> transport.py   (M3)
  - protocol (MID)   : decode/CRC/commands -> protocol.py    (M2)  <- single source of truth: docs/protocol.md
  - api (HIGH)       : meaningful actions  -> vn100.py       (M3)

Orthogonal concern: `capabilities.py` centralizes all firmware GENERATION
differences (FW 3.1.0.0 vs 2.1) in ONE place. Don't hardcode "if FW version
is X do Y" in the code; call `capabilities_for(fw)` instead. See docs/protocol.md §5.3.

See docs/protocol.md
"""
from .types import Vn100Data
from . import protocol, registers, binary, capabilities, hostlink, link, selfcheck
from .capabilities import Capabilities, capabilities_for
from .transport import Transport, SerialTransport, LoopbackTransport
from .vn100 import VN100
from .simulator import Vn100Simulator, SimTransport
from .replay import ReplayTransport, HybridTransport

__all__ = [
    "Vn100Data",
    "protocol",
    "registers",
    "binary",
    "capabilities",
    "Capabilities",
    "capabilities_for",
    "hostlink",
    "link",
    "selfcheck",
    "Transport",
    "SerialTransport",
    "LoopbackTransport",
    "VN100",
    "Vn100Simulator",
    "SimTransport",
    "ReplayTransport",
    "HybridTransport",
]
__version__ = "0.1.0"
