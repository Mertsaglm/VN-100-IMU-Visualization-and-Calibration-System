"""
pyvn100.link — Connection-mode abstraction (Strategy): STM bridge vs direct USB-TTL.

The same dashboard must work over two physical topologies:

  BRIDGE  — PC -> STM32 (ST-Link VCP) -> VN-100. Speaks the STM32 host protocol:
            `VN RAW $VN...`, `VN FREQ`, `VN MODE`, `VN TARE`, `VN SAVE`, `VN FACTORY`.
            (Builder: `pyvn100.hostlink`, mirrors the STM32 `host_link.c` contract.)
  DIRECT  — PC -> USB-TTL -> VN-100 (no STM32). Speaks the raw sensor protocol:
            `$VNRRG`, `$VNWRG`, `$VNTAR`, `$VNWNV`, `$VNRFS`, `$VNSGB` (checksum+CRLF).
            (Builder: `pyvn100.protocol`.)

The ONLY difference between the two modes is how a command is **framed**; the
receive/parse side (VN100._scan / _handle_line) handles both automatically and
NEVER changes (sensor lines always start with `$`; the bridge's dollar-less
VNERR/VNPONG is a harmless superset).

Design — PURE BUILDER, not executor: each method returns "WHAT to send" as a
wire-ready `list[str]`; "HOW to write + log it" stays at the call site
(`app._emit`, dialog `_send/_send_all`), keeping presentation/transport policy
out of the protocol layer. Pure/stateless -> verified in unit tests by plain
string equality. No mode state is kept; `set_freq`/`set_output_mode` take the
current output mode as a caller-supplied parameter (single source of truth:
`VN100.fmt`).

DIRECT output of `set_output_mode`/`set_freq` can span multiple registers, so
the return type is always `list[str]`, even for length-1 results. Command
order matches `VN100.set_output_mode` EXACTLY (parity: identical behavior).

Import rule: this module imports ONLY `hostlink`+`protocol`+`registers`,
NEVER `vn100` (to avoid a cycle). `detect_link` takes `transport` as a parameter.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Optional

from . import hostlink, protocol, registers
from .registers import HSIOutput, Reg

# Link mode labels (shared by CLI `--bridge/--direct`, the UI segment, and detect_link)
BRIDGE = "bridge"
DIRECT = "direct"

# Sensible default when no output rate is given in DIRECT mode (matches app._populate_freqs)
_DEFAULT_ASCII_HZ = registers.ADOF_DEFAULT_HZ   # SINGLE SOURCE OF TRUTH (ICD §3.2.4)
_DEFAULT_BINARY_HZ = 200


class CommandLink(ABC):
    """Interface that turns logical commands into wire-ready `list[str]` (pure, stateless builder).

    Concrete implementations: `BridgeLink` (STM bridge framing) and `DirectLink` (raw `$VN`).
    Each method returns one or more wire-ready command lines; the caller writes them in order.
    """

    mode: str = ""     # "bridge" | "direct" — for UI/log/diagnostics

    @abstractmethod
    def read_register(self, reg: int) -> list[str]: ...

    @abstractmethod
    def write_register(self, reg: int, *values) -> list[str]: ...

    @abstractmethod
    def set_output_mode(self, mode: str, rate_hz: Optional[int] = None) -> list[str]: ...

    @abstractmethod
    def set_freq(self, hz: int, binary: bool = False) -> list[str]: ...

    @abstractmethod
    def set_type(self, ador: int) -> list[str]: ...

    @abstractmethod
    def tare(self) -> list[str]: ...

    @abstractmethod
    def async_pause(self) -> list[str]: ...

    @abstractmethod
    def async_resume(self) -> list[str]: ...

    @abstractmethod
    def save(self) -> list[str]: ...

    @abstractmethod
    def factory(self) -> list[str]: ...

    @abstractmethod
    def reset(self) -> list[str]: ...

    @abstractmethod
    def gyro_bias(self) -> list[str]: ...

    @abstractmethod
    def hsi_reset(self, rate: int = 5, output: int = HSIOutput.USE_ONBOARD) -> list[str]: ...

    @abstractmethod
    def hsi_off(self, output: int = HSIOutput.USE_ONBOARD) -> list[str]: ...

    @abstractmethod
    def hsi_status(self) -> list[str]: ...


class BridgeLink(CommandLink):
    """STM32 bridge mode — frames commands via `hostlink` (VN RAW / VN <verb>).

    The STM32 translates opaque verbs like `VN MODE`/`VN FREQ` into register
    logic internally (e.g. keeps the last freq in ASCII, clamps to 50 Hz), so
    BridgeLink returns exactly ONE bridge line per logical command.
    """

    mode = BRIDGE

    def read_register(self, reg: int) -> list[str]:
        return [hostlink.read_reg(reg)]

    def write_register(self, reg: int, *values) -> list[str]:
        return [hostlink.write_reg(reg, *values)]

    def set_output_mode(self, mode: str, rate_hz: Optional[int] = None) -> list[str]:
        # rate_hz is IGNORED on the bridge: STM keeps the last `VN FREQ` across `VN MODE`.
        _validate_mode(mode)
        return [hostlink.set_mode(mode)]

    def set_freq(self, hz: int, binary: bool = False) -> list[str]:
        # STM translates `VN FREQ` to reg7/reg75 itself based on mode -> `binary` is irrelevant on the bridge.
        return [hostlink.set_freq(hz)]

    def set_type(self, ador: int) -> list[str]:
        return [hostlink.set_type(ador)]

    def tare(self) -> list[str]:
        return [hostlink.tare()]

    def async_pause(self) -> list[str]:
        return [hostlink.async_pause()]

    def async_resume(self) -> list[str]:
        return [hostlink.async_resume()]

    def save(self) -> list[str]:
        return [hostlink.save()]

    def factory(self) -> list[str]:
        return [hostlink.factory()]

    def reset(self) -> list[str]:
        return [hostlink.reset()]

    def gyro_bias(self) -> list[str]:
        return [hostlink.gyro_bias_capture()]

    def hsi_reset(self, rate: int = 5, output: int = HSIOutput.USE_ONBOARD) -> list[str]:
        return [hostlink.hsi_reset(rate=rate, output=output)]

    def hsi_off(self, output: int = HSIOutput.USE_ONBOARD) -> list[str]:
        return [hostlink.hsi_off(output=output)]

    def hsi_status(self) -> list[str]:
        return [hostlink.hsi_status()]


class DirectLink(CommandLink):
    """Direct USB-TTL mode — frames commands via `protocol` as raw `$VN...*CS\\r\\n`.

    With no STM bridge, opaque verbs (`set_output_mode`, `set_freq`) expand
    directly into a register sequence. Command order + divisor rule match
    `VN100.set_output_mode` EXACTLY (parity).
    """

    mode = DIRECT

    def read_register(self, reg: int) -> list[str]:
        return [protocol.read_register(reg)]

    def write_register(self, reg: int, *values) -> list[str]:
        return [protocol.write_register(reg, *values)]

    def set_output_mode(self, mode: str, rate_hz: Optional[int] = None) -> list[str]:
        """ASCII: [disable reg75, reg6=VNYMR, reg7=hz]; BINARY: [reg6=0, reg75 divisor].
        Order matches `VN100.set_output_mode` exactly, to avoid an inconsistent
        output if applied only partway."""
        mode = _validate_mode(mode)
        if mode == "binary":
            hz = rate_hz or _DEFAULT_BINARY_HZ
            return [
                protocol.write_register(Reg.ASYNC_DATA_OUTPUT_TYPE, 0),       # disable ASCII (reg6=0)
                protocol.binary_output(Reg.BINARY_OUTPUT_1,                   # enable binary (reg75)
                                       registers.SENSOR_ASYNC_PORT, _divisor(hz)),
            ]
        hz = rate_hz or _DEFAULT_ASCII_HZ
        return [
            protocol.binary_output(Reg.BINARY_OUTPUT_1, 0, 4),               # disable binary (reg75=0)
            protocol.write_register(Reg.ASYNC_DATA_OUTPUT_TYPE, registers.AsyncType.VNYMR),  # reg6=14
            protocol.write_register(Reg.ASYNC_DATA_OUTPUT_FREQ, hz),         # reg7=hz
        ]

    def set_freq(self, hz: int, binary: bool = False) -> list[str]:
        if binary:
            # Rewrite the reg75 rate divisor (async_mode=PORT2 is preserved) -> output Hz = 800/divisor.
            return [protocol.binary_output(Reg.BINARY_OUTPUT_1,
                                           registers.SENSOR_ASYNC_PORT, _divisor(hz))]
        return [protocol.write_register(Reg.ASYNC_DATA_OUTPUT_FREQ, hz)]     # reg7 (ADOF)

    def set_type(self, ador: int) -> list[str]:
        return [protocol.write_register(Reg.ASYNC_DATA_OUTPUT_TYPE, ador)]

    def tare(self) -> list[str]:
        return [protocol.tare()]

    def async_pause(self) -> list[str]:
        return [protocol.async_pause()]

    def async_resume(self) -> list[str]:
        return [protocol.async_resume()]

    def save(self) -> list[str]:
        return [protocol.write_settings()]

    def factory(self) -> list[str]:
        return [protocol.restore_factory()]

    def reset(self) -> list[str]:
        return [protocol.reset()]

    def gyro_bias(self) -> list[str]:
        return [protocol.set_gyro_bias()]

    def hsi_reset(self, rate: int = 5, output: int = HSIOutput.USE_ONBOARD) -> list[str]:
        return [protocol.write_register(Reg.HSI_CONTROL, registers.HSIMode.RESET, output, rate)]

    def hsi_off(self, output: int = HSIOutput.USE_ONBOARD) -> list[str]:
        return [protocol.write_register(Reg.HSI_CONTROL, registers.HSIMode.OFF, output, 5)]

    def hsi_status(self) -> list[str]:
        return [protocol.read_register(Reg.HSI_STATUS)]


# ── Helpers ──────────────────────────────────────────────────

def _validate_mode(mode: str) -> str:
    """Validate 'ascii'/'binary' (same strictness as VN100.set_output_mode); raises ValueError otherwise."""
    m = mode.lower()
    if m not in ("ascii", "binary"):
        raise ValueError(f"mode must be 'ascii' or 'binary', got: {mode!r}")
    return m


def _divisor(hz: int) -> int:
    """Binary output RateDivisor — matches VN100.set_output_mode EXACTLY (integer division, min 1)."""
    divisor = (registers.IMU_RATE_HZ // hz) if hz and hz > 0 else 4
    return max(1, divisor)


# ── Auto-detection ──────────────────────────────────────────────

def detect_link(transport, timeout: float = 0.8, forced: Optional[str] = None,
                ping_interval: float = 0.3) -> CommandLink:
    """Probe for an STM bridge and return the matching CommandLink.

    Writes `VN PING`, then watches for `VNPONG` in incoming bytes for
    `timeout` seconds. VNPONG is a dollar-less control reply produced only by
    the STM bridge (the sensor itself never produces it), so it's a clean
    "bridge present" signal. Returns BridgeLink if found, else DirectLink
    (timeout/error).

    If `forced` ('bridge'/'direct') is given, the probe is skipped. On a
    non-writable transport such as replay (writable=False), probing is
    impossible -> BridgeLink (commands are already no-ops; caught by the
    replay guard at the seam).

    NOTE: must be called BEFORE the reader thread starts, or it will consume
    those bytes. Never touches the RX path (_scan) — reads raw bytes directly
    from the transport. A stray `VN PING\\n` sent to a DIRECT sensor is
    harmless (invalid frame -> ignored).
    """
    if forced == BRIDGE:
        return BridgeLink()
    if forced == DIRECT:
        return DirectLink()
    if not getattr(transport, "writable", True):
        return BridgeLink()

    buf = bytearray()
    start = time.monotonic()
    last_ping = -ping_interval        # ping immediately on the first round
    while (time.monotonic() - start) < timeout:
        now = time.monotonic()
        if (now - last_ping) >= ping_interval:
            try:
                transport.write(hostlink.ping())
            except Exception:
                return DirectLink()   # assume no bridge if we can't write
            last_ping = now
        try:
            chunk = transport.read()
        except Exception:
            return DirectLink()
        if chunk:
            buf.extend(chunk)
            if b"VNPONG" in buf:
                return BridgeLink()
            if len(buf) > 8192:       # don't grow unbounded under a stream (VNPONG is short)
                del buf[:-1024]
        else:
            time.sleep(0.01)
    return DirectLink()
