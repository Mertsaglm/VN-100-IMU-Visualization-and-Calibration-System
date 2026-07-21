"""
pyvn100.hostlink — PC <-> STM32 host command protocol (docs/protocol.md §7).

The dashboard sends commands to the STM32; the STM32 (host_link.c) parses them
and applies them to the VN-100. Commands start with "VN " (no dollar sign) so
they can't be confused with the VN-100's own "$VN..." messages.

  VN PING              -> VNPONG
  VN MODE ASCII|BINARY -> output mode (reg 6 / reg 75)
  VN FREQ <hz>         -> ASCII: ADOF (reg 7); binary: RateDivisor (reg 75)
  VN TYPE <ador>       -> set ADOR
  VN TARE / SAVE / FACTORY
  VN RAW <text>        -> forward a raw command straight to the sensor (incl. $VNSGB gyro bias)
  (VN BAUD is INTENTIONALLY disabled in firmware; this module has no builder for it.)

Same format as host_link.c on the C side.
"""
from __future__ import annotations

from . import protocol
from .registers import HSIMode, HSIOutput, Reg


def ping() -> str:
    return "VN PING\n"


def set_freq(hz: int) -> str:
    return f"VN FREQ {int(hz)}\n"


def set_type(ador: int) -> str:
    return f"VN TYPE {int(ador)}\n"


def tare() -> str:
    """Warning: FW v2.x ONLY — $VNTAR does not exist on FW v3.1.0.0 (capabilities.has_tare)."""
    return "VN TARE\n"


def save() -> str:
    return "VN SAVE\n"


def factory() -> str:
    return "VN FACTORY\n"


def raw(text: str) -> str:
    return f"VN RAW {text}\n"


def reset() -> str:
    """$VNRST — software reset. Secures the Kalman filter after WNV
    (UM001 §5.1.3: a Write Settings while moving also requires a Reset)."""
    return raw(protocol.reset().strip())


def set_mode(mode: str) -> str:
    """'VN MODE ASCII|BINARY' — select output mode (ASCII for demos, binary for operation)."""
    return f"VN MODE {mode.upper()}\n"


def gyro_bias_capture() -> str:
    """Capture gyro bias while STATIONARY ($VNSGB, via the bridge).

    Target register depends on FW (FW3: Reg 43, FW2: Reg 74) -> capabilities.gyro_bias_reg.
    """
    return raw(protocol.set_gyro_bias().strip())


def async_pause() -> str:
    """'VN RAW $VNASY,0' — temporarily silence the stream (before writing config; see protocol.async_pause)."""
    return raw(protocol.async_pause().strip())


def async_resume() -> str:
    """'VN RAW $VNASY,1' — resume the stream."""
    return raw(protocol.async_resume().strip())


# ── Register access (via the STM32 bridge: VN RAW $VN...) ─────

def read_reg(reg: int) -> str:
    """'VN RAW $VNRRG,<reg>*CS' — triggers a register read from the sensor."""
    return raw(protocol.read_register(reg).strip())


def write_reg(reg: int, *values) -> str:
    """'VN RAW $VNWRG,<reg>,<v...>*CS' — writes a register on the sensor."""
    return raw(protocol.write_register(reg, *values).strip())


# ── HSI (onboard hard/soft-iron) control shortcuts (Reg 44) ────

def hsi_reset(rate: int = 5, output: int = HSIOutput.USE_ONBOARD) -> str:
    """RESET the onboard HSI solution and start it running (reconverge for the current environment)."""
    return write_reg(Reg.HSI_CONTROL, HSIMode.RESET, output, rate)


def hsi_run(rate: int = 5, output: int = HSIOutput.USE_ONBOARD) -> str:
    """RUN the onboard HSI (keep adapting using the existing solution).
    Unused in the repo currently; kept for symmetry with reset/off (HSI RUN primitive)."""
    return write_reg(Reg.HSI_CONTROL, HSIMode.RUN, output, rate)


def hsi_off(output: int = HSIOutput.USE_ONBOARD) -> str:
    """TURN OFF the onboard HSI (freeze the solution); output correction is preserved via 'output'."""
    return write_reg(Reg.HSI_CONTROL, HSIMode.OFF, output, 5)


def hsi_status() -> str:
    """Trigger a read of Reg 46 (calibration status: NumMeas/AvgResidual/bins — 7 or 8
    bins depending on hardware; the decode side tolerates either via len(bins))."""
    return read_reg(Reg.HSI_STATUS)


def parse(line: str):
    """'VN FREQ 200' -> ('FREQ', ['200']). Returns None if not a host command."""
    line = line.strip()
    if not line.startswith("VN "):
        return None
    parts = line[3:].split()
    if not parts:
        return None
    return parts[0].upper(), parts[1:]
