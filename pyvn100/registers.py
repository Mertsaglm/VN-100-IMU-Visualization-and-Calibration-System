"""
pyvn100.registers — VN-100 register IDs and related constants.

Primary source: VectorNav VN-100 ICD (Firmware v3.1.0.0), STM/VN100_ICD_fw3.1.0.0.pdf
— the ICD for the hardware in the field. Verified byte-for-byte against the
hardware's actual Reg 4 response (`$VNRRG,04,3.1.0.0*77`).

Secondary (older hardware): UM001 Rev 2.22 / FW v2.1.0.0. Version-dependent
differences are centralized in `pyvn100/capabilities.py`, not hardcoded here.
Full mapping/diff table: docs/protocol.md §5, §5.3.

Mirrors vn100_registers.h on the C side.
"""
from __future__ import annotations

# The baseline firmware this project was VERIFIED against (field hardware). Used by capabilities.py.
ICD_FW_BASELINE = "3.1.0.0"


class Reg:
    """VN-100 register IDs (ICD FW v3.1.0.0; differences noted)."""

    USER_TAG = 0
    MODEL_NUMBER = 1               # Reg 1 — Model (ICD §4.2.1); used for bring-up identification
    HARDWARE_REVISION = 2          # Reg 2 — Hardware Version (ICD §4.2.2)
    SERIAL_NUMBER = 3
    FIRMWARE_VERSION = 4           # Reg 4 — Firmware Version, string[20] (ICD §4.2.4)
    SERIAL_BAUD_RATE = 5           # Reg 5
    ASYNC_DATA_OUTPUT_TYPE = 6     # Reg 6 — ADOR (ASCII message type); ICD default is 14 (YMR)
    ASYNC_DATA_OUTPUT_FREQ = 7     # Reg 7 — ADOF (ASCII Hz)
    YPR = 8                        # Reg 8 — Yaw/Pitch/Roll
    MAG_COMPENSATED = 17           # Reg 17 — Compensated Magnetometer (ICD §4.4.1)
    GYRO_COMPENSATED = 19          # Reg 19 — Compensated Gyro (ICD §4.4.2/§4.4.3).
    # NOTE: Reg 19 is NOT the magnetometer — that's Reg 17. Easy to mix up; per
    # STM/VN100_ICD_fw3.1.0.0.pdf "Register Index": 17 Compensated Magnetometer,
    # 18 Compensated Accelerometer, 19 Compensated Gyro, 20 Compensated IMU.
    MAG_CALIBRATION = 23           # Reg 23 — Magnetometer Calibration (C 3x3 + B 3x1) — same in both ICDs
    REF_FRAME_ROTATION = 26        # Reg 26 — Reference Frame Rotation (axis alignment)
    YPR_COMPENSATED_IMU = 27       # Reg 27 — source of $VNYMR (ADOR 14 -> YMR header; ICD §4.3.4)
    VPE_BASIC_CONTROL = 35         # Reg 35 — VPE Basic Control
    FILTER_STARTUP_GYRO_BIAS = 43  # Reg 43 — Filter Startup Gyro Bias; $VNSGB writes HERE (ICD §3.3.5)
    HSI_CONTROL = 44               # Reg 44 — Real-Time HSI Control (Mode/ApplyComp/ConvergeRate)
    HSI_STATUS = 46                # Reg 46 — FW v2.x ONLY. Absent from the FW v3.1.0.0 ICD -> capabilities
    HSI_CALCULATED = 47            # Reg 47 — Real-Time HSI Results (C 3x3 + B 3x1) — same in both ICDs
    BINARY_OUTPUT_1 = 75           # Reg 75 — Binary Output 1 (AsyncMode/RateDivisor/Group/Field)
    BINARY_OUTPUT_2 = 76           # Reg 76 — Binary Output 2
    BINARY_OUTPUT_3 = 77           # Reg 77 — Binary Output 3
    GYRO_COMPENSATION = 84         # Reg 84 — Gyro Calibration (C 3x3 + B 3x1) — UNRELATED to $VNSGB
    LEGACY_COMPAT = 206            # Reg 206 — Legacy Compatibility Settings (FW v3.x only)


# $VNSGB's target register depends on FW version (ICD §3.3.5 -> 43; UM001 FW2.1 §7.1.3 -> 74) -> capabilities.py.
GYRO_BIAS_REG_FW2 = 74             # older hardware: Filter Startup Gyro Bias
GYRO_BIAS_REG_FW3 = Reg.FILTER_STARTUP_GYRO_BIAS


# ── Async ASCII output type (ADOR / Reg 6) values (UM001 Table 28) ──
class AsyncType:
    OFF = 0        # async ASCII output disabled
    VNYMR = 14     # Yaw/Pitch/Roll + Mag + Accel + AngularRate (the message we use)


# ── Binary Output (Reg 75-77) constants (ICD FW3 §3.2.8 / docs/protocol.md §3.3, §4.2) ──
IMU_RATE_HZ = 800          # RateDivisor divides this: output Hz = 800 / RateDivisor
# NOTE: OutputGroup/OutputField (0x01 / 0x0128) live in `binary.py`
# (`_GROUPS`, `_FIELDS_G1`) instead — that module builds/parses the frame
# and should stay the single source of truth (duplicating risks drift).


class AsyncMode:
    """Reg 75-77 field 0 — AsyncMode (UM001 §4.2)."""
    OFF = 0        # binary output disabled
    PORT1 = 1      # fixed-rate to serial port 1
    PORT2 = 2      # fixed-rate to serial port 2
    BOTH = 3       # both ports


SENSOR_ASYNC_PORT = AsyncMode.PORT2   # sensor is wired to TTL Serial Port 2 (pins 8/9)


class HSIMode:
    """Reg 44 field 0 — Mode. Values are the SAME in both ICDs (FW3 §3.5.1 Table 3.56 / UM001 Table 2)."""
    OFF = 0     # real-time HSI disabled  <- FW v3.1.0.0 FACTORY DEFAULT
    RUN = 1     # running, uses the existing solution (toggle OFF<->RUN to start/stop)
    RESET = 2   # resets the real-time HSI solution (only RESET clears the solution)


class HSIOutput:
    """
    Reg 44 field 1 — should the onboard HSI solution be applied to the mag output?

    FW v3.1.0.0 ICD names it ApplyCompensation (Disable/Enable, §3.5.1 Table
    3.57); UM001 FW2.1 named it HSIOutput (NO_ONBOARD/USE_ONBOARD). Numeric
    values are identical in both (1/3) — wire-compatible, name-only change.
    Old names kept for backward compatibility.

    Reg 23 calibration is separate and always active: raw -> factory cal ->
    Reg 23 -> (if this field on) Reg 47 -> output (ICD §4.5.1).
    """
    DISABLE = 1       # ICD FW3 name — onboard HSI not applied  <- FW v3.1.0.0 FACTORY DEFAULT
    ENABLE = 3        # ICD FW3 name — onboard HSI (Reg 47 solution) applied
    NO_ONBOARD = 1    # UM001 FW2.1 name (alias)
    USE_ONBOARD = 3   # UM001 FW2.1 name (alias)


# Reg 44 factory default VARIES BY VERSION — don't assume a fixed value, ask capabilities.py.
#   FW v3.1.0.0 (ICD §3.5.1 DEFAULT column): 0,1,5 -> Off / Disable / 5  <- MEASURED on hardware
#   FW v2.1     (UM001 §8.3):                1,3,5 -> Run / Use / 5
HSI_CONTROL_DEFAULT_FW3 = (HSIMode.OFF, HSIOutput.DISABLE, 5)

# ADOF (Reg 7) — ICD §3.2.4 Table 3.9: a CLOSED enum, not a free range.
# Single source of truth: simulator rejects values outside this list (like
# the real sensor); dashboard's ASCII_HZ list is a subset. 0 = disabled.
# Binary path uses Reg 75 RateDivisor instead (output = 800/divisor) with
# no enum restriction — this gate applies only to the ASCII/Reg 7 branch.
ADOF_VALID = (0, 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200)
# ADOF factory default (ICD §3.2.4). Single source for both `link.py` and
# `vn100.py` — reading a different constant per path could desync ADOF values.
ADOF_DEFAULT_HZ = 40
HSI_CONTROL_DEFAULT_FW2 = (HSIMode.RUN, HSIOutput.ENABLE, 5)


class HeadingMode:
    """Reg 35 field 1 — HeadingMode."""
    ABSOLUTE = 0   # absolute north via magnetometer (magnetically clean environment)
    RELATIVE = 1   # doesn't trust mag; yaw starts at 0, relatively stable
    INDOOR = 2     # keeps an absolute reference in indoor/noisy environments


# ── Typed register-response decoders (UM001 §8.1–8.2) ────────

def decode_hsi_status(fields: list[str]) -> dict | None:
    """
    Decode Reg 46 (Magnetometer Calibration Status) fields — FW v2.x ONLY.

    Warning: Reg 46 is ABSENT from the FW v3.1.0.0 ICD (register index only
    lists Reg 44 + Reg 47 for HSI). Field hardware responds to `$VNRRG,46`
    with fourteen zeros instead of `$VNERR,08` — possibly an undocumented
    legacy stub, not part of the ICD contract, could disappear at any time.

    No decision should ever depend on this function: wizard progress comes
    from `tools/coverage.py` (PC-side orientation coverage), convergence from
    Reg 47 stability (`hsi_solution_converged`) — both ICD-backed. Kept only
    for legacy FW v2.x hardware and informational display.

    Field layout was never given an official table (even on FW v2.x): first
    6 fields are the header (LastBin, NumMeas, AvgResidual, LastMeas{X,Y,Z}),
    remaining fields are bins (tolerant of 7 or 8).
    """
    # 6 header fields + at least 3 bins -> plausibly a Reg 46 response (otherwise a different/corrupt register).
    if len(fields) < 9:
        return None
    try:
        bins = [int(x) for x in fields[6:]]        # all remaining fields are bins (7 or 8)
        return {
            "last_bin": int(fields[0]),
            "num_meas": int(fields[1]),
            "avg_residual": float(fields[2]),
            "last_meas": (float(fields[3]), float(fields[4]), float(fields[5])),
            "bins": bins,
        }
    except ValueError:
        return None


def decode_mag_cal(fields: list[str]):
    """
    Decode Reg 23/47 fields (C row-major 9 + B 3) -> (C[3][3], B[3]).

    Layout is identical in both ICDs: 48 bytes = Gain float[9] (row-major) +
    Bias float[3] (FW3 ICD §4.5.1 / §3.4.1; UM001 §8.2.1 / §6.2.1).
    """
    if len(fields) < 12:
        return None
    try:
        v = [float(x) for x in fields[:12]]
    except ValueError:
        return None
    C = [v[0:3], v[3:6], v[6:9]]
    B = v[9:12]
    return C, B


# ── Reg 47 stability — an ICD-backed convergence metric that REPLACES Reg 46 ──
#
# FW v3.1.0.0 has no HSI-progress register (46); the only ICD-based way to
# tell whether the onboard solution has converged is to poll Reg 47
# periodically and wait for it to SETTLE. Thresholds are empirical (tunable
# on hardware), not fixed constants.
HSI_STABLE_TOL = 0.002      # "settled" if the max |delta| between consecutive Reg 47 reads stays below this
HSI_STABLE_SAMPLES = 3      # how many consecutive reads must stay within that tolerance


def mag_cal_max_delta(a, b) -> float:
    """Largest absolute difference between two Reg 23/47 solutions (C,B). Returns inf on malformed input."""
    if a is None or b is None:
        return float("inf")
    (ca, ba), (cb, bb) = a, b
    flat_a = [x for row in ca for x in row] + list(ba)
    flat_b = [x for row in cb for x in row] + list(bb)
    if len(flat_a) != len(flat_b):
        return float("inf")
    return max((abs(x - y) for x, y in zip(flat_a, flat_b)), default=float("inf"))


def hsi_solution_converged(history, tol: float = HSI_STABLE_TOL,
                           min_samples: int = HSI_STABLE_SAMPLES) -> bool:
    """Is the Reg 47 solution history (oldest-to-newest (C,B) list) stable?

    Settled if consecutive deltas of the last `min_samples` reads all stay
    below `tol`. An identity (never-converged) solution also looks "stable" —
    caller must additionally check it moved away from identity
    (`mag_cal_max_delta(sol, IDENTITY) > tol`).
    """
    if len(history) < min_samples + 1:
        return False
    window = history[-(min_samples + 1):]
    return all(mag_cal_max_delta(window[i], window[i + 1]) <= tol
               for i in range(len(window) - 1))


IDENTITY_MAG_CAL = ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
