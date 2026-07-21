"""
pyvn100.capabilities — Firmware capability map (ICD generation differences, ALL IN ONE PLACE).

WHY THIS EXISTS: this project was written against the UM001 Rev 2.22 / **FW v2.1.0.0**
manual, but field hardware runs **FW v3.1.0.0** (confirmed: the ICD's sample Reg 4
response `$VNRRG,04,3.1.0.0*77` matches the hardware's response byte-for-byte). Five
real things changed between the two ICDs; they're all collected HERE instead of
scattered "if FW is version X do Y" checks — call `capabilities_for(fw)` instead.

DESIGN PRINCIPLE — the version string is a HINT, not proof:
We only have two ICDs (v2.1, v3.1.0.0); a `startswith`-based gate will misclassify
v4 if/when it ships. So:
  - Known versions map to documented fact (table below, taken verbatim from the ICD).
  - An UNKNOWN version falls back to the v3 profile + `known=False` (the verified
    baseline); selfcheck surfaces that as a "please confirm" NOTE, not an error.
  - Capabilities can be OVERRIDDEN by live probing (`Capabilities.probed(...)`):
    hardware beats documentation — this project's core rule ("sim passing != hardware working").

ICD DIFFERENCES (same table as docs/protocol.md §5.3):
  1. Reg 46 (HSI Status/bins)   : present in v2.x  · **absent in v3** (not in the ICD register index)
  2. Reg 44 factory default     : v2.x `1,3,5` (Run/Enable)  · **v3 `0,1,5`** (Off/Disable)
  3. `$VNTAR` (Tare command)    : present in v2.x  · **absent in v3** (not in the §1.3 command list)
  4. `$VNSGB` target            : v2.x Reg 74  · **v3 Reg 43** (Filter Startup Gyro Bias)
  5. $VNERR code display        : v2.x decimal (10/11/12)  · **v3 hex (0A/0B/0C)**; 01-08 unchanged
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

from . import registers
from .registers import ICD_FW_BASELINE


@dataclass(frozen=True)
class Capabilities:
    """WHAT a given firmware version can do. Carries behavior, not just a version string."""

    fw: Optional[str]                 # observed Reg 4 string (None = not read yet)
    profile: str                      # "fw3" | "fw2" — which ICD behavior is being followed
    known: bool                       # was the version matched to a known ICD (False = assumed)
    has_tare: bool                    # is the $VNTAR command available (ICD §1.3)
    has_hsi_status_reg: bool          # is Reg 46 defined in the ICD
    gyro_bias_reg: int                # register that $VNSGB writes to
    hsi_control_default: tuple        # Reg 44 factory default (Mode, ApplyComp, ConvergeRate)
    hsi_on_by_default: bool           # does onboard HSI run out of the box (affects calibration strategy!)
    err_codes_hex: bool               # are $VNERR codes shown in hex

    def note(self) -> str:
        """One-line FW summary for the user/console."""
        if not self.known:
            return (f"FW '{self.fw}' not recognized — assuming the v{ICD_FW_BASELINE} "
                    "profile this project was verified against; confirm on hardware")
        return f"FW {self.fw} — {self.profile.upper()} profile (matched ICD)"

    def probed(self, **overrides) -> "Capabilities":
        """Override a capability with a live hardware probe result (hardware beats documentation).

        E.g. if probing Reg 46 shows it's actually live:
            caps = caps.probed(has_hsi_status_reg=True)
        """
        return replace(self, **overrides)


# ── Profiles taken verbatim from the ICD ────────────────────────────

_FW3 = dict(
    profile="fw3",
    has_tare=False,                       # ICD §1.3: RRG/WRG/WNV/RFS/RST/FWU/KMD/KAD/ASY/SGB/BOM — no TAR
    has_hsi_status_reg=False,             # 46 not in the ICD register index; HSI = 44 + 47
    gyro_bias_reg=registers.GYRO_BIAS_REG_FW3,      # 43 (ICD §3.3.5)
    hsi_control_default=registers.HSI_CONTROL_DEFAULT_FW3,   # (0, 1, 5) — ICD §3.5.1 DEFAULT column
    hsi_on_by_default=False,              # Mode=Off -> offline fit starts CLEAN
    err_codes_hex=True,                   # ICD §1.5 Table 1.6
)

_FW2 = dict(
    profile="fw2",
    has_tare=True,                        # UM001 §5.1.x
    has_hsi_status_reg=True,              # UM001 Reg 46 (field layout not given in an official table)
    gyro_bias_reg=registers.GYRO_BIAS_REG_FW2,      # 74 (UM001 §7.1.3)
    hsi_control_default=registers.HSI_CONTROL_DEFAULT_FW2,   # (1, 3, 5) — UM001 §8.3
    hsi_on_by_default=True,               # Mode=Run -> sensor is a "moving target" during the fit
    err_codes_hex=False,                  # UM001 §3.7 decimal
)


def capabilities_for(fw: Optional[str]) -> Capabilities:
    """Build a capability map from the Reg 4 string. Unknown/unread version -> v3 profile + known=False.

    Falling back to v3 is deliberate: field hardware runs v3, the verified baseline.
    The `known=False` flag isn't lost — selfcheck prints a "please confirm" note
    instead of assuming silently.
    """
    v = (fw or "").strip()
    if v.startswith("3."):
        return Capabilities(fw=v or None, known=True, **_FW3)
    if v.startswith("2.") or v.startswith("1."):
        # v1.x maps to the v2 profile too: no v1.1 ICD exists, but the v1/v2
        # differences (Reg 75/46 presence) align with v2 for the paths this project uses.
        return Capabilities(fw=v, known=True, **_FW2)
    return Capabilities(fw=v or None, known=False, **_FW3)
