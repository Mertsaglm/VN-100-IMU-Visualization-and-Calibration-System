"""
pyvn100.selfcheck — Hardware/no-hardware BRING-UP self-check.

Purpose: during a live office session, nobody should have to "reason it
out". Silent-failure signals found during bring-up are baked in here; this
tool probes the data path automatically and reports each item as
ok/warn/fail. Qt-free and pure, so it's testable with the simulator and
callable from the dashboard with one click.

Bring-up risks covered (see docs/bringup_checklist.md):
  - silent float-printf breakage -> every %f prints EMPTY/0 (symptom: |accel| approx 0),
  - sensor IDENTITY: Reg 1 (model) + Reg 2 (hardware) + Reg 4 (firmware) — which ICD applies?
  - firmware capabilities (does Reg 46 exist, does $VNTAR exist, which register does $VNSGB write to),
  - is the stream live, and in what format (ASCII/binary).

Usage:
  request_reads(vn)              # TRIGGER a read of Reg 1/2/4 (+ Reg 46 if v2.x)
  ... wait ~1-1.5 s (let the reader thread cache the responses) ...
  report = run_checks(vn)        # [CheckResult] — each one ok/warn/fail/unknown
  ok = all(r.status != "fail" for r in report)
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from .capabilities import capabilities_for
from .registers import ICD_FW_BASELINE, Reg, decode_hsi_status

# Thresholds (m/s^2): if |accel| is below this, float-printf is likely broken (values ~0).
ACCEL_MIN_ALIVE = 2.0      # below this -> suspect float-printf/data (FAIL)
ACCEL_G_LOW, ACCEL_G_HIGH = 7.0, 12.0   # ~1g band (expected for a stationary sensor)
FRESH_MAX_AGE_S = 3.0      # stream is considered "stale" if the last packet is older than this


@dataclass
class CheckResult:
    key: str
    label: str
    status: str        # "ok" | "warn" | "fail" | "unknown"
    detail: str

    @property
    def mark(self) -> str:
        return {"ok": "✓", "warn": "⚠", "fail": "✗", "unknown": "?"}.get(self.status, "?")

    def __str__(self) -> str:
        return f"{self.mark} {self.label}: {self.detail}"


def request_reads(vn) -> bool:
    """Trigger a read of the sensor's IDENTITY (Reg 1 model, Reg 2 hardware, Reg 4 firmware) + Reg 46.

    The identity triple is critical: this project was written against the
    UM001 Rev 2.22 (FW v2.1) manual, but field hardware runs FW v3.1.0.0, and
    five real things changed between the two ICDs. No register claim can be
    trusted without knowing which ICD applies — so identity is the FIRST
    bring-up step, not just Reg 4 (firmware) but Reg 1/2 (model/hardware) too.

    Reg 46 is requested on every version: v2.x returns real status, v3 is
    expected to return $VNERR,08 since it's not in the ICD — either way
    it's informational (the request is harmless, read-only).

    Command framing comes from `vn.link` based on the active connection mode:
    BRIDGE uses 'VN RAW $VNRRG,..' (STM bridge), DIRECT uses raw '$VNRRG,..'
    (direct USB-TTL). Returns True if all reads made it out of the PC (write
    failures aren't swallowed silently)."""
    ok = True
    cmds = [
        *vn.link.read_register(Reg.MODEL_NUMBER),
        *vn.link.read_register(Reg.HARDWARE_REVISION),
        *vn.link.read_register(Reg.FIRMWARE_VERSION),
        *vn.link.hsi_status(),
    ]
    for cmd in cmds:
        try:
            vn.send(cmd)          # on_tx -> bring-up read commands are also visible on the console
        except Exception:
            ok = False
    return ok


def _cached_reg(vn, reg: int, since: Optional[float]):
    """Return the cached register response, filtered by freshness.

    VN100._registers lives for the process lifetime: a second check while the
    sensor is silent could mistake a stale response from a previous session
    for a fresh one. If `since` is given, only a response that arrived after
    that time is accepted; an older one counts as 'not received' (unknown)."""
    r = vn.get_register(reg)
    if r is None:
        return None
    if since is not None and len(r) > 1 and r[1] is not None and r[1] < since:
        return None                      # arrived BEFORE the request -> stale, don't trust it
    return r


def _reg_first_field(vn, reg: int, since: Optional[float] = None) -> Optional[str]:
    """First field of a register response (for string registers: model/hardware/firmware)."""
    r = _cached_reg(vn, reg, since)
    if r is None or not r[0]:
        return None
    return str(r[0][0]).strip()


def _fw_version_str(vn, since: Optional[float] = None) -> Optional[str]:
    return _reg_first_field(vn, Reg.FIRMWARE_VERSION, since)


def capabilities(vn, since: Optional[float] = None):
    """Build a capability map from the observed Reg 4, corrected by a live hardware probe.

    Two layers:
      1. Reg 4 string -> ICD profile (what the documentation says). If unread,
         `known=False` + the baseline (v3) profile — the interface never
         silently turns "I don't know" into "I know".
      2. Measurement beats documentation (this project's core rule): if a
         real response came back from Reg 46 — even if the ICD doesn't define
         it — the register is live. Field hardware does exactly this:
         `$VNRRG,46` gets 14 zeros back instead of `$VNERR,08` (an
         undocumented legacy stub). Recording this lets the info panel show
         reality. No decision still depends on Reg 46 — convergence is
         measured from Reg 47 stability; this is purely for display/diagnostics.
    """
    caps = capabilities_for(_fw_version_str(vn, since))
    r46 = _cached_reg(vn, Reg.HSI_STATUS, since)
    if r46 is not None and decode_hsi_status(r46[0]) is not None and not caps.has_hsi_status_reg:
        caps = caps.probed(has_hsi_status_reg=True)   # hardware beats documentation
    return caps


def run_checks(vn, *, now: Optional[float] = None,
               since: Optional[float] = None) -> list[CheckResult]:
    """Probe the data path and return the bring-up check list. Side-effect free (read only).

    Must be called after request_reads(vn) and after the Reg 4/46 responses
    land in the cache; if a response hasn't arrived yet, that item returns
    'unknown' (it knows what it doesn't know — it doesn't guess).
    since: the time request_reads was called (time.time()). If given, any
    Reg 4/46 response cached before that time counts as stale — a check is
    never faked '✓' from a previous session's response."""
    t = time.time() if now is None else now
    out: list[CheckResult] = []

    # 1) Is the stream alive? (packets arriving + fresh)
    pkt = int(getattr(vn, "packet_count", 0) or 0)
    last = getattr(vn, "last_update", None)
    age = (t - last) if last else None
    if pkt <= 0 or last is None:
        out.append(CheckResult("stream", "Telemetry stream", "fail",
                               "no packets at all — cable/port/baud, or the sensor is silent"))
    elif age is not None and age > FRESH_MAX_AGE_S:
        out.append(CheckResult("stream", "Telemetry stream", "fail",
                               f"stream stalled (no new packet for {age:.1f} s)"))
    else:
        fmt = getattr(vn, "last_fmt", None) or "?"
        out.append(CheckResult("stream", "Telemetry stream", "ok",
                               f"{pkt} packets, live, format={fmt}"))

    d = vn.get_data()

    # 2) float-printf check — is |accel| a real number? (if broken, all %f come back ~0)
    if d is None:
        # If newlib-nano float-printf isn't linked, `%f` prints EMPTY, producing
        # a line with empty fields like "$VNYMR,,,,,,,,,,,,*XX". Such a line
        # can't be parsed, so get_data() stays None — to distinguish this from
        # "no data yet", check for frames arriving but failing to decode
        # (error_count increasing).
        errors = int(getattr(vn, "error_count", 0))
        if errors > 0:
            out.append(CheckResult(
                "float_printf", "float-printf / data sanity", "fail",
                f"NO data but {errors} undecodable frame(s) arrived — lines may have EMPTY "
                "FIELDS ($VNYMR,,,,,...), the TYPICAL symptom of newlib-nano float-printf not "
                "being linked; check the _printf_float asm directive in main.c and the "
                "'-u _printf_float' flag in .cproject (both are required)"))
        else:
            out.append(CheckResult("float_printf", "float-printf / data sanity", "unknown",
                                   "no data yet (and no undecodable frames either — the "
                                   "stream may not have started at all)"))
    else:
        amag = math.sqrt(d.accel_x**2 + d.accel_y**2 + d.accel_z**2)
        if amag < ACCEL_MIN_ALIVE:
            out.append(CheckResult("float_printf", "float-printf / data sanity", "fail",
                                   f"|accel| approx {amag:.2f} m/s^2 (~0!) — float-printf may not "
                                   "be linked; check the _printf_float asm directive in main.c "
                                   "and the '-u _printf_float' flag in .cproject"))
        elif ACCEL_G_LOW <= amag <= ACCEL_G_HIGH:
            out.append(CheckResult("float_printf", "float-printf / data sanity", "ok",
                                   f"|accel| approx {amag:.2f} m/s^2 (~1g, real float values) "
                                   "— expect approx 9.81 when stationary"))
        else:
            out.append(CheckResult("float_printf", "float-printf / data sanity", "warn",
                                   f"|accel| approx {amag:.2f} m/s^2 — numbers are coming through "
                                   "but outside 1g (sensor may be moving; hold still and recheck)"))

    # 3) IDENTITY — model + hardware revision (which device are we talking to?)
    model = _reg_first_field(vn, Reg.MODEL_NUMBER, since)
    hw = _reg_first_field(vn, Reg.HARDWARE_REVISION, since)
    if model is None and hw is None:
        out.append(CheckResult("identity", "Sensor identity (Reg 1/2)", "unknown",
                               "no response yet — wait a bit after request_reads and retry"))
    else:
        out.append(CheckResult("identity", "Sensor identity (Reg 1/2)", "ok",
                               f"model={model or '?'}, hardware revision={hw or '?'}"))

    # 4) Firmware -> capability map. The version string is a hint; what matters is which
    #    ICD applies. An unknown version is not an error — it's a "please confirm" note (no guessing).
    fw = _fw_version_str(vn, since)
    caps = capabilities_for(fw)
    if fw is None:
        out.append(CheckResult("fw", "Firmware / ICD profile (Reg 4)", "unknown",
                               "no response yet — wait a bit after request_reads and retry"))
    elif not caps.known:
        out.append(CheckResult("fw", "Firmware / ICD profile (Reg 4)", "warn",
                               f"v{fw} — not one of the recognized ICDs (v3.1.0.0 / v2.1); "
                               f"ASSUMED the v{ICD_FW_BASELINE} profile. Confirm register "
                               "behavior on hardware."))
    elif caps.profile == "fw3":
        out.append(CheckResult("fw", "Firmware / ICD profile (Reg 4)", "ok",
                               f"v{fw} — the baseline ICD this project was verified against. "
                               "Onboard HSI is OFF by factory default (Reg 44=0,1,5), no Reg 46, "
                               "no $VNTAR, $VNSGB->Reg 43."))
    else:
        out.append(CheckResult("fw", "Firmware / ICD profile (Reg 4)", "warn",
                               f"v{fw} — OLDER hardware (UM001 Rev 2.22). This project's baseline "
                               f"is v{ICD_FW_BASELINE}: there, Reg 46 present, HSI is ON by factory "
                               "default (Reg 44=1,3,5), $VNSGB->Reg 74. Verify the differences."))

    # 5) Reg 46 — interpret based on version. Absent from the ICD on v3; $VNERR,08 or
    #    silence is the expected outcome, not a fault. No decision depends on this (the
    #    wizard uses Reg 47 stability + PC-side coverage) -> this item is informational only.
    r46 = _cached_reg(vn, Reg.HSI_STATUS, since)
    st = decode_hsi_status(r46[0]) if r46 is not None else None
    if not caps.has_hsi_status_reg:
        detail = ("not defined in this firmware's ICD — expected; the wizard already uses "
                  "Reg 47 stability + PC-side coverage")
        if st is not None:
            detail += f" (a {len(st['bins'])}-field response still came back — an undocumented stub)"
        out.append(CheckResult("reg46", "Reg 46 (HSI status)", "ok", detail))
    elif st is None:
        out.append(CheckResult("reg46", "Reg 46 (HSI status)", "warn",
                               "Reg 46 was expected on a v2.x sensor but couldn't be read"))
    else:
        out.append(CheckResult("reg46", "Reg 46 (HSI status)", "ok",
                               f"{len(st['bins'])} bins (v2.x path; code tolerates via len(bins))"))

    return out


def format_report(results: list[CheckResult]) -> str:
    """Render the report list as human-readable multi-line text (dashboard/console)."""
    lines = [str(r) for r in results]
    worst = "ok"
    for r in results:
        if r.status == "fail":
            worst = "fail"; break
        if r.status in ("warn", "unknown") and worst == "ok":
            worst = r.status
    verdict = {"ok": "ALL PASSED ✓", "warn": "WARNINGS ⚠",
               "unknown": "MISSING DATA ? (wait a bit and retry)", "fail": "FAILURES ✗"}[worst]
    return f"BRING-UP CHECK — {verdict}\n" + "\n".join("  " + ln for ln in lines)
