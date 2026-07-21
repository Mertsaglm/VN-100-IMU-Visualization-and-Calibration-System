"""
Bring-up self-check (pyvn100.selfcheck) tests -- no hardware required.

Covers the silent-failure signals the check looks for (float-printf, no
stream, wrong ICD profile) and the register-map differences between FW 2.1
and FW 3.1.0.0 (docs/protocol.md §5.3, pyvn100/capabilities.py). Reports are
keyed off the `capabilities` map, not the version string: v3 is ok
(baseline), v2 warns (older hardware), and an unrecognized version also
warns -- the assumption is stated explicitly, never silently treated as ok.
"""
import time

from pyvn100 import VN100, SimTransport, selfcheck
from pyvn100.types import Vn100Data


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


def _by_key(vn):
    return {r.key: r for r in selfcheck.run_checks(vn)}


def test_selfcheck_all_ok_on_sim():
    """Baseline hardware (FW 3.1.0.0) -> every item ok; missing Reg 46 is EXPECTED."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=False)
    vn = VN100(tp)
    for _ in range(20):
        clk.adv(0.02)
        vn.poll()
    assert vn.packet_count > 0
    assert selfcheck.request_reads(vn) is True    # identity + Reg 46 read commands sent
    vn.poll()                                      # cache the $VNRRG,1/2/4 replies
    r = _by_key(vn)
    assert r["stream"].status == "ok"
    assert r["float_printf"].status == "ok"       # |accel|~=9.81 even mid-tumble (norm preserved)
    assert r["identity"].status == "ok"           # Reg 1/2 READ (identity relies on these)
    assert "VN-100" in r["identity"].detail
    assert r["fw"].status == "ok"                 # sim default = field hardware (3.1.0.0)
    # Reg 46 absent on this ICD -> 'ok' (detail states it's expected, not a fault)
    assert r["reg46"].status == "ok" and "not defined" in r["reg46"].detail


def test_selfcheck_older_fw2_hardware_warns():
    """FW 2.1 (UM001) sensor -> warn: this project's baseline is v3, differences must be flagged."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False, fw_version="2.1.0.0")
    vn = VN100(tp)
    with vn._lock:
        vn._registers[4] = (["2.1.0.0"], time.time())
    r = _by_key(vn)
    assert r["fw"].status == "warn"
    assert "OLDER" in r["fw"].detail and "Reg 46 present" in r["fw"].detail


def test_selfcheck_fw2_reg46_still_reads():
    """On the older-hardware profile, Reg 46 still returns its real value (backward compat preserved)."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=False,
                      hsi_bins=7, fw_version="2.1.0.0")
    vn = VN100(tp)
    for _ in range(20):
        clk.adv(0.02)
        vn.poll()
    selfcheck.request_reads(vn)
    vn.poll()
    r = _by_key(vn)
    assert r["reg46"].status == "ok" and "7 bin" in r["reg46"].detail


def test_selfcheck_unknown_fw_states_assumption():
    """An unrecognized version is never silently 'ok': the assumption is stated explicitly (no guessing)."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    with vn._lock:
        vn._registers[4] = (["9.9.9.9"], time.time())
    r = _by_key(vn)
    assert r["fw"].status == "warn"
    assert "ASSUMED" in r["fw"].detail


def test_selfcheck_broken_float_printf_fails():
    # Firmware didn't link float-printf -> every %f comes back EMPTY/0 -> |accel|~=0 -> FAIL.
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    with vn._lock:
        vn.data = Vn100Data()          # all 0.0 (the hardware symptom of 'blank float')
        vn.packet_count = 5
        vn.last_update = time.time()
    r = _by_key(vn)
    assert r["stream"].status == "ok"              # stream is present
    assert r["float_printf"].status == "fail"      # but values are ~0 -> caught
    assert "float-printf" in r["float_printf"].detail


def test_selfcheck_no_stream_fails():
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)                                  # never polled
    r = _by_key(vn)
    assert r["stream"].status == "fail"
    # the report text must also be meaningful
    assert "FAILURES" in selfcheck.format_report(selfcheck.run_checks(vn))


def test_selfcheck_stale_cache_not_trusted():
    """The register cache lives for the process lifetime, so while the sensor is
    SILENT, a second check must not report a previous session's Reg 4 reply as
    '✓'. With `since` given, replies older than the request are STALE -> unknown."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    old = time.time() - 60.0                        # a 'previous session' reply from 1 min ago
    with vn._lock:
        vn._registers[4] = (["3.1.0.0"], old)
        vn._registers[1] = (["VN-100T-CR"], old)
    # without `since`, old behavior: the cache is trusted (backward compat)
    r = {x.key: x for x in selfcheck.run_checks(vn)}
    assert r["fw"].status == "ok"
    assert r["identity"].status == "ok"
    # with `since` given, the stale cache counts as 'not received'
    r = {x.key: x for x in selfcheck.run_checks(vn, since=time.time())}
    assert r["fw"].status == "unknown"
    assert r["identity"].status == "unknown"


def test_capabilities_helper_derives_from_reg4():
    """selfcheck.capabilities(vn) -- all capability queries from dialogs go through one door."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    assert selfcheck.capabilities(vn).known is False        # Reg 4 not read yet -> assumed
    with vn._lock:
        vn._registers[4] = (["3.1.0.0"], time.time())
    caps = selfcheck.capabilities(vn)
    assert caps.known and caps.profile == "fw3"
    assert caps.has_tare is False and caps.has_hsi_status_reg is False
    assert caps.gyro_bias_reg == 43 and caps.hsi_on_by_default is False
    # ICD diff #5 (hex error codes) was never asserted before -- could silently regress.
    assert caps.err_codes_hex is True, "FW3 profile must report $VNERR codes as HEX"


def test_fw2_profile_err_codes_hex_is_false():
    """Counter-check (ICD difference #5): UM001/FW2.1 error codes are DECIMAL."""
    from pyvn100.capabilities import capabilities_for
    caps2 = capabilities_for("2.1.0.0")
    assert caps2.profile == "fw2"
    assert caps2.err_codes_hex is False, "FW2 profile must report decimal codes"
    caps3 = capabilities_for("3.1.0.0")
    assert caps3.err_codes_hex is True
    assert caps2.err_codes_hex != caps3.err_codes_hex, "the two profiles must DIFFER on this field"


def test_measurement_beats_documentation_when_reg46_is_live():
    """Core rule: HARDWARE > DOCUMENTATION. The FW 3.1.0.0 ICD doesn't define
    Reg 46, so the profile says `has_hsi_status_reg=False` -- but field
    hardware replies on that register anyway (14 zeros, an undocumented
    legacy stub). If the probe observes this, the capability map must be
    CORRECTED rather than trusting the ICD over what was measured.
    (Display/diagnostics only -- no decision is gated on Reg 46.)"""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    now = time.time()
    with vn._lock:
        vn._registers[4] = (["3.1.0.0"], now)
    assert selfcheck.capabilities(vn).has_hsi_status_reg is False    # ICD: absent

    with vn._lock:   # hardware answered anyway (field observation: 6 headers + 8 bins)
        vn._registers[46] = (["0"] * 6 + ["5"] * 8, now)
    caps = selfcheck.capabilities(vn)
    assert caps.has_hsi_status_reg is True, "the measurement was ignored -- hardware must beat the doc"
    assert caps.profile == "fw3" and caps.gyro_bias_reg == 43   # other capabilities unaffected
