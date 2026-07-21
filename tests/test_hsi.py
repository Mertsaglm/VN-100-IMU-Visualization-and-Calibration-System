"""
Onboard HSI (hard/soft-iron) calibration workflow tests.

Verifies the simulator's HSI emulation (Reg 44/47; also Reg 46 on v2.x) and $VNRRG
parsing without hardware, exercising RESET -> rotate -> converge -> OFF end to end.

Covers register-map differences between two firmware/ICD generations (FW 2.1 and
FW 3.1.0.0; docs/protocol.md Sec.5.3, pyvn100/capabilities.py):
  * v3 factory default is (Off, Disable, 5) - HSI ships OFF; v2.1 ships (Run, Enable, 5),
    so onboard HSI must be EXPLICITLY started on v3.
  * Reg 46 does NOT exist in the v3 ICD -> only exercised under fw_version="2.1.0.0".
"""
import numpy as np

from pyvn100 import VN100, SimTransport, hostlink, protocol
from pyvn100.registers import (HSI_CONTROL_DEFAULT_FW2, HSI_CONTROL_DEFAULT_FW3, HSIMode,
                              HSIOutput, Reg, decode_hsi_status, hsi_solution_converged,
                              mag_cal_max_delta, IDENTITY_MAG_CAL)
from pyvn100.simulator import HSIEmulator, _HARD_IRON, _SOFT_IRON

FW2 = "2.1.0.0"      # legacy hardware profile (UM001 Rev 2.22) - Reg 46 + HSI on by factory default


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


def _run(tp, vn, clk, seconds, rate=50.0):
    for _ in range(int(seconds * rate)):
        clk.adv(1.0 / rate)
        vn.poll()


def _spin(hsi, rounds=30):
    """Feed all octants -> let the onboard solution converge."""
    for _ in range(rounds):
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    hsi.observe((0.3 * sx, 0.3 * sy, 0.3 * sz))


def _running(n_bins=8):
    """An emulator with HSI ON - must be started explicitly since v3 ships off by default."""
    hsi = HSIEmulator(_SOFT_IRON, _HARD_IRON, n_bins=n_bins)
    hsi.write_control(HSIMode.RUN, HSIOutput.ENABLE, 5)
    return hsi


# -- parse/decode ---------------------------------------------------

def test_parse_vnrrg_and_decode():
    line = protocol.build_command("VNRRG,46,3,120,0.0123,0.1,0.2,0.3,10,12,9,11,8,10,13,7")
    reg, fields = protocol.parse_vnrrg(line)
    assert reg == 46
    st = decode_hsi_status(fields)
    assert st["num_meas"] == 120
    assert abs(st["avg_residual"] - 0.0123) < 1e-6
    assert st["bins"] == [10, 12, 9, 11, 8, 10, 13, 7]


def test_parse_vnrrg_bad_checksum():
    assert protocol.parse_vnrrg("$VNRRG,46,1,2,3*00") is None
    assert protocol.parse_vnrrg("$VNYMR,1,2,3*00") is None   # different message type


# -- Factory default: VERSION-DEPENDENT (a real field difference) --

def test_factory_default_fw3_hsi_off():
    """FW 3.1.0.0 ICD Sec.3.5.1 default column: Reg 44 = 0,1,5 (Off/Disable/5) - two separate
    $VNRFS calls both leave $VNRRG,44 unchanged. Differs from UM001's '1,3,5' assumption;
    calibration.md's 'on by default, moving target' premise doesn't hold for this ICD."""
    hsi = HSIEmulator(_SOFT_IRON, _HARD_IRON)
    assert (hsi.mode, hsi.output, hsi.rate) == HSI_CONTROL_DEFAULT_FW3 == (0, 1, 5)
    assert hsi.mode == HSIMode.OFF and hsi.output == HSIOutput.DISABLE


def test_factory_default_fw2_hsi_on():
    """Legacy hardware (UM001 Sec.8.3): Reg 44 = 1,3,5 -> ON by factory default."""
    hsi = HSIEmulator(_SOFT_IRON, _HARD_IRON, hsi_default=HSI_CONTROL_DEFAULT_FW2)
    assert (hsi.mode, hsi.output, hsi.rate) == (1, 3, 5)


def test_factory_reset_restores_reg44():
    """$VNRFS restores Reg 44 to its factory default - verifies the sim actually applies it."""
    hsi = _running()
    _spin(hsi, rounds=5)
    hsi.write_user_cal([1.1, 0, 0, 0, 0.9, 0, 0, 0, 1.05, 0.03, -0.02, 0.01])
    hsi.factory_reset()
    assert (hsi.mode, hsi.output, hsi.rate) == HSI_CONTROL_DEFAULT_FW3
    assert np.allclose(hsi.user_C, np.eye(3)) and np.allclose(hsi.user_B, 0)   # Reg 23 -> identity


def test_sim_vnrfs_resets_reg44_end_to_end():
    """'VN FACTORY' -> $VNRFS -> reading Reg 44 should return 0,1,5 (FW 3.1.0.0 factory default)."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=False)
    vn = VN100(tp)
    tp.write(hostlink.hsi_reset(rate=3))                 # scramble HSI state
    tp.write(hostlink.factory())                         # restore factory settings
    tp.write(hostlink.read_reg(Reg.HSI_CONTROL))
    vn.poll()
    fields = vn.get_register(Reg.HSI_CONTROL)[0]
    assert [int(x) for x in fields[:3]] == list(HSI_CONTROL_DEFAULT_FW3)   # 0,1,5


# -- HSIEmulator unit tests ------------------------------------------

def test_hsi_emulator_converges():
    hsi = _running()
    assert hsi.mode == HSIMode.RUN
    r0 = hsi.avg_residual
    _spin(hsi)
    assert hsi.avg_residual < r0                     # residual dropped
    assert all(b > 0 for b in hsi.bins)              # all 8 bins filled
    assert np.allclose(hsi.est_B, _HARD_IRON, atol=0.02)   # solution converged to the true value


def test_hsi_off_does_not_converge():
    """With v3's factory state (Off), the sensor never converges on its own, so the offline
    fit starts from a clean slate - confirms the v2.1 'moving target' problem doesn't apply here."""
    hsi = HSIEmulator(_SOFT_IRON, _HARD_IRON)        # factory default: Off
    _spin(hsi)
    assert hsi.num_meas == 0 and all(b == 0 for b in hsi.bins)
    assert np.allclose(hsi.est_C, np.eye(3)) and np.allclose(hsi.est_B, 0)


def test_hsi_reset_and_off():
    hsi = _running()
    for _ in range(50):
        hsi.observe((0.2, -0.1, 0.3))
    hsi.write_control(HSIMode.RESET, HSIOutput.ENABLE, 5)
    assert hsi.num_meas == 0 and hsi.bins == [0] * 8   # RESET cleared state
    hsi.write_control(HSIMode.OFF, HSIOutput.ENABLE, 5)
    n = hsi.num_meas
    hsi.observe((0.2, 0.1, 0.1))
    assert hsi.num_meas == n                            # frozen while OFF


def test_hsi_output_mode_changes_mag():
    hsi = _running()
    _spin(hsi, rounds=200)
    from pyvn100.types import Vn100Data
    hsi.output = HSIOutput.DISABLE
    d_raw = hsi.apply(Vn100Data(mag_x=0.5, mag_y=-0.2, mag_z=0.3))
    hsi.output = HSIOutput.ENABLE
    d_cal = hsi.apply(Vn100Data(mag_x=0.5, mag_y=-0.2, mag_z=0.3))
    assert (d_raw.mag_x, d_raw.mag_y, d_raw.mag_z) != (d_cal.mag_x, d_cal.mag_y, d_cal.mag_z)


def test_hsi_user_and_onboard_chain():
    """F11: under USE_ONBOARD, the Reg 23 (user) stage is also applied - chained ON TOP
    of the onboard solution, so a non-identity user_C output must DIFFER from onboard-only.
    FW3 ICD Sec.4.5.1 states this explicitly: Reg 47's input is mag data 'after Reg 23
    has been applied'."""
    from pyvn100.types import Vn100Data
    raw = dict(mag_x=0.5, mag_y=-0.2, mag_z=0.3)

    hsi_chain = _running()
    _spin(hsi_chain, rounds=200)
    hsi_chain.write_user_cal([1.1, 0.05, 0.0, 0.0, 0.9, 0.02, 0.0, 0.0, 1.05, 0.03, -0.02, 0.01])
    hsi_chain.output = HSIOutput.ENABLE
    d_chain = hsi_chain.apply(Vn100Data(**raw))

    hsi_only = _running()
    _spin(hsi_only, rounds=200)
    hsi_only.output = HSIOutput.ENABLE
    d_only = hsi_only.apply(Vn100Data(**raw))

    assert (d_chain.mag_x, d_chain.mag_y, d_chain.mag_z) != (d_only.mag_x, d_only.mag_y, d_only.mag_z)


# -- Reg 47 stability - a convergence metric that REPLACES Reg 46 --

def test_reg47_stability_detects_convergence():
    """Convergence without Reg 46: a solution has converged once it settles across
    consecutive readings."""
    steady = ([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.1, 0.2, 0.3])
    assert hsi_solution_converged([steady] * 4) is True
    # Still moving -> not converged
    shifting = [([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.1 + 0.05 * i, 0.2, 0.3]) for i in range(4)]
    assert hsi_solution_converged(shifting) is False
    # Too few samples -> "unknown" (False; never fake a 'converged' result)
    assert hsi_solution_converged([steady]) is False


def test_identity_solution_not_counted_as_converged():
    """Identity looks 'stable' but there IS NO solution -> callers must also check for
    identity to rule this out."""
    assert hsi_solution_converged([IDENTITY_MAG_CAL] * 4) is True          # stable, yes
    assert mag_cal_max_delta(IDENTITY_MAG_CAL, IDENTITY_MAG_CAL) == 0.0    # but identity -> no real solution


def test_sim_reg47_converges_while_tumbling():
    """End to end: turn HSI on -> rotate -> Reg 47 moves off identity and SETTLES
    (no need for Reg 46)."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=True)
    vn = VN100(tp)
    tp.write(hostlink.hsi_reset(rate=5))          # RESET -> RUN (ICD: RESET clears the solution and starts running)
    _run(tp, vn, clk, seconds=20.0)

    from pyvn100.registers import decode_mag_cal
    history = []
    for _ in range(4):
        _run(tp, vn, clk, seconds=1.0)
        tp.write(hostlink.read_reg(Reg.HSI_CALCULATED)); vn.poll()
        history.append(decode_mag_cal(vn.get_register(Reg.HSI_CALCULATED)[0]))

    assert hsi_solution_converged(history)                                   # settled
    assert mag_cal_max_delta(history[-1], IDENTITY_MAG_CAL) > 0.01           # moved off identity


# -- What's NOT present on FW 3.1.0.0 (confirmed against the ICD) --

def test_fw3_reg46_invalid_register():
    """Reg 46 does not exist in the FW 3.1.0.0 ICD -> $VNERR,08 (Invalid Register, ICD Sec.1.5)."""
    tp = SimTransport(rate_hz=50.0, motion="still", noise=False)
    vn = VN100(tp)
    tp.write(hostlink.hsi_status())
    vn.poll()
    assert vn.get_register(Reg.HSI_STATUS) is None                    # no data
    assert any("$VNERR,08" in e for e in vn.errors_since(0))          # and the reason is visible


def test_fw2_reg46_still_present():
    """Legacy hardware profile is preserved - no backward-compat regression."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=False, fw_version=FW2)
    vn = VN100(tp)
    tp.write(hostlink.hsi_status())
    vn.poll()
    st = decode_hsi_status(vn.get_register(Reg.HSI_STATUS)[0])
    assert st is not None and "bins" in st


def test_fw2_7bin_reg46_end_to_end():
    """7-bin hardware emulation (per the UM001 hsi info dump): 13 fields -> len(bins)=7,
    no fixed index assumed."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=True,
                      hsi_bins=7, fw_version=FW2)
    vn = VN100(tp)
    tp.write(hostlink.hsi_reset(rate=5))
    _run(tp, vn, clk, seconds=15.0)
    tp.write(hostlink.hsi_status()); vn.poll()
    reg, fields = protocol.parse_vnrrg(tp._hsi.resp_status())
    assert reg == 46 and len(fields) == 13            # 6 header fields + 7 bins
    st = decode_hsi_status(vn.get_register(Reg.HSI_STATUS)[0])
    assert st is not None and len(st["bins"]) == 7
    assert sum(1 for b in st["bins"] if b >= 3) >= 6


def test_hsi_emulator_7bin_hardware_path():
    hsi = _running(n_bins=7)
    assert len(hsi.bins) == 7
    _spin(hsi, rounds=40)                              # octant 7 folds into bin 6
    assert len(hsi.bins) == 7 and all(b > 0 for b in hsi.bins)
    assert hsi.avg_residual < 0.12


def test_fw2_onboard_convergence_and_residual_drop():
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=True, fw_version=FW2)
    vn = VN100(tp)
    tp.write(hostlink.hsi_reset(rate=5))

    tp.write(hostlink.hsi_status()); vn.poll()
    early = decode_hsi_status(vn.get_register(Reg.HSI_STATUS)[0])["avg_residual"]

    _run(tp, vn, clk, seconds=15.0)     # rotate (simulated tumble)

    tp.write(hostlink.hsi_status()); vn.poll()
    late_st = decode_hsi_status(vn.get_register(Reg.HSI_STATUS)[0])
    assert late_st["avg_residual"] < early          # residual dropped
    assert sum(1 for b in late_st["bins"] if b > 0) >= 6   # bins filled


def test_sim_hsi_off_write_reg44():
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, motion="calibration", clock=clk, noise=False)
    vn = VN100(tp)
    tp.write(hostlink.hsi_off())
    assert tp._hsi.mode == HSIMode.OFF
    tp.write(hostlink.read_reg(Reg.HSI_CONTROL)); vn.poll()
    fields = vn.get_register(Reg.HSI_CONTROL)[0]
    assert int(fields[0]) == HSIMode.OFF


# -- Discriminating power of the convergence detector -----------------
def test_hsi_emulator_can_produce_non_convergence():
    """Monotonic interpolation toward the ideal solution would make non-convergence
    mathematically impossible, so any 'is Reg 47 stable?' test would have no
    discriminating power (the mock always reports 'stable'). `wander` must be able to
    produce a realistic unsettled state."""
    import numpy as np
    from pyvn100.simulator import HSIEmulator
    from pyvn100.registers import HSIMode

    soft = np.array([[1.2, 0.1, 0.0], [0.1, 0.9, 0.05], [0.0, 0.05, 1.1]])
    hard = np.array([0.3, -0.2, 0.1])

    def solve_history(wander):
        em = HSIEmulator(soft, hard, wander=wander)
        em.mode = HSIMode.RUN
        history = []
        for i in range(400):
            a = i * 0.37
            em.observe((np.cos(a), np.sin(a), np.cos(a * 0.5)))
            if i % 40 == 0:
                history.append(em.est_C.copy())
        return history

    # wander=0 -> current behavior: the solution SETTLES (last readings close together)
    steady_hist = solve_history(0.0)
    last_delta = float(np.max(np.abs(steady_hist[-1] - steady_hist[-2])))
    assert last_delta < 0.01, f"default behavior should not have changed (delta {last_delta})"

    # wander>0 -> the solution OSCILLATES: consecutive readings stay noticeably apart
    wobbly_hist = solve_history(0.05)
    wobble_delta = float(np.max(np.abs(wobbly_hist[-1] - wobbly_hist[-2])))
    assert wobble_delta > 0.01, \
        f"solution settled even with wander ({wobble_delta}) - detector still untestable"


def test_convergence_detector_rejects_unsettled_solution():
    """Discriminating power: the Reg 47 stability metric must NOT count an oscillating
    solution as 'converged'."""
    from pyvn100.registers import HSI_STABLE_TOL, hsi_solution_converged

    # Settled history -> converged
    C = [[1.05, 0.01, 0.0], [0.01, 0.97, 0.02], [0.0, 0.02, 1.01]]
    B = [0.12, -0.08, 0.03]
    settled = [(C, B)] * 5
    assert hsi_solution_converged(settled, HSI_STABLE_TOL) is True

    # Oscillating history -> not converged
    oscillating = []
    for i in range(5):
        d = 0.05 * ((-1) ** i)
        oscillating.append(([[1.05 + d, 0.01, 0.0], [0.01, 0.97, 0.02], [0.0, 0.02, 1.01]], B))
    assert hsi_solution_converged(oscillating, HSI_STABLE_TOL) is False, \
        "an oscillating solution was counted as 'converged' - detector isn't discriminating"
