"""
Dual-mode (ASCII + binary) auto-detecting parser, runtime output-mode
switching, and SetGyroBias tests — verifying docs/protocol.md §4, entirely
without hardware.

Covered behaviors:
  • The parser extracts both $VNYMR/$VNRRG (ASCII) and 0xFA (binary) frames
    from the same stream.
  • When binary telemetry and an ASCII command reply arrive INTERLEAVED, both
    get decoded (on a real sensor, $VNRRG replies stay ASCII even in binary
    mode).
  • set_output_mode() writes the correct registers (reg 6 / reg 75).
  • The simulator actually changes its output mode when the register is written.
  • SetGyroBias ($VNSGB) compensates the gyro bias while stationary.
"""
from pyvn100 import VN100, LoopbackTransport, SimTransport, binary, hostlink, protocol
from pyvn100.simulator import Vn100Simulator
from pyvn100.types import Vn100Data


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


def _sample() -> Vn100Data:
    return Vn100Data(
        yaw=10.0, pitch=-5.0, roll=3.0,
        mag_x=0.2, mag_y=-0.1, mag_z=0.3,
        accel_x=0.1, accel_y=0.2, accel_z=9.81,
        gyro_x=0.01, gyro_y=-0.02, gyro_z=0.03,
    )


# ── Dual-mode parser ───────────────────────────────────────────────

def test_parser_decodes_binary_frame():
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(binary.encode(_sample()))
    assert vn.poll() == 1
    d = vn.get_data()
    assert abs(d.yaw - 10.0) < 1e-3 and abs(d.accel_z - 9.81) < 1e-3


def test_parser_consecutive_binary_frames():
    tp = LoopbackTransport()
    vn = VN100(tp)
    for _ in range(20):
        tp.feed(binary.encode(_sample()))
    assert vn.poll() == 20


def test_parser_mixed_ascii_and_binary():
    tp = LoopbackTransport()
    vn = VN100(tp)
    tp.feed(binary.encode(_sample()))
    tp.feed(protocol.build_command("VNRRG,46,0,120,0.01,0.1,0.2,0.3,1,2,3,4,5,6,7,8"))
    tp.feed(binary.encode(_sample()))
    n = vn.poll()
    assert n == 2                                  # poll() counts binary frames only; the VNRRG reply doesn't add to n
    assert vn.get_register(46) is not None         # reply captured via the register cache instead
    assert vn.get_data() is not None


def test_parser_ascii_line_and_binary_in_same_poll():
    tp = LoopbackTransport()
    vn = VN100(tp)
    ymr = protocol.build_command("VNYMR,1,2,3,0.1,0.1,0.1,0,0,9.81,0,0,0")
    tp.feed(ymr)
    tp.feed(binary.encode(_sample()))
    assert vn.poll() == 2                           # VNYMR is a measurement frame, so it IS counted (unlike VNRRG above)


# ── set_output_mode writes the correct registers ───────────────────

def test_set_output_mode_binary_writes_registers():
    tp = LoopbackTransport()
    vn = VN100(tp)
    vn.set_output_mode("binary", rate_hz=200)
    tx = bytes(tp.tx_log).decode("ascii")
    assert "VNWRG,6,0" in tx                        # turn off ASCII (ADOR=0)
    assert "VNWRG,75,2,4,01,0128" in tx             # turn on binary Port 2 (TTL): 800/4=200Hz, YPR+Rate+Accel
    assert vn.fmt == "binary"


def test_set_output_mode_ascii_writes_registers():
    tp = LoopbackTransport()
    vn = VN100(tp)
    vn.set_output_mode("ascii", rate_hz=50)
    tx = bytes(tp.tx_log).decode("ascii")
    assert "VNWRG,75,0" in tx                        # turn off binary
    assert "VNWRG,6,14" in tx                        # turn on VNYMR
    assert "VNWRG,7,50" in tx                        # 50 Hz


# ── Simulator: runtime mode switching ───────────────────────────────

def test_sim_mode_transition_ascii_to_binary():
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, clock=clk, noise=False)   # starts in ASCII
    vn = VN100(tp)
    clk.adv(0.05)
    assert vn.poll() >= 1
    assert tp._ascii_on and not tp._binary_on

    vn.set_output_mode("binary", rate_hz=200)                 # reg6=0 + reg75 on
    assert (not tp._ascii_on) and tp._binary_on
    assert abs(tp.rate_hz - 200.0) < 1e-6                     # 800/4

    clk.adv(0.05)
    assert vn.poll() >= 1
    assert vn.get_data() is not None


def test_vn_mode_command_actually_changes_stream():
    """End-to-end: the dashboard's 'VN MODE BINARY' button switches the sim stream to
    binary, and the parser actually decodes it (last_fmt)."""
    clk = Clk()
    tp = SimTransport(rate_hz=50.0, clock=clk, noise=False)   # starts as an ASCII stream
    vn = VN100(tp)
    clk.adv(0.05); vn.poll()
    assert vn.last_fmt == "ascii"

    tp.write(hostlink.set_mode("binary"))                    # "VN MODE BINARY\n"
    assert tp._binary_on and (not tp._ascii_on)
    assert b"VNMODE BINARY" in tp.read(4096)                 # STM32/sim confirmation returned
    clk.adv(0.05); vn.poll()
    assert vn.last_fmt == "binary"

    tp.write(hostlink.set_mode("ascii"))
    assert tp._ascii_on and (not tp._binary_on)
    clk.adv(0.05); vn.poll()
    assert vn.last_fmt == "ascii"


# ── SetGyroBias: bias compensation while stationary ─────────────────

def test_sim_setgyrobias_reduces_bias():
    clk = Clk()
    sim = Vn100Simulator(motion="still")            # stationary: gyro ~= constant bias
    tp = SimTransport(rate_hz=50.0, clock=clk, noise=False, sim=sim)
    vn = VN100(tp)

    clk.adv(0.05); vn.poll()
    before = vn.get_data()
    g0 = (before.gyro_x ** 2 + before.gyro_y ** 2 + before.gyro_z ** 2) ** 0.5
    assert g0 > 1e-4                                 # there's a measurable bias initially

    tp.write(hostlink.gyro_bias_capture())          # $VNSGB (via the bridge)
    clk.adv(0.05); vn.poll()
    after = vn.get_data()
    g1 = (after.gyro_x ** 2 + after.gyro_y ** 2 + after.gyro_z ** 2) ** 0.5
    assert g1 < g0 * 0.1                             # bias ~zeroed out


# ── Command builders ─────────────────────────────────────────────────

def test_binary_output_command():
    cmd = protocol.binary_output(75, 1, 4, 0x01, 0x0128)
    assert cmd.startswith("$VNWRG,75,1,4,01,0128*")
    assert protocol.verify_ascii(cmd.strip())


def test_set_gyro_bias_command():
    cmd = protocol.set_gyro_bias()
    assert cmd.startswith("$VNSGB*")
    assert protocol.verify_ascii(cmd.strip())
