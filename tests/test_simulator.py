"""
Tests for the simulator engine + SimTransport.

Verifies the full pipeline without hardware, using a deterministic clock (FakeClock):
    SimTransport -> VN100.poll() -> parse -> data
"""
import math

from pyvn100 import VN100, Vn100Simulator, SimTransport


class FakeClock:
    """Deterministic clock, advanced manually by the test."""

    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


# ── Engine ────────────────────────────────────────────────────────

def test_sample_is_deterministic():
    s = Vn100Simulator()
    a = s.sample(1.234, noise=False)
    b = s.sample(1.234, noise=False)
    assert a == b


def test_sample_is_physically_plausible():
    s = Vn100Simulator()
    d = s.sample(0.0, noise=False)
    # at t=0 angles are ~0, gravity is ~ +Z
    assert abs(d.yaw) < 1.0
    assert abs(d.pitch) < 1.0
    assert abs(d.roll) < 1.0
    assert 9.5 < d.accel_z < 10.0
    # total accel magnitude ~9.81
    mag = math.sqrt(d.accel_x**2 + d.accel_y**2 + d.accel_z**2)
    assert abs(mag - 9.81) < 0.1


def test_ascii_frame_is_parseable():
    from pyvn100 import protocol
    s = Vn100Simulator()
    line = s.ascii_frame(2.5, noise=False)
    assert protocol.verify_ascii(line.strip())
    assert protocol.parse_vnymr(line) is not None


# ── SimTransport -> VN100 end-to-end ───────────────────────────────

def test_simtransport_end_to_end():
    clk = FakeClock()
    tp = SimTransport(rate_hz=100.0, noise=False, clock=clk)
    vn = VN100(tp)

    clk.advance(1.0)   # advance 1 second -> ~100 frames

    total = 0
    while True:
        n = vn.poll()
        if n == 0:
            break
        total += n

    assert 99 <= total <= 102, f"expected ~100, got {total}"
    d = vn.get_data()
    assert d is not None
    assert d.timestamp is not None
    assert 9.0 < d.accel_z < 10.5


def test_simtransport_mimics_commands():
    clk = FakeClock()
    tp = SimTransport(rate_hz=40.0, noise=False, clock=clk, respond=True)
    vn = VN100(tp)
    vn.read_register(6)
    # command was recorded in tx_log
    assert "$VNRRG,6*" in tp.tx_log.decode("ascii")


# ── host_link.c behavior parity (sim vs firmware deviations) ──

def test_sim_mode_preserves_last_freq():
    """EXACT match with host_link.c: 'VN FREQ 10' then 'VN MODE ASCII' does NOT
    clobber the rate -- sim keeps the pre-switch rate instead of unconditionally
    resetting to 50/200 (hardware parity)."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN FREQ 10\n")
    tp.write("VN MODE ASCII\n")
    assert tp.rate_hz == 10.0
    # if FREQ was never set, fall back to the mode's default
    tp2 = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp2.write("VN MODE BINARY\n")
    assert tp2.rate_hz == 200.0


def test_sim_mode_ascii_clamps_to_50hz():
    """Setting 200 Hz in binary then switching to ASCII clamps the rate to 50
    (ADOF=200 exceeds the ~90 Hz ASCII VCP ceiling; same clamp as host_link.c)."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN MODE BINARY\n")
    tp.write("VN FREQ 200\n")
    tp.write("VN MODE ASCII\n")
    assert tp.rate_hz == 50.0


def test_sim_ASCII_FREQ_also_clamps_to_50hz():
    """The FREQ clamp in ASCII mode must be as strict as the MODE clamp -- if
    'VN FREQ 200' writes 200 to ADOF unconditionally (in firmware or sim), the
    link's bandwidth ceiling is exceeded. ASCII frames are 101-118 B -> at
    115200 baud the ceiling is ~98-114 Hz; 200 Hz is double that."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())   # default ASCII
    tp.write("VN FREQ 200\n")
    assert tp.rate_hz == 50.0, "FREQ was not clamped in ASCII (bandwidth ceiling exceeded)"
    assert tp._last_hz == 200.0, "the raw request must be kept -- the clamp must NOT overwrite last_hz"
    # switching to BINARY restores the preserved 200 (the clamp isn't permanent).
    tp.write("VN MODE BINARY\n")
    assert tp.rate_hz == 200.0


def test_sim_binary_freq_800_divisor_is_quantized():
    """In binary mode the actual output rate is quantized to 800/divisor (like
    firmware): 'VN FREQ 170' -> div=int(800/170)=4 -> 200 Hz. The raw request
    is kept in _last_hz."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN MODE BINARY\n")            # switch to binary
    tp.write("VN FREQ 170\n")               # 170 -> div=4 -> 200
    assert abs(tp.rate_hz - 200.0) < 1e-6
    assert tp._last_hz == 170.0             # raw request (kept across MODE switch)


def test_sim_mode_unknown_argument_does_not_change_mode():
    """EXACT match with host_link.c: 'VN MODE XYZ' -> VNERR, mode/rate UNCHANGED --
    an unrecognized argument must not silently be treated as ASCII."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock(), fmt="binary")
    tp.write("VN MODE XYZ\n")
    assert tp._binary_on and not tp._ascii_on          # mode preserved
    assert b"VNERR mode" in bytes(tp._buf)


def test_sim_factory_reset_also_restores_ADOF_to_40():
    """A factory reset must clear both ADOR (Reg 14) and its sibling ADOF (Reg 7);
    forgetting one would leave the sim streaming at the old rate, diverging from
    real hardware. ICD §3.2.4: ADOF factory default is 40 Hz."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN FREQ 10\n")
    assert tp.rate_hz == 10.0
    tp.write("VN FACTORY\n")
    assert tp.rate_hz == 40.0, "factory reset did not restore ADOF to 40 Hz"
    # _last_hz must stay 0 ("no FREQ given"); 40 would wrongly imply a rate to
    # preserve across a MODE switch.
    assert tp._last_hz == 0.0


def test_sim_freq_out_of_range_is_rejected():
    """EXACT match with host_link.c: FREQ outside 1..200 is rejected, rate unchanged."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN FREQ 300\n")
    assert tp.rate_hz == 50.0
    assert b"VNERR freq-range" in bytes(tp._buf)


def test_sim_corrupt_checksum_is_rejected():
    """A real sensor rejects a command with a bad checksum via $VNERR -- so does
    the sim; if the sim didn't verify the checksum, a PC-side encoding bug would
    pass every sim test and only surface on real hardware."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN RAW $VNRRG,4*00\n")                   # deliberately wrong checksum
    out = bytes(tp._buf).decode("ascii", errors="ignore")
    assert "$VNERR" in out
    assert "2.1.0.0" not in out                        # no reply was produced


def test_sim_raw_reg5_baud_write_is_rejected():
    """'VN RAW $VNWRG,005,...' (5/05/005 -> same integer) is rejected in sim too;
    a reg-5 write never REACHES _reg_write, VNERR baud-disabled is returned
    (matches host_link.c)."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN RAW $VNWRG,005,921600\n")               # zero-padded reg 5
    out = bytes(tp._buf).decode("ascii", errors="ignore")
    assert "VNERR baud-disabled" in out
    assert tp.rate_hz == 50.0                            # state unchanged (write blocked)
    assert "$VNWRG,005" not in out                       # no echo was produced


def test_sim_vn_baud_is_rejected():
    """The 'VN BAUD' host command is deliberately disabled in firmware
    (host_link.c:97-104) -- same in sim."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN BAUD 921600\n")
    assert b"VNERR baud-disabled" in bytes(tp._buf)


def test_sim_tare_is_rejected_on_fw3():
    """FW 3.1.0.0 ICD §1.3's command list has no $VNTAR -- a real sensor replies
    'Invalid Command', so the dashboard's Tare button is a no-op on this
    hardware. Sim must reject it too, or demo mode would make a dead button
    look 'working'."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN TARE\n")
    assert b"$VNERR,04*" in bytes(tp._buf)
    assert b"$VNTAR*" not in bytes(tp._buf)


def test_sim_tare_echo_on_fw2():
    """Older hardware (UM001): $VNTAR exists and is echoed -- backward compat preserved."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock(), fw_version="2.1.0.0")
    tp.write("VN TARE\n")
    assert b"$VNTAR*" in bytes(tp._buf)
    from pyvn100 import protocol
    tp.write("VN RAW " + protocol.build_command("VNWNV").strip() + "\n")  # RAW for permanent save
    assert b"$VNWNV*" in bytes(tp._buf)


def test_sim_async_pause_mutes_stream_without_touching_register():
    """$VNASY,0 mutes the stream without touching ADOR (ICD §1.3.9); $VNASY,1
    resumes it. This is the ICD-supported way to run long configuration writes
    without colliding with live telemetry -- the firmware-safe alternative to
    $VNERR,03 (see protocol.async_pause)."""
    from pyvn100 import protocol
    clk = FakeClock()
    tp = SimTransport(rate_hz=50.0, noise=False, clock=clk)
    clk.t = 0.2
    tp.read()                                          # drain the backlog
    tp.write("VN RAW " + protocol.async_pause().strip() + "\n")
    tp.read()                                          # consume the echo
    clk.t = 0.6
    assert tp.read() == b""                            # no telemetry bytes while paused
    tp.write("VN RAW " + protocol.async_resume().strip() + "\n")
    clk.t = 1.0
    assert b"$VNYMR" in tp.read()                      # stream resumed (ADOR untouched)


# ── AUDIT parity fixes (sim <-> firmware) ────────────────────
def test_sim_mode_binary_rate_is_also_quantized():
    """'FREQ 150' then 'MODE BINARY': firmware produces 800/div=160; sim must
    produce 160 too -- if the MODE branch passed _last_hz through raw, a rate
    (150) that can never exist on real hardware would appear."""
    tp = SimTransport(fmt="binary", noise=False, clock=FakeClock())
    tp.write("VN FREQ 150\n")
    assert abs(tp.rate_hz - 160.0) < 1e-6               # 800/int(800/150)=800/5=160
    tp.write("VN MODE BINARY\n")
    assert abs(tp.rate_hz - 160.0) < 1e-6               # MODE quantizes too


def test_sim_mode_trailing_garbage_is_rejected():
    """'VN MODE ASCII junk' is rejected by firmware (arg='ASCII junk'); sim must
    match -- it must not take split()[2], silently swallow the garbage, and
    change mode anyway."""
    tp = SimTransport(fmt="binary", noise=False, clock=FakeClock())
    tp.write("VN MODE ASCII junk\n")
    assert b"VNERR mode" in bytes(tp._buf)
    assert tp._binary_on is True                        # mode UNCHANGED


def test_sim_vn_type_and_ping():
    """host_link.c's VN TYPE (0/14) + VN PING must be handled in sim too (not ignored)."""
    tp = SimTransport(fmt="ascii", noise=False, clock=FakeClock())
    tp.write("VN TYPE 0\n")
    assert tp._ascii_on is False                        # stream off
    tp.write("VN TYPE 14\n")
    assert tp._ascii_on is True                          # VNYMR on
    tp._buf.clear()
    tp.write("VN TYPE 99\n")                            # invalid
    assert b"VNERR type" in bytes(tp._buf)
    tp._buf.clear()
    tp.write("VN PING\n")
    assert b"VNPONG" in bytes(tp._buf)


def test_sim_rate_zero_is_rejected():
    """--rate 0/negative must raise a meaningful error instead of a divide-by-zero crash."""
    import pytest
    with pytest.raises(ValueError):
        SimTransport(rate_hz=0)
    with pytest.raises(ValueError):
        SimTransport(rate_hz=-5)


# ── Parity with firmware's GENERIC error replies ────────────────────────────
# Counterpart: pc/host_selftest.c "host_link: generic error replies" block.
# Expected values were MEASURED from host_link.c's dispatch (not hand-written):
#   tag != "VN" -> bad · no command -> nocmd · unrecognized/no-argument -> unknown
_ERROR_PARITY = [
    ("FOO BAR", b"VNERR bad"),
    ("VN", b"VNERR nocmd"),
    ("VN XYZZY", b"VNERR unknown"),
    ("VN TYPE", b"VNERR unknown"),     # no argument -> firmware's `&& arg != NULL` gate
    ("VN MODE", b"VNERR unknown"),
    ("VN FREQ", b"VNERR unknown"),
]


def test_sim_generic_error_replies_match_firmware():
    """The sim's elif chain must end in an `else`: unrecognized/malformed host
    commands must NOT be silently swallowed, and must return VNERR like the
    hardware does. Otherwise a whole class of 'why didn't my command work?'
    bugs would never show up in sim."""
    for line, expected in _ERROR_PARITY:
        tp = SimTransport(rate_hz=1.0, noise=False, clock=FakeClock())
        tp._buf.clear()
        tp.write(line + "\n")
        assert expected in bytes(tp._buf), f"{line!r} -> {bytes(tp._buf)!r} (expected {expected!r})"


def test_sim_valid_commands_produce_NO_error():
    """Counter-check: the else branch must not catch legitimate commands (over-strictness regression)."""
    for line in ("VN PING", "VN SAVE", "VN FACTORY", "VN MODE ASCII",
                 "VN FREQ 10", "VN TYPE 14", "VN RAW $VNRRG,1*71"):
        tp = SimTransport(rate_hz=1.0, noise=False, clock=FakeClock())
        tp._buf.clear()
        tp.write(line + "\n")
        out = bytes(tp._buf)
        assert b"VNERR bad" not in out and b"VNERR nocmd" not in out \
            and b"VNERR unknown" not in out, f"{line!r} was rejected incorrectly: {out!r}"


# ── FLASH MODEL ($VNWNV is permanent, $VNRST reverts) ─────────────────────
def _reg23(tp):
    """Reg 23 on the sensor (user mag calibration) -- its first term is a sufficient marker."""
    return float(tp._hsi.user_C[0][0])


def test_sim_write_WITHOUT_WNV_is_LOST_on_RESET():
    """If $VNWNV were a no-op and $VNRST just echoed, the 'applied calibration
    but forgot to SAVE it' bug could never be caught in sim -- only real
    hardware would show the setting vanishing on power cycle. The flash model
    must be able to catch this."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    initial = _reg23(tp)
    tp.write("VN RAW $VNWRG,23,1.5,0,0,0,1,0,0,0,1,0.1,0.2,0.3\n")   # write to RAM
    assert _reg23(tp) == 1.5, "precondition failed: write did not reach RAM"

    tp.write("VN RAW $VNRST*4D\n")                                    # no WNV -> reset
    assert _reg23(tp) == initial, "unsaved write did NOT get lost on reset"


def test_sim_write_WITH_WNV_SURVIVES_RESET():
    """Counter-check: if $VNWNV was sent, the value persists (no over-strictness regression)."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    tp.write("VN RAW $VNWRG,23,1.5,0,0,0,1,0,0,0,1,0.1,0.2,0.3\n")
    tp.write("VN SAVE\n")                                             # make it permanent
    assert b"VNWNV" in bytes(tp._buf), "the echo must be PRESERVED (console/tests rely on it)"

    tp.write("VN RAW $VNRST*4D\n")
    assert _reg23(tp) == 1.5, "saved write was lost on reset"


def test_sim_flash_EXCLUDES_onboard_HSI_solution():
    """ICD §3.5.1 distinction: the onboard HSI solution (Reg 47) is NOT held in
    flash -- it reconverges from scratch on power cycle and is SEPARATE from
    the user's Reg 23 solution. Writing it to flash would incorrectly model
    the Reg 47 -> Reg 23 chain."""
    tp = SimTransport(rate_hz=50.0, noise=False, clock=FakeClock())
    assert "est_C" not in tp._flash and "est_B" not in tp._flash
    assert "bins" not in tp._flash
    # fields that SHOULD be permanent must be IN flash
    for field in ("user_C", "user_B", "hsi_mode", "ascii_on", "rate_hz", "gyro_comp"):
        assert field in tp._flash, f"a field that should persist is missing from flash: {field}"
