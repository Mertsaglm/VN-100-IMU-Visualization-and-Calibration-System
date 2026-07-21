"""Host command protocol tests."""
from pyvn100 import hostlink as hl


def test_command_formats():
    assert hl.ping() == "VN PING\n"
    assert hl.set_freq(200) == "VN FREQ 200\n"
    assert hl.set_type(14) == "VN TYPE 14\n"
    # set_baud removed: 'VN BAUD' is disabled in firmware (host_link.c -> 'VNERR baud-disabled')
    assert not hasattr(hl, "set_baud")
    assert hl.tare() == "VN TARE\n"
    assert hl.save() == "VN SAVE\n"
    assert hl.factory() == "VN FACTORY\n"
    assert hl.raw("$VNRRG,06*XX") == "VN RAW $VNRRG,06*XX\n"


def test_parse_roundtrip():
    assert hl.parse(hl.set_freq(100)) == ("FREQ", ["100"])
    assert hl.parse(hl.tare()) == ("TARE", [])
    assert hl.parse(hl.ping()) == ("PING", [])


def test_parse_invalid():
    assert hl.parse("$VNYMR,1,2,3*00") is None      # sensor message, not a host command
    assert hl.parse("random") is None
    assert hl.parse("VN ") is None


def test_parse_literal_input():
    # Must parse hand-written input too, not just its own output (avoids a round-trip tautology).
    assert hl.parse("VN MODE BINARY\n") == ("MODE", ["BINARY"])
    assert hl.parse("VN FREQ 200") == ("FREQ", ["200"])
    assert hl.parse("VN RAW $VNRRG,44\n") == ("RAW", ["$VNRRG,44"])


def test_new_command_formats():
    # Dual-mode and gyro-bias commands (docs/protocol.md §7)
    assert hl.set_mode("binary") == "VN MODE BINARY\n"
    assert hl.set_mode("ascii") == "VN MODE ASCII\n"
    assert hl.gyro_bias_capture().startswith("VN RAW $VNSGB*")
