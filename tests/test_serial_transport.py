"""
SerialTransport -- the one code path that talks to real hardware.

Tests verify `reopen()`'s reconnect behavior: after a board reset or USB
re-plug, Windows may move the ST-Link to a new COM number, and getting stuck
on the old one shows up as "the dashboard won't reconnect". pyserial is
mocked (no real port opened); the class is written so it can be injected via
its `self._serial` module reference.
"""
import sys
import types

import pytest

from pyvn100 import transport as tp_mod
from pyvn100.transport import SerialTransport


class _FakeSerial:
    """Minimal pyserial.Serial stand-in."""

    def __init__(self, port, baud, timeout=0.0, write_timeout=0.5, fail=False):
        if fail:
            raise OSError(f"no such port: {port}")
        self.port = port
        self.baud = baud
        self.is_open = True
        self._incoming = bytearray()
        self.written = bytearray()
        self.closed = False

    # --- the subset of the pyserial API that's used ---
    @property
    def in_waiting(self):
        return len(self._incoming)

    def read(self, n):
        data = bytes(self._incoming[:n])
        del self._incoming[:n]
        return data

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def close(self):
        self.closed = True
        self.is_open = False

    # --- test helper ---
    def feed(self, data: bytes):
        self._incoming.extend(data)


def _fake_module(monkeypatch, *, failing_ports=()):
    """Fakes the result of `import serial`; records which ports were opened."""
    opened = []

    def _Serial(port, baud, timeout=0.0, write_timeout=0.5):
        opened.append(port)
        return _FakeSerial(port, baud, timeout, write_timeout,
                            fail=(port in failing_ports))

    mod = types.ModuleType("serial")
    mod.Serial = _Serial
    monkeypatch.setitem(sys.modules, "serial", mod)
    return opened


def test_read_returns_only_whats_pending(monkeypatch):
    _fake_module(monkeypatch)
    t = SerialTransport("COM_TEST")
    assert t.read() == b""                     # nothing pending -> empty (non-blocking)
    t._ser.feed(b"$VNYMR,1,2,3*00\r\n")
    data = t.read()
    assert data.startswith(b"$VNYMR")
    assert t.read() == b""                     # buffer drained


def test_write_accepts_str_and_bytes(monkeypatch):
    _fake_module(monkeypatch)
    t = SerialTransport("COM_TEST")
    n = t.write("VN PING\n")                   # str -> must be ascii-encoded
    assert n == len("VN PING\n")
    t.write(b"VN SAVE\n")
    assert b"VN PING" in bytes(t._ser.written) and b"VN SAVE" in bytes(t._ser.written)


def test_reopen_reopens_the_SAME_port(monkeypatch):
    opened = _fake_module(monkeypatch)
    t = SerialTransport("COM7")
    old = t._ser
    assert t.reopen() is True
    assert old.closed, "old port handle was not closed (handle leak)"
    assert opened == ["COM7", "COM7"]
    assert t.port_name == "COM7"


def test_reopen_finds_new_name_if_port_was_RENUMBERED(monkeypatch):
    """Field scenario: after a board reset the ST-Link reappears on a different
    COM port -- retrying the old name forever is what "won't reconnect" means."""
    opened = _fake_module(monkeypatch, failing_ports={"COM7"})
    monkeypatch.setattr(tp_mod, "find_stlink_port", lambda: "COM12")

    t = SerialTransport.__new__(SerialTransport)   # __init__ couldn't have opened COM7
    t._serial = sys.modules["serial"]
    t._port, t._baud, t._timeout = "COM7", 115200, 0.0
    t._ser = _FakeSerial("COM7", 115200)

    assert t.reopen() is True
    assert t.port_name == "COM12", "the new port name was not adopted"
    assert "COM12" in opened


def test_reopen_raises_if_device_is_truly_gone(monkeypatch):
    """Fail-loud: if no new name can be found either, the original error must
    propagate so the reader keeps retrying -- silently claiming 'connected' is the worst outcome."""
    _fake_module(monkeypatch, failing_ports={"COM7"})
    monkeypatch.setattr(tp_mod, "find_stlink_port", lambda: None)

    t = SerialTransport.__new__(SerialTransport)
    t._serial = sys.modules["serial"]
    t._port, t._baud, t._timeout = "COM7", 115200, 0.0
    t._ser = _FakeSerial("COM7", 115200)

    with pytest.raises(OSError):
        t.reopen()


def test_writable_and_data_is_recorded_contract(monkeypatch):
    """Real port: commands ARE WRITABLE and data is NOT a recording -- these two
    flags drive the dashboard's hybrid/replay warnings."""
    _fake_module(monkeypatch)
    t = SerialTransport("COM_TEST")
    assert getattr(t, "writable", True) is True
    assert getattr(t, "data_is_recorded", False) is False
