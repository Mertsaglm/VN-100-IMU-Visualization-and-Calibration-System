"""
pyvn100.transport — Transport layer (LOW, environment/platform-specific).

The Python counterpart of the C-side `vn100_port_t` seam: moving bytes
around. Upper layers (the VN100 class) don't know which transport they're
talking to — serial port, in-memory loopback, or (later) a socket, it
doesn't matter.
"""
from __future__ import annotations

import abc
import threading


ST_VID = 0x0483        # STMicroelectronics USB Vendor ID (covers all ST devices, incl. ST-Link VCP)


def list_ports() -> list[tuple[str, str, int | None, int | None]]:
    """
    Return the serial ports on the system: [(device, description, vid, pid), ...].

    STM32 Nucleo exposes the ST-Link VCP as "STMicroelectronics STLink Virtual
    COM Port"; the launcher can auto-select it. Empty list if pyserial isn't
    installed. vid/pid are real USB identifiers (an ST device is reliably
    identified by VID=0x0483 even if the Windows generic CDC driver's
    description doesn't contain "STM").
    """
    try:
        from serial.tools import list_ports as _lp
    except Exception:
        return []
    return [(p.device, p.description or "", p.vid, p.pid) for p in _lp.comports()]


def find_stlink_port() -> str | None:
    """Auto-detect the ST-Link VCP port (None if not found).

    Among ST-VID=0x0483 devices, distinguishes the ST-Link VCP by PID/description
    — blindly picking the first ST-VID device could return the wrong port if a
    DFU interface or a second Nucleo is present. One clear VCP candidate ->
    return it; several -> return the first and warn; none distinguishable ->
    fall back to the first ST device. No ST-VID device at all -> fall back to
    matching the description text (Windows generic CDC driver).
    """
    ports = list_ports()
    st = [(dev, desc, pid) for dev, desc, vid, pid in ports if vid == ST_VID]
    KNOWN_VCP_PID = {0x374B, 0x3752, 0x374E, 0x374F, 0x3753}   # known ST-Link VCP PIDs
    if st:
        vcp = [dev for dev, desc, pid in st
               if (pid in KNOWN_VCP_PID) or any(k in f"{dev} {desc}".lower()
                                                for k in ("stlink", "st-link", "vcp", "virtual com"))]
        if len(vcp) == 1:
            return vcp[0]
        if len(vcp) > 1:
            import warnings
            warnings.warn(f"Multiple ST-Link VCP candidates: {vcp} — using the first")
            return vcp[0]
        return st[0][0]                          # ST-VID present but VCP undistinguishable -> first ST device
    for device, desc, _vid, _pid in ports:       # no ST-VID -> fall back to description text
        text = f"{device} {desc}".lower()
        if "stlink" in text or "st-link" in text or "stm32" in text:
            return device
    return None


class Transport(abc.ABC):
    """Abstract transport interface. All concrete transports implement this."""

    @abc.abstractmethod
    def read(self, max_bytes: int = 4096) -> bytes:
        """Return pending bytes (non-blocking); b'' if none."""

    @abc.abstractmethod
    def write(self, data: bytes | str) -> int:
        """Send bytes/text, return the number of bytes sent."""

    def close(self) -> None:
        """Release the resource (default: no-op)."""

    def reopen(self) -> bool:
        """Re-establish the connection (after a drop). Default: unsupported -> False.

        Only the real serial port (SerialTransport) implements this
        meaningfully; sim/loopback/replay never drop, so this returns False
        (the VN100 reader won't attempt reconnection in that case)."""
        return False

    @property
    def is_open(self) -> bool:
        return True

    @property
    def port_name(self) -> str:
        return ""

    @property
    def writable(self) -> bool:
        """Is writing commands meaningful on this transport? Advisory flag the UI
        should check before sending a command — False (replay/sim) means a
        written command reaches no sensor at all, so the UI shouldn't show
        'success'."""
        return True

    @property
    def data_is_recorded(self) -> bool:
        """Do measurements come from a live sensor, or from a recording (playback)?

        This axis is independent of `writable`; together they determine the mode:
          live + writable         -> normal hardware (SerialTransport)
          recorded + not writable -> pure playback   (ReplayTransport)
          recorded + writable     -> hybrid          (HybridTransport) — feeds from a recording, writes to the real sensor

        The UI cannot ignore this: data from a recording describes the
        sensor's past, not its current state, so operations that depend on
        live sensor behavior (onboard HSI convergence, the pre-$VNWNV
        stillness gate) cannot be validated against recorded data."""
        return False


class SerialTransport(Transport):
    """Real serial port (pyserial). Used with the STM32 VCP or a USB-UART.

    write_timeout=0.5 s: on a half-open port (device not draining the OS
    buffer), writing a command must not BLOCK the GUI thread forever — on
    timeout a SerialTimeoutException is raised for the caller to catch.
    After a drop, reopen() reopens the same port.
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.0):
        import serial  # lazy import: pyserial is only needed when actually used
        self._serial = serial               # module reference for reopen()
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser = serial.Serial(port, baud, timeout=timeout, write_timeout=0.5)

    def read(self, max_bytes: int = 4096) -> bytes:
        n = self._ser.in_waiting
        if n:
            return self._ser.read(min(n, max_bytes))
        return b""

    def write(self, data: bytes | str) -> int:
        if isinstance(data, str):
            data = data.encode("ascii")
        return self._ser.write(data)

    def close(self) -> None:
        self._ser.close()

    def reopen(self) -> bool:
        """Close and reopen the port. Returns True on success; raises the
        underlying serial exception if the device is still gone, so the
        caller (reader) can catch it and retry.

        Tries the OLD port name first. On failure, RE-RESOLVES the ST-Link
        VCP (find_stlink_port): on Windows, a board/sensor reset or USB
        replug often moves the ST-Link to a DIFFERENT COM number; rather than
        retrying a fixed name forever, we look up the new name and connect to
        that. If no new name can be found either, the original exception
        propagates (the reader retries)."""
        try:
            self._ser.close()
        except Exception:
            pass
        try:
            self._ser = self._serial.Serial(self._port, self._baud,
                                            timeout=self._timeout, write_timeout=0.5)
        except Exception:
            newport = find_stlink_port()            # find the new name if the port got renumbered
            if not newport:
                raise                               # device truly gone -> caller retries
            self._ser = self._serial.Serial(newport, self._baud,
                                            timeout=self._timeout, write_timeout=0.5)
            self._port = newport                    # use the new name going forward
        return True

    @property
    def is_open(self) -> bool:
        return self._ser.is_open

    @property
    def port_name(self) -> str:
        return self._port


class LoopbackTransport(Transport):
    """
    In-memory transport — for tests and simulation.

    feed(): injects bytes into the RX direction (as if the sensor sent them).
    write(): accumulates written bytes in tx_log (for command tests).
    """

    def __init__(self):
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.tx_log = bytearray()

    def feed(self, data: bytes | str) -> None:
        if isinstance(data, str):
            data = data.encode("ascii")
        with self._lock:
            self._rx.extend(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        with self._lock:
            if not self._rx:
                return b""
            chunk = bytes(self._rx[:max_bytes])
            del self._rx[:max_bytes]
            return chunk

    def write(self, data: bytes | str) -> int:
        if isinstance(data, str):
            data = data.encode("ascii")
        self.tx_log.extend(data)
        return len(data)
