"""
pyvn100.vn100 — High-level API (HIGH, platform-independent).

Python mirror of the C core's `vn100_t` context + `vn100_rx_feed()` push
model. Wraps a Transport; decodes incoming bytes via the protocol layer,
keeps the latest data and statistics, and sends commands.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Callable, Optional

from . import binary, protocol, registers
from .link import BridgeLink, CommandLink
from .transport import Transport
from .types import Vn100Data

# Longest valid $VN line is ~150 bytes (Reg 47: 12 floats). If no '\n' shows up
# beyond this, the leading '$' isn't a real line start (e.g. 0x24 inside a
# binary payload) -> skip it.
MAX_ASCII_LINE = 256


class VN100:
    """High-level interface to the VN-100 driver."""

    def __init__(
        self,
        transport: Transport,
        on_packet: Optional[Callable[[Vn100Data], None]] = None,
        on_error: Optional[Callable[[str], None]] = None,
        fmt: str = "ascii",
        on_register: Optional[Callable[[int, list], None]] = None,
        link: Optional[CommandLink] = None,
        on_tx: Optional[Callable[[str], None]] = None,
    ):
        self.transport = transport
        self.on_packet = on_packet
        self.on_error = on_error
        self.on_register = on_register
        # Hook that observes EVERY raw command sent (console TX visibility). Triggered by send().
        self.on_tx = on_tx
        # Hook called periodically while a verified write waits for the sensor's
        # response (~every 5 ms). The GUI wires this to `QApplication.processEvents`
        # so the window doesn't freeze (waits can reach 1 s per attempt on real
        # hardware, 3 s for an unresponsive sensor). Library stays Qt-unaware; if
        # no hook is set, the calling thread just blocks for the wait duration.
        self.on_wait: Optional[Callable[[], None]] = None
        self.fmt = fmt                 # last selected output mode ("ascii"|"binary"); parsing AUTO-DETECTS
        # Command-framing strategy (BRIDGE=STM bridge / DIRECT=direct USB-TTL). Defaults to
        # bridge -> preserves existing behavior; the dashboard updates it via detect_link on connect (pyvn100.link).
        self.link: CommandLink = link or BridgeLink()

        self.data: Optional[Vn100Data] = None
        self.packet_count = 0
        self.error_count = 0
        self.last_update: Optional[float] = None
        self.last_fmt: Optional[str] = None   # format of the last DECODED frame ("ascii"|"binary")
        self._registers: dict[int, tuple] = {}   # reg -> (fields, timestamp)
        # Latest command response (bring-up visibility): sensor $VN echo/errors ($VNWRG/$VNERR/$VNTAR...)
        # A queue instead of a single slot: prevents responses arriving back-to-back
        # in one frame (especially $VNERR) from overwriting each other before the
        # GUI polls (drained via drain_responses).
        self.last_response: Optional[str] = None          # backward compat: most recent response
        self.last_response_err: bool = False
        self.last_response_ts: Optional[float] = None
        self._responses: deque = deque(maxlen=64)         # (text, err, ts) — console queue (destructive drain)
        # Non-destructive log of sensor errors ($VNERR...). The dashboard empties
        # _responses every tick via drain_responses(); a dialog asking "was my
        # write rejected?" could race that consumer and miss the $VNERR (e.g.
        # $VNERR,03 lands on the console and the calibration dialog never learns
        # about it). This log is never drained, only queried by timestamp via
        # errors_since() -> the two consumers never starve each other.
        self._error_log: deque = deque(maxlen=64)         # (text, ts) — $VNERR only

        # Connection state (real serial port): False if the reader hits an error/drop, True on reconnect.
        self.connected: bool = True
        self.last_error: Optional[str] = None

        self._buf = bytearray()
        self._lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ════════════════════════════════════════════════════
    #   RX — reading and decoding (push model)
    # ════════════════════════════════════════════════════

    def poll(self, max_bytes: int = 4096) -> int:
        """Read from the transport and decode. Returns the number of new packets decoded in this call."""
        chunk = self.transport.read(max_bytes)
        if not chunk:
            return 0
        return self._feed(chunk)

    def _commit(self, d: Vn100Data) -> None:
        """Record a decoded packet + update stats/callback."""
        d.timestamp = time.time()
        with self._lock:
            self.data = d
            self.packet_count += 1
            self.last_update = d.timestamp
        if self.on_packet:
            self.on_packet(d)

    def _feed(self, chunk: bytes) -> int:
        """
        Decode incoming raw bytes in DUAL MODE: auto-detects and extracts both
        ASCII lines ('$'...'\\n') and binary frames (0xFA...) from the same stream.

        This is required: even in binary mode, command responses ($VNRRG/$VNWRG/$VNERR)
        arrive as ASCII; a parser locked to a single mode would swallow them (docs/protocol.md §4.3).
        """
        self._buf.extend(chunk)
        n = self._scan()
        if len(self._buf) > 4096:      # trim buildup of garbage/noise
            del self._buf[:-512]
        return n

    def _handle_host_line(self, s: bytes) -> None:
        """A dollar-less control-plane line from the STM32 bridge (VNERR/VNMODE/VNACK/VNPONG).

        VNERR -> error (both the console queue and the NON-DESTRUCTIVE error log),
        VNMODE -> info (console only), VNACK/VNPONG -> deliberately DROPPED."""
        is_err = s.startswith(b"VNERR")
        if not (is_err or s.startswith(b"VNMODE")):
            return
        txt = s.decode("ascii", errors="ignore")
        ts = time.time()
        with self._lock:
            self.last_response = txt
            self.last_response_err = is_err
            self.last_response_ts = ts
            self._responses.append((txt, is_err, ts))
            if is_err:
                # The BRIDGE's own failure report ('VNERR fail' = the STM32 could
                # not WRITE the command to the sensor UART) must also land in the
                # non-destructive error log: for commands with no readback (like
                # $VNWNV/$VNSGB), errors_since() is the ONLY channel for checking
                # "was this rejected?". The '$VN' branch already does this — this
                # restores the symmetry.
                self._error_log.append((txt, ts))

    def _consume_host_lines(self, chunk: bytes) -> None:
        """Process the complete ('\\n'-terminated) host lines inside `chunk`.

        Bytes discarded before a frame start also pass through here: while the
        stream is running, the buffer can hold 'VNERR fail\\r\\n' followed by a
        '$VNYMR'; if those VNERR bytes were treated as "leading garbage" and
        dropped, the bridge's error/ack lines would never reach the console
        (this project's rule of 'check the error line, not the ACK' must still
        hold while streaming) — hence they're processed here.

        The remainder after the last '\\n' is deliberately discarded: sitting in
        front of a frame, it can never complete on its own, and keeping it would
        report the split line twice. (The branch that preserves a split line
        when neither '$' nor 0xFA is present is a separate one.)"""
        cut = chunk.rfind(b"\n")
        if cut < 0:
            return
        for hl_line in chunk[:cut + 1].split(b"\n"):
            s = hl_line.strip()
            if s:
                self._handle_host_line(s)

    def _scan(self) -> int:
        """Decode complete ASCII lines and binary frames from the buffer, in order."""
        new_packets = 0
        while self._buf:
            i_ascii = self._buf.find(b"$")
            i_bin = self._buf.find(bytes([binary.SYNC]))
            if i_ascii < 0 and i_bin < 0:
                # Neither '$' nor 0xFA: plain host_link control-plane bytes (STM32 bridge).
                # Surfaces host-level VNERR (bad/nocmd/freq-range/type/baud-disabled/mode/
                # unknown/overflow/fail) as an error, and VNMODE (mode-change ack) as info
                # -> visible on the console. VNACK (just an 'STM wrote to UART' ack, repeats
                # on every command + every poll -> noise; already covered by the TX log +
                # sensor echo) and VNPONG (only during the startup probe, consumed before
                # the reader starts) are deliberately dropped. Sensor lines always start
                # with '$', so they never land here -> misclassification is impossible.
                nl = self._buf.rfind(b"\n")
                if nl < 0:
                    # No complete line yet. Preserve the leading 'VN' fragment so a split
                    # 'VNERR...' isn't lost; a remainder that's too long or doesn't start
                    # with 'VN' is garbage.
                    if len(self._buf) > MAX_ASCII_LINE or not bytes(self._buf).lstrip().startswith(b"VN"):
                        self._buf.clear()
                    break
                complete = bytes(self._buf[:nl + 1])
                del self._buf[:nl + 1]
                self._consume_host_lines(complete)
                continue

            # Which frame type starts first? (A binary payload can contain '$'; if
            # 0xFA comes earlier, binary is processed first.)
            ascii_first = i_ascii >= 0 and (i_bin < 0 or i_ascii < i_bin)

            if ascii_first:
                nl = self._buf.find(b"\n", i_ascii)
                if nl < 0:                       # complete line not received yet
                    if i_ascii > 0:
                        # The leading block may not be "garbage": a bridge VNERR/VNMODE
                        # line could sit right there -> process it BEFORE discarding.
                        self._consume_host_lines(bytes(self._buf[:i_ascii]))
                        del self._buf[:i_ascii]  # drop the leading remainder, wait for the line
                    elif len(self._buf) - i_ascii > MAX_ASCII_LINE:
                        # '$' present but no '\n' within a reasonable length -> a stray '$'
                        # (likely 0x24 inside a binary payload). Skip this '$' so the
                        # binary stream doesn't get stuck.
                        del self._buf[:1]
                        continue
                    break
                # Resync on an embedded '$' (mirrors the C core, vn100.c): if the line
                # contains another '$' further in, the leading part is a half-frame that
                # lost a '\n' (line noise / dropped byte). The C side unconditionally
                # resyncs on seeing '$' and decodes the latest frame; without this, the
                # whole block would be treated as one line, fail the checksum, and also
                # discard the sound $VNERR/$VNRRG right after it — putting a command
                # response at risk, not just one telemetry frame.
                seg = self._buf[i_ascii:nl + 1]
                j = seg.rfind(b"$")
                if j > 0:
                    self.error_count += 1        # the discarded half-frame is counted ONCE
                    i_ascii += j
                line = bytes(self._buf[i_ascii:nl + 1])
                if i_ascii > 0:                  # host lines BEFORE the '$'
                    self._consume_host_lines(bytes(self._buf[:i_ascii]))
                del self._buf[:nl + 1]
                if self._handle_line(line):
                    new_packets += 1
            else:
                if i_bin > 0:
                    # Don't lose host lines BEFORE the 0xFA either — this is the path
                    # most likely to drop them in binary mode (200 Hz stream).
                    self._consume_host_lines(bytes(self._buf[:i_bin]))
                    del self._buf[:i_bin]        # drop the remainder before 0xFA
                # Early header resync (mirrors the C core, vn100.c:245): if the 3 header
                # bytes after 0xFA (0x01|0x28|0x01) don't match, drop just that 0xFA
                # without waiting for a full frame. Otherwise a stray 0xFA sitting in
                # front of a short ASCII response (e.g. $VNRRG) would wait forever for
                # 42 bytes a quiet stream will never send, locking up register reads
                # (get_register stays None). A valid frame is still fully verified
                # afterward, so this doesn't break the "0x24 in the payload" case.
                if len(self._buf) >= 4 and not (
                        self._buf[1] == 0x01 and self._buf[2] == 0x28 and self._buf[3] == 0x01):
                    del self._buf[:1]
                    continue
                if len(self._buf) < binary.FRAME_LEN:
                    break                        # full frame not received yet
                d = binary.decode(bytes(self._buf[:binary.FRAME_LEN]))
                if d is not None:
                    del self._buf[:binary.FRAME_LEN]
                    self.last_fmt = "binary"
                    self._commit(d)
                    new_packets += 1
                elif (self._buf[1] == 0x01) and (self._buf[2] == 0x28) and (self._buf[3] == 0x01):
                    # Header is CORRECT (0xFA|01|0128) but CRC/data is bad -> a genuinely
                    # corrupt frame. Consistent with the C core: consume the whole frame +
                    # count an error (not a 1-byte shift).
                    del self._buf[:binary.FRAME_LEN]
                    with self._lock:
                        self.error_count += 1
                    if self.on_error:
                        self.on_error("binary crc")
                else:
                    del self._buf[:1]            # bad 0xFA sync — advance one byte
        return new_packets

    def _handle_line(self, raw: bytes) -> bool:
        """Decode a single ASCII line. Returns True if it's a valid $VNYMR."""
        line = raw.decode("ascii", errors="ignore")
        d = protocol.parse_vnymr(line)
        if d is not None:
            self.last_fmt = "ascii"
            self._commit(d)
            return True

        # Register-read response ($VNRRG,<reg>,...) -> cache + callback + surface on the CONSOLE
        resp = protocol.parse_vnrrg(line)
        if resp is not None:
            reg, fields = resp
            s = line.strip()
            ts = time.time()
            with self._lock:
                self._registers[reg] = (fields, ts)
                # Visibility: also push the register-read response ($VNRRG,4/46/47/23...)
                # onto the response queue -> visible on the console via GUI
                # drain_responses (the user should 'see everything' except $VNYMR).
                self.last_response = s
                self.last_response_err = False
                self.last_response_ts = ts
                self._responses.append((s, False, ts))
            if self.on_register:
                self.on_register(reg, fields)
            return False

        s = line.strip()

        # A $ line WITH a checksum that FAILS = corrupt (noise). Count as an error,
        # do NOT count it as a command response. (Otherwise a $VNRRG/$VNYMR with a
        # bad checksum would silently look like a 'response' on the console and
        # error_count would under-report -> a "CRC 0 ERR" indicator would look clean
        # when the link isn't.)
        if s.startswith("$") and "*" in s and not self._checksum_ok(s):
            with self._lock:
                self.error_count += 1
            if self.on_error:
                self.on_error("checksum")
            return False

        # A malformed $VNYMR (checksum passes but field count/format is wrong) = data error.
        if s.startswith("$VNYMR"):
            with self._lock:
                self.error_count += 1
            if self.on_error:
                self.on_error(s)
            return False

        # Remaining VALID $VN* lines = command responses/echoes ($VNWRG/$VNERR/$VNTAR/$VNWNV/$VNSGB...).
        # Also appended to the queue -> the GUI sees them all via drain_responses, no $VNERR is ever overwritten.
        # (The STM32 host_link's dollar-less VNACK/VNMODE responses are stripped in
        # _scan and never reach here.)
        if s.startswith("$VN"):
            err = s.startswith("$VNERR")
            ts = time.time()
            with self._lock:
                self.last_response = s
                self.last_response_err = err
                self.last_response_ts = ts
                self._responses.append((s, err, ts))
                if err:
                    self._error_log.append((s, ts))   # non-destructive -> visible to errors_since()
        return False

    @staticmethod
    def _checksum_ok(line: str) -> bool:
        """Verify the 8-bit XOR checksum of a '$<body>*<CS>' line (2 hex digits)."""
        star = line.rfind("*")
        if star < 1:
            return False
        try:
            recv = int(line[star + 1:star + 3], 16)
        except (ValueError, IndexError):
            return False
        return protocol.xor_checksum(line[1:star]) == recv

    def get_register(self, reg: int):
        """Return the last-received register response: (fields, timestamp), or None."""
        with self._lock:
            return self._registers.get(reg)

    def get_data(self) -> Optional[Vn100Data]:
        """Return the last valid measurement (thread-safe)."""
        with self._lock:
            return self.data

    def drain_responses(self) -> list[tuple[str, bool, float]]:
        """Return accumulated command responses (text, err, ts) and empty the queue.
        Called by the GUI every tick -> none of the back-to-back responses (especially $VNERR) are ever lost."""
        with self._lock:
            out = list(self._responses)
            self._responses.clear()
            return out

    def errors_since(self, since: float) -> list[str]:
        """Sensor $VNERR lines that arrived AFTER `since` (NON-DESTRUCTIVE).

        drain_responses() is for the console and empties the queue; this query is
        independent of it -> use it to check whether a write was rejected without
        racing the console consumer.
        """
        with self._lock:
            return [txt for txt, ts in self._error_log if ts >= since]

    def stats(self) -> dict:
        with self._lock:
            return {
                "packets": self.packet_count,
                "errors": self.error_count,
                "last_update": self.last_update,
                "connected": self.connected,
                "last_error": self.last_error,
            }

    # ════════════════════════════════════════════════════
    #   Reader thread (convenience for the dashboard)
    # ════════════════════════════════════════════════════

    def start_reader(self, interval: float = 0.001) -> None:
        """Start a thread that continuously calls poll() in the background."""
        if self._reader and self._reader.is_alive():
            return
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop, args=(interval,), daemon=True
        )
        self._reader.start()

    def _read_loop(self, interval: float) -> None:
        """Guarded read loop — the reader thread never dies silently, for any reason.

        If poll() raises (serial disconnect, USB glitch, processing error), the
        thread enters a 'disconnected' state instead of dying, and for
        SerialTransport periodically tries reopen(). Disconnect-then-reconnect
        is normal on real hardware; sim/loopback returns reopen()=False so they
        never disconnect (unchanged behavior there)."""
        while not self._stop.is_set():
            if self.connected:
                try:
                    if self.poll() == 0:
                        time.sleep(interval)
                except Exception as exc:            # noqa: BLE001 — the reader must never crash
                    self._mark_disconnected(exc)
            else:
                if self._stop.wait(0.5):            # disconnected: wait 0.5s (or exit if stopped)
                    break
                self._try_reconnect()

    def _mark_disconnected(self, exc: Exception) -> None:
        with self._lock:
            self.error_count += 1
            self.last_error = repr(exc)
            self.connected = False
        if self.on_error:
            try:
                self.on_error(f"reader: {exc}")
            except Exception:
                pass

    def _try_reconnect(self) -> None:
        try:
            ok = bool(self.transport.reopen())
        except Exception:
            ok = False                              # device still gone -> retry next round
        if ok:
            with self._lock:
                del self._buf[:]                    # discard the half-frame
                self.connected = True
                self.last_error = None
            if self.on_error:
                try:
                    self.on_error("reconnected")
                except Exception:
                    pass

    def stop_reader(self) -> None:
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)

    # ════════════════════════════════════════════════════
    #   TX — commands (HIGH). ($VNRRG responses are handled in _handle_line.)
    # ════════════════════════════════════════════════════

    def send_raw(self, text: str) -> int:
        return self.transport.write(text)

    def send(self, text: str) -> int:
        """Write a raw command line and trigger the on_tx observer (console TX visibility).

        Used by dashboard/dialog/selfcheck commands so every command goes
        through this single choke point onto the console — a path that writes
        directly to the transport would leave dialog commands (gyro bias,
        calibration) invisible there. on_tx only fires on a successful write;
        a failure propagates to the caller, which reports it."""
        n = self.transport.write(text)
        if self.on_tx is not None:
            try:
                self.on_tx(text)
            except Exception:      # noqa: BLE001 — the logging hook must never break the command path
                pass
        return n

    def read_register(self, reg: int) -> int:
        return self.transport.write(protocol.read_register(reg))

    def write_register(self, reg: int, *values) -> int:
        return self.transport.write(protocol.write_register(reg, *values))

    # ════════════════════════════════════════════════════
    #   Verified write — NOT "I wrote it", but "the sensor accepted it"
    # ════════════════════════════════════════════════════

    def _wait_fresh_register(self, reg: int, since: float, timeout: float):
        """Wait for a $VNRRG,<reg> response that arrives after `since`; None if none arrives.

        If the reader thread isn't running (test/CLI), pump the transport
        ourselves -> the same helper works both in the GUI and in tests.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not (self._reader and self._reader.is_alive()):
                try:
                    self.poll()
                except Exception:      # noqa: BLE001 — a disconnect is _mark_disconnected's job
                    pass
            r = self.get_register(reg)
            if r is not None and r[1] >= since:
                return r
            if self.on_wait is not None:
                try:
                    self.on_wait()     # GUI: processEvents -> window doesn't freeze
                except Exception:      # noqa: BLE001 — the presentation hook must not break the command path
                    pass
            time.sleep(0.005)
        return None

    @staticmethod
    def _fields_match(sent, got, tol: float) -> bool:
        """Do the written values match the read-back fields within numeric tolerance?

        The sensor stores float32 and writes it back in its own formatting
        (1.002110 -> 1.00211), so an exact string comparison would be wrong;
        numeric tolerance is used instead. Fields that can't be converted to a
        number (string registers) are compared exactly.

        NaN/inf fail closed: `abs(nan - x) > tol` is always False, so a naive
        comparison would count NaN as "matched". Since this function's only
        job is to prevent a false positive, any non-finite readback counts as
        not matched.
        """
        if len(got) < len(sent):
            return False
        for s, g in zip(sent, got):
            try:
                fs, fg = float(s), float(g)
            except (TypeError, ValueError):
                if str(s).strip() != str(g).strip():
                    return False
                continue
            if not (math.isfinite(fs) and math.isfinite(fg)):
                return False        # NaN/inf -> not verified (don't silently call it a match)
            if abs(fs - fg) > tol:
                return False
        return True

    def write_register_verified(self, reg: int, *values, retries: int = 2,
                                timeout: float = 1.0, tol: float = 1e-4,
                                quiet_stream: bool = True):
        """Write a register, verify the sensor accepted it by reading it back, retrying if needed.

        Why this exists: a successful `transport.write()` only means "the bytes
        left the PC" — it does not prove the sensor accepted them (VNACK isn't
        an acceptance guarantee). The sensor can reject a write with `$VNERR`
        while, without a readback, the UI shows "applied" — the user could
        think a value that was actually rejected (e.g. a 12-float Reg 23
        calibration) got saved to flash. This closes that gap via readback +
        tolerance comparison.

        Flow (each attempt):  [$VNASY,0] -> $VNWRG -> $VNRRG -> wait for a fresh response -> tolerance compare
        Always at the end:    [$VNASY,1]

        quiet_stream: silences the telemetry stream during the write (ICD
        §1.3.9). This isn't just a convenience — on the STM32 bridge, a
        streaming telemetry byte can collide with the host command and get
        dropped, producing `$VNERR,03` (see protocol.async_pause). Pass False
        to disable it.

        Returns: dict — ok, reason, attempts, readback, errors. `ok` is True ONLY if the readback matches.
        """
        if not getattr(self.transport, "writable", True):
            return {"ok": False, "reason": "replay mode — commands never reach the sensor",
                    "attempts": 0, "readback": None, "errors": []}

        paused = False
        if quiet_stream:
            try:
                for c in self.link.async_pause():
                    self.send(c)
                paused = True
            except Exception as exc:   # noqa: BLE001 — try the write anyway even if silencing the stream failed
                self.last_error = f"async_pause: {exc}"

        errors: list[str] = []
        attempts = 0
        readback = None
        reason = "unknown"
        try:
            for attempt in range(retries + 1):
                attempts = attempt + 1
                t0 = time.time()
                try:
                    for c in self.link.write_register(reg, *values):
                        self.send(c)
                    for c in self.link.read_register(reg):
                        self.send(c)
                except Exception as exc:            # noqa: BLE001 — port dropped -> report honestly
                    reason = f"failed to send command: {exc}"
                    break

                r = self._wait_fresh_register(reg, t0, timeout)
                errs = self.errors_since(t0)
                errors.extend(e for e in errs if e not in errors)

                if r is None:
                    reason = (f"Reg {reg} could not be read back (no response within {timeout:.1f} s)"
                              + (f" — sensor error: {errs[-1]}" if errs else ""))
                    continue

                readback = r[0]
                if self._fields_match(values, readback, tol):
                    return {"ok": True, "reason": f"Reg {reg} verified (readback matched)",
                            "attempts": attempts, "readback": readback, "errors": errors}

                reason = (f"Reg {reg} readback MISMATCH "
                          f"(sent={list(values)[:3]}... got={readback[:3]}...)"
                          + (f" — sensor error: {errs[-1]}" if errs else ""))
            return {"ok": False, "reason": reason, "attempts": attempts,
                    "readback": readback, "errors": errors}
        finally:
            if paused:
                try:
                    for c in self.link.async_resume():
                        self.send(c)
                except Exception:      # noqa: BLE001 — if the stream can't be resumed, the reader already reports it
                    pass

    def set_async_output_type(self, ador: int) -> int:
        """Reg 6 (ADOR) — set the async output type."""
        return self.write_register(6, ador)

    def set_async_output_freq(self, hz: int) -> int:
        """Reg 7 (ADOF) — set the async output frequency."""
        return self.write_register(7, hz)

    def write_settings(self) -> int:
        return self.transport.write(protocol.write_settings())

    def restore_factory(self) -> int:
        return self.transport.write(protocol.restore_factory())

    def set_binary_output(self, async_mode: int = 1, rate_divisor: int = 4,
                          group: int = 0x01, fields: int = 0x0128, reg: int = 75) -> int:
        """Configure a Binary Output register (75-77) — output Hz = 800 / rate_divisor."""
        return self.transport.write(
            protocol.binary_output(reg, async_mode, rate_divisor, group, fields))

    def set_output_mode(self, mode: str, rate_hz: int | None = None) -> None:
        """
        Select the output mode at runtime (docs/protocol.md §4.3): 'ascii' | 'binary'.
        ASCII (demo): reg6=VNYMR + reg75 disabled. BINARY (operation): reg6=disabled + reg75.
        No reflash needed since the registers are independent.
        """
        mode = mode.lower()
        # Mode-dependent default: 200 Hz for binary, `registers.ADOF_DEFAULT_HZ`
        # (the ICD's ADOF default, 40 Hz) for ASCII — a single shared default
        # would be wrong for one of them: 200 Hz in ASCII is double the band's
        # ceiling (~98-114 Hz).
        if rate_hz is None:
            rate_hz = 200 if mode == "binary" else registers.ADOF_DEFAULT_HZ
        if mode not in ("ascii", "binary"):
            # Raise explicitly rather than silently coercing a typo (e.g. 'binry')
            # into ASCII.
            raise ValueError(f"mode must be 'ascii' or 'binary', got: {mode!r}")
        if mode == "binary":
            self.set_async_output_type(0)                       # disable ASCII (ADOR=0)
            # Divisor rule matches the C core exactly (vn100.c set_output_mode):
            # 800/hz using integer division (truncation), hz<=0 -> 4 (200 Hz), div 0 -> 1.
            # Must be identical on both sides — a different rounding (e.g. round())
            # would give a different rate for the same request (300 Hz: truncation
            # gives 400 Hz, round() would give 266.7 Hz).
            divisor = (registers.IMU_RATE_HZ // rate_hz) if rate_hz > 0 else 4
            divisor = max(1, divisor)
            self.set_binary_output(async_mode=registers.SENSOR_ASYNC_PORT, rate_divisor=divisor)
        else:                                                    # ascii
            self.set_binary_output(async_mode=0)                # disable binary
            self.set_async_output_type(14)                      # enable VNYMR
            self.set_async_output_freq(rate_hz)
        self.fmt = mode

    def set_gyro_bias(self) -> int:
        """$VNSGB — capture the gyro bias while the sensor is STATIONARY (stationary drift correction)."""
        return self.transport.write(protocol.set_gyro_bias())

    def reset(self) -> int:
        """$VNRST — software reset (secures the Kalman filter after WNV)."""
        return self.transport.write(protocol.reset())
