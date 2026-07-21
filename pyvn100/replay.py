"""
pyvn100.replay — Transport that replays a recorded CSV log (playback) + hybrid mode.

Replays a `logs/vn100_*.csv` file produced by the dashboard (columns:
timestamp, yaw, pitch, roll, gyro_x/y/z, accel_x/y/z, mag_x/y/z) as a
`$VNYMR` stream, timed against the original timestamps. This lets a session
(especially a calibration run) be reviewed offline without hardware — the
existing VN100/dashboard pipeline consumes the same data unmodified.

Two transports:
  ReplayTransport — pure playback: feeds the recording, commands CANNOT be written (writable=False).
  HybridTransport — feeds the recording + commands go to the REAL sensor (writable=True).

Usage:
  python vn100_dashboard.py --replay logs/vn100_YYYYmmdd_HHMMSS.csv               # pure playback
  python vn100_dashboard.py --replay logs/altin.csv --port auto --replay-speed 8   # hybrid
"""
from __future__ import annotations

import csv
import threading
import time
from typing import Callable, Optional

from . import binary, protocol
from .simulator import Vn100Simulator
from .transport import Transport
from .types import Vn100Data

# Longest valid $VN line is ~150 bytes (same rationale as vn100.MAX_ASCII_LINE;
# not imported from there — coupling replay to vn100 would invert the layer direction).
_MAX_ASCII_LINE = 256


def _rows_from_csv(path: str):
    """Convert a CSV file into a list of (timestamp, Vn100Data); malformed rows are skipped."""
    rows: list[tuple[Optional[float], Vn100Data]] = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                d = Vn100Data(
                    yaw=float(r["yaw"]), pitch=float(r["pitch"]), roll=float(r["roll"]),
                    gyro_x=float(r["gyro_x"]), gyro_y=float(r["gyro_y"]), gyro_z=float(r["gyro_z"]),
                    accel_x=float(r["accel_x"]), accel_y=float(r["accel_y"]), accel_z=float(r["accel_z"]),
                    mag_x=float(r.get("mag_x") or 0.0),
                    mag_y=float(r.get("mag_y") or 0.0),
                    mag_z=float(r.get("mag_z") or 0.0),
                )
                # timestamp parsing is inside the try too: a truncated final row
                # (missing fields -> None -> TypeError) or a hand-edited ISO
                # date (ValueError) should skip the row, not crash playback.
                ts_raw = r.get("timestamp") or ""
                ts: Optional[float] = float(ts_raw) if ts_raw else None
            except (ValueError, KeyError, TypeError):
                continue
            rows.append((ts, d))
    return rows


class ReplayTransport(Transport):
    """
    Replays a CSV recording while presenting itself as a transport.

    read()  : produces $VNYMR lines whose time has come, based on elapsed time.
    write() : ignored (recorded in tx_log).
    clock/speed can be injected for testing (deterministic playback).
    """

    def __init__(self, csv_path: str, clock: Callable[[], float] = time.perf_counter,
                 speed: float = 1.0, loop: bool = False):
        self._rows = _rows_from_csv(csv_path)
        self._clock = clock
        self._speed = max(0.01, float(speed))
        self._loop = loop
        self._i = 0
        self._t0_wall = clock()
        self._t_base = next((t for (t, _) in self._rows if t is not None), None)
        self._synth_dt = 1.0 / 50.0   # default 50 Hz if no timestamp is present
        self.tx_log = bytearray()

    def _due(self, idx: int) -> float:
        t, _ = self._rows[idx]
        if (self._t_base is not None) and (t is not None):
            return (t - self._t_base) / self._speed
        return (idx * self._synth_dt) / self._speed

    def read(self, max_bytes: int = 4096) -> bytes:
        if self._i >= len(self._rows):
            if self._loop and self._rows:
                self._i = 0
                self._t0_wall = self._clock()
            else:
                return b""
        elapsed = self._clock() - self._t0_wall
        out = bytearray()
        while (self._i < len(self._rows)) and (self._due(self._i) <= elapsed):
            out.extend(Vn100Simulator.encode_ascii(self._rows[self._i][1]).encode("ascii"))
            self._i += 1
            if len(out) >= max_bytes:
                break
        return bytes(out)

    def write(self, data: bytes | str) -> int:
        if isinstance(data, str):
            data = data.encode("ascii")
        self.tx_log.extend(data)
        return len(data)

    @property
    def writable(self) -> bool:
        """During playback, commands never reach a sensor -> False. Advisory only:
        the UI should check it before sending a command, to avoid showing a false
        'saved/applied' confirmation."""
        return False

    @property
    def data_is_recorded(self) -> bool:
        return True

    @property
    def n_rows(self) -> int:
        return len(self._rows)

    @property
    def finished(self) -> bool:
        return (not self._loop) and (self._i >= len(self._rows))


class HybridTransport(Transport):
    """Hybrid: measurements from the RECORDING, commands to the REAL sensor —
    "reproducible calibration from a golden recording".

    Purpose: with the sensor sitting still (no manual rotation), replay a
    good recording into the calibration wizard repeatedly and write the
    resulting solution to the real sensor (offline ellipsoid fit; details/
    limits: `docs/calibration.md` §4b).

    Composition (no new protocol code): `source` = ReplayTransport (RX/
    telemetry), `sensor` = SerialTransport (TX + command responses). Both
    injectable for testing.

    ── STREAM SEPARATION (the reason this class exists) ──
    Even sitting still, the sensor keeps broadcasting its own telemetry. If
    passed through, it would mix with the (rotating) recording samples,
    silently poisoning the calibration point cloud with stationary-sensor
    data. So:

      - sensor telemetry  ($VNYMR + binary frame)                     -> stripped, never passed through
      - command responses ($VNRRG/$VNWRG/$VNERR/$VNTAR... plus the
        bridge's dollar-less VNERR/VNMODE/VNACK/VNPONG)               -> passed through

    Passing responses through is mandatory: the Reg 23/44 snapshot 'Cancel'
    relies on, Reg 46/47 reads, and the echo check confirming the sensor
    accepted a command all come from these lines — VNACK only proves the
    STM32 wrote the command to UART, not that the sensor accepted it (see
    `docs/protocol.md` §8.2).

    Stripped telemetry isn't discarded — it's exposed via `live_data`/
    `live_age`, so the pre-$VNWNV stillness gate (UM001 §5.1.3) reads the
    REAL sensor's state, not the recording's (which always says "moving").
    """

    def __init__(self, source: Transport, sensor: Transport):
        self.source = source
        self.sensor = sensor
        self._buf = bytearray()
        self._live: Optional[Vn100Data] = None
        self._live_ts: Optional[float] = None
        self._lock = threading.Lock()

    # ── RX: recording (telemetry) + sensor (responses only) ─────────
    def read(self, max_bytes: int = 4096) -> bytes:
        out = bytearray(self.source.read(max_bytes))
        chunk = self.sensor.read(max_bytes)     # exceptions are NOT swallowed: the reader must see a disconnect and reopen
        if chunk:
            out.extend(self._split(chunk))
        return bytes(out)

    def _split(self, chunk: bytes) -> bytes:
        """Split the sensor stream: telemetry is stripped into `live_data`, the rest passes through.

        Framing follows the same rules as VN100._scan (ASCII '$'...'\\n' /
        binary 0xFA + 3-byte header + FRAME_LEN) — goal here is to SEPARATE
        the two streams, not decode them."""
        buf = self._buf
        buf.extend(chunk)
        out = bytearray()
        while buf:
            i_ascii = buf.find(b"$")
            i_bin = buf.find(bytes([binary.SYNC]))
            if i_ascii < 0 and i_bin < 0:
                # Neither '$' nor 0xFA -> plain host_link control lines (VNACK/VNERR/VNMODE/VNPONG).
                # Not telemetry -> pass through as a full line (VN100._scan does the classification).
                nl = buf.rfind(b"\n")
                if nl < 0:
                    if len(buf) > _MAX_ASCII_LINE:
                        buf.clear()             # garbage buildup
                    break                        # line not complete yet -> wait
                out.extend(buf[:nl + 1])
                del buf[:nl + 1]
                continue

            ascii_first = i_ascii >= 0 and (i_bin < 0 or i_ascii < i_bin)
            if ascii_first:
                if i_ascii > 0:                  # control bytes before '$' -> pass through
                    out.extend(buf[:i_ascii])
                    del buf[:i_ascii]
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > _MAX_ASCII_LINE:
                        del buf[:1]              # stray '$' (0x24 inside a binary payload) -> skip
                        continue
                    break                        # line not complete yet -> wait
                line = bytes(buf[:nl + 1])
                del buf[:nl + 1]
                if not self._capture_ascii(line):
                    out.extend(line)             # not telemetry -> a command response, pass through
            else:
                if i_bin > 0:                    # bytes before 0xFA -> pass through
                    out.extend(buf[:i_bin])
                    del buf[:i_bin]
                if len(buf) >= 4 and not (buf[1] == 0x01 and buf[2] == 0x28 and buf[3] == 0x01):
                    del buf[:1]                  # false sync (mirrors the early resync in vn100.c:245)
                    continue
                if len(buf) < binary.FRAME_LEN:
                    break                        # full frame not received yet -> wait
                d = binary.decode(bytes(buf[:binary.FRAME_LEN]))
                del buf[:binary.FRAME_LEN]
                if d is not None:
                    self._note_live(d)           # valid binary telemetry -> strip it
                # A frame with a bad CRC is also telemetry -> never passed through
                # (no point polluting the recording stream's error counter with sensor noise).
        if len(buf) > 4096:
            del buf[:-512]
        return bytes(out)

    def _capture_ascii(self, line: bytes) -> bool:
        """Strip if $VNYMR (True = telemetry, do NOT pass through). Otherwise False -> a response."""
        s = line.decode("ascii", errors="ignore")
        if not s.lstrip().startswith("$VNYMR"):
            return False
        d = protocol.parse_vnymr(s)
        if d is not None:
            self._note_live(d)
        return True          # a malformed $VNYMR is also telemetry -> drop it

    def _note_live(self, d: Vn100Data) -> None:
        with self._lock:
            self._live = d
            self._live_ts = time.time()

    @property
    def live_data(self) -> Optional[Vn100Data]:
        """Latest telemetry from the REAL sensor (not the recording) — for the stillness gate.
        None if the sensor isn't broadcasting (async output disabled)."""
        with self._lock:
            return self._live

    @property
    def live_age(self) -> Optional[float]:
        """Time elapsed since the last telemetry from the real sensor [s]; None if none has arrived."""
        with self._lock:
            return None if self._live_ts is None else time.time() - self._live_ts

    # ── TX + source management: always the REAL sensor ──────────────────
    def write(self, data: bytes | str) -> int:
        return self.sensor.write(data)

    def close(self) -> None:
        self.source.close()
        self.sensor.close()

    def reopen(self) -> bool:
        """Only the sensor link can drop (the recording file never does) -> reopen delegates to the sensor."""
        return self.sensor.reopen()

    @property
    def is_open(self) -> bool:
        return self.sensor.is_open

    @property
    def port_name(self) -> str:
        return self.sensor.port_name

    @property
    def writable(self) -> bool:
        """Commands reach the REAL sensor -> True (this is what distinguishes it from pure replay)."""
        return True

    @property
    def data_is_recorded(self) -> bool:
        """Measurements come from the recording -> True. UI should disable operations
        that depend on LIVE sensor behavior (e.g. onboard HSI convergence)."""
        return True

    @property
    def n_rows(self) -> int:
        return getattr(self.source, "n_rows", 0)

    @property
    def finished(self) -> bool:
        return getattr(self.source, "finished", False)
