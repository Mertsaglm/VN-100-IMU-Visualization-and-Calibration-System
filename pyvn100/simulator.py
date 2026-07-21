"""
pyvn100.simulator — Fake VN-100 (for development without hardware).

Two pieces:
  - Vn100Simulator : the engine that generates realistic physics (t -> Vn100Data / $VNYMR line)
  - SimTransport   : presents that engine as a Transport -> plugs directly into the VN100 API

This lets the sim -> transport -> parse -> data -> dashboard pipeline be
tested without a VN-100 in hand.

Physics: an orientation (yaw/pitch/roll) is generated; acceleration and
magnetometer are derived from the SAME rotation matrix (mutually consistent).
A realistic hard-iron + soft-iron distortion is applied to the magnetometer,
so the calibration wizard has to correct a real ellipsoid (end-to-end testing
without hardware).

Motion modes:
  - "gentle"      : small oscillation (default; dashboard demo, sensor stays ~level)
  - "calibration" : continuous yaw + large pitch/roll sweep -> covers the WHOLE sphere
                    (for testing the calibration wizard without hardware)
"""
from __future__ import annotations

import math
import random
import re
import threading
import time
from typing import Callable, Optional

import numpy as np

from . import binary, protocol, registers
from .capabilities import capabilities_for
from .registers import ADOF_VALID, HSIMode, HSIOutput, Reg
from .transport import Transport
from .types import Vn100Data

_DEG = math.pi / 180.0
_G = 9.81  # m/s^2

# Magnetic field in the body frame when the sensor is level (yaw=pitch=roll=0) [Gauss].
# |.| approx 0.45 Gauss (typical order of magnitude for Earth's field).
_MAG_LEVEL = np.array([0.228, -0.015, -0.387])

# Default magnetic distortion (what calibration must correct):
#   raw = SOFT @ mag_body + HARD      (SOFT: soft-iron, HARD: hard-iron offset)
_SOFT_IRON = np.array([[1.15, 0.05, 0.00],
                       [0.05, 0.92, 0.03],
                       [0.00, 0.03, 1.08]])
_HARD_IRON = np.array([0.050, -0.030, 0.040])

# Realistic small constant gyro bias [rad/s] (approx [0.17, -0.26, 0.11] deg/s, |.|~0.33 deg/s)
# — the gyro bias tool recovers this via motion="still".
_GYRO_BIAS = np.array([0.0030, -0.0045, 0.0020])

# Noise standard deviations (on the order of a real VN-100)
NOISE_GYRO = 0.002    # rad/s
NOISE_ACCEL = 0.015   # m/s^2
NOISE_MAG = 0.002     # Gauss

_GYRO_EPS = 1.0e-3    # time step for deriving gyro via finite differences [s]


def _rot_body_from_world(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """
    World-to-body rotation matrix from a ZYX (yaw-pitch-roll) orientation (v_body = R @ v_world).

    With this convention, R @ [0,0,G] = G*[-sin(theta), sin(phi)cos(theta), cos(phi)cos(theta)];
    i.e. accel_z=+G when level, matching the existing sim formulas exactly.
    """
    y, p, r = yaw_deg * _DEG, pitch_deg * _DEG, roll_deg * _DEG
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    r_world_from_body = Rz @ Ry @ Rx
    return r_world_from_body.T


class Vn100Simulator:
    """Deterministic engine that generates realistic IMU motion."""

    def __init__(
        self,
        seed: Optional[int] = None,
        motion: str = "gentle",
        soft_iron: Optional[np.ndarray] = None,
        hard_iron: Optional[np.ndarray] = None,
        gyro_bias: Optional[np.ndarray] = None,
    ):
        self._rng = random.Random(seed)
        self.motion = motion
        self._soft = np.eye(3) if soft_iron is None else np.asarray(soft_iron, float)
        self._hard = _HARD_IRON if hard_iron is None else np.asarray(hard_iron, float)
        self._gyro_bias = _GYRO_BIAS if gyro_bias is None else np.asarray(gyro_bias, float)
        # Default: distortion ON (so it's realistic and calibration makes sense)
        if soft_iron is None and hard_iron is None:
            self._soft = _SOFT_IRON
        self._tare = np.zeros(3)   # VN TARE reference (subtracted from the reported YPR)

    def tare(self, t: float) -> None:
        """Emulate $VNTAR: take the current orientation as the ZERO reference -> reported YPR resets.
        Physics (accel/mag/gyro) is UNCHANGED (a real Tare also only shifts the attitude reference)."""
        self._tare = np.array(self._orientation(t), dtype=float)

    def _noise(self, std: float, enable: bool) -> float:
        return self._rng.gauss(0.0, std) if enable else 0.0

    # ── Orientation generators ─────────────────────────────────────
    def _orientation(self, t: float) -> tuple[float, float, float]:
        """(yaw, pitch, roll) at time t — degrees — based on the selected motion mode."""
        if self.motion == "still":
            return 0.0, 0.0, 0.0          # perfectly stationary (for gyro bias measurement)
        if self.motion == "calibration":
            # Continuous yaw rotation + large, unbounded pitch/roll sweep -> whole sphere.
            # Periods are chosen mutually incommensurate so the orientation sweeps
            # the sphere quasi-uniformly (a "figure-8 tumble").
            yaw = (360.0 / 3.0) * t
            yaw = ((yaw + 180.0) % 360.0) - 180.0
            pitch = 88.0 * math.sin(2 * math.pi * t / 7.0)    # avoid gimbal lock: +-88deg
            roll = 175.0 * math.sin(2 * math.pi * t / 11.0)   # +-175deg -> includes Z-down
            return yaw, pitch, roll
        # "gentle" (default): small oscillation — sensor stays ~level
        yaw = 6.0 * math.sin(2 * math.pi * 0.05 * t)
        yaw = ((yaw + 180.0) % 360.0) - 180.0
        pitch = 5.0 * math.sin(2 * math.pi * 0.10 * t)
        roll = 3.0 * math.sin(2 * math.pi * 0.07 * t)
        return yaw, pitch, roll

    def _gyro_body(self, t: float) -> np.ndarray:
        """Derive body angular rate (rad/s) from the finite difference of orientation."""
        r1 = _rot_body_from_world(*self._orientation(t))
        r2 = _rot_body_from_world(*self._orientation(t + _GYRO_EPS))
        d = r2 @ r1.T
        skew = (d - d.T) * 0.5
        # vee(skew) / dt approx angular rate vector
        return np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) / _GYRO_EPS

    # ── Sampling ───────────────────────────────────────────────
    def sample(self, t: float, noise: bool = True) -> Vn100Data:
        """Generate a measurement for the given t (seconds)."""
        yaw, pitch, roll = self._orientation(t)
        r = _rot_body_from_world(yaw, pitch, roll)

        accel = r @ np.array([0.0, 0.0, _G])
        mag_body = r @ _MAG_LEVEL
        mag_raw = self._soft @ mag_body + self._hard   # hard/soft-iron distortion
        gyro = self._gyro_body(t) + self._gyro_bias    # true rotation + constant bias

        # Tare: reported YPR is RELATIVE to the orientation at VN TARE time (reference shift); yaw/roll wrap.
        yaw = ((yaw - self._tare[0] + 180.0) % 360.0) - 180.0
        pitch = pitch - self._tare[1]
        roll = ((roll - self._tare[2] + 180.0) % 360.0) - 180.0

        return Vn100Data(
            yaw=yaw, pitch=pitch, roll=roll,
            mag_x=float(mag_raw[0]) + self._noise(NOISE_MAG, noise),
            mag_y=float(mag_raw[1]) + self._noise(NOISE_MAG, noise),
            mag_z=float(mag_raw[2]) + self._noise(NOISE_MAG, noise),
            accel_x=float(accel[0]) + self._noise(NOISE_ACCEL, noise),
            accel_y=float(accel[1]) + self._noise(NOISE_ACCEL, noise),
            accel_z=float(accel[2]) + self._noise(NOISE_ACCEL, noise),
            gyro_x=float(gyro[0]) + self._noise(NOISE_GYRO, noise),
            gyro_y=float(gyro[1]) + self._noise(NOISE_GYRO, noise),
            gyro_z=float(gyro[2]) + self._noise(NOISE_GYRO, noise),
        )

    @staticmethod
    def encode_ascii(d: Vn100Data) -> str:
        """Encode a measurement into a complete $VNYMR line (same format as the C vn100_encode_vnymr)."""
        body = (
            "VNYMR,"
            f"{d.yaw:+.3f},{d.pitch:+.3f},{d.roll:+.3f},"
            f"{d.mag_x:+.4f},{d.mag_y:+.4f},{d.mag_z:+.4f},"
            f"{d.accel_x:+.3f},{d.accel_y:+.3f},{d.accel_z:+.3f},"
            f"{d.gyro_x:+.4f},{d.gyro_y:+.4f},{d.gyro_z:+.4f}"
        )
        return protocol.build_command(body)

    def ascii_frame(self, t: float, noise: bool = True) -> str:
        """A complete $VNYMR line for time t ('...\\r\\n')."""
        return self.encode_ascii(self.sample(t, noise=noise))


class HSIEmulator:
    """
    Emulates the sensor's onboard real-time HSI calibration (UM001 §7.44-7.46).

    Purpose: test the onboard HSI workflow without hardware. Mirrors the
    sensor's real behavior:
      - Factory default is version-dependent (`hsi_default`): FW v3.1.0.0 -> (Off, Disable, 5),
        i.e. HSI comes up OFF (ICD §3.5.1 DEFAULT column, measured on hardware);
        FW v2.1 -> (Run, Enable, 5), comes up ON (UM001 §8.3). This difference
        drives the calibration strategy, so it comes from capabilities, not hardcoded.
      - While RUN, orientation bins fill in as the sensor rotates; the
        solution converges toward the inverse of the true distortion, so
        AvgResidual drops.
      - RESET clears the solution (for reconverging in a new environment).
      - OFF freezes the solution (ICD: "the solution is NOT cleared on a Run->Off transition").
      - Output mode: RAW=raw, USER=the Reg 23 user solution, CALC=the computed Reg 47.
    Produces Reg 44/47 read responses (Reg 46 only makes sense on the FW v2.x profile).
    """

    BIN_TARGET = 15   # samples for a bin to be considered "full" (convergence scale)

    def __init__(self, true_soft: np.ndarray, true_hard: np.ndarray, n_bins: int = 8,
                 hsi_default: tuple = registers.HSI_CONTROL_DEFAULT_FW3,
                 wander: float = 0.0):
        # n_bins: number of bins Reg 46 produces — used ONLY on the FW v2.x profile
        # (Reg 46 doesn't exist in the FW v3.1.0.0 ICD). Field hardware returned 8
        # bins; UM001's `hsi info` dump showed 7 -> both are supported, the code
        # tolerates either via len(bins).
        self.n_bins = int(n_bins)
        # If >0, the onboard solution never SETTLES (it wanders) -> lets the
        # discriminating power of a "has it converged?" detector be tested.
        # 0.0 = deterministic behavior (default).
        self.wander = float(wander)
        self._true_soft = np.asarray(true_soft, float)
        self._true_hard = np.asarray(true_hard, float)
        # Ideal solution: raw = soft . body + hard  ->  body = soft^-1 . (raw - hard)
        self._target_C = np.linalg.inv(self._true_soft)
        self._target_B = self._true_hard.copy()

        self._hsi_default = tuple(hsi_default)
        self.factory_reset()

    def factory_reset(self) -> None:
        """Emulate $VNRFS: reset Reg 44 to the VERSION's factory default, Reg 23 to identity.

        Actually modeling this matters: otherwise the version-specific factory
        assumption (`0,1,5` FW3 / `1,3,5` FW2) would never be exercised by sim
        tests and would only surface on real hardware.
        """
        self.mode, self.output, self.rate = self._hsi_default
        self._reset_solution()
        # User Reg 23 solution — factory default is identity + zero bias (same in both ICDs)
        self.user_C = np.eye(3)
        self.user_B = np.zeros(3)

    def _reset_solution(self) -> None:
        self.bins = [0] * self.n_bins
        self.num_meas = 0
        self.last_bin = 0
        self.last_meas = (0.0, 0.0, 0.0)
        self._frac = 0.0
        self.est_C = np.eye(3)
        self.est_B = np.zeros(3)

    @staticmethod
    def _octant(v) -> int:
        return ((1 if v[0] >= 0 else 0) << 2) | ((1 if v[1] >= 0 else 0) << 1) | (1 if v[2] >= 0 else 0)

    def observe(self, raw_mag) -> None:
        """Ingest one raw mag sample while in RUN: fill a bin, converge the solution."""
        if self.mode == HSIMode.OFF:
            return
        b = min(self._octant(raw_mag), self.n_bins - 1)   # fold the last octant on 7-bin hardware
        if self.bins[b] < 255:
            self.bins[b] += 1
        self.num_meas = min(self.num_meas + 1, 65535)
        self.last_bin = b
        self.last_meas = (float(raw_mag[0]), float(raw_mag[1]), float(raw_mag[2]))
        # convergence ratio: how full each bin is relative to BIN_TARGET (0..1)
        self._frac = sum(min(c, self.BIN_TARGET) for c in self.bins) / (self.n_bins * self.BIN_TARGET)
        # converge the solution toward the ideal one (interpolation via frac -> deterministic, no drift)
        self.est_C = np.eye(3) + (self._target_C - np.eye(3)) * self._frac
        self.est_B = self._target_B * self._frac
        # Monotonic interpolation ALONE cannot produce non-convergence; if `wander`
        # is given, the solution never settles and keeps oscillating instead —
        # exercises the discriminating power of the "has Reg 47 stabilized?"
        # detector, mimicking real-world behavior under magnetic noise/insufficient
        # coverage. Default 0.0 -> deterministic behavior (doesn't affect existing tests).
        if self.wander:
            faz = self.num_meas * 0.7
            self.est_C = self.est_C + np.eye(3) * (self.wander * math.sin(faz))
            self.est_B = self.est_B + self.wander * math.cos(faz)

    def apply(self, d: Vn100Data) -> Vn100Data:
        """
        Correct the mag reading (UM001 HSIOutput): Reg 23 user compensation is always
        applied; the onboard real-time solution (est_C, est_B) chains on top only
        under USE_ONBOARD.
          NO_ONBOARD  -> Reg 23 only (default identity -> raw mag)
          USE_ONBOARD -> Reg 23 then onboard HSI (registers.py: 'Reg 23 is separate, always active').
        The two are not mutually exclusive — Reg 23 always applies, and USE_ONBOARD
        chains on top of it, mirroring the real sensor's chaining behavior.
        """
        raw = np.array([d.mag_x, d.mag_y, d.mag_z])
        m = self.user_C @ (raw - self.user_B)          # Reg 23 stage: ALWAYS applied
        if self.output == HSIOutput.USE_ONBOARD:
            m = self.est_C @ (m - self.est_B)          # onboard HSI: chained ON TOP of the user solution
        d.mag_x, d.mag_y, d.mag_z = float(m[0]), float(m[1]), float(m[2])
        return d

    @property
    def avg_residual(self) -> float:
        """Average residual that drops as convergence progresses (on the order of a real sensor's AvgResidual)."""
        return 0.005 + 0.120 * (1.0 - self._frac)

    # ── Register commands/responses ────────────────────────────────
    def write_control(self, mode: int, output: int, rate: int) -> None:
        if mode == HSIMode.RESET:
            self._reset_solution()
            self.mode = HSIMode.RUN     # reset + keep running
        else:
            self.mode = mode
        self.output = output
        self.rate = rate

    def write_user_cal(self, vals) -> None:
        v = [float(x) for x in vals[:12]]
        self.user_C = np.array([v[0:3], v[3:6], v[6:9]])
        self.user_B = np.array(v[9:12])

    def resp_control(self) -> str:
        return protocol.build_command(f"VNRRG,{Reg.HSI_CONTROL},{self.mode},{self.output},{self.rate}")

    def resp_status(self) -> str:
        lm = self.last_meas
        fields = [self.last_bin, self.num_meas, f"{self.avg_residual:.6f}",
                  f"{lm[0]:.6f}", f"{lm[1]:.6f}", f"{lm[2]:.6f}"] + list(self.bins)
        return protocol.build_command(f"VNRRG,{Reg.HSI_STATUS}," + ",".join(str(x) for x in fields))

    def resp_calculated(self) -> str:
        c = self.est_C.flatten()
        vals = [f"{x:.6f}" for x in c] + [f"{x:.6f}" for x in self.est_B]
        return protocol.build_command(f"VNRRG,{Reg.HSI_CALCULATED}," + ",".join(vals))


class SimTransport(Transport):
    """
    Presents Vn100Simulator as a Transport — stands in for the real serial port.

    read()  : produces the appropriate number of frames based on elapsed time (rate_hz).
    write() : accepts commands; emulates a simple command/response cycle (generates responses to VNRRG).

    The clock parameter can be injected for testing (deterministic time).
    """

    def __init__(
        self,
        rate_hz: float = 40.0,
        noise: bool = True,
        sim: Optional[Vn100Simulator] = None,
        clock: Callable[[], float] = time.perf_counter,
        respond: bool = True,
        fmt: str = "ascii",
        motion: str = "gentle",
        hsi_bins: int = 8,
        fw_version: str = registers.ICD_FW_BASELINE,
    ):
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0 (division by zero), got: {rate_hz!r}")
        # Emulated firmware. Default = field hardware (3.1.0.0); the sim models v3's
        # real differences (no Reg 46, Reg 44 default is 0,1,5, no $VNTAR), or "the
        # sim passed" wouldn't validate actual field behavior. fw_version="2.1.0.0"
        # lets the older hardware profile be tested too (backward-compatibility tests).
        self.fw_version = str(fw_version)
        self.caps = capabilities_for(self.fw_version)
        self.sim = sim or Vn100Simulator(motion=motion)
        self.rate_hz = rate_hz
        self._dt = 1.0 / rate_hz
        self._noise = noise
        self._clock = clock
        self._respond = respond
        self._fmt = fmt                # initial output mode ("ascii" | "binary")
        # Two independent output-mode switches (like the real sensor): ADOR (reg6) + Binary (reg75)
        self._ascii_on = (fmt != "binary")
        self._binary_on = (fmt == "binary")
        self._gyro_comp = np.zeros(3)  # gyro bias compensation applied after SetGyroBias
        self._last_hz = 0.0            # last 'VN FREQ' (0 = never set) — preserved across MODE changes
        # Paused via $VNASY,0 (ICD §1.3.9): registers are UNCHANGED, only the stream is silenced.
        self._async_paused = False
        # Emulates the sensor's onboard HSI calibration (for testing the onboard workflow).
        # Factory default comes from the emulated firmware (v3: HSI off; v2: on).
        self._hsi = HSIEmulator(self.sim._soft, self.sim._hard, n_bins=hsi_bins,
                                hsi_default=self.caps.hsi_control_default)
        # Startup = state loaded from flash (factory values). $VNWNV refreshes this,
        # $VNRST reverts to it -> a "forgot to WNV" mistake becomes VISIBLE in the sim.
        self._flash = self._persist_snapshot()

        self._t0 = clock()
        self._next = 0.0            # time of the next frame to be generated
        self._buf = bytearray()     # generated but not-yet-read bytes
        self.tx_log = bytearray()
        # read() (the reader thread) and write()/_emulate_response() (the GUI
        # thread) both mutate the same _buf -> serialize access (the dashboard
        # sim demo is multi-threaded)
        self._lock = threading.Lock()

    def _generate_due(self) -> None:
        now = self._clock() - self._t0
        # Avoid excessive buildup (e.g. after a long pause) — at most ~1s worth of frames
        limit = now + self._dt
        max_frames = int(self.rate_hz) + 2
        produced = 0
        while self._next <= now and produced < max_frames:
            d = self.sim.sample(self._next, noise=self._noise)
            # Gyro bias compensation after SetGyroBias (stationary drift correction)
            d.gyro_x -= float(self._gyro_comp[0])
            d.gyro_y -= float(self._gyro_comp[1])
            d.gyro_z -= float(self._gyro_comp[2])
            self._hsi.observe((d.mag_x, d.mag_y, d.mag_z))   # raw mag -> convergence
            d = self._hsi.apply(d)                            # correct based on output mode
            # ASCII (reg6) and binary (reg75) are independent; if both are on, both stream.
            # $VNASY,0 silences both but does NOT change the registers (ICD §1.3.9) ->
            # while paused the sim's internal physics/HSI observation still advances,
            # only telemetry bytes stop being emitted.
            if not self._async_paused:
                if self._ascii_on:
                    self._buf.extend(self.sim.encode_ascii(d).encode("ascii"))
                if self._binary_on:
                    self._buf.extend(binary.encode(d))
            self._next += self._dt
            produced += 1
        if self._next < limit - 1.0:
            # if we've fallen far behind, catch up (stay in sync with real time)
            self._next = now

    def read(self, max_bytes: int = 4096) -> bytes:
        with self._lock:
            self._generate_due()
            if not self._buf:
                return b""
            chunk = bytes(self._buf[:max_bytes])
            del self._buf[:max_bytes]
            return chunk

    def set_motion(self, motion: str) -> str | None:
        """Change the simulated motion at runtime; returns the previous motion.

        Used by the calibration wizard: the default 'gentle' oscillation is too
        small to pass the wizard's motion gate (|gyro|>0.1 rad/s) -> switching to
        'calibration' (whole-sphere sweep) lets the wizard actually fill up.
        Meaningful only in the simulator."""
        with self._lock:
            prev = getattr(self.sim, "motion", None)
            self.sim.motion = motion
        return prev

    def write(self, data: bytes | str) -> int:
        if isinstance(data, str):
            data = data.encode("ascii")
        with self._lock:
            self.tx_log.extend(data)
            if self._respond:
                self._emulate_response(data)
        return len(data)

    def _emulate_response(self, data: bytes) -> None:
        """Emulates a fake VN-100 + STM32 bridge.

        - 'VN RAW $VN...'    -> unwrap the frame, process as a sensor command
        - '$VN...' (direct)  -> a sensor command (PC<->VN-100 direct testing)
        - 'VN FREQ <hz>'     -> change the data output rate (host command)
        Register reads ($VNRRG,44/46/47/23) generate the matching response;
        HSI control writes ($VNWRG,44/23) update the emulated HSI state.
        """
        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            return
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if not line:
                continue
            if line.startswith("VN RAW "):
                raw = line[7:].strip()
                # Mirrors host_link.c: block writes to reg 5 (baud) (5/05/005 -> integer) —
                # host_link.c parses the reg number with atoi and rejects it if 5; the sim matches.
                if raw.startswith("$VNWRG,"):
                    p = raw[7:].lstrip()
                    num = ""
                    for ch in p:
                        if ch.isdigit():
                            num += ch
                        else:
                            break
                    if num and int(num) == 5:
                        self._buf.extend(b"VNERR baud-disabled\r\n")   # same as firmware (dollar-less)
                        continue
                self._sensor_cmd(raw)
            elif line.startswith("$VN"):
                self._sensor_cmd(line)
            # Argument requirement (mirrors the firmware's `&& (arg != NULL)` gate): an
            # argument-less 'VN MODE'/'VN FREQ'/'VN TYPE' falls through to the end of the
            # chain in firmware and produces 'VNERR unknown', not its own specific error
            # message. Without this check the sim used to say 'VNERR mode(...)' (and stayed
            # silent for FREQ) — out of parity.
            elif line.startswith("VN MODE") and line[len("VN MODE"):].strip():
                # Emulate the STM32 relay: actually switch the stream to the selected format.
                # Matches host_link.c's MODE behavior exactly: only 'ASCII'/'BINARY' accepted
                # (unknown argument -> VNERR, mode unchanged); preserves the last VN FREQ if
                # set, otherwise the mode's default; clamps to 50 Hz in ASCII (VCP ~90 Hz ceiling).
                # Firmware's strcmp is case-sensitive ('BINARY'/'ASCII') — the sim must match,
                # or it would accept an argument real hardware rejects. In firmware,
                # arg = strtok(NULL,"") -> everything after 'MODE' is one argument; 'VN MODE
                # ASCII junk' is rejected by firmware, so the sim rejects it too (silently
                # swallowing trailing garbage would be a deviation from hardware).
                mode_arg = line[len("VN MODE"):].strip()
                if mode_arg not in ("ASCII", "BINARY"):
                    self._buf.extend(b"VNERR mode(ASCII|BINARY)\r\n")
                    continue
                want_binary = (mode_arg == "BINARY")
                self._binary_on = want_binary
                self._ascii_on = not want_binary
                hz = self._last_hz if self._last_hz > 0 else (200.0 if want_binary else 50.0)
                if want_binary:
                    # Firmware's vn100_set_output_mode(BINARY,hz) quantizes output to
                    # 800/divisor (div=800/hz integer division); if the sim passed
                    # _last_hz through raw (e.g. 'FREQ 150' then 'MODE BINARY') it would
                    # produce 150 Hz instead of the firmware's actual 160 Hz, and diverge.
                    div = max(1, int(800.0 / hz))
                    hz = 800.0 / div
                else:
                    hz = min(hz, 50.0)
                self.rate_hz = hz
                self._dt = 1.0 / self.rate_hz
                self._buf.extend(
                    (("VNMODE BINARY" if want_binary else "VNMODE ASCII") + "\r\n").encode("ascii"))
            elif line.startswith("VN FREQ") and line[len("VN FREQ"):].strip():
                # Same as host_link.c: valid range is 1..200, otherwise rejected.
                # Firmware uses atoi -> parses a leading integer ('10.5'->10, 'x'->0); if
                # the sim accepted float(), it would diverge from host_link.c.
                parts = line.split()
                if len(parts) >= 3:
                    m = re.match(r'[+-]?\d+', parts[2])
                    hz = float(m.group()) if m else 0.0
                    if 1.0 <= hz <= 200.0:
                        self._last_hz = hz          # RAW request — preserved across a MODE switch (firmware last_hz=hz)
                        if self._binary_on:
                            # In binary, the real output rate is quantized to 800/divisor (like firmware).
                            div = max(1, int(800.0 / hz))
                            hz = 800.0 / div
                        else:
                            # Mirrors host_link.c: rate is CLAMPED to 50 Hz in ASCII (FREQ branch).
                            # The MODE branch already does this; the FREQ branch must apply the same clamp.
                            hz = min(hz, 50.0)
                        self.rate_hz = hz
                        self._dt = 1.0 / hz
                    else:
                        self._buf.extend(b"VNERR freq-range(1..200)\r\n")
            elif line.startswith("VN TYPE") and line[len("VN TYPE"):].strip():
                # Same as host_link.c: only 0 (stream off) or 14 (VNYMR) are valid;
                # arg[0] must be a digit and t in {0,14}, otherwise VNERR. On success, set ADOR
                # (0=ASCII stream off, 14=on).
                parts = line.split()
                arg = parts[2] if len(parts) >= 3 else ""
                m = re.match(r'\d+', arg)
                if m and int(m.group()) in (0, 14):
                    self._ascii_on = (int(m.group()) != 0)
                else:
                    self._buf.extend(b"VNERR type(0|14)\r\n")
            elif line.startswith("VN PING"):
                # host_link.c: 'VN PING' -> 'VNPONG' (a dollar-less bridge response; the PC
                # scanner deliberately drops it, but the sim still produces it for parity with firmware).
                self._buf.extend(b"VNPONG\r\n")
            elif line.startswith("VN TARE"):
                # The bridge translates 'VN TARE' into $VNTAR and forwards it to the
                # sensor -> whatever the sensor's response is, that's what comes back.
                # $VNTAR does NOT EXIST on FW v3.1.0.0 (ICD §1.3) -> $VNERR,04. Modeling
                # this rejection matters: otherwise the dashboard's disabled button
                # could look like it "works" in the sim.
                if not self.caps.has_tare:
                    self._buf.extend(protocol.build_command("VNERR,04").encode("ascii"))
                else:
                    self.sim.tare(self._clock() - self._t0)
                    self._buf.extend(protocol.build_command("VNTAR").encode("ascii"))
            elif line.startswith("VN SAVE"):
                # The real sensor ECHOES $VNWNV back (the echo is preserved EXACTLY —
                # console/tests check it) AND persists the current RAM state.
                self._flash_commit()
                self._buf.extend(protocol.build_command("VNWNV").encode("ascii"))
            elif line.startswith("VN FACTORY"):
                # host_link.c -> vn100_restore_factory -> $VNRFS. Actually resetting
                # matters: otherwise the factory-default claim (`0,1,5` FW3 / `1,3,5`
                # FW2) would never be exercised by the sim.
                self._factory_reset()
                self._buf.extend(protocol.build_command("VNRFS").encode("ascii"))
            elif line.startswith("VN BAUD"):
                # host_link.c:97-104 deliberately rejects 'VN BAUD' (a baud change would drop the link) — the sim matches.
                self._buf.extend(b"VNERR baud-disabled\r\n")
            else:
                # Models the firmware's generic error responses: unrecognized/malformed
                # host commands must not be silently swallowed, they should return VNERR
                # like the hardware. Matches host_link.c's dispatch exactly:
                #   tag != "VN"          -> VNERR bad      (:54-57)
                #   no command           -> VNERR nocmd    (:60-63)
                #   unrecognized command -> VNERR unknown  (:212-215)
                # The argument-less form of 'VN TYPE'/'VN MODE' also lands here: in
                # firmware the `&& (arg != NULL)` gate isn't satisfied, so the chain
                # falls through to the end -> unknown.
                parts_ = line.split()
                if not parts_ or parts_[0] != "VN":
                    self._buf.extend(b"VNERR bad\r\n")
                elif len(parts_) < 2:
                    self._buf.extend(b"VNERR nocmd\r\n")
                else:
                    self._buf.extend(b"VNERR unknown\r\n")
                # Decision (documented): on successful commands the sim does not produce
                # the firmware's VNACK, only the sensor's own echo ($VNWNV/$VNWRG...). On
                # real hardware both arrive, but the PC side deliberately drops VNACK
                # (vn100.py `_scan`: repeats on every command -> noise), so there's no
                # observable difference. Error paths have no VNACK at all, which is why
                # the VNERR responses above are required for parity.

    def _sensor_cmd(self, cmd: str) -> None:
        """Process a '$VNRRG/$VNWRG,...' sensor command + produce a response if needed."""
        # Verify the checksum if present — the real sensor rejects a command with a bad
        # checksum via $VNERR. The simulator verifies it the same way, or an encoding
        # bug on the PC side would pass every sim test and only surface on real
        # hardware. A command with no checksum (typed manually as 'VN RAW $VNRRG,46')
        # is accepted (the real sensor's tolerance for this needs confirming during
        # bring-up — NEEDS-RUNTIME).
        if "*" in cmd and not protocol.verify_ascii(cmd):
            self._buf.extend(protocol.build_command("VNERR,03").encode("ascii"))
            return
        core = cmd.split("*")[0].lstrip("$")
        parts = core.split(",")
        if not parts:
            return
        mnem = parts[0]
        try:
            if mnem == "VNRRG" and len(parts) >= 2:
                resp = self._reg_read(int(parts[1]))
                if resp:
                    self._buf.extend(resp.encode("ascii"))
            elif mnem == "VNWRG" and len(parts) >= 2:
                ok = self._reg_write(int(parts[1]), parts[2:])
                # Produce the same echo as a real VN-100: accepted -> $VNWRG echo, rejected -> $VNERR.
                # This makes a malformed write visible and verifiable in the SIM too.
                self._buf.extend(protocol.build_command(core if ok else "VNERR,03").encode("ascii"))
            elif mnem == "VNSGB":
                # SetGyroBias: capture the current gyro output as bias while stationary.
                # Target register depends on version (v3: Reg 43, v2: Reg 74) but the
                # effect is the same in the sim: bias compensation applied to the output (caps.gyro_bias_reg).
                s = self.sim.sample(self._next, noise=False)
                self._gyro_comp = np.array([s.gyro_x, s.gyro_y, s.gyro_z])
                self._buf.extend(protocol.build_command("VNSGB").encode("ascii"))
            elif mnem == "VNTAR":
                if not self.caps.has_tare:
                    # $VNTAR does NOT EXIST in the FW v3.1.0.0 ICD §1.3 -> real sensor says "Invalid Command".
                    self._buf.extend(protocol.build_command("VNERR,04").encode("ascii"))
                else:
                    self.sim.tare(self._clock() - self._t0)
                    self._buf.extend(protocol.build_command("VNTAR").encode("ascii"))
            elif mnem == "VNASY" and len(parts) >= 2:
                # ICD §1.3.9: pause/resume the stream; does NOT TOUCH the registers.
                self._async_paused = (int(parts[1]) == 0)
                self._buf.extend(protocol.build_command(core).encode("ascii"))
            elif mnem == "VNRFS":
                # Restore factory settings + reset. Actually resets: Reg 44 -> the
                # version's default (v3: 0,1,5), Reg 23 -> identity, ADOR -> 14,
                # binary -> off — otherwise the version-specific factory assumption
                # would never be exercised by the sim.
                self._factory_reset()
                self._buf.extend(protocol.build_command("VNRFS").encode("ascii"))
            elif mnem == "VNRST":
                # Reset = power cycle: UNSAVED changes (without WNV) are LOST.
                self._flash_reload()
                self._buf.extend(protocol.build_command("VNRST").encode("ascii"))
            elif mnem == "VNWNV":
                # Persist + echo (the echo is preserved EXACTLY; consistent with the VNACK != acceptance rule).
                self._flash_commit()
                self._buf.extend(protocol.build_command("VNWNV").encode("ascii"))
        except ValueError:
            return

    # ── Flash model (RAM <-> persistent) ──────────────────────────────────────────
    # Why: $VNWNV and $VNRST need to genuinely distinguish RAM from flash, or the
    # "I applied calibration but forgot $VNWNV" mistake would never be caught by
    # the sim, and losing a setting on power cycle would only ever surface on
    # real hardware.
    #
    # Scope (deliberate): only user registers are persistent. The onboard HSI
    # solution (est_C/est_B/bins/num_meas) is NOT written to flash — it lives in
    # Reg 47, reconverges from scratch on power cycle, and is separate from the
    # user's Reg 23 solution (ICD §3.5.1). Breaking this distinction would
    # misrepresent the Reg 47 -> Reg 23 chain.
    def _persist_snapshot(self) -> dict:
        return {
            "hsi_mode": self._hsi.mode,
            "hsi_output": self._hsi.output,
            "hsi_rate": self._hsi.rate,
            "user_C": np.array(self._hsi.user_C, float).copy(),   # Reg 23
            "user_B": np.array(self._hsi.user_B, float).copy(),
            "ascii_on": self._ascii_on,                            # Reg 6 (ADOR)
            "binary_on": self._binary_on,                          # Reg 75
            "last_hz": self._last_hz,
            "rate_hz": self.rate_hz,                               # Reg 7 (ADOF)
            "gyro_comp": np.array(self._gyro_comp, float).copy(),  # Reg 43/84
        }

    def _persist_restore(self, snap: dict) -> None:
        self._hsi.mode = snap["hsi_mode"]
        self._hsi.output = snap["hsi_output"]
        self._hsi.rate = snap["hsi_rate"]
        self._hsi.user_C = snap["user_C"].copy()
        self._hsi.user_B = snap["user_B"].copy()
        self._ascii_on = snap["ascii_on"]
        self._binary_on = snap["binary_on"]
        self._last_hz = snap["last_hz"]
        self.rate_hz = snap["rate_hz"]
        if self.rate_hz > 0:
            self._dt = 1.0 / self.rate_hz
        self._gyro_comp = snap["gyro_comp"].copy()

    def _flash_commit(self) -> None:
        """$VNWNV / 'VN SAVE' — persist the current RAM state."""
        self._flash = self._persist_snapshot()

    def _flash_reload(self) -> None:
        """$VNRST — power cycle/reset: UNSAVED changes are LOST."""
        self._persist_restore(self._flash)

    def _factory_reset(self) -> None:
        """$VNRFS / 'VN FACTORY' — restore factory settings (ICD §1.3.4).

        Factory defaults come from the emulated firmware; the ADOR default is
        14 in both ICDs (FW3 ICD §3.2.3 DEFAULT column) -> the sensor streams
        $VNYMR out of the box.
        """
        self._hsi.factory_reset()
        self._gyro_comp = np.zeros(3)
        self._ascii_on = True          # ADOR default is 14 (YMR)
        self._binary_on = False        # Reg 75 AsyncMode default is 0
        self._async_paused = False
        self._last_hz = 0.0            # distinguishes "FREQ never set" — NOT 40
        # ADOF (Reg 7) factory default is 40 Hz (ICD §3.2.4). This neighboring
        # register must be reset too; otherwise the sim would keep streaming at
        # the old rate after $VNRFS and diverge from the real sensor.
        self.rate_hz = 40.0
        self._dt = 1.0 / self.rate_hz
        # $VNRFS makes the factory values PERSISTENT (both RAM and flash) —
        # otherwise a $VNRST after a factory reset would bring back the old user settings.
        self._flash_commit()

    def _reg_read(self, reg: int) -> str:
        if reg == Reg.HSI_CONTROL:
            return self._hsi.resp_control()
        if reg == Reg.HSI_STATUS:
            if not self.caps.has_hsi_status_reg:
                # Reg 46 does not exist in the FW v3.1.0.0 ICD -> "Invalid Register".
                # (Field hardware responds with a string of zeros — possibly an
                # undocumented legacy stub; the sim models the ICD since that's the
                # contract. Code is resilient to either case: no decision depends on Reg 46.)
                return protocol.build_command("VNERR,08")
            return self._hsi.resp_status()
        if reg == Reg.HSI_CALCULATED:
            return self._hsi.resp_calculated()
        if reg == Reg.MAG_CALIBRATION:
            vals = [f"{x:.6f}" for x in self._hsi.user_C.flatten()] + \
                   [f"{x:.6f}" for x in self._hsi.user_B]
            return protocol.build_command(f"VNRRG,{reg}," + ",".join(vals))
        if reg == self.caps.gyro_bias_reg:
            # Filter Startup Gyro Bias — the register $VNSGB writes to. ID is
            # version-dependent: FW3 ICD §3.3.5 -> Reg 43; UM001 FW2.1 §7.1.3 -> Reg 74.
            # Being readable back is what lets the gyro-bias write be VERIFIED.
            vals = ",".join(f"{x:.6f}" for x in self._gyro_comp)
            return protocol.build_command(f"VNRRG,{reg},{vals}")
        if reg == Reg.FIRMWARE_VERSION:
            return protocol.build_command(f"VNRRG,{reg},{self.fw_version}")
        if reg == Reg.MODEL_NUMBER:
            return protocol.build_command(f"VNRRG,{reg},VN-100T-CR")
        if reg == Reg.HARDWARE_REVISION:
            return protocol.build_command(f"VNRRG,{reg},7")     # complete the bring-up identity triple
        # Like the real sensor: an unknown/unemulated register -> 08 Invalid Register
        # (ICD §1.5 Table 1.6). 03/Invalid Checksum is NOT RETURNED: saying "bad
        # checksum" when the checksum is actually valid would lead to a wrong diagnosis.
        return protocol.build_command("VNERR,08")

    def _reg_write(self, reg: int, vals: list) -> bool:
        """Emulate a register write. Returns True (accepted) for a recognized write with
        enough fields; otherwise False -> a $VNERR response is produced like the real sensor (see _sensor_cmd)."""
        if reg == Reg.HSI_CONTROL and len(vals) >= 3:
            self._hsi.write_control(int(vals[0]), int(vals[1]), int(vals[2]))
            return True
        if reg == Reg.MAG_CALIBRATION and len(vals) >= 12:
            self._hsi.write_user_cal(vals)
            return True
        if reg == Reg.ASYNC_DATA_OUTPUT_TYPE and len(vals) >= 1:
            self._ascii_on = (int(vals[0]) != 0)          # ADOR: 0=off, 14=VNYMR
            return True
        if reg == Reg.ASYNC_DATA_OUTPUT_FREQ and len(vals) >= 1:
            hz = float(vals[0])
            # ICD §3.2.4 Table 3.9: ADOF is a CLOSED enum. The sim must enforce this
            # enum; otherwise a write the real sensor would reject (e.g. 30 Hz) would
            # look "accepted" in the sim and only produce $VNERR on hardware.
            if int(hz) != hz or int(hz) not in ADOF_VALID:
                return False                      # -> caller echoes back $VNERR
            if hz > 0:
                self.rate_hz = hz
                self._dt = 1.0 / hz
            return True
        if reg in (Reg.BINARY_OUTPUT_1, Reg.BINARY_OUTPUT_2, Reg.BINARY_OUTPUT_3) and len(vals) >= 2:
            async_mode = int(vals[0])
            self._binary_on = (async_mode != 0)           # AsyncMode: 0=off
            divisor = int(vals[1]) if str(vals[1]).isdigit() else 0
            if async_mode != 0 and divisor > 0:
                self.rate_hz = 800.0 / divisor            # output Hz = 800 / RateDivisor
                self._dt = divisor / 800.0
            return True
        return False
