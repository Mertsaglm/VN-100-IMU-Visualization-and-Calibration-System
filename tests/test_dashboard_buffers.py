"""
_RingBuffers mag-channel test: KEYS <-> push() <-> snapshot() consistency.

`tools/verify.py` only imports the GUI module and never runs `update_plots`, so a
mismatch between `KEYS` and `push()`/`snapshot()` — a missing mag key causing a
`KeyError` in `update_plots` at runtime — is only caught here. Skipped if PySide6
isn't available (CI).
"""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # ensure headless mode before import
pytest.importorskip("PySide6")   # _RingBuffers lives in app.py; app.py imports PySide6/pyqtgraph

from dashboard.app import _RingBuffers, BUF_LEN            # noqa: E402
from pyvn100 import Vn100Data                              # noqa: E402


def test_snapshot_mag_keys_carry_correct_values():
    """Mag values passed to push() are preserved in snapshot() under mx/my/mz."""
    rb = _RingBuffers()
    rb.push(Vn100Data(
        yaw=10.0, pitch=-5.0, roll=3.0,
        mag_x=0.11, mag_y=-0.22, mag_z=0.33,
        accel_x=0.1, accel_y=0.2, accel_z=9.81,
        gyro_x=0.01, gyro_y=0.02, gyro_z=0.03,
    ))
    snap = rb.snapshot()
    for key in ("mx", "my", "mz"):
        assert key in snap
        assert snap[key].shape == (BUF_LEN,)
    assert snap["mx"][-1] == pytest.approx(0.11)
    assert snap["my"][-1] == pytest.approx(-0.22)
    assert snap["mz"][-1] == pytest.approx(0.33)


def test_keys_push_snapshot_consistent_and_warmup_nan():
    """Snapshot key set matches KEYS exactly; warm-up is NaN, last value is finite."""
    rb = _RingBuffers()
    rb.push(Vn100Data())                                   # default mag=0.0
    snap = rb.snapshot()
    # all keys read by update_plots are present (no missing/extra) -> guards against KeyError
    assert set(snap.keys()) == set(_RingBuffers.KEYS)
    assert {"mx", "my", "mz"}.issubset(snap.keys())
    assert np.isfinite(snap["mx"][-1])                     # last element is real (0.0)
    assert np.isnan(snap["mx"][0])                         # warm-up NaN -> connect="finite" skips it
