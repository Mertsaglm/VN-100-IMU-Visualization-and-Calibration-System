"""
Shared pytest setup.

The Qt tests (`test_calibration_dialog.py`, `test_orientation_axes.py`) create real widgets,
so default to the **offscreen** platform: on a display-less machine (CI, SSH, container) the
Qt platform plugin crashes without a display, and `importorskip` only catches PySide6 not
being installed, not a missing display. Offscreen also avoids windows flashing on screen
during a normal run.

`setdefault` leaves an externally set `QT_QPA_PLATFORM` alone, so debugging with a real
window still works via `QT_QPA_PLATFORM=cocoa pytest ...`.
"""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
