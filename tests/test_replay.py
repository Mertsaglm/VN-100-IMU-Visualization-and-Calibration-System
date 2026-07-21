"""
Playback (ReplayTransport) test — a recorded CSV session is replayed without a
sensor and decoded through the same VN100 pipeline (docs: dashboard --replay).
"""
import csv

from pyvn100 import VN100, ReplayTransport


class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


_HEADER = ["timestamp", "yaw", "pitch", "roll",
           "gyro_x", "gyro_y", "gyro_z",
           "accel_x", "accel_y", "accel_z",
           "mag_x", "mag_y", "mag_z"]


def _write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(_HEADER)
        for r in rows:
            w.writerow(r)


def test_replay_plays_back_recording(tmp_path):
    p = tmp_path / "rec.csv"
    rows = [
        [100.00, 10, -5, 3, 0.01, 0.02, 0.03, 0.1, 0.2, 9.81, 0.2, -0.1, 0.3],
        [100.02, 11, -6, 4, 0.01, 0.02, 0.03, 0.1, 0.2, 9.81, 0.2, -0.1, 0.3],
        [100.04, 12, -7, 5, 0.01, 0.02, 0.03, 0.1, 0.2, 9.81, 0.2, -0.1, 0.3],
    ]
    _write_csv(str(p), rows)

    clk = Clk()
    tp = ReplayTransport(str(p), clock=clk)
    assert tp.n_rows == 3

    vn = VN100(tp)
    clk.adv(0.05)                    # all recorded timestamps have elapsed
    n = vn.poll()
    assert n == 3
    d = vn.get_data()
    assert abs(d.yaw - 12.0) < 1e-2 and abs(d.roll - 5.0) < 1e-2
    assert tp.finished


def test_replay_follows_recorded_timing(tmp_path):
    """Rows arrive gradually, following their recorded timestamps (not all at once)."""
    p = tmp_path / "rec.csv"
    rows = [[100.0 + i * 0.10, i, 0, 0, 0, 0, 0, 0, 0, 9.81, 0, 0, 0] for i in range(5)]
    _write_csv(str(p), rows)

    clk = Clk()
    vn = VN100(ReplayTransport(str(p), clock=clk))
    clk.adv(0.25)                    # only t=0,0.1,0.2 are due (3 rows)
    assert vn.poll() == 3
    clk.adv(0.25)                    # remaining 2 rows
    assert vn.poll() == 2


def test_replay_skips_truncated_last_row(tmp_path):
    """If recording is interrupted, the last row may be truncated (missing fields).
    DictReader fills missing fields with None, and float(None) raises TypeError --
    such rows must be filtered so playback doesn't crash."""
    p = tmp_path / "truncated.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        f.write(",".join(_HEADER) + "\n")
        f.write("100.00,10,-5,3,0.01,0.02,0.03,0.1,0.2,9.81,0.2,-0.1,0.3\n")
        f.write("100.02,11,-6\n")                       # truncated row
    tp = ReplayTransport(str(p), clock=Clk())
    assert tp.n_rows == 1                                # only the complete row


def test_replay_skips_iso_timestamp(tmp_path):
    """A hand-edited/foreign CSV may contain an ISO timestamp string; rows where
    float(ts) fails must be filtered, not raise ValueError, so playback doesn't crash."""
    p = tmp_path / "foreign.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        f.write(",".join(_HEADER) + "\n")
        f.write("2026-07-10T12:00:00,10,-5,3,0.01,0.02,0.03,0.1,0.2,9.81,0.2,-0.1,0.3\n")
        f.write("100.02,11,-6,4,0.01,0.02,0.03,0.1,0.2,9.81,0.2,-0.1,0.3\n")
    tp = ReplayTransport(str(p), clock=Clk())
    assert tp.n_rows == 1


def test_replay_not_writable_flag(tmp_path):
    """In replay, commands never reach a sensor -> writable=False (so the UI
    doesn't lie about 'saved'); other transports default to True."""
    from pyvn100 import LoopbackTransport, SimTransport
    p = tmp_path / "rec.csv"
    _write_csv(str(p), [[100.0, 0, 0, 0, 0, 0, 0, 0, 0, 9.81, 0, 0, 0]])
    assert ReplayTransport(str(p), clock=Clk()).writable is False
    assert LoopbackTransport().writable is True
    assert SimTransport(clock=Clk()).writable is True
