"""
vn100_simulator.py — VN-100 synthetic data generator (hardware-free testing).

Writes $VNYMR messages to stdout; can be consumed by the dashboard or any other
reader. The physics engine comes from a single source: pyvn100.simulator.Vn100Simulator

Usage:
    python vn100_simulator.py [--rate 40] [--no-noise]
"""
import argparse
import sys
import time

from pyvn100.simulator import Vn100Simulator


def _positive_float(s: str) -> float:
    """argparse type: a non-positive rate (0/negative) causes a divide-by-zero crash -> reject."""
    v = float(s)
    if v <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0 [Hz]")
    return v


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="VN-100 synthetic data generator (ASCII $VNYMR)")
    ap.add_argument("--rate", type=_positive_float, default=40.0, help="Output rate [Hz] (>0)")
    ap.add_argument("--no-noise", action="store_true", help="Generate noise-free (deterministic) output")
    ap.add_argument("--motion", choices=["gentle", "calibration", "still"], default="gentle",
                    help="Motion: gentle (oscillation) | calibration (full sphere) | still (stationary)")
    args = ap.parse_args()

    sim = Vn100Simulator(motion=args.motion)
    dt = 1.0 / args.rate
    t0 = time.perf_counter()

    print(f"[SIM] VN-100 simulator — {args.rate:g} Hz. Press Ctrl+C to stop.", file=sys.stderr)
    try:
        while True:
            t = time.perf_counter() - t0
            sys.stdout.write(sim.ascii_frame(t, noise=not args.no_noise))
            sys.stdout.flush()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[SIM] Simulator stopped.", file=sys.stderr)


if __name__ == "__main__":
    main()
