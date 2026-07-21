"""
vn100_dashboard.py — real-time launcher for the VN-100 IMU dashboard.

Modes:
  Simulation (no hardware):   python vn100_dashboard.py --sim
  Auto-detect (find ST-Link): python vn100_dashboard.py --port auto
  Specific port:               python vn100_dashboard.py --port COM5 --baud 115200
  List ports:                  python vn100_dashboard.py --list-ports
  Playback (replay a log):     python vn100_dashboard.py --replay logs/vn100_*.csv
  HYBRID (log -> sensor):      python vn100_dashboard.py --replay logs/gold.csv --port auto --replay-speed 8

Hybrid mode: measurements come FROM THE LOG, commands go TO THE REAL sensor —
replay a single "gold" recording repeatedly to tune a calibration and write
it to the sensor, without rotating the sensor by hand each time. Only
'Offline ellipsoid fit' is valid here (the onboard HSI needs a LIVE data
stream to converge, not a replay); the recording must have been captured in
RAW mode — see docs/calibration.md §4b.

Link mode (command framing): auto-detected on connect (VN PING -> VNPONG
reply means STM bridge, no reply means direct). Force it manually with
--bridge / --direct. A direct USB-TTL adapter is NOT an ST-Link, so
--port auto won't find it — give the port explicitly in direct mode:
  STM bridge:            python vn100_dashboard.py --port auto            (or --bridge)
  Direct USB-TTL:        python vn100_dashboard.py --port COM7 --direct

Data path (STM bridge): VN-100 -> STM32 (USART6) -> relay (ASCII $VNYMR or
binary) -> VCP -> PC. The STM32 relays whichever format is selected (set via
VN MODE); the PC parser auto-detects each frame from its header. (--fmt is
only needed when the PC connects directly to the VN-100.)

Requirements: pip install -r requirements.txt  (pyqtgraph, PySide6, pyserial, numpy)
"""
import argparse
import sys


def _positive_float(s: str) -> float:
    """argparse type: reject non-positive values (--rate: avoids div-by-zero
    in SimTransport; --replay-speed: zero/negative is meaningless)."""
    v = float(s)
    if v <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0")
    return v


def _print_ports() -> int:
    from pyvn100.transport import list_ports
    ports = list_ports()
    if not ports:
        print("No serial port found. (Is the STM32 connected via USB? Is pyserial installed?)")
        return 1
    print("Serial ports found:")
    for device, desc, vid, pid in ports:
        vidpid = f"  [VID:PID={vid:04X}:{pid:04X}]" if vid and pid else ""
        print(f"  {device:<12} {desc}{vidpid}")
    return 0


def main():
    # Reconfigure stdout/stderr to UTF-8 so the Windows console (cp1254) doesn't
    # choke on Unicode output.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ap = argparse.ArgumentParser(description="VN-100 IMU Dashboard")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--sim", action="store_true", help="Simulation mode (no hardware required)")
    grp.add_argument("--port", type=str, default=None,
                     help="Serial port (COM5, /dev/ttyUSB0) or 'auto' (find the ST-Link)")
    # --replay is OUTSIDE the group: combined with --port it's hybrid mode
    # (log feeds data, real sensor gets commands); alone, pure playback.
    ap.add_argument("--replay", type=str, default=None, metavar="CSV",
                    help="Replay a recorded logs/vn100_*.csv file (playback). "
                         "Combined WITH --port: hybrid (log feeds data, commands go to the real sensor)")
    ap.add_argument("--replay-speed", type=_positive_float, default=1.0, metavar="X",
                    help="Playback speed multiplier (>0, default 1). Up to 8x doesn't break "
                         "the fit for calibration recordings; beyond that the wizard's 30 ms "
                         "polling drops samples and the coverage gate may not close")
    ap.add_argument("--replay-loop", action="store_true",
                    help="Loop back to the start when the recording ends (continuous playback)")
    ap.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--rate", type=_positive_float, default=40.0, help="Simulation data rate [Hz] (>0)")
    ap.add_argument("--sim-motion", choices=["gentle", "calibration", "still"], default="gentle",
                    help="Simulated motion: gentle (oscillation) | calibration (full sphere) | still (stationary, gyro bias)")
    ap.add_argument("--fmt", choices=["ascii", "binary"], default="ascii",
                    help="Data format (only for a direct PC-to-VN-100 connection; the STM32 relay sends whichever format was selected)")
    link_grp = ap.add_mutually_exclusive_group()
    link_grp.add_argument("--bridge", action="store_true",
                          help="Send commands via the STM32 bridge (VN RAW $VN...). Default: auto-detect.")
    link_grp.add_argument("--direct", action="store_true",
                          help="Send commands directly to the sensor (raw $VN...); no STM32. Give --port COMx for a USB-TTL adapter.")
    args = ap.parse_args()

    # Link mode: --bridge/--direct force it manually; if neither is given, None -> auto-detected inside run().
    link_mode = "bridge" if args.bridge else "direct" if args.direct else None
    if args.direct and args.port in (None, "auto"):
        print("[NOTE] --direct is for a direct USB-TTL adapter; give the port explicitly "
              "(e.g. --port COM7). 'auto' only finds the ST-Link VCP, not a USB-TTL adapter.", file=sys.stderr)

    if args.list_ports:
        return _print_ports()

    from dashboard.app import run
    from pyvn100 import SimTransport, SerialTransport, ReplayTransport, HybridTransport
    from pyvn100.transport import find_stlink_port, list_ports

    if args.replay and args.sim:
        print("[ERROR] --replay and --sim cannot be used together (both are data sources). "
              "To write a recording to the real sensor: --replay CSV --port COMx", file=sys.stderr)
        return 1

    # ── Playback / hybrid: replay a recorded CSV ───────
    if args.replay:
        try:
            source = ReplayTransport(args.replay, speed=args.replay_speed, loop=args.replay_loop)
        except OSError as e:
            print(f"[ERROR] Could not open recording ({args.replay}): {e}", file=sys.stderr)
            return 1
        if source.n_rows == 0:
            print(f"[ERROR] No valid rows in recording ({args.replay}) — column headers "
                  "must be timestamp,yaw,pitch,roll,gyro_*,accel_*,mag_*.", file=sys.stderr)
            return 1
        speed_txt = f", {args.replay_speed:g}x" if args.replay_speed != 1.0 else ""
        loop_txt = ", loop" if args.replay_loop else ""

        if args.port is None:                       # ── pure playback (no commands can be sent)
            label = f"Source: PLAYBACK ({args.replay}, {source.n_rows} samples{speed_txt}{loop_txt})"
            print(f"[DASHBOARD] Playback — {args.replay} ({source.n_rows} samples{speed_txt}{loop_txt})")
            return run(source, label, fmt="ascii")

        # ── hybrid: log feeds data, commands go to the REAL sensor
        port = find_stlink_port() if args.port == "auto" else args.port
        if port is None:
            print("[ERROR] Could not auto-detect the ST-Link VCP (hybrid mode needs a real sensor). "
                  "For a specific port: --port COMx", file=sys.stderr)
            return 1
        try:
            sensor = SerialTransport(port, args.baud)
        except Exception as e:
            print(f"[ERROR] Could not open serial port ({port}): {e}", file=sys.stderr)
            return 1
        transport = HybridTransport(source, sensor)
        label = (f"Source: HYBRID (log {args.replay}, {source.n_rows} samples{speed_txt}{loop_txt} "
                 f"-> commands {port})")
        print(f"[DASHBOARD] HYBRID — measurements from log ({args.replay}, {source.n_rows} samples"
              f"{speed_txt}{loop_txt}), commands to the REAL sensor ({port} @ {args.baud}).")
        print("[DASHBOARD] Only 'Offline ellipsoid fit' is valid here; the onboard HSI cannot "
              "converge on replayed data (the wizard disables it). The recording must have been "
              "captured in RAW mode — see docs/calibration.md §4b.")
        return run(transport, label, fmt="ascii", link_mode=link_mode)

    # ── Source selection ────────────────────────────────────────────
    if args.sim:
        port = None
    elif args.port in (None, "auto"):
        port = find_stlink_port()
        if port is None:
            if args.port == "auto":
                print("[ERROR] Could not auto-detect the ST-Link VCP port.", file=sys.stderr)
                ports = list_ports()
                if ports:
                    print("Available ports: " + ", ".join(d for d, *_ in ports), file=sys.stderr)
                print("For a specific port: --port COMx", file=sys.stderr)
                return 1
            # Neither --sim nor --port was given and no hardware found -> fall back to simulation
            print("[DASHBOARD] No hardware found -> falling back to simulation mode "
                  "(use --port COMx for real hardware).")
            args.sim = True
        else:
            print(f"[DASHBOARD] ST-Link VCP auto-detected: {port}")
    else:
        port = args.port

    if args.sim:
        transport = SimTransport(rate_hz=args.rate, motion=args.sim_motion, fmt=args.fmt)
        label = f"Source: SIMULATION ({args.rate:g} Hz, {args.sim_motion})"
        print(f"[DASHBOARD] Simulation mode — {args.rate:g} Hz, motion={args.sim_motion}")
    else:
        try:
            transport = SerialTransport(port, args.baud)
        except Exception as e:
            print(f"[ERROR] Could not open serial port ({port}): {e}", file=sys.stderr)
            print("To see available ports: python vn100_dashboard.py --list-ports", file=sys.stderr)
            return 1
        label = f"Source: {port} @ {args.baud}"
        print(f"[DASHBOARD] {label}")

    return run(transport, label, fmt=args.fmt, link_mode=link_mode)


if __name__ == "__main__":
    sys.exit(main())
