#!/usr/bin/env python3
"""Task 5: PASS/FAIL cadence gate for the evtstream pipeline.

Wraps evtstream_receiver.py in --measure-cadence --duration N mode,
greps the [CADENCE] markers it prints, and asserts:

  1. MCU window inter-arrival std <= 50 us  (the original cadence
     target from the task brief and DESIGN.md / RISK1_FINDINGS §7a).
     Computed across consecutive-sequence pairs only, after the
     receiver drains pre-test stale CDC FIFO bytes.
  2. resync_count == 0 over the test (no magic-byte slips ->
     wire-level integrity is intact).
  3. Post-drain packet count in [1000 - tol, 1000 + tol] /sec band
     (default tol = 50/sec). Two-sided check catches both
     catastrophic packet loss and unexpected packet surges.

PASS exits 0, FAIL exits 1. Captured numbers are printed regardless
so a failed run is debuggable from the log.

Usage:

    # Default: 10-second test against /dev/ttyACM0 with the OpenMV
    # streaming via evtstream.start(window_us=1000).
    python3 test_cadence.py

    # Stricter / looser thresholds:
    python3 test_cadence.py --max-mcu-std-us 25
    python3 test_cadence.py --max-resyncs 1

    # Different device or duration:
    python3 test_cadence.py --device /dev/ttyACM1 --duration 30
"""
import argparse
import re
import subprocess
import sys


CADENCE_MARKER_RE = re.compile(r"^\[CADENCE\]\s+(\S+)=([-\d.eE+]+)\s*$")


def parse_markers(stdout):
    """Pull all [CADENCE] key=value markers from stdout into a dict.

    Last value wins if a key repeats (which it does: every periodic
    summary plus the final summary all emit the same keys, and we want
    the final values).
    """
    out = {}
    for line in stdout.splitlines():
        m = CADENCE_MARKER_RE.match(line)
        if m:
            key, val = m.group(1), m.group(2)
            try:
                out[key] = float(val)
            except ValueError:
                pass
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="/dev/ttyACM0")
    p.add_argument("--duration", type=float, default=10.0,
                   help="seconds of cadence measurement (default 10)")
    p.add_argument("--max-mcu-std-us", type=float, default=50.0,
                   help="MCU window inter-arrival std threshold (default 50)")
    p.add_argument("--max-resyncs", type=int, default=0,
                   help="maximum allowed resync events (default 0)")
    p.add_argument("--drain-seconds", type=float, default=1.0,
                   help="receiver startup discard window (default 1.0). "
                        "Forwarded to evtstream_receiver.py and used here "
                        "to compute the expected post-drain packet count.")
    p.add_argument("--rate-tolerance-per-sec", type=float, default=50.0,
                   help="acceptable deviation from the nominal 1000 "
                        "packets/sec rate (default +/- 50/sec)")
    p.add_argument("--receiver", default="evtstream_receiver.py",
                   help="path to receiver script (default ./evtstream_receiver.py)")
    args = p.parse_args()

    cmd = [
        sys.executable,
        args.receiver,
        "--measure-cadence",
        "--device", args.device,
        "--duration", str(args.duration),
        "--summary-interval", str(max(args.duration, 1.0)),  # one final summary is enough
        "--drain-seconds", str(args.drain_seconds),
    ]
    print("[test_cadence] running:", " ".join(cmd))

    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(proc.stdout)
    if proc.stderr:
        print("STDERR:", proc.stderr, file=sys.stderr)

    if proc.returncode != 0:
        print("[test_cadence] FAIL: receiver exited %d" % proc.returncode)
        sys.exit(1)

    markers = parse_markers(proc.stdout)

    mcu_std = markers.get("mcu_interval_std_us")
    mcu_mean = markers.get("mcu_interval_mean_us")
    resync = markers.get("resync_count")
    packets = markers.get("packets")
    drained = markers.get("drained_packets")
    consec = markers.get("consecutive_intervals")
    drain_sec = markers.get("drain_seconds", args.drain_seconds)

    # Effective measurement window after subtracting the drain.
    measured_seconds = max(args.duration - drain_sec, 0.0)
    expected_packets = measured_seconds * 1000.0  # nominal 1 kHz
    rate_band = args.rate_tolerance_per_sec * measured_seconds
    min_packets = expected_packets - rate_band
    max_packets = expected_packets + rate_band

    print("[test_cadence] measured:")
    print("  drain_seconds       :", drain_sec)
    print("  drained_packets     :", drained)
    print("  packets (post-drain):", packets)
    print("  consecutive_intervals:", consec)
    print("  mcu_interval_mean_us:", mcu_mean)
    print("  mcu_interval_std_us :", mcu_std)
    print("  resync_count        :", resync)
    print("  expected packets    : %.0f in [%.0f, %.0f] (1000/sec +/- %.0f/sec "
          "over %.1fs measurement window)"
          % (expected_packets, min_packets, max_packets,
             args.rate_tolerance_per_sec, measured_seconds))

    fails = []
    if mcu_std is None:
        fails.append("no mcu_interval_std_us marker captured "
                     "(receiver produced no consecutive intervals?)")
    elif mcu_std > args.max_mcu_std_us:
        fails.append("mcu_interval_std_us %.2f > threshold %.2f"
                     % (mcu_std, args.max_mcu_std_us))

    if resync is None:
        fails.append("no resync_count marker captured")
    elif resync > args.max_resyncs:
        fails.append("resync_count %d > threshold %d"
                     % (int(resync), args.max_resyncs))

    if packets is None:
        fails.append("no packets marker captured")
    elif packets < min_packets or packets > max_packets:
        # Two-sided rate band (+/- rate-tolerance-per-sec / sec). Catches
        # both catastrophic packet loss (rate too low) and unexpected
        # surges (rate too high, which would imply duplicate packets or
        # a misconfigured PIT period).
        fails.append("packet count %d outside [%.0f, %.0f] "
                     "(post-drain rate not 1 kHz +/- %.0f/sec)"
                     % (int(packets), min_packets, max_packets,
                        args.rate_tolerance_per_sec))

    if fails:
        print()
        print("=" * 64)
        print(" FAIL")
        print("=" * 64)
        for f in fails:
            print(" -", f)
        sys.exit(1)

    print()
    print("=" * 64)
    print(" PASS")
    print("=" * 64)
    print(" mcu_interval_std_us=%.2f (<= %.2f)" % (mcu_std, args.max_mcu_std_us))
    print(" resync_count=%d (<= %d)" % (int(resync), args.max_resyncs))
    print(" packets=%d in [%d, %d]"
          % (int(packets), int(min_packets), int(max_packets)))
    if consec is not None:
        print(" consecutive_intervals=%d" % int(consec))
    sys.exit(0)


if __name__ == "__main__":
    main()
