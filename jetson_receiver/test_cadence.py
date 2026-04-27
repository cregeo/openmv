#!/usr/bin/env python3
"""Task 5: PASS/FAIL cadence gate for the evtstream pipeline.

Wraps evtstream_receiver.py in --measure-cadence --duration N mode,
greps the [CADENCE] markers it prints, and asserts:

  1. MCU window inter-arrival std <= 50 us  (the original cadence
     target from the task brief and DESIGN.md / RISK1_FINDINGS §7a).
  2. resync_count == 0 over the test (no magic-byte slips ->
     wire-level integrity is intact).

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

    print("[test_cadence] measured:")
    print("  packets             :", packets)
    print("  mcu_interval_mean_us:", mcu_mean)
    print("  mcu_interval_std_us :", mcu_std)
    print("  resync_count        :", resync)

    fails = []
    if mcu_std is None:
        fails.append("no mcu_interval_std_us marker captured "
                     "(receiver produced no packets?)")
    elif mcu_std > args.max_mcu_std_us:
        fails.append("mcu_interval_std_us %.2f > threshold %.2f"
                     % (mcu_std, args.max_mcu_std_us))

    if resync is None:
        fails.append("no resync_count marker captured")
    elif resync > args.max_resyncs:
        fails.append("resync_count %d > threshold %d"
                     % (int(resync), args.max_resyncs))

    if packets is None or packets < 0.9 * args.duration * 1000:
        # Sanity: at 1 kHz cadence we should see ~duration*1000 packets.
        # 90% floor catches catastrophic packet loss before threshold
        # checks declare PASS on near-zero data.
        fails.append("packet count %s well below expected ~%d"
                     % (packets, int(args.duration * 1000)))

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
    print(" packets=%d" % int(packets))
    sys.exit(0)


if __name__ == "__main__":
    main()
