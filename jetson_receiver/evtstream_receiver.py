#!/usr/bin/env python3
"""evtstream Jetson-side receiver.

Reads the 20-byte-header + 8-byte-event packets emitted by the OpenMV
RT1062 firmware over USB CDC, decodes them via numpy, and either:

  - default mode: streams events to a timestamped .npz file on Ctrl+C
                  with a summary printed at exit, or
  - --measure-cadence: prints periodic stats on MCU window timing,
                       Jetson arrival timing, resync events, and
                       sequence gaps. Greppable [CADENCE] markers in
                       the output are consumed by test_cadence.py.

Reference: DESIGN.md §2 (wire format) on the firmware fork.
"""
import argparse
import os
import signal
import struct
import sys
import time
from collections import deque

import numpy as np
import serial

from packet_format import (
    EVENT_DTYPE,
    EVENT_SIZE,
    EVT_FLAG_TRUNCATED,
    EVT_FLAG_USB_RETRY,
    EVT_PACKET_MAGIC,
    HEADER_FIELDS,
    HEADER_FMT,
    HEADER_SIZE,
    MAGIC_BYTES,
    unpack_header,
)


# ---------------------------------------------------------------------------
# Streaming reader
# ---------------------------------------------------------------------------

class StreamReader:
    """Buffered packet parser with magic-byte resync.

    Owns a `bytearray` working buffer, accumulates bytes from the serial
    port, and yields (header_dict, events_ndarray) per fully-received
    packet. Any byte-stream slip (USB drop, partial write, host-buffer
    truncation) triggers magic-byte resync forward to the next aligned
    EVT_PACKET_MAGIC. resync_count tracks how often that happens.
    """

    READ_CHUNK = 16 * 1024  # bytes per ser.read; matches MCU max packet
    MAX_EVENTS_PER_PACKET = 4096  # generous; firmware caps at 512 currently

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.resync_count = 0
        self.bytes_read = 0
        self.packets_read = 0

    def _resync(self):
        """Discard bytes up to the next magic. Called when the first
        4 bytes of self.buf don't match magic. Returns True on found,
        False if magic not in buf yet (caller should read more)."""
        # Search from offset 1 so we don't re-find the same failed magic
        # at offset 0.
        idx = self.buf.find(MAGIC_BYTES, 1)
        if idx == -1:
            # Not in buf. Keep last 3 bytes in case the magic is split
            # across this and the next read; discard the rest.
            del self.buf[:max(0, len(self.buf) - 3)]
            return False
        del self.buf[:idx]
        self.resync_count += 1
        return True

    def packets(self):
        """Generator yielding (header_dict, events_ndarray) per packet."""
        while True:
            try:
                chunk = self.ser.read(self.READ_CHUNK)
            except serial.SerialException as e:
                print("[receiver] serial error: %s" % e, file=sys.stderr)
                return
            if chunk:
                self.buf.extend(chunk)
                self.bytes_read += len(chunk)

            # Drain as many complete packets as the buffer holds.
            while True:
                if len(self.buf) < HEADER_SIZE:
                    break

                # Magic check.
                if bytes(self.buf[:4]) != MAGIC_BYTES:
                    if not self._resync():
                        break  # need more bytes to find magic
                    continue

                # Header looks aligned; decode it.
                hdr = unpack_header(bytes(self.buf[:HEADER_SIZE]))
                n_events = hdr["event_count"]
                if n_events > self.MAX_EVENTS_PER_PACKET:
                    # Implausibly large -- treat as corruption, resync.
                    del self.buf[:1]
                    self.resync_count += 1
                    continue

                payload_bytes = n_events * EVENT_SIZE
                total = HEADER_SIZE + payload_bytes
                if len(self.buf) < total:
                    break  # need more bytes for the payload

                if n_events == 0:
                    events = np.empty((0,), dtype=EVENT_DTYPE)
                else:
                    payload = bytes(self.buf[HEADER_SIZE:total])
                    events = np.frombuffer(payload, dtype=EVENT_DTYPE)

                del self.buf[:total]
                self.packets_read += 1
                yield hdr, events


# ---------------------------------------------------------------------------
# Cadence tracking
# ---------------------------------------------------------------------------

class CadenceTracker:
    """Online stats for MCU window timing + Jetson arrival timing.

    For each received packet we record:
      - hdr['window_start_us'] from the MCU
      - time.monotonic_ns() at the moment the packet was decoded

    Inter-arrival deltas are the per-window cadence we care about. The
    MCU one is the firmware's PIT-driven schedule (target: 1000 us
    +/- 50 us per the RISK1_FINDINGS §7a budget). The Jetson one
    folds in USB scheduling jitter on top.
    """

    def __init__(self):
        self.last_mcu_us = None
        self.last_jetson_ns = None
        self.last_sequence = None

        self._mcu_intervals = deque()       # microseconds
        self._jetson_intervals = deque()    # microseconds
        self.sequence_gaps = 0
        self.packets = 0
        self.events = 0
        self.flags_truncated = 0
        self.flags_usb_retry = 0

    def record(self, hdr, events, jetson_ns):
        self.packets += 1
        self.events += len(events)
        if hdr["flags"] & EVT_FLAG_TRUNCATED:
            self.flags_truncated += 1
        if hdr["flags"] & EVT_FLAG_USB_RETRY:
            self.flags_usb_retry += 1

        # Sequence-gap detection (modular over uint32).
        if self.last_sequence is not None:
            expected = (self.last_sequence + 1) & 0xFFFFFFFF
            if hdr["sequence"] != expected:
                # Number of missed packets between last and current.
                gap = (hdr["sequence"] - expected) & 0xFFFFFFFF
                # Cap reasonably; uint32 wrap would otherwise show as huge.
                if gap < 1_000_000:
                    self.sequence_gaps += gap
        self.last_sequence = hdr["sequence"]

        # MCU inter-arrival (uint32 microseconds, modular subtract).
        mcu = hdr["window_start_us"]
        if self.last_mcu_us is not None:
            delta = (mcu - self.last_mcu_us) & 0xFFFFFFFF
            self._mcu_intervals.append(delta)
        self.last_mcu_us = mcu

        # Jetson inter-arrival (nanoseconds -> microseconds).
        if self.last_jetson_ns is not None:
            self._jetson_intervals.append((jetson_ns - self.last_jetson_ns) / 1000.0)
        self.last_jetson_ns = jetson_ns

    @staticmethod
    def _stats(samples):
        if not samples:
            return None
        a = np.fromiter(samples, dtype=np.float64)
        return dict(
            n=len(a),
            mean=float(a.mean()),
            std=float(a.std()),
            min=float(a.min()),
            max=float(a.max()),
            p50=float(np.percentile(a, 50)),
            p99=float(np.percentile(a, 99)),
        )

    def summary(self):
        return dict(
            packets=self.packets,
            events=self.events,
            sequence_gaps=self.sequence_gaps,
            flags_truncated=self.flags_truncated,
            flags_usb_retry=self.flags_usb_retry,
            mcu=self._stats(self._mcu_intervals),
            jetson=self._stats(self._jetson_intervals),
        )


def print_cadence_summary(tracker, resync_count, label="periodic"):
    """Emit a cadence summary block.

    Lines starting with `[CADENCE] key=value` are the greppable markers
    consumed by test_cadence.py. Free-form text around them is for human
    operators.
    """
    s = tracker.summary()
    print()
    print("=" * 64)
    print(" Cadence summary (%s)" % label)
    print("=" * 64)
    print(" packets         : %d" % s["packets"])
    print(" events          : %d" % s["events"])
    print(" sequence_gaps   : %d" % s["sequence_gaps"])
    print(" flags_truncated : %d" % s["flags_truncated"])
    print(" flags_usb_retry : %d" % s["flags_usb_retry"])
    print(" resync_count    : %d" % resync_count)
    if s["mcu"]:
        m = s["mcu"]
        print(" MCU window inter-arrival (us):  "
              "n=%d mean=%.2f std=%.2f min=%.2f max=%.2f p50=%.2f p99=%.2f"
              % (m["n"], m["mean"], m["std"], m["min"], m["max"],
                 m["p50"], m["p99"]))
    if s["jetson"]:
        j = s["jetson"]
        print(" Jetson arrival   inter-arrival (us): "
              "n=%d mean=%.2f std=%.2f min=%.2f max=%.2f p50=%.2f p99=%.2f"
              % (j["n"], j["mean"], j["std"], j["min"], j["max"],
                 j["p50"], j["p99"]))

    # Greppable markers.
    print()
    print("[CADENCE] packets=%d" % s["packets"])
    print("[CADENCE] events=%d" % s["events"])
    print("[CADENCE] sequence_gaps=%d" % s["sequence_gaps"])
    print("[CADENCE] resync_count=%d" % resync_count)
    if s["mcu"]:
        print("[CADENCE] mcu_interval_mean_us=%.3f" % s["mcu"]["mean"])
        print("[CADENCE] mcu_interval_std_us=%.3f" % s["mcu"]["std"])
        print("[CADENCE] mcu_interval_p99_us=%.3f" % s["mcu"]["p99"])
    if s["jetson"]:
        print("[CADENCE] jetson_interval_mean_us=%.3f" % s["jetson"]["mean"])
        print("[CADENCE] jetson_interval_std_us=%.3f" % s["jetson"]["std"])
        print("[CADENCE] jetson_interval_p99_us=%.3f" % s["jetson"]["p99"])


# ---------------------------------------------------------------------------
# Save mode (default) -- accumulate events to .npz on Ctrl+C
# ---------------------------------------------------------------------------

def run_save_mode(reader, args):
    """Stream events into in-memory chunks; flush to .npz on exit.

    Each packet's events get its absolute MCU time appended as a column
    (window_start_us + t_us) so the saved array is self-describing on
    later analysis. Keeps everything in numpy from the start to avoid
    Python-loop overhead at 1 kHz packet rate.
    """
    chunks_events = []     # list of ndarray (per packet, copy of frombuffer)
    chunks_abs_us = []     # list of int64 ndarray, absolute MCU us
    total_events = 0
    total_packets = 0

    print("[receiver] saving events; press Ctrl+C to stop and write %s"
          % args.save)

    try:
        for hdr, events in reader.packets():
            total_packets += 1
            if len(events) > 0:
                # absolute_us = (uint32 window_start_us << 0) + t_us; use
                # int64 to keep monotonic across the uint32 wrap.
                abs_us = (np.int64(hdr["window_start_us"])
                          + events["t_us"].astype(np.int64))
                # Snapshot since np.frombuffer view aliases reader.buf
                # contents that get freed after del self.buf[:total].
                chunks_events.append(events.copy())
                chunks_abs_us.append(abs_us)
                total_events += len(events)
            if total_packets % 1000 == 0:
                print("[receiver] %d packets / %d events / %d resyncs"
                      % (total_packets, total_events, reader.resync_count))
    except KeyboardInterrupt:
        print("\n[receiver] Ctrl+C, finalising save...")

    if total_events == 0:
        print("[receiver] no events captured; nothing to save")
        return

    events_arr = np.concatenate(chunks_events)
    abs_us_arr = np.concatenate(chunks_abs_us)
    np.savez_compressed(
        args.save,
        events=events_arr,
        abs_us=abs_us_arr,
        bytes_read=np.int64(reader.bytes_read),
        packets=np.int64(total_packets),
        resync_count=np.int64(reader.resync_count),
    )
    print("[receiver] wrote %s: %d events, %d packets, %d resyncs"
          % (args.save, total_events, total_packets, reader.resync_count))


# ---------------------------------------------------------------------------
# Cadence mode
# ---------------------------------------------------------------------------

def run_cadence_mode(reader, args):
    """Track timing stats, print periodic summaries, end after duration_s."""
    tracker = CadenceTracker()
    start_ns = time.monotonic_ns()
    next_summary_ns = start_ns + int(args.summary_interval * 1e9)
    end_ns = (start_ns + int(args.duration * 1e9)
              if args.duration > 0 else None)

    print("[receiver] cadence mode, duration=%s, summary_interval=%.1fs"
          % ("inf" if args.duration <= 0 else "%.1fs" % args.duration,
             args.summary_interval))

    try:
        for hdr, events in reader.packets():
            now_ns = time.monotonic_ns()
            tracker.record(hdr, events, now_ns)
            if now_ns >= next_summary_ns:
                print_cadence_summary(tracker, reader.resync_count, "interval")
                next_summary_ns = now_ns + int(args.summary_interval * 1e9)
            if end_ns is not None and now_ns >= end_ns:
                break
    except KeyboardInterrupt:
        print("\n[receiver] Ctrl+C, final summary follows")

    print_cadence_summary(tracker, reader.resync_count, "final")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", default="/dev/ttyACM0",
                   help="serial device (default /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200,
                   help="baud rate (CDC ignores it but pyserial requires "
                        "a value; default 115200)")
    p.add_argument("--save", default="evtstream_capture.npz",
                   help="output .npz (default mode); 'none' disables")
    p.add_argument("--measure-cadence", action="store_true",
                   help="cadence-measurement mode -- print stats, "
                        "don't save events")
    p.add_argument("--duration", type=float, default=10.0,
                   help="cadence-mode duration in seconds; 0 = until "
                        "Ctrl+C (default 10.0)")
    p.add_argument("--summary-interval", type=float, default=2.0,
                   help="cadence-mode summary interval in seconds "
                        "(default 2.0)")
    args = p.parse_args()

    if not os.path.exists(args.device):
        # Permissions errors will surface on .open below; the existence
        # check is a friendlier early error.
        print("[receiver] %s not found; is the OpenMV camera plugged in?"
              % args.device, file=sys.stderr)
        sys.exit(2)

    ser = serial.Serial(
        args.device,
        baudrate=args.baud,
        # Short timeout so the reader loop can periodically check
        # KeyboardInterrupt + duration. 50 ms is short enough to feel
        # responsive without busy-spinning on empty reads.
        timeout=0.05,
        dsrdtr=False,
    )
    print("[receiver] opened %s @ %d baud" % (args.device, args.baud))

    reader = StreamReader(ser)
    try:
        if args.measure_cadence:
            run_cadence_mode(reader, args)
        else:
            run_save_mode(reader, args)
    finally:
        ser.close()


if __name__ == "__main__":
    # Disable the default SIGINT handler so KeyboardInterrupt propagates
    # cleanly through serial.Serial.read (which on some platforms will
    # otherwise catch and swallow signals).
    signal.signal(signal.SIGINT, signal.default_int_handler)
    main()
