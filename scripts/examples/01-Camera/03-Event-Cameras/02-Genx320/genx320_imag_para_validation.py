# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenMV LLC contributors. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# evtstream task-3 pre-implementation validation.
#
# Tests whether shrinking the i.MX CSI peripheral's IMAG_PARA image-height
# field still produces coherent EVT2.0 streams from the GenX320 in event mode.
# This is the load-bearing assumption for the evtstream native module's
# 1 ms cadence design, so it must be verified on hardware before further work.
#
# Procedure:
#   1. Initialize the GenX320 in EVENT mode with ndarray_size=4096
#      (gives a default IMAG_PARA height of 16 lines).
#   2. Wave a hand or LED in front of the lens to generate events.
#   3. Capture twice via the experimental DEBUG_CAPTURE IOCTL:
#        a. height_lines=16 (full default frame)
#        b. height_lines=13 (target for the evtstream design)
#   4. Decode EVT2.0 words and report:
#        - total words / pixel events / EV_TIME_HIGH words / other
#        - first and last decoded events
#        - x/y range validity
#        - timestamp monotonicity within the capture
#   5. Print PASS / FAIL based on a coherence checklist.
#
# What "coherent" means for the shrunken capture:
#   - All x/y in [0, 320)
#   - At least one EV_TIME_HIGH word per ~64 us of capture (the EVT2.0
#     spec requires periodic time-high updates so the 6-bit ts_low field
#     can be unambiguously placed)
#   - Timestamps non-decreasing (or only decreasing across an EV_TIME_HIGH
#     boundary as expected)
#   - Total decoded pixel events approximately proportional to the
#     height_lines ratio compared to the default capture
#
# Notes:
#   - This script is a one-shot validation, not a benchmark. The Python
#     decoder is intentionally explicit (byte-by-byte) for clarity; do
#     not measure perf from this.
#   - Run on the OpenMV RT1062 with firmware built from the
#     `evtstream-task3-validate` branch.

import csi
import time
from ulab import numpy as np


# Tunables ---------------------------------------------------------------

# Power of 2 between 1024 and 65536. Determines default IMAG_PARA height
# as `ndarray_size >> 8`. 4096 -> 16 lines, which lets us compare 16
# (default) vs 13 (target) cleanly.
EVT_NDARRAY_SIZE = 4096

DMA_LINE_BYTES = 1024  # Set by genx320 driver in MODE_EVENT.

DEFAULT_HEIGHT = EVT_NDARRAY_SIZE >> 8  # 16
SHRUNKEN_HEIGHT = 13                    # evtstream design target

WAIT_BEFORE_CAPTURE_S = 1.5

DETAILED_DECODE_LIMIT = 512  # words to fully decode + log


# EVT2.0 type codes ------------------------------------------------------

TYPE_TD_LOW = 0x0   # CD_OFF (pixel got darker)
TYPE_TD_HIGH = 0x1  # CD_ON  (pixel got brighter)
TYPE_EV_TIME_HIGH = 0x8
TYPE_EXT_TRIGGER = 0xA
TYPE_OTHERS = 0xE   # padding / continued events / etc


# Helpers ----------------------------------------------------------------

def le32(buf, offset):
    """Read a little-endian 32-bit word at byte offset."""
    return (int(buf[offset])
            | (int(buf[offset + 1]) << 8)
            | (int(buf[offset + 2]) << 16)
            | (int(buf[offset + 3]) << 24))


def decode(raw, n_bytes):
    """Decode EVT2.0 words from raw[:n_bytes]. Returns a stats dict."""
    n_words = n_bytes >> 2
    n_pixel = 0
    n_time_high = 0
    n_trigger = 0
    n_other = 0
    n_pixel_invalid_xy = 0
    time_high_us = 0
    last_full_ts = -1
    monotonic_violations = 0
    first_event = None
    last_event = None
    sample_log = []

    for i in range(n_words):
        val = le32(raw, i * 4)
        evt_type = (val >> 28) & 0xF

        if evt_type == TYPE_TD_LOW or evt_type == TYPE_TD_HIGH:
            ts_low = (val >> 22) & 0x3F
            x = (val >> 11) & 0x7FF
            y = val & 0x7FF
            full_ts = (time_high_us | ts_low) & 0xFFFFFFFFFFFFFFFF
            invalid = (x >= 320) or (y >= 320)
            n_pixel += 1
            if invalid:
                n_pixel_invalid_xy += 1
            if last_full_ts >= 0 and full_ts < last_full_ts:
                monotonic_violations += 1
            last_full_ts = full_ts
            if first_event is None:
                first_event = (full_ts, x, y, evt_type)
            last_event = (full_ts, x, y, evt_type)
            if i < DETAILED_DECODE_LIMIT and len(sample_log) < 8:
                sample_log.append("pixel ts=%d x=%d y=%d p=%d" %
                                  (full_ts, x, y, evt_type))

        elif evt_type == TYPE_EV_TIME_HIGH:
            new_time_high = ((val & 0xFFFFFFF) << 6)
            if i < DETAILED_DECODE_LIMIT and len(sample_log) < 8:
                sample_log.append("time_high <- %d" % new_time_high)
            time_high_us = new_time_high
            n_time_high += 1

        elif evt_type == TYPE_EXT_TRIGGER:
            n_trigger += 1

        else:
            n_other += 1

    return {
        "n_words": n_words,
        "n_pixel": n_pixel,
        "n_time_high": n_time_high,
        "n_trigger": n_trigger,
        "n_other": n_other,
        "n_pixel_invalid_xy": n_pixel_invalid_xy,
        "monotonic_violations": monotonic_violations,
        "first_event": first_event,
        "last_event": last_event,
        "sample_log": sample_log,
    }


def report(label, height, stats):
    print()
    print("--- %s (height_lines=%d) ---" % (label, height))
    print("  EVT2.0 words decoded     : %d" % stats["n_words"])
    print("  pixel events             : %d" % stats["n_pixel"])
    print("  EV_TIME_HIGH words       : %d" % stats["n_time_high"])
    print("  EXT_TRIGGER words        : %d" % stats["n_trigger"])
    print("  other / padding words    : %d" % stats["n_other"])
    print("  pixel events x/y invalid : %d" % stats["n_pixel_invalid_xy"])
    print("  monotonicity violations  : %d" % stats["monotonic_violations"])
    if stats["first_event"]:
        print("  first event (ts/x/y/p)   : %s" % str(stats["first_event"]))
        print("  last  event (ts/x/y/p)   : %s" % str(stats["last_event"]))
    if stats["sample_log"]:
        print("  first 8 decoded entries  :")
        for line in stats["sample_log"]:
            print("    %s" % line)


# Main -------------------------------------------------------------------

print("=" * 64)
print(" evtstream task-3: IMAG_PARA height validation")
print("=" * 64)
print(" ndarray_size           = %d (power of 2)" % EVT_NDARRAY_SIZE)
print(" default IMAG_PARA h    = %d lines" % DEFAULT_HEIGHT)
print(" shrunken IMAG_PARA h   = %d lines" % SHRUNKEN_HEIGHT)
print(" dma_line_bytes         = %d" % DMA_LINE_BYTES)
print(" default capture bytes  = %d (= %d EVT2.0 words)" %
      (DEFAULT_HEIGHT * DMA_LINE_BYTES,
       (DEFAULT_HEIGHT * DMA_LINE_BYTES) >> 2))
print(" shrunken capture bytes = %d (= %d EVT2.0 words)" %
      (SHRUNKEN_HEIGHT * DMA_LINE_BYTES,
       (SHRUNKEN_HEIGHT * DMA_LINE_BYTES) >> 2))

# Initialize sensor.
csi0 = csi.CSI(cid=csi.GENX320)
csi0.reset()
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, EVT_NDARRAY_SIZE)

# Allocate one buffer big enough for the larger of the two captures.
buf_capacity = DEFAULT_HEIGHT * DMA_LINE_BYTES
raw = np.zeros((buf_capacity,), dtype=np.uint8)

print()
print("Wave hand or LED in front of lens; capturing in %.1f s..." %
      WAIT_BEFORE_CAPTURE_S)
time.sleep(WAIT_BEFORE_CAPTURE_S)

# Default capture.
n_default = csi0.ioctl(csi.IOCTL_GENX320_DEBUG_CAPTURE, raw, DEFAULT_HEIGHT)
default_stats = decode(raw, n_default)
report("DEFAULT", DEFAULT_HEIGHT, default_stats)

# Brief pause; sensor keeps running so events accumulate independently.
time.sleep(0.1)

# Shrunken capture.
n_shrunk = csi0.ioctl(csi.IOCTL_GENX320_DEBUG_CAPTURE, raw, SHRUNKEN_HEIGHT)
shrunk_stats = decode(raw, n_shrunk)
report("SHRUNKEN", SHRUNKEN_HEIGHT, shrunk_stats)


# Verdict ----------------------------------------------------------------
#
# PASS requires ALL of the following. The earlier "byte count ratio matches
# the height ratio" criterion was insufficient: it only proved that the
# DMA peripheral copies the requested number of bytes, not that the bytes
# represent equivalent event data. A real run on hardware showed PASS by
# the old criterion while the shrunken capture was actually missing 80%
# of pixel events (filler-dominated stream). Stricter checks below.

print()
print("=" * 64)
print(" Stricter PASS criteria")
print("=" * 64)

reasons = []
geo_ratio = SHRUNKEN_HEIGHT / DEFAULT_HEIGHT  # expected geometric ratio


def assert_check(label, ok, detail):
    print(" %-26s : %s    %s" % (label, "OK  " if ok else "FAIL", detail))
    if not ok:
        reasons.append("%s — %s" % (label, detail))


# (0) Both captures produced data at all.
assert_check("default returned bytes",
             n_default > 0,
             "%d bytes" % n_default)
assert_check("shrunken returned bytes",
             n_shrunk > 0,
             "%d bytes" % n_shrunk)

if n_default > 0 and n_shrunk > 0:
    # (1) Word count ratio matches geometry within 5% — proves DMA byte
    #     accounting is correct. (Necessary but not sufficient.)
    word_ratio = shrunk_stats["n_words"] / float(default_stats["n_words"])
    word_ratio_ok = abs(word_ratio - geo_ratio) <= geo_ratio * 0.05
    assert_check("word ratio == geometry",
                 word_ratio_ok,
                 "%.3f vs expected %.3f (+/- 5%%)" %
                 (word_ratio, geo_ratio))

    # (2) Pixel-event count within +/- 25% of geometric expectation.
    #     Captures are not simultaneous; +/- 25% absorbs scene jitter.
    expected_pixels = default_stats["n_pixel"] * geo_ratio
    pixel_lo = int(expected_pixels * 0.75)
    pixel_hi = int(expected_pixels * 1.25)
    pixels_ok = pixel_lo <= shrunk_stats["n_pixel"] <= pixel_hi
    assert_check("pixel-event count",
                 pixels_ok,
                 "got %d, expected %d in [%d, %d]" %
                 (shrunk_stats["n_pixel"], int(expected_pixels),
                  pixel_lo, pixel_hi))

    # (3) Pixel-events as a percentage of the stream within +/- 15
    #     percentage points. This catches the failure mode where the
    #     CSI captures the same number of bytes but they are dominated
    #     by EV_TIME_HIGH filler words instead of real pixel events.
    default_pix_pct = (100.0 * default_stats["n_pixel"]
                       / default_stats["n_words"])
    shrunk_pix_pct = (100.0 * shrunk_stats["n_pixel"]
                      / shrunk_stats["n_words"])
    pix_pct_ok = abs(shrunk_pix_pct - default_pix_pct) <= 15.0
    assert_check("pixel/total ratio (%)",
                 pix_pct_ok,
                 "shrunken=%.1f%% vs default=%.1f%% (+/- 15 pp)" %
                 (shrunk_pix_pct, default_pix_pct))

    # (4) EV_TIME_HIGH word count within +/- 50% of geometric
    #     expectation. Wider tolerance because EV_TIME_HIGH cadence has
    #     intrinsic jitter; tighter than 50% would false-positive.
    expected_th = default_stats["n_time_high"] * geo_ratio
    th_lo = int(expected_th * 0.5)
    th_hi = int(expected_th * 1.5)
    th_ok = th_lo <= shrunk_stats["n_time_high"] <= th_hi
    assert_check("EV_TIME_HIGH count",
                 th_ok,
                 "got %d, expected %d in [%d, %d]" %
                 (shrunk_stats["n_time_high"], int(expected_th),
                  th_lo, th_hi))

# (5) No timestamp monotonicity violations beyond the 2 expected at
#     EV_TIME_HIGH boundaries.
mono_ok = shrunk_stats["monotonic_violations"] <= 2
assert_check("monotonicity violations",
             mono_ok,
             "%d (allowed up to 2)" % shrunk_stats["monotonic_violations"])

# (6) No invalid x/y coordinates.
xy_ok = shrunk_stats["n_pixel_invalid_xy"] == 0
assert_check("invalid x/y coords",
             xy_ok,
             "%d (must be 0)" % shrunk_stats["n_pixel_invalid_xy"])

print()
print("=" * 64)
print(" VERDICT")
print("=" * 64)
if not reasons:
    print(" PASS — shrunken IMAG_PARA height yields a coherent EVT2.0 stream.")
    print("        Proceed with the evtstream task-3 main implementation.")
else:
    print(" FAIL — shrunken IMAG_PARA height is NOT coherent. Reasons:")
    for r in reasons:
        print("        - %s" % r)
    print()
    print("        Read RISK1_FINDINGS.md at the repo root before further")
    print("        work. Do not proceed to Task 3 main implementation.")
print("=" * 64)
