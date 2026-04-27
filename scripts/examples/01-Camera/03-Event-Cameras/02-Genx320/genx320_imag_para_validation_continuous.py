# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenMV LLC contributors. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# evtstream task-3 RISK1 follow-up validation: continuous-DMA mode.
#
# LESSON FROM THE FIRST VALIDATION ROUND (genx320_imag_para_validation.py):
#   The validation patch must exercise the actual configuration the design
#   will use in production. The first round used `omv_csi_snapshot()` (one
#   capture per call, CSI-off in between) to validate a design that intends
#   to run CSI continuously. Result: a "PASS" verdict that fell apart under
#   stricter criteria, and a confused multi-hypothesis post-mortem. This
#   round uses the IOCTL_GENX320_DEBUG_CAPTURE_CONTINUOUS IOCTL, which
#   configures CSI in non-`one_shot` ping-pong mode and never disables it
#   during a capture window. That is the same configuration the evtstream
#   module will use in production.
#
# LESSON FROM THE FIRST CONTINUOUS-MODE RUN (this script, original criteria):
#   Strict aggregate ratio criteria — pixel-total range, pixel/total ratio
#   drift, filler-total range — assumed wire content scales linearly with
#   buffer size across heights. It does not. The pixel/filler ratio is
#   scene-dependent, and the GenX320's CPI block paces filler differently at
#   different buffer sizes (h=8 saw 99% pixel/wire; h=4 saw 70-98% depending
#   on activity). The ratio criteria were FAIL-ing on perfectly coherent
#   data. The criterion that actually discriminates Hypothesis A
#   (CPI-burst-truncation) from Hypothesis B (snapshot lifecycle gap) is
#   per-FB pixel-count distribution unimodality. Keep that one strict;
#   loosen the rest to floor checks.
#
# LESSON FROM IDE OUTPUT TRUNCATION:
#   The first run's PASS verdict never reached the console because the
#   per-FB log filled the buffer first. Print verdict BEFORE detailed
#   stats so future runs are easy to grep for "OVERALL:".
#
# Procedure:
#   1. Initialize the GenX320 in EVENT mode with ndarray_size = 4096.
#   2. Wave a hand or LED in front of the lens to generate events.
#   3. For each height in HEIGHTS, run a continuous-DMA capture for
#      DURATION_MS of wall-clock time, accumulating per-FB-completion stats.
#   4. Compute aggregates AND per-FB distribution stats per capture.
#   5. Print VERDICT BLOCK first (per-height + OVERALL).
#   6. Print detailed stats and per-FB log after the verdict.
#
# PASS criteria (loosened per validation feedback):
#   1. Per-FB pixel-count std/mean <= 0.5 after excluding first 2 transient
#      FBs. PRIMARY discriminator. If bimodal distribution shows up,
#      Hypothesis A is back in play and we stop.
#   2. Aggregate pixel-event count >= 20% of the linear-scaling
#      expectation (loose floor — catches "captured almost nothing" while
#      tolerating scene-dependent variation).
#   3. Steady-state per-FB pixel count >= 1 (sensor is producing events).
#   4. No invalid x/y coordinates.
#   (Monotonicity is checked inside the C decoder; not surfaced here.)

import csi
import time
from ulab import numpy as np


# Tunables ---------------------------------------------------------------

# Heights to sweep. Largest = reference for geometric scaling. The IOCTL
# caps height at 8 lines (DEBUG_STREAM_MAX_FB_BYTES = 8 KB). Useful sweep
# patterns:
#   [8, 6]      — confirm the production candidate (h=6) against reference
#   [8, 6, 4, 2, 1] — full sweep
HEIGHTS = [8, 6]

DURATION_MS = 1000

# Per-FB stats array capacity. At burst rates (saccade) FBs come ~700/s for
# h=4 / 350/s for h=8. 2048 is comfortable headroom.
MAX_FBS = 2048

EVT_NDARRAY_SIZE = 4096
DMA_LINE_BYTES = 1024  # set by genx320 driver in MODE_EVENT

WAIT_BEFORE_CAPTURE_S = 1.0


# Helpers ----------------------------------------------------------------

def my_std(arr):
    # ulab numpy's np.std availability is uncertain; do it ourselves.
    n = arr.shape[0]
    if n == 0:
        return 0.0
    m = float(np.sum(arr)) / float(n)
    diffs_sq = (arr - m) ** 2
    return (float(np.sum(diffs_sq)) / float(n)) ** 0.5


def summarize(stats_view):
    n_fbs = stats_view.shape[0]
    if n_fbs == 0:
        return None

    pixel_col = stats_view[:, 2]
    filler_col = stats_view[:, 3]
    trigger_col = stats_view[:, 4]
    other_col = stats_view[:, 5]
    invxy_col = stats_view[:, 6]

    pixel_total = int(np.sum(pixel_col))
    filler_total = int(np.sum(filler_col))
    trigger_total = int(np.sum(trigger_col))
    other_total = int(np.sum(other_col))
    invxy_total = int(np.sum(invxy_col))

    transient_pixels = int(np.sum(pixel_col[:2])) if n_fbs >= 2 else 0
    if n_fbs > 2:
        steady = pixel_col[2:]
        steady_n = steady.shape[0]
        steady_mean = float(np.sum(steady)) / float(steady_n)
        steady_std = my_std(steady)
        steady_min = int(np.min(steady))
        steady_max = int(np.max(steady))
    else:
        steady_n = 0
        steady_mean = 0.0
        steady_std = 0.0
        steady_min = 0
        steady_max = 0

    return {
        "n_fbs": n_fbs,
        "pixel_total": pixel_total,
        "filler_total": filler_total,
        "trigger_total": trigger_total,
        "other_total": other_total,
        "invxy_total": invxy_total,
        "transient_pixels": transient_pixels,
        "steady_n": steady_n,
        "steady_mean": steady_mean,
        "steady_std": steady_std,
        "steady_min": steady_min,
        "steady_max": steady_max,
    }


def evaluate(h, s, ref_h, ref_s):
    """Apply loosened PASS criteria. Returns (ok, reasons, key_metrics)."""
    if s is None or s["n_fbs"] < 3:
        return False, ["less than 3 FBs captured"], {}

    reasons = []
    metrics = {}

    # (1) Primary: per-FB pixel-count std/mean <= 0.5 (unimodal).
    if s["steady_mean"] > 0:
        std_over_mean = s["steady_std"] / s["steady_mean"]
        metrics["std_over_mean"] = std_over_mean
        if std_over_mean > 0.5:
            reasons.append(
                "per-FB std/mean %.3f > 0.5 (suspect bimodal — Hypothesis A)"
                % std_over_mean)
    else:
        metrics["std_over_mean"] = 0.0
        reasons.append("steady-state mean is 0 (sensor produced no events)")

    # (2) Loose floor: aggregate pixel total >= 20% of geometric expectation.
    geo_ratio = h / float(ref_h)
    fb_ratio = s["n_fbs"] / float(ref_s["n_fbs"]) if ref_s["n_fbs"] else 1.0
    expected = ref_s["pixel_total"] * fb_ratio * geo_ratio
    floor = expected * 0.20
    metrics["pixel_total"] = s["pixel_total"]
    metrics["pixel_floor"] = floor
    if h != ref_h and s["pixel_total"] < floor:
        reasons.append(
            "pixel total %d < floor %.0f (20%% of geometric expectation %.0f)"
            % (s["pixel_total"], floor, expected))

    # (3) Sensor-is-on: steady-state per-FB pixels >= 1.
    metrics["steady_min"] = s["steady_min"]
    if s["steady_min"] < 1:
        reasons.append(
            "steady-state min pixels/FB = %d (sensor not producing)"
            % s["steady_min"])

    # (4) No invalid x/y.
    metrics["invxy_total"] = s["invxy_total"]
    if s["invxy_total"] != 0:
        reasons.append(
            "%d invalid x/y events" % s["invxy_total"])

    return (len(reasons) == 0), reasons, metrics


def report_detailed(h, s):
    print()
    print("--- h=%d (FB = %d bytes = %d EVT2.0 words) ---" %
          (h, h * DMA_LINE_BYTES, (h * DMA_LINE_BYTES) >> 2))
    print("  total FBs                : %d" % s["n_fbs"])
    print("  total pixel events       : %d" % s["pixel_total"])
    print("  total EV_TIME_HIGH       : %d" % s["filler_total"])
    print("  total triggers           : %d" % s["trigger_total"])
    print("  total others             : %d" % s["other_total"])
    print("  total invalid x/y        : %d" % s["invxy_total"])
    print("  steady-state pixels/FB   : mean=%.1f std=%.1f min=%d max=%d (n=%d)" %
          (s["steady_mean"], s["steady_std"], s["steady_min"], s["steady_max"],
           s["steady_n"]))
    if s["steady_mean"] > 0:
        ratio = s["steady_std"] / s["steady_mean"]
        verdict = "unimodal" if ratio <= 0.5 else "BIMODAL/wide"
        print("  std/mean (target <=0.5)  : %.3f  [%s]" % (ratio, verdict))
    print("  first-2 FBs pixel total  : %d (transient — excluded from steady)"
          % s["transient_pixels"])


def print_per_fb_rows(label, stats_view):
    n_fbs = stats_view.shape[0]
    show = min(n_fbs, 50)
    print()
    print("  per-FB log for %s (first %d of %d):" % (label, show, n_fbs))
    print("    fb_idx |     ts_us |  pixel | filler | trig | other | inv_xy")
    for i in range(show):
        ts_us = (int(stats_view[i, 0])
                 | (int(stats_view[i, 1]) << 16))
        print("    %6d | %9d | %6d | %6d | %4d | %5d | %6d" %
              (int(stats_view[i, 7]), ts_us,
               int(stats_view[i, 2]), int(stats_view[i, 3]),
               int(stats_view[i, 4]), int(stats_view[i, 5]),
               int(stats_view[i, 6])))


# Main -------------------------------------------------------------------

print("=" * 70)
print(" evtstream task-3 RISK1 follow-up: CONTINUOUS-DMA validation")
print("=" * 70)
print(" heights swept    : %s" % str(HEIGHTS))
print(" duration each    : %d ms" % DURATION_MS)
print(" stats capacity   : %d rows" % MAX_FBS)
print(" reference height : %d (largest in sweep)" % max(HEIGHTS))

csi0 = csi.CSI(cid=csi.GENX320)
csi0.reset()
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, EVT_NDARRAY_SIZE)

stats = np.zeros((MAX_FBS, 8), dtype=np.uint16)

print()
print("Wave hand or LED in front of lens; capturing in %.1f s..." %
      WAIT_BEFORE_CAPTURE_S)
time.sleep(WAIT_BEFORE_CAPTURE_S)

# Phase 1: capture all heights, compute summaries. Minimal output here so
# the verdict block below isn't pushed off-screen by per-FB logs.
results = {}
for h in HEIGHTS:
    n_fbs = csi0.ioctl(csi.IOCTL_GENX320_DEBUG_CAPTURE_CONTINUOUS,
                       stats, h, DURATION_MS)
    if n_fbs > 0:
        snap = np.array(stats[:n_fbs])
        s = summarize(snap)
        results[h] = (snap, s)
    else:
        results[h] = (None, None)
    print("  captured h=%d -> %d FBs" % (h, n_fbs))
    time.sleep(0.3)


# Phase 2: VERDICT BLOCK (printed first so it never gets truncated).
print()
print("=== VERDICT START ===")

ref_h = max(HEIGHTS)
_, ref_s = results[ref_h]

if ref_s is None or ref_s["n_fbs"] < 3:
    print(" REFERENCE FAIL — h=%d had < 3 FBs; cannot evaluate." % ref_h)
    print("OVERALL: FAIL")
    print("=== VERDICT END ===")
    raise SystemExit

overall_pass = True
verdicts = {}
for h in HEIGHTS:
    snap, s = results[h]
    ok, reasons, metrics = evaluate(h, s, ref_h, ref_s)
    verdicts[h] = (ok, reasons, metrics)
    if not ok:
        overall_pass = False

    # One concise verdict line per height.
    if s is None:
        print(" h=%d: FAIL (no FBs captured)" % h)
        continue
    sm = metrics.get("std_over_mean", 0.0)
    pix = metrics.get("pixel_total", 0)
    floor = metrics.get("pixel_floor", 0.0)
    print(" h=%d: %s — std/mean=%.3f, pixel total=%d (floor %.0f), "
          "invxy=%d, n_fbs=%d"
          % (h, "PASS" if ok else "FAIL",
             sm, pix, floor, metrics.get("invxy_total", 0), s["n_fbs"]))
    for r in reasons:
        print("    - %s" % r)

print("OVERALL: %s" % ("PASS" if overall_pass else "FAIL"))
print("=== VERDICT END ===")


# Phase 3: detailed per-height reports.
print()
print("=" * 70)
print(" Detailed stats")
print("=" * 70)
for h in HEIGHTS:
    snap, s = results[h]
    if s is not None:
        report_detailed(h, s)
    else:
        print()
        print("--- h=%d: no FBs captured ---" % h)


# Phase 4: per-FB log (last; safe to truncate without losing the verdict).
print()
print("=" * 70)
print(" Per-FB log (truncatable; verdict is above)")
print("=" * 70)
for h in HEIGHTS:
    snap, s = results[h]
    if snap is not None:
        print_per_fb_rows("h=%d" % h, snap)


# Phase 5: trailing reminder — overall verdict, repeated.
print()
print("=" * 70)
print(" OVERALL: %s" % ("PASS" if overall_pass else "FAIL"))
print("=" * 70)
if overall_pass:
    print(" Continuous-DMA mode preserves event coherence at all swept heights.")
    print(" Hypothesis B is confirmed. Original Option B holds; proceed to")
    print(" Task 3 main implementation per DESIGN.md (with §3a / §7a updates).")
else:
    print(" Continuous-DMA mode does NOT meet PASS criteria.")
    print(" Per RISK1_FINDINGS.md §8d, do NOT escalate to C-line speculatively.")
    print(" Next deliverable is LINE_CALLBACK_ANALYSIS.md, not another patch.")
print("=" * 70)
