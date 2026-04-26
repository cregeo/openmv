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
# Procedure:
#   1. Initialize the GenX320 in EVENT mode with ndarray_size = 4096 (default
#      IMAG_PARA height = 16 lines if we ever fell back to snapshot mode).
#   2. Wave a hand or LED in front of the lens to generate events.
#   3. For each height in HEIGHTS, run a continuous-DMA capture for
#      DURATION_MS of wall-clock time, accumulating per-FB-completion stats.
#   4. Compute aggregates AND per-FB distribution stats per capture.
#   5. Apply stricter criteria across heights (largest height = reference).
#   6. Print PASS / FAIL.
#
# Per-FB stats discriminate the two hypotheses from RISK1_FINDINGS.md §2/§3:
#   - Hypothesis A (CPI-burst-truncation): per-FB pixel-count distribution
#     should be **bimodal** (full FBs vs filler-only FBs) — std/mean > 0.5.
#   - Hypothesis B (snapshot lifecycle gap): in continuous-mode, the gap
#     disappears, so per-FB pixel-count distribution should be **unimodal**
#     and aggregate counts should match the geometric expectation.
#
# Per RISK1_FINDINGS.md §8c, PASS requires:
#   1. Aggregate pixel-event count within +/- 25% of geometric expectation.
#   2. Pixel/total ratio within +/- 15 percentage points of reference.
#   3. EV_TIME_HIGH count within +/- 50% of geometric expectation.
#   4. Per-FB pixel-count std/mean <= 0.5 (after excluding first 2 transient).
#   5. No invalid x/y coordinates.
#   (Monotonicity is checked inside the C decoder; not surfaced here.)

import csi
import time
from ulab import numpy as np


# Tunables ---------------------------------------------------------------

# Heights to sweep. Largest = reference for geometric scaling. The IOCTL
# caps height at 8 lines (DEBUG_STREAM_MAX_FB_BYTES = 8 KB). Useful sweep
# patterns:
#   [8, 4]      — quick: reference + design default
#   [8, 4, 2, 1]— full sweep, ~4 seconds total runtime
HEIGHTS = [8, 4]

DURATION_MS = 1000

# Per-FB stats array capacity. At 50K evt/s with h=4 (4 KB FB) the FB rate
# is ~70 FBs/s -> ~70 entries per second. Saccade rates push this higher.
# 2048 rows is comfortable.
MAX_FBS = 2048

EVT_NDARRAY_SIZE = 4096
DMA_LINE_BYTES = 1024  # set by genx320 driver in MODE_EVENT

WAIT_BEFORE_CAPTURE_S = 1.0


# Helpers ----------------------------------------------------------------

def my_std(arr):
    # ulab numpy's np.std availability is uncertain; do it ourselves to
    # avoid a missing-symbol failure on some firmware builds.
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


def report(h, s):
    print("\n--- h=%d (FB = %d bytes = %d EVT2.0 words) ---" %
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


def print_per_fb_rows(stats_view):
    # Compact per-FB log so the user can eyeball the distribution.
    n_fbs = stats_view.shape[0]
    show = min(n_fbs, 50)
    print("  per-FB log (first %d of %d):" % (show, n_fbs))
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

results = {}
for h in HEIGHTS:
    print("\nh=%d capture (%d ms)..." % (h, DURATION_MS))
    n_fbs = csi0.ioctl(csi.IOCTL_GENX320_DEBUG_CAPTURE_CONTINUOUS,
                        stats, h, DURATION_MS)
    print(" -> %d FBs captured" % n_fbs)
    if n_fbs > 0:
        # Snapshot the just-captured rows into a fresh ndarray so we can
        # reuse `stats` for the next capture. ulab slice + np.array() makes
        # a contiguous copy.
        snap = np.array(stats[:n_fbs])
        s = summarize(snap)
        results[h] = (snap, s)
        report(h, s)
        print_per_fb_rows(snap)
    else:
        results[h] = (None, None)
        print("  WARNING: zero FBs captured at h=%d — invalid run" % h)
    time.sleep(0.3)


# Stricter criteria ------------------------------------------------------

ref_h = max(HEIGHTS)
ref_snap, ref_stats = results[ref_h]

print()
print("=" * 70)
print(" Stricter PASS criteria (each height vs reference h=%d)" % ref_h)
print("=" * 70)

if ref_stats is None or ref_stats["n_fbs"] < 3:
    print(" REFERENCE FAILED — h=%d capture had < 3 FBs. Cannot evaluate." %
          ref_h)
    raise SystemExit

ref_words_per_fb = ref_h * (DMA_LINE_BYTES // 4)
ref_pix_pct = (100.0 * ref_stats["pixel_total"]
               / (ref_stats["n_fbs"] * ref_words_per_fb))

print(" reference h=%d  : %d FBs, %d pixels, %.1f%% pixel-of-stream" %
      (ref_h, ref_stats["n_fbs"], ref_stats["pixel_total"], ref_pix_pct))

overall_pass = True

for h in HEIGHTS:
    if h == ref_h:
        continue
    snap, s = results[h]
    if s is None:
        print("\n h=%d: FAIL — no FBs captured" % h)
        overall_pass = False
        continue

    print("\n h=%d:" % h)
    reasons = []

    # Per-FB scaling: if h captured proportionally fewer FBs of the same wall
    # time, bigger sample → expected aggregates scale by FBs * (h/ref_h).
    geo_ratio = h / float(ref_h)
    fb_ratio = s["n_fbs"] / float(ref_stats["n_fbs"])

    # (1) Pixel total within +/-25% of geometric expectation.
    expected_pix = ref_stats["pixel_total"] * fb_ratio * geo_ratio
    pix_lo = expected_pix * 0.75
    pix_hi = expected_pix * 1.25
    pix_ok = pix_lo <= s["pixel_total"] <= pix_hi
    print("   pixel total              : got %d, expected %.0f in [%.0f, %.0f]    %s" %
          (s["pixel_total"], expected_pix, pix_lo, pix_hi,
           "OK" if pix_ok else "FAIL"))
    if not pix_ok:
        reasons.append("pixel total out of range")

    # (2) Pixel/total percentage within +/-15 pp of reference.
    h_words_per_fb = h * (DMA_LINE_BYTES // 4)
    h_pix_pct = (100.0 * s["pixel_total"]
                 / (s["n_fbs"] * h_words_per_fb))
    pct_delta = abs(h_pix_pct - ref_pix_pct)
    pct_ok = pct_delta <= 15.0
    print("   pixel/total ratio (%%)    : got %.1f%%, ref %.1f%% (delta %.1f pp)    %s" %
          (h_pix_pct, ref_pix_pct, pct_delta,
           "OK" if pct_ok else "FAIL"))
    if not pct_ok:
        reasons.append("pixel/total ratio drift > 15 pp")

    # (3) EV_TIME_HIGH count within +/-50% of geometric expectation.
    expected_filler = ref_stats["filler_total"] * fb_ratio * geo_ratio
    f_lo = expected_filler * 0.5
    f_hi = expected_filler * 1.5
    f_ok = f_lo <= s["filler_total"] <= f_hi
    print("   EV_TIME_HIGH total       : got %d, expected %.0f in [%.0f, %.0f]    %s" %
          (s["filler_total"], expected_filler, f_lo, f_hi,
           "OK" if f_ok else "FAIL"))
    if not f_ok:
        reasons.append("filler count out of range")

    # (4) Per-FB unimodality (steady-state std/mean <= 0.5).
    if s["steady_mean"] > 0:
        sm = s["steady_std"] / s["steady_mean"]
        sm_ok = sm <= 0.5
        print("   per-FB std/mean          : %.3f (target <= 0.5)             %s" %
              (sm, "OK" if sm_ok else "FAIL"))
        if not sm_ok:
            reasons.append("per-FB distribution suggests bimodal "
                           "(Hypothesis A still in play)")
    else:
        print("   per-FB std/mean          : zero mean — too few FBs    SKIP")

    # (5) Invalid x/y is zero.
    xy_ok = s["invxy_total"] == 0
    print("   invalid x/y total        : %d                                       %s" %
          (s["invxy_total"], "OK" if xy_ok else "FAIL"))
    if not xy_ok:
        reasons.append("%d invalid x/y events" % s["invxy_total"])

    if reasons:
        print("   verdict: FAIL —")
        for r in reasons:
            print("     - %s" % r)
        overall_pass = False
    else:
        print("   verdict: PASS")


# Final verdict ----------------------------------------------------------

print()
print("=" * 70)
if overall_pass:
    print(" OVERALL PASS — continuous-DMA mode preserves event coherence at")
    print("                all swept heights. Original Option B is viable;")
    print("                proceed to Task 3 main implementation with")
    print("                'always continuous, never snapshot-per-call'.")
else:
    print(" OVERALL FAIL — continuous-DMA mode does NOT preserve coherence.")
    print("                Per RISK1_FINDINGS.md §8d, do NOT escalate to")
    print("                C-line speculatively. Next step is to read and")
    print("                document the line-callback path in")
    print("                LINE_CALLBACK_ANALYSIS.md before any further")
    print("                validation patches.")
print("=" * 70)
