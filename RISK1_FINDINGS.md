# RISK1_FINDINGS — IMAG_PARA validation result

Status: **FAIL** under the stricter criteria. Do not proceed to Task 3 main
implementation.

But: the failure mode is not exactly what the original DESIGN.md §9 Risk #1
predicted, and may not invalidate Option B as designed. Read on.

---

## 1. Observed data

Captured side-by-side on the same scene with `csi.IOCTL_GENX320_DEBUG_CAPTURE`.
`ndarray_size = 4096` → default `IMAG_PARA` height = 16 lines (5120 bytes /
1280 EVT2.0 words available); shrunken = 13 lines (4160 bytes / 1040 words).

| Metric                        | Default (h=16) | Shrunken (h=13) | Delta              |
|-------------------------------|----------------|------------------|--------------------|
| EVT2.0 words decoded          | 4096           | 3328             | -19% (= geometry)  |
| **Pixel events**              | **2938**       | **560**          | **-81%**           |
| EV_TIME_HIGH words            | 1158           | 2768            | **+139%**          |
| Pixel events / total words    | 71.7%          | 16.8%            | -55 pp             |
| Wall-clock span (last ts)     | 1.732 s        | 2.064 s          | comparable         |
| Invalid x/y                   | 0              | 0                | OK                 |
| Monotonicity violations       | 0              | 0                | OK                 |

The shrunken capture saw **5× fewer pixel events** over a *longer* wall-clock
span. The "missing" capacity is almost entirely filled with `EV_TIME_HIGH`
filler. Coordinates are valid and timestamps monotonic — so the bytes
themselves are well-formed; the *content* is just wrong.

Stricter criteria added to `genx320_imag_para_validation.py` flag this as
FAIL on the same data:

- pixel-event count `560` vs expected `2387` in `[1790, 2984]` → **FAIL**
- pixel/total `16.8%` vs default `71.7%` (Δ 54.9 pp > 15 pp) → **FAIL**
- EV_TIME_HIGH count `2768` vs expected `941` in `[470, 1411]` → **FAIL**

Word-count ratio, monotonicity, and x/y-validity all PASS — confirming the
DMA byte-accounting is correct.

---

## 2. Hypothesis A — CPI bursts truncated by small DMA windows

This is the hypothesis in the validation pushback (validation_pushback.md §3):
the GenX320's CPI emits events in bursts, the small DMA buffer captures only a
fraction of each burst, and post-burst quiet (dominated by EV_TIME_HIGH
filler) makes up the rest.

**Plausible**, but the data partially contradicts it. If bursts were being
truncated *inside* a buffer fill, we'd expect the captured *bytes* per
buffer to match expectation (DMA fills its quota), with *fewer* pixel events
and *more* filler — which we do see. But we'd also expect the pixel events
to land at the *start* of each captured buffer (the burst lands on the
buffer that's currently arming, then the rest is filler). I cannot verify
this from the current data because the script reports aggregated stats over
the whole capture, not per-FB.

If this hypothesis is right, larger buffers would help — at the cost of
latency.

---

## 3. Hypothesis B — snapshot lifecycle gap, not buffer size

The DEBUG_CAPTURE IOCTL uses `omv_csi_snapshot()` for each capture. That
path:

1. Aborts the CSI (`omv_csi_abort` is called by the IOCTL handler before
   `omv_csi_snapshot` to make the new IMAG_PARA take effect).
2. Reconfigures CSI registers, arms the DMA.
3. Waits for the EOF interrupt.
4. Returns. **Between `return` and the next snapshot call, the CSI is
   either disabled or re-arming.**

During the off-time, the GenX320's CPI keeps producing events on the wire
into nowhere. The sensor's internal CPI FIFO can hold only so much; once
full, it either stalls real events or the CPI block flushes filler
(EV_TIME_HIGH) once the wire is drained. When the CSI re-arms, the next
buffer is dominated by whatever the CPI was producing during the gap —
which may be filler, not real events.

This hypothesis predicts:

- Larger buffers don't help much: the gap between captures is still there,
  scaled by the same per-snapshot overhead.
- **Continuous ping-pong DMA — without ever disabling the CSI — should
  preserve event coherence even with small buffers**, because there is no
  off-time.

**This is significant for the design.** The DESIGN.md Option B as written
*does* run continuous DMA — it never reverts to the snapshot lifecycle
once started. The validation we just ran does not exercise that mode; it
exercises the snapshot-per-call mode. So the validation result tells us
about the snapshot path, but does not directly disprove Option B's
continuous-DMA path.

---

## 4. Why I think Hypothesis B is more likely right

Three indirect arguments:

1. **The wall-clock span ratio**. Default span 1.732 s, shrunken 2.064 s —
   shrunken is *longer* on the wire even though it captures less data. If
   bursts were merely truncated, we'd expect shrunken span < default span
   (smaller buffer → less wall-clock time captured). The fact that
   shrunken span is *longer* fits a model where the sensor is mostly
   idle/filler-emitting between captures.

2. **EV_TIME_HIGH cadence**. Default capture saw 1158 anchors over 1.732 s
   = ~1.5 ms apart. Shrunken capture saw 2768 anchors over 2.064 s =
   ~0.75 ms apart. The shrunken capture sampled the wire at a moment when
   anchors were 2× more frequent — consistent with a higher fraction of
   "wire-idle filler" time.

3. **Geometry intuition**. h=13 vs h=16 is not a dramatic shrink. It's
   ~80% of the buffer size. A factor-of-5 pixel-event drop from a
   factor-of-1.25 buffer-size drop is unphysical for a "buffer truncates
   the burst" model unless bursts are exactly tuned to the larger size,
   which they aren't.

I am not 100% sure. Hypothesis A could still be partially correct. But B
fits the data better and has a clean test: re-run the validation with
continuous DMA instead of snapshot-per-call, and see if event coherence
returns.

---

## 5. Implications for the design

- **Option B as written in DESIGN.md (continuous ping-pong DMA + PIT
  cadence) is not directly disproved by this validation**. The snapshot-
  based DEBUG_CAPTURE was a poor fixture for testing it.
- **We need a second validation** that runs the actual continuous-DMA
  configuration we'd use in production, before deciding to abandon
  Option B for B-modified, C, or A.
- **Worst case**: continuous DMA also fails. Then Hypothesis A is right
  (or partly right), and we genuinely need larger buffers or a different
  capture strategy. We re-evaluate at that point.

---

## 6. Three options (per validation_pushback.md §5)

### Option B-modified — larger DMA buffers, snapshot-per-window mode

Keep the snapshot lifecycle but inflate FB1/FB2 to e.g. 32 KB (~32 lines).
Each PIT window does one snapshot at default-or-larger geometry.

- **Pros**: similar architecture to original Option B; addresses
  Hypothesis A; preserves event coherence per the validation pattern.
- **Cons**: latency. At ~50K evt/s typical eye-tracking rates, 32 KB /
  280 KB/s ≈ 115 ms to fill at typical rates (longer at low rates). At
  bursty 500K evt/s, ~30 ms. **Both blow the 5 ms latency budget for
  the eye-tracker**.
- **Verdict**: **does not meet the use case**.

### Option C — continuous DMA, decode in CSI ISR

Configure CSI in continuous (non-`one_shot`) mode. The peripheral
ping-pongs FB1/FB2 with no off-time. Two flavors:

- **C-frame** (lighter): DMA-done IRQ fires per FB-completion. Decode
  the just-completed FB in the IRQ. Buffer size sets latency: 4 KB
  buffers at 50K evt/s = ~14 ms decode interval; at bursty 500K evt/s =
  ~1.4 ms. Tunable — pick a buffer size that fits the latency budget at
  the rates the use case actually sees.
- **C-line** (heavier): Use IMAG_PARA height = 1 so DMA-done fires per
  line (256 EVT2.0 words / line / ~12.5 µs at peak wire rate). Decode
  per line. Worst-case decode-to-availability ~12.5 µs. **Significantly
  more ISR overhead** at high rates but bounded sub-ms latency.
- **Pros**: no off-time → addresses Hypothesis B; matches DESIGN.md's
  intent; sub-5-ms latency achievable.
- **Cons**: more complex than C-frame; needs careful CSI register
  manipulation outside the existing snapshot path; line-callback for
  non-JPEG pixformat is unverified.
- **Verdict**: **most likely path to the use case being met**, but
  requires another small validation to confirm continuous mode
  preserves coherence.

### Option A revisited — accept sensor-driven cadence

Use `omv_csi_snapshot()` with default 320-line frames. Frame time at peak
event rate ~4 ms; at typical 50K evt/s ~25 s per frame (since the buffer
is huge relative to the rate). Pack events into Jetson-side packets
using event-embedded MCU timestamps; the wire arrival rate is
event-driven, not 1 kHz fixed.

- **Pros**: no firmware changes; uses well-tested snapshot path.
- **Cons**: latency at low event rate is unbounded (one snapshot can
  take seconds to complete). For eye-tracking at 50K evt/s, the tracker
  would have multi-second gaps between updates. **Useless for the use
  case**.
- **Verdict**: **does not meet the use case** at typical event rates.
  Could be a fallback if we configure a much smaller frame size, but at
  that point we're back to a flavor of Option B/C.

---

## 7. Recommendation

**Option C (continuous DMA)**, but with a cheap **follow-up validation
first** before committing to which flavor (C-frame vs C-line) and which
buffer size.

### Why C and not B-modified

The eye-tracking latency budget is ~5 ms (per validation_pushback.md §5,
"half the NIR re-seed interval" = half of 10 ms). Any snapshot-based
option scaled to fit eye-tracking rates blows this budget at low scene
activity. Continuous DMA decouples the buffer size from the latency
floor.

### Why a follow-up validation before locking the design

The current validation result is consistent with both Hypothesis A
(buffer size) and Hypothesis B (snapshot lifecycle). They imply
different designs. One small targeted test discriminates them and lets
us pick C-frame (sufficient if B), C-line (sufficient if A), or
recognise that we need a deeper rethink.

### Use-case fit — see §7a for the latency derivation that backs this

| Requirement              | Source         | C-frame fit | C-line fit |
|--------------------------|----------------|-------------|------------|
| ≤5 ms saccade latency    | NIR re-seed    | yes — 1.4 ms DMA fill + 1 ms PIT + ~1 ms USB | yes — sub-ms |
| Fixation latency unbounded | Use case     | yes         | yes        |
| 1 kHz packet cadence     | Tracker spec   | yes (PIT)   | yes (PIT)  |
| 50K evt/s nominal load   | Eye scenes     | yes         | yes        |
| 500K evt/s saccade rate  | Saccade physics| yes         | yes        |
| ISR CPU < 5%             | Headroom       | yes (~0.7%) | unmeasured — pending §8d |

**Default to C-frame with 4 KB buffers (h=4 lines)**. Latency budget for the
eye-tracking use case is met during saccades (when it matters) and explicitly
relaxed during fixation (when NIR ground truth carries the tracker). Do not
proceed to C-line speculatively — see §8c.

---

## 7a. Latency derivation

The earlier draft of §6/§7 conflated two distinct quantities. Separating
them properly:

### Components

The end-to-end latency from a sensor event being emitted to that event
landing in a Jetson packet is the sum of three terms:

| Term              | Formula                                              | Bound        |
|-------------------|------------------------------------------------------|--------------|
| **DMA fill latency** `L_fill` | (FB_size_bytes / wire_byte_rate)        | per-FB time  |
| **PIT drain latency** `L_pit` | (≤ window_us = 1000 µs)                 | one window   |
| **USB CDC latency** `L_usb`   | (host scheduling, ~1 ms typ., spike-prone) | ≈ 1–3 ms |
| **Total**         | `L_fill + L_pit + L_usb`                              | sum          |

`L_fill` is the dominant term. It is *not* the average decode interval
(events arrive in the ring as soon as their FB completes); it is the
worst-case wait for the very first event of a new FB to be processable.
At any given instant the in-flight FB has been filling for between 0 and
`FB_size_bytes / wire_byte_rate` of wall-clock time.

`wire_byte_rate` includes both pixel events and EV_TIME_HIGH filler. From
the validation data the wire was ~70/30 split pixel-vs-filler at
fixation activity. So at a given pixel-event rate, multiply by ~1.4× to
get wire word rate, then by 4 to get bytes.

### Saccade vs fixation

Eye-tracking event-stream latency only matters during **saccades**:

- **Fixation** (eye is still): pupil position changes are sub-pixel. NIR
  ground truth at 100 Hz handles position estimation. Event-stream
  latency does not affect tracker quality.
- **Saccade** (eye is moving): pupil moves several pixels per ms. NIR
  re-seeds at 100 Hz aren't enough; the event stream must keep up. This
  is when the 5 ms latency budget applies (half the NIR re-seed
  interval).

Saccade event rates are high (saccades produce ~50–200K pupil-edge
events/sec while moving; this dominates the wire). So our latency
analysis must be sized for **high event rate**, where DMA fills quickly
and `L_fill` is small.

### Worked numbers at h=4 lines (4 KB FB)

`dma_line_bytes = 1024` for GenX320 EVENT mode (sensor's CPI line width).
At h=4 lines the FB is 4 KB / 1024 EVT2.0 words / ~700 pixel-event capacity.

| Scenario        | Pixel rate | Wire byte rate | `L_fill` | Total latency |
|-----------------|-----------:|---------------:|---------:|--------------:|
| Fixation        |    50K/s   |    ~280 KB/s   |  ~14 ms  |  ~16 ms (OK)  |
| Onset transition|    rising  |    rising      |  ≤1.4 ms |  ≤3.4 ms (OK) |
| Saccade peak    |   500K/s   |    ~2.8 MB/s   |  ~1.4 ms |   ~3.4 ms (OK) |
| Blink burst     |   1M/s+    |    ~5.6 MB/s+  |  ~0.7 ms |   ~2.7 ms (OK) |

The fixation row's 16 ms exceeds the budget; that's accepted per the
saccade-vs-fixation framing. Every other row is comfortably under 5 ms.

### Why not 1 KB or 2 KB?

The pushback raised whether smaller buffers (1 KB / 2 KB) would do
better. Working through it:

| Size  | Saccade `L_fill` | Onset `L_fill` (worst) | Saccade IRQ rate | IRQ CPU at saccade |
|-------|------------------:|-----------------------:|-----------------:|-------------------:|
| 1 KB  |  0.36 ms          |  0.36 ms               |  ~2,734 / s      |  ~0.8%             |
| 2 KB  |  0.72 ms          |  0.72 ms               |  ~1,367 / s      |  ~0.7%             |
| 4 KB  |  1.4 ms           |  1.4 ms                |  ~683 / s        |  ~0.7%             |

All three meet the budget cleanly. Smaller is faster but with diminishing
returns, and the IRQ overhead is negligible at all three sizes. No
operational reason picks 1 or 2 over 4.

**Default: h=4 (4 KB FB).** Recommended for the design.

But: the validation patch in §8a exposes `height_lines` as a Python
parameter so the next hardware run can sweep h ∈ {1, 2, 4, 8} cheaply.
If the data surfaces a reason to prefer a different size, we revise
before locking the design.

### Caveats baked into this derivation

1. **Ground truth wire rate is from the failed snapshot validation.** The
   70/30 pixel/filler split is from a specific scene (waving hand). At
   true saccade rates the ratio likely shifts toward 90+% pixel as the
   sensor saturates. Re-derive after the §8 follow-up validation
   produces a clean continuous-mode wire rate.
2. **`L_usb` of ~1 ms is unverified.** The original DESIGN.md §9 Risk #3
   flagged this; the synthetic CDC benchmark in task3 instructions §6 is
   the artifact that confirms or refutes it. Don't trust the 1 ms
   estimate yet — it just isn't the bottleneck if it's anywhere near
   correct.
3. **First-FB transient.** When CSI is first armed in continuous mode,
   FB1 fills from a randomly-aligned moment in the wire stream. The
   first FB-done IRQ may be unrepresentative. The validation patch in
   §8a captures per-FB stats so we can see the transient and exclude it
   from steady-state analysis.

---

## 8. What we need from cregeo

A second validation patch + run, on the same hardware, before any main
module work.

**Lesson from the first validation, baked into this one**: the original
`DEBUG_CAPTURE` IOCTL exercised the snapshot-per-call lifecycle, which
is *not* the configuration the production design will use. The follow-up
validation must exercise the *actual* production configuration —
continuous ping-pong DMA — or its result is moot.

### 8a. Patch — continuous-mode validation IOCTL

Add a sibling IOCTL `OMV_CSI_IOCTL_GENX320_DEBUG_CAPTURE_CONTINUOUS`
(0x28). Keep the original `DEBUG_CAPTURE` (0x27) intact so we can run
side-by-side comparisons if needed during analysis. Implementation
sketch:

1. Configure the CSI peripheral in **non-`one_shot` mode** with
   `IMAG_PARA` height set per the §7a recommendation (default h=4
   lines, but exposed as a Python argument so we can sweep).
2. Allocate two FB buffers (sized `height_lines × dma_line_bytes`)
   and point `CSI_REG_DMASA_FB1` / `_FB2` at them. CSI ping-pongs
   FB1↔FB2 with no off-time between them.
3. Install a CSI-IRQ hook (in `ports/mimxrt/omv_csi.c`) that bypasses
   the existing `omv_csi_line_callback` line-copy path and instead
   calls our event-decode-from-FB function on every FB-done IRQ.
4. Run for a caller-specified `duration_ms` of wall-clock time.
5. Per-FB, accumulate stats (pixel count, EV_TIME_HIGH count,
   trigger count, invalid-xy count, FB-completion timestamp) into a
   caller-provided ndarray.
6. After the window expires, stop CSI cleanly and return the per-FB
   stats array.

Python interface (final shape):

```python
# Per-FB stats: (n_fbs, 8) uint16 array, one row per FB-done IRQ.
# Columns: ts_us_lo, ts_us_hi, pixels, fillers, triggers, others, invalid_xy, reserved
stats = np.zeros((MAX_FBS, 8), dtype=np.uint16)
n_fbs = csi0.ioctl(csi.IOCTL_GENX320_DEBUG_CAPTURE_CONTINUOUS,
                   stats, height_lines, duration_ms)
```

### 8b. Run on hardware

Same procedure as the first validation. Suggested sweep:

1. Run with `height_lines=4, duration_ms=1000` (the design default).
2. Run with `height_lines=2` and `=1` to sanity-check that the answer
   doesn't depend on the specific size.
3. Optional: run with `height_lines=16` to compare against the original
   snapshot-mode default-height capture.

The script will print aggregate stats AND per-FB distribution analysis
so the same log can be inspected for Hypothesis A vs B signature.

### 8c. Decision criteria — strict

Pass criteria for **continuous-mode capture at any tested
`height_lines`**:

1. Aggregate pixel-event count is within ±25 % of the geometric
   expectation calibrated against a reference-height continuous-mode
   capture in the same run.
2. Pixel/total ratio across the run is within ±15 percentage points of
   the reference-height capture in the same run.
3. EV_TIME_HIGH count is within ±50 % of the geometric expectation.
4. Per-FB pixel count distribution is **unimodal** with std dev ≤ 50%
   of the mean (after excluding the first 2 FBs as transient). Bimodal
   distribution would be the Hypothesis-A signature; if present we
   stop and revisit.
5. No timestamp monotonicity violations beyond the EV_TIME_HIGH
   boundary count (≤ 2× the EV_TIME_HIGH count).
6. No invalid x/y coordinates.

If the script reports PASS at h=4, h=2, AND h=1: continuous-mode
ping-pong DMA preserves event coherence at all relevant sizes →
**Option C-frame is viable** → original Option B design holds with the
clarification "always continuous, never snapshot-per-call". Proceed to
Task 3 main implementation with h=4 as the default size.

If the script reports FAIL at any tested size: **stop**. Do not
escalate to C-line speculatively.

### 8d. If C-frame fails — what to do *before* writing C-line

C-line as sketched in §6 introduces a new unverified assumption: that
the existing `imx_csi_line_callback` path can be repurposed for raw
EVT2.0 word capture in non-JPEG pixformat. The callback was written
for JPEG and grayscale-pixel-line copying; whether it can carry raw
EVT2.0 words without modification is a research question, not an
implementation step.

If §8c produces a FAIL, the next deliverable is **NOT** another
validation patch. It is a written analysis (`LINE_CALLBACK_ANALYSIS.md`)
that:

1. Reads `omv_csi_line_callback()` end-to-end and documents what each
   branch does.
2. Walks through what changes (if any) are needed for non-JPEG,
   non-grayscale-pixel raw-word capture.
3. Identifies whether `IMAG_PARA height = 1` actually gives per-line
   FB-done IRQs in this CSI peripheral configuration, or whether the
   semantics are different.
4. Flags any state assumptions that the existing line-callback makes
   that would be violated in evtstream's continuous mode (e.g.,
   `framebuffer_acquire`/`framebuffer_release` ordering, EDMA channel
   sharing, `vbuffer_t` ring lifecycle).

Only after that document is reviewed and we agree C-line is feasible
do I write the C-line validation patch.

### 8e. If C-line analysis says it isn't feasible

Then the options are:

- **B-modified with relaxed latency budget**: accept that latency floor
  is the FB fill time at typical (not saccade) event rate. With 4 KB
  FBs at fixation 50K evt/s = ~14 ms latency. Combined with the
  saccade-vs-fixation framing, this might be acceptable: at fixation
  the tracker doesn't need fast events anyway. Worth re-evaluating
  against the eye-tracker's actual saccade detection behavior.
- **Accept `Option A revisited` with smaller frames**: configure the
  GenX320 sensor's CPI packet size to a smaller value (currently
  hard-coded at 320 × 320 in `set_active_mode`). This shrinks the
  *sensor*-side frame, not the CSI's IMAG_PARA. May or may not be
  supported by the sensor — Prophesee public registers should
  document the lower bound; we'd need to read them.
- **Reframe the use case**: drop the 1 kHz cadence target and run at
  whatever cadence the sensor naturally produces, with MCU-side
  timestamps so the Jetson can resequence.

We pick from these in a third document if we get there.

---

## 9. Things I still don't know

- **Exact source of the EV_TIME_HIGH surplus**. My Hypothesis B is the
  most parsimonious fit but is not proven. A second test could record
  per-FB stats during continuous DMA (events vs filler in each
  ping-pong cycle) to see if filler is concentrated at the start of
  each FB (consistent with "wire was idle right before this FB started
  capturing").
- **Whether `imx_csi_line_callback` works for non-JPEG pixformat without
  modifications**. The existing path runs it for non-JPEG, but it
  copies pixel lines, not raw EVT2.0 words. A small intercept hook may
  be needed for C-line; need to read more of the line-callback path.
- **Sensor-internal CPI FIFO depth**. Don't have it from the
  Prophesee-public registers in `drivers/genx320/include/`. If the
  FIFO is small (e.g. ~256 words), Hypothesis B is more plausible
  because brief CSI off-times overflow it quickly.

---

## 10. Process

- Validation script tightened (committed as part of this patch).
- This document committed alongside.
- Both pushed to `evtstream-task3-validate` for review.
- **Stopped. Awaiting decision** on whether to write the continuous-DMA
  follow-up validation patch (recommended) or pivot to a different
  option directly.
