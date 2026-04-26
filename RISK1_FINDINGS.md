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

### Use-case fit

| Requirement              | Source         | C-frame fit | C-line fit |
|--------------------------|----------------|-------------|------------|
| 5 ms latency budget      | NIR re-seed    | yes (≤4 ms at typical rate) | yes (≤1 ms) |
| 1 kHz packet cadence     | Tracker spec   | yes (PIT)   | yes (PIT)  |
| 50K evt/s nominal load   | Eye scenes     | yes         | yes        |
| 500K evt/s burst (blink) | Blink physics  | yes (≤1.4 ms decode IRQ) | yes (per-line) |
| ISR CPU < 20%            | Headroom       | yes (~5%)   | borderline (~16%) |

**Default to C-frame with 4 KB buffers**. Escalate to C-line only if
follow-up validation shows C-frame still loses events.

---

## 8. What we need from cregeo

A second validation patch + run, on the same hardware, before any main
module work:

### 8a. Patch (I will write it; small)

Extend `OMV_CSI_IOCTL_GENX320_DEBUG_CAPTURE` (or add a sibling IOCTL)
with a "continuous mode" flavor that:

1. Configures the CSI peripheral with `IMAG_PARA` height shrunk to e.g.
   13 lines AND `one_shot = false` (continuous ping-pong).
2. Lets DMA run continuously into FB1/FB2 for a fixed wall-clock window
   (e.g. 1 second).
3. On every FB-done IRQ, decodes events into a host-side ring buffer.
4. Returns the decoded events to the caller for the same coherence
   checks the current script runs.

This is the same statistical comparison (default vs shrunken) but with
the actual continuous-DMA configuration the design will use, not the
snapshot-per-call lifecycle.

### 8b. Run on hardware

Same procedure as the current validation: wave hand / LED, run the
script, paste the output.

### 8c. Decision criteria

- If continuous-DMA shrunken capture passes the stricter criteria →
  **Option C-frame is viable**, original Option B design holds (with
  the small change of "always run continuous, never snapshot-per-call").
- If continuous-DMA shrunken still fails on pixel-event count or pixel/
  total ratio → **escalate to C-line**, write a third validation patch
  using IMAG_PARA height = 1 + per-line decode.
- If C-line also fails → genuinely a sensor-side issue (Hypothesis A
  partly correct, sensor's CPI block can't service tight pacing). Then
  we revisit: B-modified with looser latency, or accept Option A with
  the use case implications spelled out.

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
