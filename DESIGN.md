# DESIGN — `evtstream` MicroPython Native Module

Target board: **`OPENMV_RT1060`** (i.MX RT1062DVJ6A).
Repo: this fork, layout matches `EXPLORATION.md`.
Status: design only — no C written yet.

This document is the deliverable for Task 2. It picks Option A vs B with
reasoning, fixes the wire format, and enumerates failure modes. Implementation
(Task 3) waits on review.

---

## 0. Build verification (resolution to Q1 from EXPLORATION.md)

`/.github/workflows/firmware.yml` already includes `OPENMV_RT1060` in its
build matrix and uploads a per-target artifact via `actions/upload-artifact@v7`.
The artifact is named `OPENMV_RT1060` and contains the `build/bin` outputs
(renamed by `ci_build_target` when `artifacts=true`). The runner is
`ubuntu-24.04` (x86_64), so the SDK 1.1.0 fetch works.

**No CI changes needed.** Once a feature branch is pushed to a fork, the
existing workflow rebuilds RT1060 firmware automatically; you `wget` the
artifact from the Actions tab. Build verification is therefore unblocked
without doing anything on the Jetson.

---

## 1. Recommendation: Option B (continuous CSI DMA + PIT timer)

I am recommending **Option B**, not A. The brief asks me to lean toward A
unless I find something making it infeasible — and I did. Reasoning below.

### Why Option A as described doesn't reach 1 ms cadence

The brief's Option A is "configure the GenX320's CPI packet timing so the
sensor itself produces output at ~1 ms intervals." Two findings against this:

1. **The CPI packet size in `set_active_mode(MODE_EVENT)` is hard-coded to
   the framesize.** `drivers/sensors/genx320.c:716`:
   ```c
   psee_sensor_write(csi, CPI_PACKET_SIZE_CONTROL, packet_width);
   psee_sensor_write(csi, CPI_PACKET_TIME_CONTROL,
                     packet_width << CPI_PACKET_TIME_CONTROL_PERIOD_Pos |
                     packet_hsync << CPI_PACKET_TIME_CONTROL_BLANKING_Pos);
   ```
   `packet_width = 320`, `packet_hsync = EVENT_HSYNC_CLOCK_CYCLES = 280`.
   At the EVT clock (~48 MHz: 24 MHz × `EVT_CLK_MULTIPLIER=2`), one EVT
   "line" is `(320 + 280) / 48e6 ≈ 12.5 µs`. A full 320-line frame is
   ≈ 4 ms minimum and only at peak event rate. Below peak, the sensor
   stalls between events; the 4 ms is a floor, not a ceiling.

2. **The i.MX CSI peripheral's `IMAG_PARA` register pegs DMA frame size
   to `dma_line_bytes × image_height`.** `ports/mimxrt/omv_csi.c:387`. In
   EVENT mode this is 320 × 1 × 320 = 102400 bytes / 25600 EVT2.0 words.
   Each `imx_csi_snapshot()` call collects one such frame and blocks
   waiting on its EOF interrupt. So even if we shrunk the sensor's CPI
   packet (which has unknown lower bounds), the CSI DMA framing on the
   MCU side would also need shrinking — IMAG_PARA × dma_line_bytes
   reconfiguration on every snapshot.

To make Option A actually emit packets at 1 ms cadence we'd have to: (a) find
a small CPI packet size the GenX320 supports, (b) verify it doesn't break
sensor-internal AFK/STC/ESP block alignment, (c) reconfigure CSI `IMAG_PARA`
to match. That's three unknowns vs. Option B's one (USB CDC throughput at
1 kHz). Option B reuses every existing piece more cleanly.

### Why Option B is feasible without DMA surgery

Two existing pieces of infrastructure make Option B much smaller than the
brief implied:

1. **CSI ping-pong DMA already exists.** `ports/mimxrt/omv_csi.c` programs
   `CSI_REG_DMASA_FB1` and `_FB2` and enables both `FB1_DMA_DONE_INTEN` and
   `FB2_DMA_DONE_INTEN`. The non-`one_shot` JPEG path (line 380, 387) sets
   `IMAG_PARA` height = 1 and runs the DMA continuously line-by-line via
   `omv_csi_line_callback()`. That code path is the existence proof that
   continuous double-buffered DMA works on this peripheral with this driver.
2. **A frame callback hook is already wired through.** `common/omv_csi.h:617`
   defines `omv_csi_set_frame_callback(csi, cb)`, and
   `omv_csi_frame_callback()` in `ports/mimxrt/omv_csi.c:193` calls
   `csi->frame_cb.fun(csi->frame_cb.arg)` from the CSI IRQ after each
   frame's DMA completes. We can register our own callback and run packing
   work from the CSI ISR without touching the existing snapshot path at all.

So Option B's "modify `omv_csi.c`" risk in the brief is overstated — we
register a callback and add a PIT timer; the CSI driver itself stays
unchanged.

### Where the cadence comes from

The 1 ms cadence is **MCU-timer driven** (PIT), not sensor driven. The CSI
DMA runs at sensor pace (faster than 1 kHz when events stream, slower
during static scenes), filling a software ring of decoded events. The PIT
ISR fires every 1 ms regardless of CSI activity, drains whatever is in the
ring, packs a packet, hands it to the USB CDC TX path. Heartbeats fall out
naturally — if the ring is empty, the packet has `event_count = 0` and
ships anyway.

**Expected jitter**: PIT ISR latency on a Cortex-M7 with sensible NVIC
priority is well under 5 µs unless preempted by a higher-priority handler.
USB OTG IRQ is the other high-rate ISR; we set PIT priority below USB so
USB never preempts, and structure the PIT handler to exit fast (no
blocking USB writes — see §4). Realistic ±std dev: under 10 µs for the
PIT-fire-to-window-start moment, well within the ±50 µs target. Worst-case
single-window jitter from a stacked USB IRQ: ~30 µs. Under load, actual
end-to-end Jetson arrival jitter will dominate (USB scheduling) — but that's
exactly what the MCU timestamp in the packet header lets the Jetson
correct for.

---

## 2. Wire format — final

Per task2_instructions.md §3, both per the recommendation that landed there.
Endianness: **little-endian** throughout (RT1062 is little-endian; Jetson
aarch64 is little-endian; matches `np.frombuffer` defaults).

### Packet header — 20 bytes

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;              // 0xE7E7E7E7 — for resync
    uint32_t window_start_us;    // MCU clock (PIT lifetime tick, µs) at window start
    uint16_t window_duration_us; // nominal window length, typically 1000
    uint16_t event_count;        // number of wire_event_t in payload
    uint16_t flags;              // see below
    uint16_t reserved;           // alignment, must be 0
    uint32_t sequence;           // monotonic packet counter (wraps at 2^32)
} evt_packet_header_t;
```

`flags` bits:
- bit 0  `EVT_FLAG_TRUNCATED` — `event_count` was clamped to
  `max_events_per_window`; the window held more
- bit 1  `EVT_FLAG_USB_RETRY` — the previous window's packet was dropped
  by USB backpressure; this is informational, not an error from this
  window's perspective
- bits 2..15 reserved, must be 0

### Wire event — 8 bytes each

```c
typedef struct __attribute__((packed)) {
    uint16_t t_us;       // 0..999, µs offset from window_start_us
    uint16_t x;           // 0..319
    uint16_t y;           // 0..319
    uint8_t  polarity;    // 0 = OFF, 1 = ON
    uint8_t  flags;       // bit 0 = trigger event (then x/y/polarity may be
                          //   reinterpreted by Jetson); bits 1..7 reserved
} evt_wire_event_t;
```

8 bytes naturally aligned. The Jetson decodes via:

```python
EVENT_DTYPE = np.dtype([('t_us', '<u2'), ('x', '<u2'), ('y', '<u2'),
                        ('polarity', 'u1'), ('flags', 'u1')])
events = np.frombuffer(payload, dtype=EVENT_DTYPE)
```

A 1 ms window with `max_events_per_window = 2048` gives `20 + 2048*8 = 16404`
bytes per packet. At full 1 kHz cadence the bandwidth is 16.4 MB/s on USB.
USB OTG1 on the RT1062 is high-speed (480 Mbps theoretical / ~50 MB/s
practical CDC). This is comfortably within the ceiling.

### Trigger / reset events

`ec_event_t` distinguishes pixel events from `EXT_TRIGGER_*` /
`RST_TRIGGER_*`. We collapse them: trigger and reset events get
`flags |= 0x01`, `polarity` carries the rising/falling bit, `x`/`y` carry
trigger-id (zero-extended). Pixel events get `flags = 0`. Cheap to
disambiguate on the Jetson.

---

## 3. Memory layout and sizing

All buffers pre-allocated at `evtstream.start()`. Nothing in the hot path
calls `malloc`, `m_new`, `fb_alloc`, or otherwise touches the MicroPython GC.

| Buffer | Size | Region | Why |
|---|---|---|---|
| **Decoded event ring** (`evt_wire_event_t[]`) | `4096 × 8 = 32 KB` | DTCM | M7 has fastest path to DTCM; written in CSI ISR, read in PIT ISR. 4096 = 2× max_events_per_window so the PIT can drain a full window without overlapping the writer. |
| **Packet TX buffer A** | `20 + 2048 × 8 = 16404 B` | DTCM | Filled in PIT ISR, handed to USB CDC. |
| **Packet TX buffer B** | `16404 B` | DTCM | Double-buffered with A so the next PIT ISR can fill while USB drains the previous one. |
| **EVT2.0 raw DMA buffers (FB1, FB2)** | `4096 B each = 8 KB` | DTCM | Replaces the existing `_line_buf` for our ioctl. Smaller than full 102400 B frame so DMA-done IRQ fires more often, reducing event-to-decode latency to ≤ 1 ms even at low event rates. |
| **State struct** | `~128 B` | DTCM | counters, indices, sensor mode save/restore. |

**Total: ~64 KB DTCM**. Board provides 384 KB DTCM (`OMV_DTCM_LENGTH 384K`).
Plenty of headroom.

### DTCM reachability

The RT1062's DTCM is on the FlexRAM bank tied to the M7 core. **USB OTG1's
EHCI DMA controller can read system memory, including DTCM, via AXI**
(verified by NXP RM 11.4 — USB DMA reaches all AXI-attached memory). So
DTCM is fine for USB TX source; no need to bounce through OCRAM.

For CSI DMA into FB1/FB2, the existing driver already targets
`_line_buf[]` declared in DTCM (board's `OMV_DMA_MEMORY = DTCM`), so
that's also fine.

### Linker placement

The state struct, ring, and TX buffers go in DTCM via the existing
`OMV_DMA_MEMORY` section (no new linker template work). The hot-path
function code can be placed in `OMV_RAMFUNC_MEMORY = ITCM2` via the existing
`__attribute__((section(".ramfunc")))` if profiling shows the flash-XIP
fetch latency matters. Default to flash-resident for v1; promote to ITCM
if cadence jitter exceeds budget.

---

## 3a. Memory layout — measured (added 2026-04-26, revised 2026-04-27)

The "~64 KB total ... plenty of headroom" claim in §3 was wrong twice over:
once for assuming DTCM had room (it doesn't), and once for assuming OCRM1
was a fallback (it isn't either). Both are full. The validation IOCTL
overflowed DTCM by ~15 KB just trying to add two 8 KB static FBs.

### Measured allocations on a stock OPENMV_RT1060 build

`arm-none-eabi-readelf -S build/bin/firmware.elf`:

**DTCM (393,216 B / 384 KB):**

| Section          | Address      | Size       | Bytes    |
|------------------|--------------|-----------:|---------:|
| `.data`          | 0x20000000   | 0x600      | 1,536    |
| `.bss`           | 0x20000800   | 0x17888    | 96,392   |
| `.gc.block1`     | 0x20018088   | 0x43c00    | 277,504  |
| `.dma.memory0`   | 0x2005bcc0   | 0x4000     | 16,384   |
| **Used**         |              |            | **391,816** |
| **Free**         |              |            | **1,400 B** |

**OCRM1 (524,288 B / 512 KB):** entirely consumed by `.fb_overlay_storage`
(0x80000 = 512 KB at 0x20200000).

**OCRM2 (65,536 B / 64 KB):** `.gc.block0` claims most of it.

So the previous §3a "three cases best/mid/worst" framing is moot. There
isn't a meaningful amount of free TCM-or-OCRAM anywhere. The only viable
"shave a region" candidate is `.gc.block1` at 277 KB (MicroPython's GC
heap), which is user-visible (reduces Python heap ceiling).

### Pivot: everything-in-DRAM via fb_alloc, state struct in DTCM remainder

DRAM is the only region with meaningful free space (~22 MB unallocated
in `.fb_memory` at 0x80900000). All evtstream working buffers go there
via `fb_alloc()`. The validation IOCTL already proves this works for
CSI capture with cache invalidation in the ISR (commit 9ec4d5d) — extending
to the rest of the buffers is a mechanical extension of the same pattern,
not a new architectural decision.

### Production memory placement — final

Production buffer geometry uses the §7a-recommended **h=6 lines**, so each
ping-pong FB is 6 KB.

| Buffer | Size | Region | Allocation | Cache management |
|--------|-----:|--------|------------|------------------|
| Decoded event ring | 32 KB | DRAM | `fb_alloc()` at `start()` | `SCB_InvalidateDCache_by_Addr` after CSI-ISR appends; `SCB_CleanDCache_by_Addr` is unnecessary because PIT-ISR is read-only on the ring |
| Packet TX buffer A | 16 KB | DRAM | `fb_alloc()` at `start()` | `SCB_CleanDCache_by_Addr` before USB submit (write direction) |
| Packet TX buffer B | 16 KB | DRAM | `fb_alloc()` at `start()` | Same as A |
| Raw FB1 (h=6) | 6 KB | DRAM | `fb_alloc()` at `start()` | `SCB_InvalidateDCache_by_Addr` in line-callback hook (already in port code from validation patch) |
| Raw FB2 (h=6) | 6 KB | DRAM | `fb_alloc()` at `start()` | Same as FB1 |
| evtstream state struct | 128 B | DTCM | static (fits in 1.4 KB free) | None — TCM is non-cacheable |

Total DRAM: ~76 KB out of ~22 MB free in `.fb_memory`. Trivial.
Total DTCM: 128 B out of 1.4 KB free. Comfortable.

### Cache management — concrete patterns

- **Decoded event ring (DRAM):** CSI ISR walks the just-completed FB and
  appends decoded events to the ring. PIT ISR drains the ring into a TX
  packet. Both ISRs run on the M7 with D-cache enabled. Two correctness
  rules:
  1. *After CSI ISR writes:* `SCB_CleanDCache_by_Addr(ring_chunk, size)`
     so the lines are pushed to DRAM. (Necessary only if a different bus
     master could read them — for our case, the PIT ISR is also on the
     M7, so cache coherency holds without an explicit clean. Document
     the assumption.)
  2. *Before PIT ISR reads:* No invalidate needed when both writer and
     reader are the same M7 — the cache is coherent with itself. If we
     ever moved drain to a DMA engine, this would change.
- **TX buffers (DRAM):** PIT ISR fills, then calls
  `SCB_CleanDCache_by_Addr(buf, used_bytes)` to push CPU writes to
  physical memory before handing the buffer to USB EHCI DMA. Identical
  pattern to OpenMV's existing `framebuffer.c:223` (read-direction
  invalidate); we apply the write-direction analogue.
- **Raw FB1/FB2 (DRAM):** Already implemented in
  `omv_csi_line_callback`'s streaming branch (commit 9ec4d5d).
  `SCB_InvalidateDCache_by_Addr(addr, fb_size_bytes)` runs before
  the user callback gets the buffer.

### What this costs in latency

See §7a's amended worked numbers for the full breakdown. Short version:
cache invalidation/clean of a 6 KB FB is ~96 cache-lines × ~30 cycles ≈
5 µs at 600 MHz. Per-IRQ overhead is negligible relative to the 1 ms PIT
window or the 5 ms saccade budget.

### Alternative considered: shave `.gc.block1`

The MicroPython GC heap at 277 KB could be reduced by 32 KB (changing
`OMV_GC_BLOCK1_SIZE` from `271K` to `239K`) to make room for the
decoded event ring in DTCM. This would lower L_cache by ~5 µs per
FB-IRQ. Not worth doing for v1: the cache cost is already negligible,
and shrinking the user-visible Python heap is a regression for users
who don't care about evtstream. Reserve this option as a future
optimisation if measurements show ring access is a bottleneck.

---

## 4. ISR / non-ISR responsibility split

Three execution contexts:

### CSI IRQ (`CSI_IRQn`) — fast path, runs on every FB1/FB2 DMA done

Already wired. We register a `frame_cb` via
`omv_csi_set_frame_callback(csi, ...)`. On each fire:

1. Read DMA destination buffer pointer (the just-completed FB).
2. **Decode EVT2.0 words → `evt_wire_event_t`** writing into the decoded
   event ring at `ring_head`. This is the same logic as
   `post_process_event` in `drivers/sensors/genx320.c:635`, lifted into
   `py_evtstream.c`. ~5 instructions per event; at 10M evt/s peak that's
   ~5% CPU on the M7 — fine.
3. Update the running 64-bit `event_time_us` accumulator from any
   `EV_TIME_HIGH` words seen.
4. Advance `ring_head` (atomic-by-construction since CSI IRQ is the
   sole writer).
5. If ring is full, set the lost-events sticky counter and skip the
   overflowing words. (See §5.)

The CSI IRQ does **not** call USB or touch the packet buffers. Bounded
runtime even at peak event rate.

### PIT IRQ — cadence path, runs every `window_us`

PIT0 is dedicated to evtstream. Configured at PIT input clock /
window_us-µs period. NVIC priority **below** USB OTG, **above**
SysTick.

On each fire:

1. Capture `t_now_us` from PIT lifetime counter.
2. Pick the inactive packet TX buffer (`A` if `B` was last, else `A`).
3. Fill header: `magic`, `window_start_us = last_window_start_us + window_us`,
   `window_duration_us = window_us`, `flags = pending_flags`,
   `sequence++`.
4. Read `ring_tail` and `ring_head` atomically (single 32-bit load).
5. Drain min(ring_count, max_events_per_window) events from ring into
   payload.
6. If `ring_count > max_events_per_window`, set `EVT_FLAG_TRUNCATED` and
   advance `ring_tail` past the discarded events (decision logged in §5).
7. Set `event_count`.
8. Hand the buffer to the USB CDC writer via `tud_cdc_write` — non-
   blocking; if the CDC TX FIFO can't accept the full packet, **do not
   block**. Three sub-cases:
   - Whole packet accepted → set `pending_flags = 0` for next window.
   - Short write → roll back `ring_tail` (we'll retry next window with
     same data + a fresh header — but that violates cadence). **Cleaner:
     drop the packet entirely**, set `EVT_FLAG_USB_RETRY` for the next
     window's flags so the Jetson knows there's a gap, increment
     `usb_drops` stat.
   - Zero accepted → same as short write.
9. Call `tud_cdc_write_flush()` (cheap, just kicks the EP submission;
   the actual USB transaction happens in the USB OTG IRQ).
10. Return.

Bounded runtime: 1 memcpy of up to 16 KB (~5 µs in DTCM at M7 speed) plus
constant overhead. Well under the 1 ms window.

### USB OTG IRQ — driven by tinyusb, untouched

tinyusb's USB OTG IRQ services the EHCI scheduler, drains the CDC TX
FIFO into USB packets, and signals back to the host. We don't touch this
code; we rely on `tud_cdc_write` being safe to call from a peer IRQ at
equal-or-lower priority. (Tinyusb documents this; the CDC class uses a
single-producer FIFO with critical sections gated by the BASEPRI
mechanism.)

### Main loop / MicroPython context

`evtstream.start()`, `.stats()`, `.stop()` run here. None of them touch
the ring head or TX buffers' "ownership flag" without first masking the
PIT and CSI IRQs (or by reading-only from a stable snapshot).

---

## 5. Overrun handling

Per task2_instructions.md §"Overrun handling": **truncate + flag**.

Two distinct overrun cases, both surfaced in the packet:

### Case A: ring buffer overflow (CSI ISR's view)

The decoded event ring fills (e.g. blink-induced burst, 50K events in
1 ms, but ring only holds 4096). The CSI ISR drops new events on the
floor and increments `ring_lost_events_total`. The next PIT window
**does not** set a packet-level flag for this case (the dropped events
weren't in the window's payload anyway). Instead, the count is exposed
in `stats()` for observability.

Rationale: the Jetson tracker can detect this via `stats()` when it
matters. We don't burn a header bit on a counter that's monotonic.

### Case B: window has more events than `max_events_per_window`

The PIT ISR drained the ring and found > `max_events_per_window` events
ready. It clamps to the max, advances `ring_tail` past the discarded
extras, and sets `EVT_FLAG_TRUNCATED` in this window's header. Counter
`window_truncated_total` ticks.

Rationale: per the task instructions, blink-induced overruns are a
useful tracker signal, not a failure. The flag preserves cadence and
gives the Jetson an honest "the window's rate exceeded our packet
ceiling" signal.

### Why not multi-packet split-on-overrun

Considered. Rejected because: (1) it breaks the "one packet per window"
invariant the cadence-measurement code on the Jetson relies on; (2) the
Jetson can already infer overflow from the flag and request the full
recording later if it cares; (3) added complexity not justified by v1
use case.

---

## 6. Heartbeats — confirmed mandatory

Every PIT fire emits a packet, even if `event_count = 0`. Without this,
USB stalls and "static scene" look identical from the Jetson side. The
heartbeat-or-data dichotomy is what lets the Jetson's
`--measure-cadence` mode run.

Cost: 20 bytes/ms = 20 KB/s of USB during static periods. Negligible.

---

## 7. Python API — semantics

```python
import evtstream

evtstream.start(window_us=1000, max_events_per_window=2048)
evtstream.stats()  # dict — see below
evtstream.stop()
```

### `start(window_us, max_events_per_window)`

- Validates: `100 <= window_us <= 10000`,
  `64 <= max_events_per_window <= 512`. (Originally 4096; lowered to 512
  after the step-3 CDC benchmark — see §3a-revision below.)
- If already started: raises `OSError(EBUSY)`.
- Saves the GenX320's current mode (`OMV_CSI_GENX320_MODE_HISTO` by
  default).
- Switches sensor to `MODE_EVENT` via the existing
  `OMV_CSI_IOCTL_GENX320_SET_MODE` IOCTL — reuses the I2C reg writes in
  `set_active_mode()`. No duplication.
- Reconfigures CSI `IMAG_PARA` height to a small value (e.g. 4 KB / 320
  = ~13 lines) so DMA-done IRQ fires every ~3 ms instead of every full
  102400-byte frame. **This is the only port-side change** — done by
  briefly stopping the CSI, reprogramming, restarting. No edit to
  `ports/mimxrt/omv_csi.c` itself; it's all from `py_evtstream.c` calling
  CSI register macros.
- Registers our `frame_cb` via `omv_csi_set_frame_callback()`.
- Configures PIT0 channel 0 for `window_us` interval, IRQ enabled.
- Initializes counters and ring.
- Issues a one-shot `omv_csi_snapshot(NON_BLOCK)` to kick off DMA, then
  returns. Subsequent CSI cycles run autonomously via the existing SOF
  callback chain.

### `stats()`

Returns a dict, atomically snapshotted (PIT IRQ briefly masked):

```python
{
    'windows_sent': int,             # PIT fires that produced a packet (incl. heartbeats)
    'events_sent': int,              # total events in shipped packets
    'usb_drops': int,                # packets dropped because USB couldn't accept
    'window_truncated_total': int,   # windows that hit max_events_per_window
    'ring_lost_events_total': int,   # events lost to ring overflow (CSI side)
    'csi_dma_underruns': int,        # CSI buffer not ready when expected
    'last_window_event_count': int,  # debugging aid
    'last_window_us': int,           # most-recent window_start_us
    'sequence': int,                 # next packet sequence number
}
```

### `stop()`

1. **Refuse if called from an IRQ** (check `__get_IPSR()`); raise
   `OSError(EPERM)`. Tighter than the brief — the brief said "stop()
   called from within the ISR (must not be allowed)" but didn't say how.
2. If not running: return silently. (Brief: "Be safe to call from
   MicroPython without crashing if streaming wasn't active.")
3. Disable PIT IRQ, mask the channel.
4. Disable the CSI's frame_cb (set to `{NULL, NULL}`).
5. Call `omv_csi_abort()` to stop CSI DMA cleanly.
6. Drain in-flight USB writes with a bounded wait
   (`tud_cdc_write_flush()` + 100 ms ceiling polling
   `tud_cdc_write_available()`).
7. Restore the GenX320's pre-`start` mode via SET_MODE IOCTL.
8. Restore the CSI's original `IMAG_PARA` setting.
9. Clear the running flag.

`stop()` is **not** safe to call concurrently with itself from two
threads — but MicroPython is GIL-effectively-single-threaded, so this is
fine.

---

## 8. Failure mode table

| Failure | Detection | Behavior |
|---|---|---|
| USB host disconnects mid-stream (Jetson crash, cable yank) | `tud_cdc_connected()` returns false | PIT keeps firing; packets are dropped at `tud_cdc_write` (returns 0 written). `usb_drops` increments every window. No crash, no MicroPython exception. Resumes automatically when the host reconnects. |
| USB write returns short (host can't keep up) | `tud_cdc_write` returns < requested | Drop the packet entirely (don't ship a partial). Set `EVT_FLAG_USB_RETRY` for the next window. Increment `usb_drops`. |
| Sensor stops producing events (I2C glitch, brown-out) | CSI DMA-done IRQs cease | PIT keeps firing → heartbeat-only packets ship. Jetson sees `event_count=0` for an extended period and can decide policy. `csi_dma_underruns` ticks if CSI is expected to have produced data but hasn't (TBD: define "expected"). No automatic recovery in v1. |
| Timer ISR latency spike from another ISR (e.g. USB IRQ) | `t_now_us - last_window_start_us > 2 × window_us` | Don't try to "catch up" by emitting two packets. Emit one packet for the current window, log the gap implicitly via `sequence` and `window_start_us`. |
| `start()` called twice without `stop()` | Internal `running` flag | Raise `OSError(EBUSY)`. |
| `stop()` called from within an ISR | `__get_IPSR() != 0` | Raise `OSError(EPERM)`. |
| `stop()` called when not running | Internal flag | Return silently. |
| Sensor mode was something exotic before `start()` | Read pre-call mode | Save and restore exactly what we read. Don't assume HISTO. |
| Out-of-bounds args to `start()` | Range check at entry | `ValueError` with the offending field. |
| `max_events_per_window` exceeds the validated FIFO ceiling | Range check | `ValueError` — accepted range is [64, 512] (see §7-revision). |
| MicroPython GC sweep runs in the middle of a 1 ms window | N/A — heap not touched in hot path | No effect. Per §3a, ring + TX bufs + FBs are `fb_alloc()`'d from the framebuffer arena in DRAM (not the GC heap); state struct is a static in DTCM. None of these are visible to the MicroPython GC mark phase. |

---

## 9. Open risks — things to verify on hardware

1. **CSI `IMAG_PARA` height < native frame.** We're betting we can set
   `IMAG_PARA` height to ~13 lines instead of 320 and still have the
   sensor's CPI block deliver coherent EVT2.0 word streams across that
   boundary. The sensor runs free; the CSI just samples 4 KB at a time.
   Should be fine but I haven't seen this exact configuration exercised
   in the existing OpenMV examples — verify on hardware.
2. **DMA-done IRQ rate at peak event flux.** If the CSI is configured
   for 4 KB FB1/FB2 buffers, peak event rate (e.g. 10M evt/s saturated)
   means IRQ every ~100 µs. At a 5-instruction-per-event decode, that's
   ~5 µs IRQ duration → 5% CPU. Acceptable. But if real peak is 50M
   evt/s during a calibration burst, this could grow. Verify with the
   existing `--save my_recording.csv` baseline and inspect peak rate.
3. ~~**USB CDC throughput at sustained 16 MB/s.**~~ **Resolved by
   step-3 benchmark (2026-04-27)**: 99 % efficiency at packets up to
   480 events (3860 bytes), 100 % drops at packets ≥ 4116 bytes —
   sharp cliff at the 4 KB FIFO size. The `max_events_per_window`
   ceiling was lowered to 512 to fit the validated FIFO size; see §7
   `start()` revision and the resolved-questions block at the bottom.
4. ~~**`tud_cdc_write` behavior in IRQ context.**~~ **Resolved**:
   step-3 benchmark calls `tud_cdc_write` from the PIT ISR at 1 kHz
   for 2 s × 6 sweep points without any crashes or stalls. Tinyusb
   CDC FIFO is IRQ-safe in this configuration.
5. **Effect on other CSI users while streaming is active.** While
   `evtstream.start()` is running, the existing GenX320 IOCTLs
   (`READ_EVENTS`, `CALIBRATE`) and other sensor scripts cannot use the
   CSI. Document this — it's expected, but if a user runs both they get
   confused error messages from `omv_csi_snapshot()`. Possible
   mitigation: have `start()` set a "claimed" flag that other CSI
   IOCTLs check and bail out cleanly.
6. **PIT lifetime counter wrap.** PIT lifetime is 64-bit but we expose
   `window_start_us` as 32-bit (rolls over at 2^32 µs ≈ 71.5 minutes).
   For an eye-tracking session this is fine, but the Jetson side must
   handle wrap. Document that on the Jetson receiver.
7. **Inter-IRQ ordering between PIT and CSI.** If both fire in the same
   1 ms window, we need PIT to read a stable ring head. We use a
   single 32-bit `ring_head` and rely on the M7's atomic 32-bit store.
   The PIT ISR is at lower NVIC priority than CSI, so CSI can preempt
   PIT mid-drain. Verify drain logic re-reads `ring_head` after each
   chunk and is safe across preemption. (This is straightforward but
   the implementation needs care; flagging here so review can challenge.)

---

## 10. Out of scope (matches brief)

- No custom USB bulk endpoint. CDC stays.
- No tracker on MCU.
- No Ethernet path.
- No N6 / H7 Plus / OPENMV4P support — RT1060 only. Build flag
  `OMV_GENX320_ENABLE` is gated by `BOARD == OPENMV_RT1060` for the
  evtstream module's `MODULE_EVTSTREAM_ENABLE` (added in
  `boards/OPENMV_RT1060/omv_boardconfig.mk`).

---

## 11. Files I expect to add/modify in Task 3

For your reference; nothing written yet.

| File | Add / Modify | Why |
|---|---|---|
| `modules/py_evtstream.c` | Add | The module. Contains state, ISRs, `start/stats/stop`. |
| `modules/modules.mk` (or equivalent) | Modify | Compile py_evtstream.c when `MICROPY_PY_EVTSTREAM=1`. |
| `boards/OPENMV_RT1060/omv_boardconfig.mk` | Modify | Set `MICROPY_PY_EVTSTREAM=1` to wire it into RT1060 builds. |
| `ports/mimxrt/omv_mpconfigport.h` | Modify (small) | Register the `evtstream` module symbol with MicroPython's module table, gated on `MICROPY_PY_EVTSTREAM`. |
| `jetson_receiver/` | Add | Task 4 — Jetson-side reader & cadence tool. |
| `test_cadence.py` | Add | Task 5. |

No edits to `drivers/sensors/genx320.c`, `common/omv_csi.{c,h}`, or
`ports/mimxrt/omv_csi.c`. The module is purely additive.

(Caveat: the validation IOCTL series did add temporary code to
`common/omv_csi.h`, `drivers/sensors/genx320.c`, `ports/mimxrt/omv_csi.c`,
and `modules/py_csi_ng.c` — the streaming-hook variables, the line-callback
short-circuit with cache invalidation, the IOCTL bindings, etc. The
streaming-hook + cache invalidation infrastructure stays for production
because the evtstream module reuses it. The DEBUG_CAPTURE / DEBUG_CAPTURE_CONTINUOUS
IOCTLs themselves can be removed once the design is locked.)

---

## 11a. Implementation-order amendment for the DRAM-resident layout (added 2026-04-27)

§3a's pivot to DRAM-via-fb_alloc for all working buffers introduces cache
management as a first-class correctness concern. task3_instructions.md §8
specifies a six-step implementation order; one extra step is needed
**between synthetic CDC benchmark (step 3) and CSI integration (step 4)**:

### New step 3a: cache-management stress test

Before integrating the CSI capture path, verify the cache-management
patterns end-to-end under sustained load. Write a self-contained test
mode in `py_evtstream.c` (gated by a Python entry point, e.g.
`evtstream.bench_cache()`) that:

1. Allocates the four DRAM working buffers via `fb_alloc()` (one ring,
   two TX bufs, one combined-FB1+FB2 mock) at the production sizes.
2. From the PIT ISR, simulates the production write/read pattern:
   - **Mock CSI ISR**: writes a counter pattern into a "FB" buffer in
     DRAM, then `SCB_InvalidateDCache_by_Addr` (the production code
     does the same in the line-callback streaming hook; here we just
     test the round-trip).
   - **Read back & verify**: a follow-up read inside the same IRQ
     reads the buffer and checks the counter values match. If the cache
     management is wrong, mismatches will show up here.
   - **TX direction (best-effort, partially verifiable)**: write a
     counter pattern into a TX buffer, call `SCB_CleanDCache_by_Addr`,
     and run a follow-up read after a `__DSB()`. This confirms the
     SCB call doesn't crash and that the buffer holds the expected
     bytes — but it does *not* prove the cache lines actually reached
     physical DRAM, since the M7 will hit its own clean cache on the
     follow-up read. True end-to-end TX-direction verification needs
     a separate bus master reading the cache-bypass path; the cleanest
     such master in this system is USB EHCI DMA itself.
3. Runs at the production PIT rate (1 kHz) for 10 seconds.
4. Reports counters via `stats()`: cache mismatches, sequence gaps,
   SCB call counts, total bytes round-tripped.

PASS criteria for step 3a: zero ring/FB-direction mismatches over 10
seconds at 1 kHz cadence. The TX-direction is *expected* to round-trip
cleanly in this test — that result tells us the SCB call sequence
doesn't crash, not that the cache reached DRAM.

**Real TX-direction verification happens at step 5 (full path):** if
the Jetson-side reader runs cleanly with no magic-byte resync events
under sustained load, the TX cache-clean is working in production.
That's the load-bearing test for this direction; bench_cache() is the
load-bearing test for the ring/FB direction only.

This split is honest about what each test actually proves. Don't
over-engineer bench_cache() to chase symbolic TX verification — the
gain is small and step 5 covers it for real.

This is cheap to write (few hundred lines), runs without sensor
hardware, and decouples the cache-correctness verification from the
CSI integration. If it passes, we know the ring/FB cache pattern is
solid before adding the CSI complexity. If it fails, we debug cache
ordering in isolation rather than amid CSI ISR latency.

### Revised six-step order

| Step | Source | Status |
|------|--------|--------|
| 1. Skeleton (start/stats/stop flags only) | task3_instructions.md §8 step 1 | unchanged |
| 2. PIT-only path | step 2 | unchanged |
| 3. Synthetic CDC benchmark | step 3 (also covers task3_instructions §6) | unchanged |
| **3a. Cache-management stress test** | **new** (this section) | **mandatory before step 4** |
| 4. CSI integration | step 4 | unchanged |
| 5. Full path | step 5 | unchanged |
| 6. CSI exclusivity hook | step 6 | unchanged |
| 7. Failure mode handling | step 7 | unchanged |

Stop points stay where they were: after step 1 (skeleton), after step
3 (CDC benchmark). Add a third optional stop after step 3a if the
stress test surfaces anything unexpected.

---

## Resolved questions (preserved for context)

The questions originally listed under "Stop point" at task-2 review time
are now resolved. Recording the outcomes here so the doc is internally
consistent and reviewers don't re-debate decisions:

1. **Option A vs Option B** — Option B (continuous CSI ping-pong DMA +
   PIT-driven cadence). Decided after RISK1_FINDINGS.md showed the
   GenX320's CPI packet timing floors the sensor frame rate at ~4 ms,
   making Option A's "sensor-driven 1 ms cadence" unreachable. Validated
   on hardware: see RISK1_FINDINGS.md continuous-mode results
   (Hypothesis B confirmed, std/mean ~0.002 across heights).
2. **FB1/FB2 ping-pong size** — h=6 lines (6 KB each). Originally
   considered h=4 / h=8; h=6 is the saccade-latency-margin vs
   pixel/wire-efficiency compromise per RISK1_FINDINGS.md §7a.
3. **Packet header size** — 20 bytes. Originally 16 bytes per the
   task-2 strawman; expanded to add `flags` (1 bit used for TRUNCATED,
   rest reserved) plus alignment padding. Overhead negligible at
   1 kHz × 16 KB packets.
4. **`max_events_per_window` ceiling** — 512 events (revised 2026-04-27
   from the original 4096 after the step-3 CDC throughput benchmark).
   At 1 kHz cadence the tinyusb CDC TX FIFO at the existing
   `CFG_TUD_CDC_TX_BUFSIZE = 4096` (in
   `lib/micropython/ports/mimxrt/boards/OPENMV_RT1060/mpconfigboard.h`)
   sustains 99 % efficiency for packets up to ~480 events (3860 bytes)
   and drops 100 % at packets ≥ 4116 bytes. The eye-tracking saccade
   peak rate is ~50–200 K events/s, i.e. ~50–200 events per 1 ms window
   with worst-case bursts ~500. 512 fits cleanly inside the validated
   FIFO ceiling without any board-config changes; if a future use case
   needs higher per-window rates, bumping `CFG_TUD_CDC_TX_BUFSIZE` to
   16 KB or 32 KB is a one-line change. The ring buffer was sized at
   32 KB / 4096 events to give 2× headroom over the original 2048
   max; with the new 512 ceiling that headroom is now 8×, comfortable
   for blink bursts that overshoot the per-window cap.
5. **USB short-write behavior** — drop the packet, set
   `EVT_FLAG_USB_RETRY` on the *next* window's header. Drop is preferred
   over retry-with-stale-data so cadence is preserved; `sequence` field
   lets the Jetson detect the gap independently of the flag.
6. **Memory placement** — DRAM-via-`fb_alloc` for ring + TX bufs +
   FB1/FB2 + state-struct-in-DTCM. Per §3a after readelf measurement
   showed DTCM has only 1.4 KB free.

## Status

Design approved (task3_instructions.md confirmation). Implementation
proceeds per §11a's revised six-step order, one step per commit, with
review pauses at the points called out in §11a.
