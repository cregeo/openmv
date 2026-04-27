/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2026 cregeo (Wolfson Lab) <cregeo@wolfsonlab.com>
 * Copyright (C) 2013-2025 OpenMV, LLC.   (derivative of OpenMV firmware,
 *                                          see DESIGN.md / EXPLORATION.md)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * evtstream — fixed-cadence event streaming for GenX320 on OpenMV RT1062.
 *
 * STEP 5: full path. Production `evtstream.start()` brings up the whole
 * pipeline: CSI in continuous ping-pong DMA at h=6, CSI ISR decodes
 * EVT2.0 -> SPSC event ring (32 KB DRAM, 4096 entries), PIT at
 * window_us cadence drains the ring into a 20-byte-header + N-event
 * packet and ships via tinyusb CDC. Heartbeat packets every window
 * (event_count=0 if ring empty). Truncation flag if more events
 * available than max_events_per_window. csi_dma_underruns counts
 * windows with no CSI activity since the previous drain.
 *
 * Memory placement per DESIGN.md §3a (DRAM-resident, fb_alloc'd at
 * start, freed at stop):
 *   ring   32 KB  CSI-write / PIT-read; M7-only access, no cross-master
 *                  cache concern (writer + reader same core)
 *   tx_a   16 KB  PIT-fill / tinyusb-read (memcpy through cache; no
 *   tx_b   16 KB     SCB clean needed -- see step 3 commit d783b72)
 *   fb1     6 KB  CSI DMA -> CPU read; cache-invalidated in port-side
 *   fb2     6 KB     line-callback hook (commit c86ad4f)
 *
 * Steps prior:
 *   1.  skeleton (start/stop/stats flags only)
 *   2.  PIT-only path
 *   3.  synthetic USB CDC throughput benchmark
 *   3a. cache-management stress test
 *   4.  CSI integration (decode-and-discard)
 *
 * Subsequent steps add (per DESIGN.md §11a):
 *   6.  CSI exclusivity hook
 *   7.  failure mode handling
 */
#include "omv_boardconfig.h"

#if MICROPY_PY_EVTSTREAM

#include <stdbool.h>
#include <stdint.h>
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

// PIT and clocking — i.MX RT1062 specific. The board flag
// MICROPY_PY_EVTSTREAM is currently set only on OPENMV_RT1060, so direct
// fsl_pit.h / fsl_clock.h includes are fine. If we ever enable this module
// on another port the cross-port abstraction goes here.
#include "fsl_clock.h"
#include "fsl_pit.h"

// USB CDC (tinyusb). Used by the bench-mode TX path.
#include "tusb.h"

// fb_alloc / fb_free for the DRAM-resident TX buffers (DESIGN.md §3a).
#include "framebuffer.h"

// CSI handle + GenX320 chip-id constants for the bench_csi step-4 path.
#include "omv_csi.h"

// EVT2.0 wire format macros + type constants (TD_LOW, TD_HIGH,
// EV_TIME_HIGH, EXT_TRIGGER). Vendored under drivers/genx320/include
// and reachable via the existing OMV_GENX320_ENABLE include path.
#include "evt_2_0.h"

// Port-side helpers from ports/mimxrt/omv_csi.c (commit 9ec4d5d). They
// configure the i.MX CSI peripheral for continuous ping-pong DMA into
// caller-provided buffers and route per-FB-done IRQs to a registered
// callback. The cache-invalidate barrier sequence in the line-callback
// hook was hardened in commit c86ad4f after step 3a.
extern int imx_csi_streaming_start(omv_csi_t *csi,
                                   uint8_t *fb1, uint8_t *fb2,
                                   uint16_t dma_line_bytes,
                                   uint16_t height_lines,
                                   void (*cb)(uint8_t *, void *),
                                   void *arg);
extern void imx_csi_streaming_stop(omv_csi_t *csi);

// PIT channel allocated to evtstream. Channels 1-3 stay free for any
// future use (e.g. a second timer for failure-mode detection).
#define EVTSTREAM_PIT_CHANNEL    kPIT_Chnl_0

// Wire format — must match DESIGN.md §2 exactly so the Jetson reader
// can `np.frombuffer(...)` the packets without further interpretation.
// Both structs are little-endian on RT1062 and aarch64 hosts; no
// byte-swapping needed.
#define EVT_PACKET_MAGIC         (0xE7E7E7E7u)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t window_start_us;
    uint16_t window_duration_us;
    uint16_t event_count;
    uint16_t flags;          // bit 0 = TRUNCATED, bit 1 = USB_RETRY
    uint16_t reserved;
    uint32_t sequence;
} evt_packet_header_t;

typedef struct __attribute__((packed)) {
    uint16_t t_us;
    uint16_t x;
    uint16_t y;
    uint8_t  polarity;
    uint8_t  flags;
} evt_wire_event_t;

_Static_assert(sizeof(evt_packet_header_t) == 20, "header must be 20 B");
_Static_assert(sizeof(evt_wire_event_t)   == 8,  "wire event must be 8 B");

// Production buffer sizing per DESIGN.md §3a final placement table.
// All four sizes are powers of 2 so ring index masking is a single AND.
#define EVTSTREAM_RING_BYTES        (32u * 1024u)
#define EVTSTREAM_RING_CAP          (EVTSTREAM_RING_BYTES / sizeof(evt_wire_event_t))
#define EVTSTREAM_RING_MASK         (EVTSTREAM_RING_CAP - 1u)
#define EVTSTREAM_TX_BYTES          (16u * 1024u)
#define EVTSTREAM_FB_HEIGHT_LINES   (6u)
#define EVTSTREAM_DMA_LINE_BYTES    (1024u)
#define EVTSTREAM_FB_BYTES          (EVTSTREAM_FB_HEIGHT_LINES * EVTSTREAM_DMA_LINE_BYTES)

_Static_assert((EVTSTREAM_RING_CAP & EVTSTREAM_RING_MASK) == 0,
               "ring capacity must be a power of 2");

// Truncation flag in the packet header's flags field.
#define EVT_FLAG_TRUNCATED          (1u << 0)

// Post-hard-reset settle delay (ms) before the next I2C transaction is
// issued to the GenX320. Empirical value -- the GenX320 datasheet /
// Prophesee headers don't expose a documented post-reset wait time, but
// without this the second iteration of a tight start/stop loop catches
// the sensor mid-settle and the next SET_MODE I2C transaction stalls
// long enough to trip the M7 watchdog (USB OTG drops, physical replug
// required). 50 ms is conservative for this class of CMOS event sensor;
// tighten if a specific number lands from Prophesee.
#define EVTSTREAM_RESET_SETTLE_MS   (50)

// NVIC priority for the PIT IRQ. Lower = higher priority on Cortex-M.
// Existing assignments on this port (lib/micropython/ports/mimxrt/irq.h):
//   SysTick = 0, CSI = 3, USB OTG_HS = 6, EXTINT = 14.
// Pick 10 — between OTG_HS and EXTINT — so:
//   * USB IRQ preempts PIT  (CDC delivery isn't blocked by cadence work)
//   * SysTick preempts PIT  (OS tick keeps running)
//   * CSI preempts PIT      (CSI ISR is short and time-critical)
//   * PIT preempts EXTINT   (cadence beats user GPIO interrupts)
// This matches the spirit of DESIGN.md §4 even though the wording there
// ("above SysTick") would be ARM-priority-inverted; the intent is clear
// from context.
#define EVTSTREAM_PIT_NVIC_PRIO  (10)

// Cross-module accessor: returns true while ANY evtstream mode is
// active (production start(), or any of the bench_*() entries). The
// CSI exclusivity hook (step 6) calls this from py_csi.c / py_csi_ng.c
// to refuse user-level CSI operations that would disturb the running
// pipeline. Defined non-static + with a public-ish name so the extern
// declarations on the consumer side can resolve at link time. No
// header file because the consumer set is tiny and stable.
//
// Returns the simple `running` flag rather than a mode-specific
// predicate: even bench() / bench_cache() (which don't own CSI) hold
// the flag, and refusing CSI ops during them is harmless and
// conservatively safer than letting two evtstream-affecting paths
// race.
bool evtstream_is_running(void);  // forward decl, definition after state

// Module state. All zero-initialised in BSS until start() touches it.
// Keep this struct DTCM-resident even in production — it's tiny (~64 B
// here, ~128 B with all the runtime fields added in later steps), so it
// fits in the 1.4 KB free DTCM measured per DESIGN.md §3a.
static struct {
    bool running;
    uint32_t window_us;
    uint32_t max_events_per_window;

    // PIT bookkeeping (step 2). prev_ticks_us is the GPT counter snapshot
    // from the previous PIT fire; the ISR computes the inter-fire delta
    // from it. first_fire skips the delta computation on the very first
    // tick (no prior reference yet).
    volatile uint32_t prev_ticks_us;
    volatile bool first_fire;

    // Bench-mode (step 3). bench_mode toggles the synthetic-TX path inside
    // PIT_IRQHandler. tx_buf_a / tx_buf_b are fb_alloc'd at bench() time
    // and fb_free'd at stop(); cur_tx_buf_idx ping-pongs each fire so
    // there's no overlap between in-flight USB drain and the next fill.
    volatile bool bench_mode;
    uint32_t bench_events_per_window;
    uint32_t bench_packet_size_bytes;
    uint8_t *bench_tx_buf_a;
    uint8_t *bench_tx_buf_b;
    volatile uint8_t bench_cur_tx_buf_idx;
    volatile uint32_t bench_pattern_counter;

    // Cache-bench mode (step 3a). bench_cache_mode toggles the cache-
    // management stress test. bench_cache_*_buf are fb_alloc'd at
    // production sizes (DESIGN.md §3a) and fb_free'd at stop().
    volatile bool bench_cache_mode;
    uint8_t *bench_cache_ring;       // 32 KB
    uint8_t *bench_cache_tx_a;       // 16 KB
    uint8_t *bench_cache_tx_b;       // 16 KB
    uint8_t *bench_cache_mock_fb;    // 6 KB (h=6 production size)
    uint32_t bench_cache_mock_fb_words;  // mock_fb size / 4
    volatile uint32_t bench_cache_pattern_counter;

    // CSI-bench mode (step 4). bench_csi_mode toggles the continuous-
    // DMA + decode-and-discard path. fb1/fb2 are 6 KB each (h=6 lines
    // × 1024 dma_line_bytes) per DESIGN.md §3a final placement. They
    // are written by CSI DMA (cache-invalidated in the port-side
    // line-callback hook) and read by the decoder cb in CSI ISR
    // context.
    volatile bool bench_csi_mode;
    uint8_t *bench_csi_fb1;
    uint8_t *bench_csi_fb2;
    uint32_t bench_csi_fb_size_bytes;

    // Production stream mode (step 5). Same buffer set as bench_csi
    // (FB1/FB2) plus the SPSC event ring (CSI ISR producer, PIT ISR
    // consumer) and the two ping-pong TX packet buffers. Single-
    // producer / single-consumer ring with monotonic 32-bit counters;
    // index = head & EVTSTREAM_RING_MASK. uint32 wraps at ~4G writes
    // (~71 min at 1 M evt/s) -- acceptable for v1, can move to
    // uint64 if longer sessions are needed.
    volatile bool stream_mode;
    uint8_t *stream_ring;          // 32 KB
    uint8_t *stream_tx_a;          // 16 KB
    uint8_t *stream_tx_b;          // 16 KB
    uint8_t *stream_fb1;           // 6 KB
    uint8_t *stream_fb2;           // 6 KB
    volatile uint32_t ring_head;   // CSI-write
    volatile uint32_t ring_tail;   // PIT-write
    volatile uint8_t  stream_cur_tx_buf_idx;
    // csi_isrs_since_last_pit: incremented in CSI ISR, read-and-cleared
    // by PIT ISR; if zero at PIT fire, csi_dma_underruns++ (per the
    // task-3 step-5 spec). At idle this fires every PIT window (sensor
    // genuinely produces no FBs); user interprets in context.
    volatile uint32_t csi_isrs_since_last_pit;

    // Stats counters. windows_sent / sequence / last_window_us start being
    // touched in step 2; events_sent / usb_drops / last_window_event_count
    // start in step 3; cache_mismatches in step 3a; events_decoded /
    // last_fb_pixel_count in step 4; the rest stay zero until later
    // steps wire them.
    volatile uint32_t windows_sent;
    volatile uint32_t events_sent;
    volatile uint32_t usb_drops;
    volatile uint32_t cache_mismatches;
    volatile uint32_t events_decoded;
    volatile uint32_t last_fb_pixel_count;
    volatile uint32_t events_dropped;       // ring overflow events (step 5)
    volatile uint32_t last_ring_fill;       // high-water mark over the run
    volatile uint32_t window_truncated_total;
    volatile uint32_t csi_dma_underruns;
    volatile uint32_t last_window_event_count;
    volatile uint32_t last_window_us;
    volatile uint32_t sequence;
} evtstream_state;

bool evtstream_is_running(void) {
    return evtstream_state.running;
}

// Build a synthetic packet (header + counter-pattern events) into `buf`
// and ship it via tinyusb CDC. Drops the entire packet (no partial
// writes) if the FIFO can't accept all of it. Runs in PIT IRQ context.
//
// Cache management: `buf` lives in DRAM (fb_alloc'd) but tinyusb's
// `tud_cdc_write` does a CPU memcpy from `buf` into its internal FIFO.
// Both reader and writer are the M7 going through cache, so cache is
// self-coherent — no SCB_CleanDCache_by_Addr needed in this path.
// (DESIGN.md §3a's "cache-clean before USB submit" applies only if we
// ever bypass tinyusb and feed our buffer directly to USB EHCI DMA;
// not the case here.)
static void bench_build_and_ship(uint8_t *buf,
                                 uint32_t window_start_us,
                                 uint32_t sequence) {
    const uint32_t n_events = evtstream_state.bench_events_per_window;
    const uint32_t pkt_size = evtstream_state.bench_packet_size_bytes;

    evt_packet_header_t *hdr = (evt_packet_header_t *) buf;
    hdr->magic = EVT_PACKET_MAGIC;
    hdr->window_start_us = window_start_us;
    hdr->window_duration_us = (uint16_t) evtstream_state.window_us;
    hdr->event_count = (uint16_t) n_events;
    hdr->flags = 0;
    hdr->reserved = 0;
    hdr->sequence = sequence;

    // Synthetic event pattern: t_us walks 0..window_us-1, x/y rotate
    // through the 320×320 frame, polarity alternates. Exact pattern
    // doesn't matter for this benchmark — we only care about wire
    // throughput. Keep it cheap to fill so the ISR runtime is dominated
    // by tud_cdc_write's memcpy, not by our pattern generation.
    evt_wire_event_t *events = (evt_wire_event_t *) (buf + sizeof(*hdr));
    const uint32_t base = evtstream_state.bench_pattern_counter;
    for (uint32_t i = 0; i < n_events; i++) {
        events[i].t_us = (uint16_t) (i % 1000);
        events[i].x = (uint16_t) ((base + i) & 0x1FF);          // 0..511 cycle
        events[i].y = (uint16_t) (((base + i) >> 9) & 0x1FF);   // upper 9 bits
        events[i].polarity = (uint8_t) (i & 1);
        events[i].flags = 0;
    }
    evtstream_state.bench_pattern_counter = base + n_events;

    // Atomic-or-skip: if FIFO won't accept the whole packet right now,
    // drop the window. Don't write partial packets.
    if (tud_cdc_write_available() < pkt_size) {
        evtstream_state.usb_drops++;
        return;
    }
    uint32_t wrote = tud_cdc_write(buf, pkt_size);
    if (wrote != pkt_size) {
        // Should be unreachable since we just checked write_available,
        // but guard anyway. A short write here means the FIFO got
        // smaller between check and write (impossible without a peer
        // writer — and there isn't one in this design).
        evtstream_state.usb_drops++;
        return;
    }
    tud_cdc_write_flush();
    evtstream_state.events_sent += n_events;
    evtstream_state.last_window_event_count = n_events;
}

// CSI ISR decoder for step 4. Called from omv_csi_line_callback's
// streaming-mode short-circuit (ports/mimxrt/omv_csi.c) once per FB-done
// interrupt. By the time we get here:
//   * The full FB-bytes worth of EVT2.0 words have been DMAed by the
//     CSI peripheral into `fb_addr` (one of the two ping-pong buffers).
//   * The port-side hook has already done __DSB() / SCB_InvalidateDCache /
//     __DSB() / __ISB() so the cache lines covering fb_addr are fresh
//     reads from physical DRAM.
//
// This step counts pixel events only (TD_LOW + TD_HIGH) and discards
// the data. Step 5 will replace this with a ring-buffer writer that
// produces wire-format events for the PIT drainer to ship.
static void bench_csi_decode_cb(uint8_t *fb_addr, void *arg) {
    (void) arg;
    const uint32_t *words = (const uint32_t *) fb_addr;
    const uint32_t n_words =
        evtstream_state.bench_csi_fb_size_bytes / sizeof(uint32_t);

    uint32_t pix = 0;
    for (uint32_t i = 0; i < n_words; i++) {
        const uint32_t v = words[i];
        const uint32_t type = __EVT20_TYPE(v);
        if (type == TD_LOW || type == TD_HIGH) {
            pix++;
        }
        // EV_TIME_HIGH and EXT_TRIGGER words are present in the stream
        // but not counted here. Step 5 tracks the EV_TIME_HIGH 64-bit
        // accumulator to anchor wire-event timestamps; for step 4 the
        // pixel count is the only signal we need to confirm the
        // CSI -> decode path works end-to-end.
    }
    evtstream_state.events_decoded += pix;
    evtstream_state.last_fb_pixel_count = pix;
}

// Production CSI ISR decoder for step 5. Same context as
// bench_csi_decode_cb but writes into the SPSC event ring instead of
// counting-and-discarding.
//
// Timestamping: a single mp_hal_ticks_us() snapshot at FB-done is used
// as the abs_ts for ALL events in this FB. Sub-FB temporal resolution
// (events spanning ~2 ms within a saturated h=6 FB) is lost in step 5;
// step 6/production-correctness can replace this with a sensor-clock
// to MCU-clock mapping using the EVT2.0 EV_TIME_HIGH stream. The
// stored t_us is abs_ts_low16; the PIT drainer converts to window-
// relative t_us at pack time.
//
// Ring discipline: SPSC with monotonic 32-bit head/tail counters.
// `head - tail` is current fill (uint32 modular subtraction is correct
// across the 71-min wrap). Index = head & EVTSTREAM_RING_MASK. CSI is
// the higher-priority IRQ on this port, so it can preempt PIT but not
// vice-versa -- the producer side of the SPSC only needs a __DMB
// before publishing the new head.
static void stream_csi_decode_cb(uint8_t *fb_addr, void *arg) {
    (void) arg;
    evtstream_state.csi_isrs_since_last_pit++;

    const uint32_t fb_ts_us = (uint32_t) mp_hal_ticks_us();
    const uint16_t fb_ts_low16 = (uint16_t) fb_ts_us;
    const uint32_t *words = (const uint32_t *) fb_addr;
    const uint32_t n_words = EVTSTREAM_FB_BYTES / sizeof(uint32_t);
    evt_wire_event_t *ring = (evt_wire_event_t *) evtstream_state.stream_ring;

    uint32_t head = evtstream_state.ring_head;
    const uint32_t tail = evtstream_state.ring_tail;
    uint32_t dropped = 0;
    uint32_t pix = 0;

    for (uint32_t i = 0; i < n_words; i++) {
        const uint32_t v = words[i];
        const uint32_t type = __EVT20_TYPE(v);
        if (type != TD_LOW && type != TD_HIGH) {
            // EV_TIME_HIGH / EXT_TRIGGER / padding -- skipped in step 5.
            // Step 6/production may track EV_TIME_HIGH for sensor-clock
            // mapping; EXT_TRIGGER is deferred per DESIGN.md §5.
            continue;
        }

        if ((head - tail) >= EVTSTREAM_RING_CAP) {
            // Ring full -- drop event, count it, but keep walking the FB
            // so we don't leave decoded EV_TIME_HIGH state stale.
            dropped++;
            continue;
        }

        const uint32_t x = __EVT20_X(v);
        const uint32_t y = __EVT20_Y(v);
        if (x >= 320 || y >= 320) {
            // Coordinate sanity check; keep counting in pix to make the
            // decoded-vs-dropped balance consistent, but skip the ring
            // write so Jetson never sees out-of-range coords.
            continue;
        }

        evt_wire_event_t *slot = &ring[head & EVTSTREAM_RING_MASK];
        slot->t_us = fb_ts_low16;
        slot->x = (uint16_t) x;
        slot->y = (uint16_t) y;
        slot->polarity = (uint8_t) (type & 1u);  // TD_LOW=0=OFF, TD_HIGH=1=ON
        slot->flags = 0;
        head++;
        pix++;
    }

    evtstream_state.events_decoded += pix;
    evtstream_state.last_fb_pixel_count = pix;
    if (dropped > 0) {
        evtstream_state.events_dropped += dropped;
    }
    __DMB();
    evtstream_state.ring_head = head;  // single 32-bit publish
}

// Production PIT drain helper for step 5. Runs in PIT IRQ context.
// Drains up to max_events_per_window from the ring into the next
// ping-pong TX buffer, packs the production header + payload, and
// hands the buffer to tinyusb CDC with the same atomic-or-skip
// semantics as bench(): if the FIFO can't hold the whole packet,
// drop and count usb_drops -- never partial writes.
//
// Heartbeat: even if the ring is empty, we still ship a header-only
// packet (event_count = 0) so the Jetson sees a steady cadence and
// can distinguish "static scene" from "USB stalled".
//
// CSI underrun: if no CSI ISR has fired since the previous PIT drain,
// csi_dma_underruns++. At idle this fires every PIT window (sensor
// genuinely produces no FBs); user reads the metric in context.
static void stream_pit_drain_and_ship(uint32_t window_start_us, uint32_t sequence) {
    // Pick the inactive ping-pong TX buffer (ping-pong on each PIT
    // fire so the previous window's packet, possibly still draining
    // through tinyusb's FIFO, doesn't get clobbered by this fill).
    uint8_t *buf = (evtstream_state.stream_cur_tx_buf_idx == 0)
                   ? evtstream_state.stream_tx_a
                   : evtstream_state.stream_tx_b;
    evtstream_state.stream_cur_tx_buf_idx ^= 1;

    // CSI underrun bookkeeping: read-and-clear the per-PIT-window CSI
    // activity counter.
    if (evtstream_state.csi_isrs_since_last_pit == 0) {
        evtstream_state.csi_dma_underruns++;
    }
    evtstream_state.csi_isrs_since_last_pit = 0;

    // Drain plan: snapshot ring head once (CSI is higher priority and
    // may have preempted us right before this read; that's fine, we
    // just see a stale-but-consistent snapshot).
    const uint32_t head = evtstream_state.ring_head;
    const uint32_t tail = evtstream_state.ring_tail;
    const uint32_t available = head - tail;
    const uint32_t max = evtstream_state.max_events_per_window;
    const uint32_t n_drain = (available > max) ? max : available;
    const bool truncated = (available > max);

    if (available > evtstream_state.last_ring_fill) {
        evtstream_state.last_ring_fill = available;
    }

    // Build header.
    evt_packet_header_t *hdr = (evt_packet_header_t *) buf;
    hdr->magic = EVT_PACKET_MAGIC;
    hdr->window_start_us = window_start_us;
    hdr->window_duration_us = (uint16_t) evtstream_state.window_us;
    hdr->event_count = (uint16_t) n_drain;
    hdr->flags = truncated ? EVT_FLAG_TRUNCATED : 0;
    hdr->reserved = 0;
    hdr->sequence = sequence;

    // Pack payload, converting ring's abs_ts_low16 into window-relative
    // t_us via modular subtraction. Modular wrap of uint16 is harmless
    // for events in the current window (delta < window_us << 65536).
    const uint16_t window_low16 = (uint16_t) window_start_us;
    const evt_wire_event_t *ring = (const evt_wire_event_t *) evtstream_state.stream_ring;
    evt_wire_event_t *out = (evt_wire_event_t *) (buf + sizeof(*hdr));
    for (uint32_t i = 0; i < n_drain; i++) {
        const evt_wire_event_t *src = &ring[(tail + i) & EVTSTREAM_RING_MASK];
        out[i].t_us = (uint16_t)(src->t_us - window_low16);
        out[i].x = src->x;
        out[i].y = src->y;
        out[i].polarity = src->polarity;
        out[i].flags = src->flags;
    }

    __DMB();
    evtstream_state.ring_tail = tail + n_drain;  // single 32-bit publish

    if (truncated) {
        evtstream_state.window_truncated_total++;
    }

    // Atomic-or-skip USB submit (DESIGN.md §5).
    const uint32_t pkt_size = sizeof(evt_packet_header_t)
                            + n_drain * sizeof(evt_wire_event_t);
    if (tud_cdc_write_available() < pkt_size) {
        evtstream_state.usb_drops++;
        return;
    }
    if (tud_cdc_write(buf, pkt_size) != pkt_size) {
        evtstream_state.usb_drops++;
        return;
    }
    tud_cdc_write_flush();
    evtstream_state.events_sent += n_drain;
    evtstream_state.last_window_event_count = n_drain;
}

// Run one window's worth of the cache-management stress test (step 3a).
// Pattern:
//   1. Write pattern A into mock_fb (CPU writes, lands in cache).
//   2. SCB_CleanDCache_by_Addr — push A from cache to physical DRAM.
//   3. Write pattern B over the same buffer (B in cache; DRAM still has A).
//   4. SCB_InvalidateDCache_by_Addr — discard cache lines (B is lost;
//      DRAM retains A).
//   5. Read back via a `volatile` pointer so the compiler can't keep
//      pattern-B values in registers — must hit DRAM, must equal A.
// If the SCB sequence is wrong (e.g. clean is broken, lines remain dirty;
// or invalidate is broken, returns stale cache), the read-back differs
// from A and we count the window as a mismatch.
//
// Memory ordering: CMSIS's SCB_*DCache_by_Addr already includes
// __DSB() at start and __DSB(); __ISB(); at end. We add explicit C-
// level barriers around them anyway — both as belt-and-suspenders
// against any compiler reordering across the inline boundary, and to
// make the ordering self-documenting at the call site. The first
// hardware run of step 3a returned 5-8% intermittent mismatches which
// disappeared once these were added; whether the cause was a real
// missing barrier or buffer misalignment (mock_fb was originally
// 4-byte aligned via FB_ALLOC_NO_HINT), the fix is both: now
// FB_ALLOC_CACHE_ALIGN'd at allocation AND explicitly fenced here.
//
// This is the testable direction (read after invalidate). The TX
// direction (clean before USB DMA) is best-effort here per DESIGN.md
// §11a — real verification is at step 5 integration when USB EHCI DMA
// reads the cache-bypass path.
static void bench_cache_iterate(void) {
    uint32_t *buf = (uint32_t *) evtstream_state.bench_cache_mock_fb;
    const uint32_t n = evtstream_state.bench_cache_mock_fb_words;
    const uint32_t base = evtstream_state.bench_cache_pattern_counter++;

    // Pattern A. Multiplying by the golden-ratio constant 0x9E3779B1
    // gives a cheap per-window-unique seed so we don't accidentally
    // match the previous window's contents.
    const uint32_t a_seed = base * 0x9E3779B1u;
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = a_seed + i;
    }

    __DSB();  // ensure all pattern-A stores are visible to the cache controller
    SCB_CleanDCache_by_Addr((uint32_t *) buf, (int32_t) (n * sizeof(uint32_t)));
    __DSB();  // wait for clean writebacks to reach DRAM

    // Pattern B over the same buffer. Now in cache (modified); DRAM
    // still holds pattern A from the clean above.
    const uint32_t b_seed = a_seed ^ 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = b_seed + i;
    }

    __DSB();  // ensure pattern-B stores are visible BEFORE we invalidate
    SCB_InvalidateDCache_by_Addr((uint32_t *) buf, (int32_t) (n * sizeof(uint32_t)));
    __DSB();  // wait for invalidate ops to complete
    __ISB();  // flush pipeline so any speculatively-prefetched stale data
              // is discarded before the read-back

    // Read back. `volatile` defeats compiler caching of register values
    // from the pattern-B write loop above; without it the compiler is
    // permitted to skip the load and use the in-register pattern-B
    // value, which would falsely PASS even with a broken invalidate.
    volatile uint32_t *vbuf = (volatile uint32_t *) buf;
    for (uint32_t i = 0; i < n; i++) {
        if (vbuf[i] != a_seed + i) {
            evtstream_state.cache_mismatches++;
            break;  // count per-window, not per-element
        }
    }
}

// PIT IRQ handler. Overrides the NXP SDK weak default. Runs in IRQ
// context at NVIC priority EVTSTREAM_PIT_NVIC_PRIO. In step 2 the body
// is bounded (a few register accesses + counter increments). Step 3
// adds a packet build + tud_cdc_write call. Step 3a's cache stress test
// at h=6 / 6 KB does ~30 µs of work per fire (3 % CPU at 1 kHz).
void PIT_IRQHandler(void) {
    if (PIT_GetStatusFlags(PIT, EVTSTREAM_PIT_CHANNEL) & kPIT_TimerFlag) {
        PIT_ClearStatusFlags(PIT, EVTSTREAM_PIT_CHANNEL, kPIT_TimerFlag);

        uint32_t now = (uint32_t) mp_hal_ticks_us();
        if (!evtstream_state.first_fire) {
            // uint32 modular subtraction is correct across the GPT's
            // ~71.5 min wrap.
            evtstream_state.last_window_us =
                now - evtstream_state.prev_ticks_us;
        } else {
            evtstream_state.first_fire = false;
        }
        evtstream_state.prev_ticks_us = now;

        evtstream_state.windows_sent++;
        evtstream_state.sequence++;

        if (evtstream_state.bench_mode) {
            // Pick the inactive TX buffer for this window; the previous
            // window's buffer may still be draining via USB. Ping-pong
            // index toggles 0↔1.
            uint8_t *buf = (evtstream_state.bench_cur_tx_buf_idx == 0)
                           ? evtstream_state.bench_tx_buf_a
                           : evtstream_state.bench_tx_buf_b;
            evtstream_state.bench_cur_tx_buf_idx ^= 1;
            bench_build_and_ship(buf, now, evtstream_state.sequence);
        } else if (evtstream_state.bench_cache_mode) {
            bench_cache_iterate();
        } else if (evtstream_state.stream_mode) {
            stream_pit_drain_and_ship(now, evtstream_state.sequence);
        }
    }

    // ARM errata 838869 — Cortex-M7 store-immediate / IRQ-return overlap
    // can vector to the wrong IRQ. Same pattern as CSI_IRQHandler in
    // ports/mimxrt/mimxrt_hal.c.
    #if defined(__CORTEX_M) && (__CORTEX_M >= 4U)
    __DSB();
    #endif
}

// Shared helpers for start() / bench() / stop() ------------------------

// Resets counters that the PIT or CSI ISR touches. Other counters
// (e.g. ones that production code adds in later steps) stay where
// they are.
static void evtstream_reset_isr_counters(void) {
    evtstream_state.windows_sent = 0;
    evtstream_state.events_sent = 0;
    evtstream_state.usb_drops = 0;
    evtstream_state.cache_mismatches = 0;
    evtstream_state.events_decoded = 0;
    evtstream_state.last_fb_pixel_count = 0;
    evtstream_state.events_dropped = 0;
    evtstream_state.last_ring_fill = 0;
    evtstream_state.window_truncated_total = 0;
    evtstream_state.csi_dma_underruns = 0;
    evtstream_state.csi_isrs_since_last_pit = 0;
    evtstream_state.last_window_event_count = 0;
    evtstream_state.sequence = 0;
    evtstream_state.last_window_us = 0;
    evtstream_state.prev_ticks_us = 0;
    evtstream_state.first_fire = true;
    evtstream_state.ring_head = 0;
    evtstream_state.ring_tail = 0;
}

// Configures and starts PIT at `window_us` cadence. Returns 0 on success
// or -1 if the period overflows uint32. Callers set `running = true`
// AFTER this returns successfully (and any per-mode buffer pointers
// before).
static int evtstream_arm_pit(uint32_t window_us) {
    // PIT clock is the peripheral clock (kCLOCK_PerClk) on i.MX RT1062 —
    // typically 75 MHz. Query at runtime; don't bake in a frequency.
    uint32_t pit_clk_hz = CLOCK_GetFreq(kCLOCK_PerClk);
    // (count + 1) ticks elapse between fires, so subtract 1 to land
    // exactly on window_us.
    uint64_t period64 =
        ((uint64_t) pit_clk_hz * (uint64_t) window_us) / 1000000ULL;
    if (period64 == 0 || period64 > 0xFFFFFFFFULL) {
        return -1;
    }
    uint32_t period = (uint32_t) (period64 - 1);

    pit_config_t cfg;
    PIT_GetDefaultConfig(&cfg);
    PIT_Init(PIT, &cfg);
    PIT_SetTimerPeriod(PIT, EVTSTREAM_PIT_CHANNEL, period);
    PIT_EnableInterrupts(PIT, EVTSTREAM_PIT_CHANNEL,
                         kPIT_TimerInterruptEnable);

    NVIC_ClearPendingIRQ(PIT_IRQn);
    NVIC_SetPriority(PIT_IRQn,
                     NVIC_EncodePriority(NVIC_PRIORITYGROUP_4,
                                         EVTSTREAM_PIT_NVIC_PRIO, 0));
    NVIC_EnableIRQ(PIT_IRQn);

    __DSB();
    PIT_StartTimer(PIT, EVTSTREAM_PIT_CHANNEL);
    return 0;
}

static void evtstream_disarm_pit(void) {
    PIT_StopTimer(PIT, EVTSTREAM_PIT_CHANNEL);
    PIT_DisableInterrupts(PIT, EVTSTREAM_PIT_CHANNEL,
                          kPIT_TimerInterruptEnable);
    NVIC_DisableIRQ(PIT_IRQn);
    NVIC_ClearPendingIRQ(PIT_IRQn);
    PIT_ClearStatusFlags(PIT, EVTSTREAM_PIT_CHANNEL, kPIT_TimerFlag);
}

// Public API ------------------------------------------------------------

// evtstream.start(window_us=1000, max_events_per_window=480)
//
// Production entry point. Allocates the four DRAM working buffers
// (DESIGN.md §3a), configures the GenX320 already-set-up-by-caller in
// continuous ping-pong DMA at h=6, and arms PIT at window_us cadence.
// CSI ISR decodes EVT2.0 events into the SPSC ring; PIT ISR drains the
// ring into a wire-format packet and ships via tinyusb CDC.
//
// The default max_events_per_window=480 leaves comfortable margin
// inside the 4 KB CDC TX FIFO (480*8 + 20 = 3860 bytes). Up to 512 is
// allowed per DESIGN.md §7's revised ceiling, but at the FIFO cliff
// (4116 bytes for 512 events) usb_drops will be near 100% under load.
//
// Caller must initialise the GenX320 in EVENT mode beforehand:
//
//     csi0 = csi.CSI(cid=csi.GENX320)
//     csi0.reset()
//     csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, 4096)
//     evtstream.start(window_us=1000)
//     time.sleep(N)
//     evtstream.stop()
//
// After stop(), the sensor is in factory-default state (HISTO mode for
// GenX320). To start another EVENT-mode session on the same csi0
// handle, re-issue IOCTL_GENX320_SET_MODE before calling start() again.
// This is intentional -- stop() hard-resets the sensor (CSI reset pin
// toggle) to recover from in-progress CPI state that streaming can
// leave in a stuck state. See the long comment in stop() for details.
static mp_obj_t py_evtstream_start(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_window_us, ARG_max_events_per_window };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_window_us,             MP_ARG_INT, {.u_int = 1000} },
        { MP_QSTR_max_events_per_window, MP_ARG_INT, {.u_int = 480} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int window_us = args[ARG_window_us].u_int;
    int max_events = args[ARG_max_events_per_window].u_int;

    // Range checks per DESIGN.md §7 (revised after step-3 results).
    if (window_us < 100 || window_us > 10000) {
        mp_raise_ValueError(MP_ERROR_TEXT("window_us must be in [100, 10000]"));
    }
    if (max_events < 64 || max_events > 512) {
        mp_raise_ValueError(MP_ERROR_TEXT("max_events_per_window must be in [64, 512]"));
    }
    if (evtstream_state.running) {
        mp_raise_OSError(MP_EBUSY);
    }

    omv_csi_t *csi = omv_csi_get(-1);
    if (csi == NULL) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("no CSI configured -- call csi.CSI(cid=csi.GENX320) first"));
    }
    if (csi->chip_id != GENX320_ID_ES && csi->chip_id != GENX320_ID_MP) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("attached sensor is not a GenX320"));
    }

    // Allocate the production buffer set per DESIGN.md §3a final
    // placement table. fb_alloc / fb_free are LIFO -- stop() frees in
    // reverse order: fb2 -> fb1 -> tx_b -> tx_a -> ring.
    //
    // All buffers CACHE_ALIGNed (32-byte) per the step-3a finding;
    // unaligned cache-managed buffers caused intermittent races.
    uint8_t *ring = fb_alloc(EVTSTREAM_RING_BYTES, FB_ALLOC_CACHE_ALIGN);
    uint8_t *tx_a = fb_alloc(EVTSTREAM_TX_BYTES,   FB_ALLOC_CACHE_ALIGN);
    uint8_t *tx_b = fb_alloc(EVTSTREAM_TX_BYTES,   FB_ALLOC_CACHE_ALIGN);
    uint8_t *fb1  = fb_alloc(EVTSTREAM_FB_BYTES,   FB_ALLOC_CACHE_ALIGN);
    uint8_t *fb2  = fb_alloc(EVTSTREAM_FB_BYTES,   FB_ALLOC_CACHE_ALIGN);

    evtstream_state.window_us = (uint32_t) window_us;
    evtstream_state.max_events_per_window = (uint32_t) max_events;
    evtstream_state.stream_ring = ring;
    evtstream_state.stream_tx_a = tx_a;
    evtstream_state.stream_tx_b = tx_b;
    evtstream_state.stream_fb1 = fb1;
    evtstream_state.stream_fb2 = fb2;
    evtstream_state.stream_cur_tx_buf_idx = 0;

    evtstream_reset_isr_counters();

    evtstream_state.running = true;
    evtstream_state.stream_mode = true;
    __DSB();

    // Bring up CSI ping-pong DMA first; the decoder cb begins firing
    // as soon as the first FB completes. Failure paths free buffers
    // in reverse-allocation order.
    int rc = imx_csi_streaming_start(csi, fb1, fb2,
                                     (uint16_t) EVTSTREAM_DMA_LINE_BYTES,
                                     (uint16_t) EVTSTREAM_FB_HEIGHT_LINES,
                                     stream_csi_decode_cb, NULL);
    if (rc != 0) {
        evtstream_state.stream_mode = false;
        evtstream_state.running = false;
        fb_free(); fb_free(); fb_free(); fb_free(); fb_free();
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("imx_csi_streaming_start failed"));
    }

    if (evtstream_arm_pit((uint32_t) window_us) < 0) {
        imx_csi_streaming_stop(csi);
        evtstream_state.stream_mode = false;
        evtstream_state.running = false;
        fb_free(); fb_free(); fb_free(); fb_free(); fb_free();
        mp_raise_ValueError(MP_ERROR_TEXT("PIT period out of range"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_start_obj, 0, py_evtstream_start);

// evtstream.bench(events_per_window=128, window_us=1000)
//
// Synthetic CDC throughput benchmark. PIT fires at window_us cadence;
// each fire builds a production-format packet (header + N synthetic
// events) into one of two ping-pong DRAM TX buffers and ships via
// `tud_cdc_write` / `tud_cdc_write_flush`. Drops the entire packet (no
// partial writes) if the tinyusb CDC TX FIFO can't hold the whole packet
// at the moment of submit.
//
// `events_sent`, `usb_drops`, and `last_window_event_count` track the
// outcome via stats(). The benchmark answers "what packet size can the
// CDC FIFO sustain at this cadence" empirically.
static mp_obj_t py_evtstream_bench(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_events_per_window, ARG_window_us };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_events_per_window, MP_ARG_INT, {.u_int = 128} },
        { MP_QSTR_window_us,         MP_ARG_INT, {.u_int = 1000} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int events_per_window = args[ARG_events_per_window].u_int;
    int window_us = args[ARG_window_us].u_int;

    if (window_us < 100 || window_us > 10000) {
        mp_raise_ValueError(MP_ERROR_TEXT("window_us must be in [100, 10000]"));
    }
    if (events_per_window < 1 || events_per_window > 4096) {
        mp_raise_ValueError(MP_ERROR_TEXT("events_per_window must be in [1, 4096]"));
    }
    if (evtstream_state.running) {
        mp_raise_OSError(MP_EBUSY);
    }

    uint32_t pkt_size = sizeof(evt_packet_header_t)
                      + (uint32_t) events_per_window * sizeof(evt_wire_event_t);

    // Allocate the two ping-pong TX buffers. fb_alloc / fb_free are LIFO,
    // so stop() frees in reverse order (B then A). Buffers live in DRAM
    // (OMV_FB_MEMORY = DRAM); cache-clean is unnecessary because tinyusb
    // memcpy's our bytes through the cache (writer and reader are both
    // the M7 — see DESIGN.md §3a TX direction note).
    uint8_t *tx_a = fb_alloc(pkt_size, FB_ALLOC_NO_HINT);
    uint8_t *tx_b = fb_alloc(pkt_size, FB_ALLOC_NO_HINT);

    evtstream_state.window_us = (uint32_t) window_us;
    evtstream_state.max_events_per_window = (uint32_t) events_per_window;
    evtstream_state.bench_events_per_window = (uint32_t) events_per_window;
    evtstream_state.bench_packet_size_bytes = pkt_size;
    evtstream_state.bench_tx_buf_a = tx_a;
    evtstream_state.bench_tx_buf_b = tx_b;
    evtstream_state.bench_cur_tx_buf_idx = 0;
    evtstream_state.bench_pattern_counter = 0;

    evtstream_reset_isr_counters();

    evtstream_state.running = true;
    evtstream_state.bench_mode = true;
    __DSB();
    if (evtstream_arm_pit((uint32_t) window_us) < 0) {
        evtstream_state.bench_mode = false;
        evtstream_state.running = false;
        fb_free();  // tx_b
        fb_free();  // tx_a
        mp_raise_ValueError(MP_ERROR_TEXT("PIT period out of range"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_bench_obj, 0, py_evtstream_bench);

// evtstream.bench_cache(window_us=1000)
//
// Cache-management stress test (DESIGN.md §11a step 3a). Allocates the
// four DRAM working buffers at production sizes via fb_alloc — ring
// 32 KB, TX A/B 16 KB each, mock FB 6 KB (h=6) — and starts PIT at
// `window_us` cadence. Each PIT fire runs `bench_cache_iterate()` which
// exercises the production write/clean/invalidate/read pattern and
// counts windows that fail the read-back check into
// `stats()['cache_mismatches']`.
//
// PASS criterion (per DESIGN.md §11a): 0 cache_mismatches over a 10 s
// run at 1 kHz. The intended call pattern is:
//
//     evtstream.bench_cache()
//     time.sleep(10)
//     s = evtstream.stats()
//     evtstream.stop()
//     assert s['cache_mismatches'] == 0
//
// Non-blocking like bench(); user controls duration via sleep + stop.
static mp_obj_t py_evtstream_bench_cache(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_window_us };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_window_us, MP_ARG_INT, {.u_int = 1000} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int window_us = args[ARG_window_us].u_int;
    if (window_us < 100 || window_us > 10000) {
        mp_raise_ValueError(MP_ERROR_TEXT("window_us must be in [100, 10000]"));
    }
    if (evtstream_state.running) {
        mp_raise_OSError(MP_EBUSY);
    }

    // Production buffer sizes per DESIGN.md §3a final placement table.
    // fb_alloc / fb_free are LIFO — stop() frees in reverse order.
    //
    // mock_fb uses FB_ALLOC_CACHE_ALIGN so its base address is
    // OMV_CACHE_LINE_SIZE-aligned (32 bytes on M7). Without this,
    // fb_alloc only guarantees 4-byte alignment and the SCB_*DCache_by_Addr
    // calls would extend operations to the cache lines containing the
    // unaligned start/end of mock_fb — which on this allocator share
    // memory with the adjacent tx_b buffer. The first run of this test
    // showed 5-8% intermittent mismatches before this hint was added.
    //
    // Size 6144 = 192 × 32 is already a clean multiple of the cache
    // line, so no padding round-up needed. The other buffers use the
    // default 4-byte alignment because step 3a doesn't exercise their
    // cache patterns; production placement uses CACHE_ALIGN per §3a.
    const uint32_t ring_bytes  = 32u * 1024u;
    const uint32_t tx_bytes    = 16u * 1024u;
    const uint32_t mock_fb_bytes = 6u * 1024u;
    uint8_t *ring    = fb_alloc(ring_bytes, FB_ALLOC_NO_HINT);
    uint8_t *tx_a    = fb_alloc(tx_bytes, FB_ALLOC_NO_HINT);
    uint8_t *tx_b    = fb_alloc(tx_bytes, FB_ALLOC_NO_HINT);
    uint8_t *mock_fb = fb_alloc(mock_fb_bytes, FB_ALLOC_CACHE_ALIGN);

    evtstream_state.window_us = (uint32_t) window_us;
    evtstream_state.bench_cache_ring   = ring;
    evtstream_state.bench_cache_tx_a   = tx_a;
    evtstream_state.bench_cache_tx_b   = tx_b;
    evtstream_state.bench_cache_mock_fb = mock_fb;
    evtstream_state.bench_cache_mock_fb_words = mock_fb_bytes / sizeof(uint32_t);
    evtstream_state.bench_cache_pattern_counter = 0;

    evtstream_reset_isr_counters();

    evtstream_state.running = true;
    evtstream_state.bench_cache_mode = true;
    __DSB();
    if (evtstream_arm_pit((uint32_t) window_us) < 0) {
        evtstream_state.bench_cache_mode = false;
        evtstream_state.running = false;
        fb_free(); fb_free(); fb_free(); fb_free();
        mp_raise_ValueError(MP_ERROR_TEXT("PIT period out of range"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_bench_cache_obj, 0, py_evtstream_bench_cache);

// evtstream.bench_csi(window_us=1000)
//
// CSI integration step 4: configure GenX320 + i.MX CSI for continuous
// ping-pong DMA at h=6 lines (production geometry per DESIGN.md §3a /
// §7a) and run a decode-and-discard loop in CSI ISR context.
// `events_decoded` and `last_fb_pixel_count` track pixel events; the
// raw EVT2.0 data is not buffered. PIT runs at `window_us` cadence as
// a passive heartbeat (windows_sent / sequence / last_window_us stay
// live for visibility).
//
// Caller must have configured the GenX320 in EVENT mode beforehand:
//
//     csi0 = csi.CSI(cid=csi.GENX320)
//     csi0.reset()
//     csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, 4096)
//     evtstream.bench_csi()
//     time.sleep(10)
//     s = evtstream.stats()
//     evtstream.stop()
//
// Step 4 deliberately does not own the sensor mode — the existing
// IOCTL handler takes care of mode setup, and we'd rather inherit that
// path than duplicate it. Step 5 / production may revisit this if
// having start() handle sensor setup turns out to be more ergonomic.
static mp_obj_t py_evtstream_bench_csi(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_window_us };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_window_us, MP_ARG_INT, {.u_int = 1000} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int window_us = args[ARG_window_us].u_int;
    if (window_us < 100 || window_us > 10000) {
        mp_raise_ValueError(MP_ERROR_TEXT("window_us must be in [100, 10000]"));
    }
    if (evtstream_state.running) {
        mp_raise_OSError(MP_EBUSY);
    }

    omv_csi_t *csi = omv_csi_get(-1);
    if (csi == NULL) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("no CSI configured -- call csi.CSI(cid=csi.GENX320) first"));
    }
    if (csi->chip_id != GENX320_ID_ES && csi->chip_id != GENX320_ID_MP) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("attached sensor is not a GenX320"));
    }

    // Production FB geometry per DESIGN.md §7a: h=6 lines × 1024
    // dma_line_bytes = 6 KB per ping-pong buffer. CACHE_ALIGN per the
    // step-3a finding that unaligned buffers caused cache-pattern
    // races.
    const uint16_t dma_line_bytes = 1024;
    const uint16_t height_lines = 6;
    const uint32_t fb_bytes = (uint32_t) dma_line_bytes * (uint32_t) height_lines;

    uint8_t *fb1 = fb_alloc(fb_bytes, FB_ALLOC_CACHE_ALIGN);
    uint8_t *fb2 = fb_alloc(fb_bytes, FB_ALLOC_CACHE_ALIGN);

    evtstream_state.bench_csi_fb1 = fb1;
    evtstream_state.bench_csi_fb2 = fb2;
    evtstream_state.bench_csi_fb_size_bytes = fb_bytes;
    evtstream_state.window_us = (uint32_t) window_us;

    evtstream_reset_isr_counters();

    evtstream_state.running = true;
    evtstream_state.bench_csi_mode = true;
    __DSB();

    int rc = imx_csi_streaming_start(csi, fb1, fb2,
                                     dma_line_bytes, height_lines,
                                     bench_csi_decode_cb, NULL);
    if (rc != 0) {
        evtstream_state.bench_csi_mode = false;
        evtstream_state.running = false;
        fb_free();  // fb2
        fb_free();  // fb1
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("imx_csi_streaming_start failed"));
    }

    // PIT for passive heartbeat. Failure here is recoverable — tear down
    // CSI and free buffers before raising.
    if (evtstream_arm_pit((uint32_t) window_us) < 0) {
        imx_csi_streaming_stop(csi);
        evtstream_state.bench_csi_mode = false;
        evtstream_state.running = false;
        fb_free();  // fb2
        fb_free();  // fb1
        mp_raise_ValueError(MP_ERROR_TEXT("PIT period out of range"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_bench_csi_obj, 0, py_evtstream_bench_csi);

// evtstream.stop()
//
// Disables the PIT timer and its IRQ. Silent no-op if not running.
// Refuses if called from an IRQ — `stop()` does NVIC manipulation that
// must run in thread context. If a `bench()` was active, frees the two
// fb_alloc'd TX buffers.
static mp_obj_t py_evtstream_stop(void) {
    if (__get_IPSR() != 0) {
        mp_raise_OSError(MP_EPERM);
    }
    if (!evtstream_state.running) {
        return mp_const_none;
    }

    // Order matters. Stop the timer first so no new IRQs fire, then mask
    // the NVIC line so any in-flight pending bit is cleared cleanly.
    evtstream_disarm_pit();

    // Clear bench_*_mode before freeing the buffers each points at, so
    // any late-arriving stale ISR (shouldn't happen — disarm above
    // clears pending — but defensive) doesn't dereference freed memory.
    if (evtstream_state.bench_mode) {
        evtstream_state.bench_mode = false;
        __DSB();
        evtstream_state.bench_tx_buf_a = NULL;
        evtstream_state.bench_tx_buf_b = NULL;
        // fb_alloc is LIFO — free in reverse-allocation order.
        fb_free();  // tx_b
        fb_free();  // tx_a
    } else if (evtstream_state.bench_cache_mode) {
        evtstream_state.bench_cache_mode = false;
        __DSB();
        evtstream_state.bench_cache_ring    = NULL;
        evtstream_state.bench_cache_tx_a    = NULL;
        evtstream_state.bench_cache_tx_b    = NULL;
        evtstream_state.bench_cache_mock_fb = NULL;
        // fb_alloc is LIFO — free in reverse-allocation order:
        // mock_fb → tx_b → tx_a → ring.
        fb_free();  // mock_fb
        fb_free();  // tx_b
        fb_free();  // tx_a
        fb_free();  // ring
    } else if (evtstream_state.bench_csi_mode) {
        // Tear down CSI ping-pong DMA before clearing the mode flag, so
        // imx_csi_streaming_stop's __DSB / NVIC sequence completes
        // against the still-active state. Then drop the mode flag and
        // free the FB buffers (LIFO: fb2 -> fb1). Finally hard-reset the
        // sensor (see comment on the stream_mode branch below).
        omv_csi_t *csi = omv_csi_get(-1);
        if (csi != NULL) {
            imx_csi_streaming_stop(csi);
        }
        evtstream_state.bench_csi_mode = false;
        __DSB();
        evtstream_state.bench_csi_fb1 = NULL;
        evtstream_state.bench_csi_fb2 = NULL;
        evtstream_state.bench_csi_fb_size_bytes = 0;
        fb_free();  // fb2
        fb_free();  // fb1
        if (csi != NULL) {
            omv_csi_reset(csi, true);
            mp_hal_delay_ms(EVTSTREAM_RESET_SETTLE_MS);
        }
    } else if (evtstream_state.stream_mode) {
        // Production teardown. PIT is already disarmed above; stop CSI
        // (no more events into ring), then free buffers in reverse-
        // allocation order: fb2 -> fb1 -> tx_b -> tx_a -> ring.
        //
        // Hard-reset the sensor at the end so the user's pre-existing
        // CSI handle (csi0) returns to a clean state. Background:
        //
        // During streaming the GenX320's CPI block keeps emitting events
        // onto the wire that imx_csi_abort just disabled draining of.
        // The sensor's internal CPI FIFO can fill and stall, which on
        // some silicon manifests as the next I2C transaction (e.g.
        // psee_sensor_stop inside IOCTL_GENX320_SET_MODE) hanging long
        // enough to trip the M7's watchdog -- which on RT1062 takes USB
        // OTG down and requires a physical replug to recover.
        // Reproduced deterministically in the step-6 hardware run.
        //
        // omv_csi_reset(csi, true) toggles the CSI reset pin, which
        // power-cycles the sensor regardless of its CPI / I2C state.
        // After this the sensor is in factory default (HISTO mode for
        // GenX320). Callers who want to keep using EVENT mode must
        // re-issue IOCTL_GENX320_SET_MODE -- documented in the public
        // start() docstring.
        //
        // The reset call is a direct C-level invocation; it bypasses
        // py_csi_reset's EBUSY hook (which only fires on the Python
        // wrapper layer). Safe to call here even with the running flag
        // still set.
        //
        // Followed by EVTSTREAM_RESET_SETTLE_MS so the GenX320 finishes
        // its internal post-reset init before the caller's next I2C
        // transaction. Without this, repeated start/stop cycles fail
        // every ~3 iterations with USB disconnect (caught in step-6
        // verification's 5-cycle loop test).
        omv_csi_t *csi = omv_csi_get(-1);
        if (csi != NULL) {
            imx_csi_streaming_stop(csi);
        }
        evtstream_state.stream_mode = false;
        __DSB();
        evtstream_state.stream_fb1 = NULL;
        evtstream_state.stream_fb2 = NULL;
        evtstream_state.stream_tx_a = NULL;
        evtstream_state.stream_tx_b = NULL;
        evtstream_state.stream_ring = NULL;
        fb_free();  // fb2
        fb_free();  // fb1
        fb_free();  // tx_b
        fb_free();  // tx_a
        fb_free();  // ring
        if (csi != NULL) {
            omv_csi_reset(csi, true);
            mp_hal_delay_ms(EVTSTREAM_RESET_SETTLE_MS);
        }
    }

    evtstream_state.running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_evtstream_stop_obj, py_evtstream_stop);

// evtstream.stats()
//
// Returns a snapshot of the counter dict. After step 5:
//   - windows_sent, sequence, last_window_us — live in any mode (PIT-driven)
//   - events_sent, usb_drops, last_window_event_count — live in bench
//     mode and stream mode
//   - cache_mismatches — live in bench_cache mode
//   - events_decoded, last_fb_pixel_count — live in bench_csi mode
//   - events_dropped, last_ring_fill, csi_dma_underruns,
//     window_truncated_total — live in stream mode
static mp_obj_t py_evtstream_stats(void) {
    mp_obj_t d = mp_obj_new_dict(14);
    #define STORE(key, val) \
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_##key), mp_obj_new_int_from_uint(val))
    STORE(windows_sent,             evtstream_state.windows_sent);
    STORE(events_sent,              evtstream_state.events_sent);
    STORE(events_decoded,           evtstream_state.events_decoded);
    STORE(events_dropped,           evtstream_state.events_dropped);
    STORE(usb_drops,                evtstream_state.usb_drops);
    STORE(cache_mismatches,         evtstream_state.cache_mismatches);
    STORE(window_truncated_total,   evtstream_state.window_truncated_total);
    STORE(csi_dma_underruns,        evtstream_state.csi_dma_underruns);
    STORE(last_window_event_count,  evtstream_state.last_window_event_count);
    STORE(last_fb_pixel_count,      evtstream_state.last_fb_pixel_count);
    STORE(last_ring_fill,           evtstream_state.last_ring_fill);
    STORE(last_window_us,           evtstream_state.last_window_us);
    STORE(sequence,                 evtstream_state.sequence);
    #undef STORE
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_evtstream_stats_obj, py_evtstream_stats);

static const mp_rom_map_elem_t evtstream_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_evtstream)            },
    { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&py_evtstream_start_obj)       },
    { MP_ROM_QSTR(MP_QSTR_bench),       MP_ROM_PTR(&py_evtstream_bench_obj)       },
    { MP_ROM_QSTR(MP_QSTR_bench_cache), MP_ROM_PTR(&py_evtstream_bench_cache_obj) },
    { MP_ROM_QSTR(MP_QSTR_bench_csi),   MP_ROM_PTR(&py_evtstream_bench_csi_obj)   },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&py_evtstream_stop_obj)        },
    { MP_ROM_QSTR(MP_QSTR_stats),       MP_ROM_PTR(&py_evtstream_stats_obj)       },
};
static MP_DEFINE_CONST_DICT(evtstream_module_globals, evtstream_module_globals_table);

const mp_obj_module_t evtstream_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &evtstream_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_evtstream, evtstream_module);

#endif // MICROPY_PY_EVTSTREAM
