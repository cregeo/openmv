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
 * STEP 3a: cache-management stress test. `evtstream.bench_cache()`
 * allocates the four DRAM working buffers (ring 32 KB, TX A/B 16 KB
 * each, mock FB 6 KB) at production sizes via fb_alloc and exercises
 * the SCB_CleanDCache_by_Addr / SCB_InvalidateDCache_by_Addr sequence
 * from the PIT ISR at 1 kHz. Per-window mismatches are counted and
 * surfaced via stats()['cache_mismatches']. PASS = zero mismatches over
 * 10 seconds (DESIGN.md §11a step 3a).
 *
 * Steps prior:
 *   1.  skeleton (start/stop/stats flags only)
 *   2.  PIT-only path
 *   3.  synthetic USB CDC throughput benchmark
 *
 * Subsequent steps add (per DESIGN.md §11a):
 *   4.  CSI integration (continuous-mode capture + decode)
 *   5.  full path (PIT drains ring into CDC packets)
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

    // Stats counters. windows_sent / sequence / last_window_us start being
    // touched in step 2; events_sent / usb_drops / last_window_event_count
    // start in step 3; cache_mismatches in step 3a; the rest stay zero
    // until later steps wire them.
    volatile uint32_t windows_sent;
    volatile uint32_t events_sent;
    volatile uint32_t usb_drops;
    volatile uint32_t cache_mismatches;
    uint32_t window_truncated_total;
    uint32_t ring_lost_events_total;
    uint32_t csi_dma_underruns;
    volatile uint32_t last_window_event_count;
    volatile uint32_t last_window_us;
    volatile uint32_t sequence;
} evtstream_state;

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

    SCB_CleanDCache_by_Addr((uint32_t *) buf, (int32_t) (n * sizeof(uint32_t)));

    // Pattern B over the same buffer. Now in cache (modified); DRAM
    // still holds pattern A from the clean above.
    const uint32_t b_seed = a_seed ^ 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = b_seed + i;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *) buf, (int32_t) (n * sizeof(uint32_t)));

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

// Resets counters that the PIT ISR touches. Other counters (e.g. ones
// that production code adds in later steps) stay where they are.
static void evtstream_reset_isr_counters(void) {
    evtstream_state.windows_sent = 0;
    evtstream_state.events_sent = 0;
    evtstream_state.usb_drops = 0;
    evtstream_state.cache_mismatches = 0;
    evtstream_state.last_window_event_count = 0;
    evtstream_state.sequence = 0;
    evtstream_state.last_window_us = 0;
    evtstream_state.prev_ticks_us = 0;
    evtstream_state.first_fire = true;
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

// evtstream.start(window_us=1000, max_events_per_window=2048)
//
// Validates args, takes EBUSY if already running, then configures and
// starts PIT channel 0 at the requested cadence. Step 2: just the timer;
// CSI / USB are wired in later steps. Step 3 keeps start() data-path-free
// — `bench()` is the synthetic-TX entry point.
static mp_obj_t py_evtstream_start(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_window_us, ARG_max_events_per_window };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_window_us,             MP_ARG_INT, {.u_int = 1000} },
        { MP_QSTR_max_events_per_window, MP_ARG_INT, {.u_int = 2048} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int window_us = args[ARG_window_us].u_int;
    int max_events = args[ARG_max_events_per_window].u_int;

    if (window_us < 100 || window_us > 10000) {
        mp_raise_ValueError(MP_ERROR_TEXT("window_us must be in [100, 10000]"));
    }
    if (max_events < 64 || max_events > 4096) {
        mp_raise_ValueError(MP_ERROR_TEXT("max_events_per_window must be in [64, 4096]"));
    }
    if (evtstream_state.running) {
        mp_raise_OSError(MP_EBUSY);
    }

    evtstream_state.window_us = (uint32_t) window_us;
    evtstream_state.max_events_per_window = (uint32_t) max_events;
    evtstream_reset_isr_counters();

    evtstream_state.running = true;  // see arm_pit comment about ordering
    __DSB();
    if (evtstream_arm_pit((uint32_t) window_us) < 0) {
        evtstream_state.running = false;
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
    const uint32_t ring_bytes  = 32u * 1024u;
    const uint32_t tx_bytes    = 16u * 1024u;
    const uint32_t mock_fb_bytes = 6u * 1024u;
    uint8_t *ring    = fb_alloc(ring_bytes, FB_ALLOC_NO_HINT);
    uint8_t *tx_a    = fb_alloc(tx_bytes, FB_ALLOC_NO_HINT);
    uint8_t *tx_b    = fb_alloc(tx_bytes, FB_ALLOC_NO_HINT);
    uint8_t *mock_fb = fb_alloc(mock_fb_bytes, FB_ALLOC_NO_HINT);

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
    }

    evtstream_state.running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_evtstream_stop_obj, py_evtstream_stop);

// evtstream.stats()
//
// Returns a snapshot of the counter dict. After step 3a:
//   - windows_sent, sequence, last_window_us — live in any mode (PIT-driven)
//   - events_sent, usb_drops, last_window_event_count — live in bench mode
//   - cache_mismatches — live in bench_cache mode
//   - window_truncated_total, ring_lost_events_total, csi_dma_underruns —
//     wired in steps 4-7
static mp_obj_t py_evtstream_stats(void) {
    mp_obj_t d = mp_obj_new_dict(10);
    #define STORE(key, val) \
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_##key), mp_obj_new_int_from_uint(val))
    STORE(windows_sent,             evtstream_state.windows_sent);
    STORE(events_sent,              evtstream_state.events_sent);
    STORE(usb_drops,                evtstream_state.usb_drops);
    STORE(cache_mismatches,         evtstream_state.cache_mismatches);
    STORE(window_truncated_total,   evtstream_state.window_truncated_total);
    STORE(ring_lost_events_total,   evtstream_state.ring_lost_events_total);
    STORE(csi_dma_underruns,        evtstream_state.csi_dma_underruns);
    STORE(last_window_event_count,  evtstream_state.last_window_event_count);
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
