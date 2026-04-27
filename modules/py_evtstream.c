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
 * STEP 2: PIT-only path. A periodic interval timer (PIT channel 0) ticks
 * at the configured `window_us` cadence. The ISR increments `windows_sent`
 * and `sequence`, and records the inter-fire delta into `last_window_us`
 * via the GPT-backed `mp_hal_ticks_us()`. No CSI, no events, no USB —
 * just verifying the timer infrastructure is solid before we wire any
 * data path to it.
 *
 * Subsequent steps add (per DESIGN.md §11a):
 *   3.  synthetic USB CDC throughput benchmark
 *   3a. cache-management stress test
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

// PIT channel allocated to evtstream. Channels 1-3 stay free for any
// future use (e.g. a second timer for failure-mode detection).
#define EVTSTREAM_PIT_CHANNEL    kPIT_Chnl_0

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

    // Stats counters. windows_sent / sequence / last_window_us start being
    // touched in step 2; the rest stay zero until later steps wire them.
    volatile uint32_t windows_sent;
    uint32_t events_sent;
    uint32_t usb_drops;
    uint32_t window_truncated_total;
    uint32_t ring_lost_events_total;
    uint32_t csi_dma_underruns;
    uint32_t last_window_event_count;
    volatile uint32_t last_window_us;
    volatile uint32_t sequence;
} evtstream_state;

// PIT IRQ handler. Overrides the NXP SDK weak default. Runs in IRQ
// context at NVIC priority EVTSTREAM_PIT_NVIC_PRIO. Must be bounded:
// only loads/stores + a single mp_hal_ticks_us() read (which is itself a
// single 32-bit GPT register read on this port — atomic on Cortex-M7).
//
// At nominal 1 ms cadence the ISR takes a few hundred nanoseconds at
// most, so PIT cadence jitter is dominated by stacked higher-priority
// IRQs (USB, CSI, SysTick) rather than ISR runtime.
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
    }

    // ARM errata 838869 — Cortex-M7 store-immediate / IRQ-return overlap
    // can vector to the wrong IRQ. Same pattern as CSI_IRQHandler in
    // ports/mimxrt/mimxrt_hal.c.
    #if defined(__CORTEX_M) && (__CORTEX_M >= 4U)
    __DSB();
    #endif
}

// evtstream.start(window_us=1000, max_events_per_window=2048)
//
// Validates args, takes EBUSY if already running, then configures and
// starts PIT channel 0 at the requested cadence. Step 2: just the timer;
// CSI / USB are wired in later steps.
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

    // Range checks per DESIGN.md §7 start() semantics.
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

    // Reset stats touched by the ISR. Don't memset the full struct — we
    // want stats() to retain monotonic counters across start/stop cycles
    // until later steps decide otherwise.
    evtstream_state.windows_sent = 0;
    evtstream_state.sequence = 0;
    evtstream_state.last_window_us = 0;
    evtstream_state.prev_ticks_us = 0;
    evtstream_state.first_fire = true;

    // PIT clock source on i.MX RT1062 is the peripheral clock (kCLOCK_PerClk),
    // typically 75 MHz on this board. Query at runtime so we don't bake in
    // a hardcoded value that breaks on a different clock config.
    uint32_t pit_clk_hz = CLOCK_GetFreq(kCLOCK_PerClk);
    // Period in PIT ticks. (count + 1) ticks elapse between fires, so
    // subtract 1 to land exactly on window_us.
    uint64_t period64 =
        ((uint64_t) pit_clk_hz * (uint64_t) window_us) / 1000000ULL;
    if (period64 == 0 || period64 > 0xFFFFFFFFULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("PIT period out of range"));
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

    // Mark running BEFORE starting the timer so the very first IRQ sees
    // a coherent state struct (compiler's `running = true` could otherwise
    // race the first PIT fire on this priority).
    evtstream_state.running = true;
    __DSB();
    PIT_StartTimer(PIT, EVTSTREAM_PIT_CHANNEL);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_start_obj, 0, py_evtstream_start);

// evtstream.stop()
//
// Disables the PIT timer and its IRQ. Silent no-op if not running.
// Refuses if called from an IRQ — `stop()` does NVIC manipulation that
// must run in thread context.
static mp_obj_t py_evtstream_stop(void) {
    if (__get_IPSR() != 0) {
        mp_raise_OSError(MP_EPERM);
    }
    if (!evtstream_state.running) {
        return mp_const_none;
    }

    // Order matters. Stop the timer first so no new IRQs fire, then mask
    // the NVIC line so any in-flight pending bit is cleared cleanly.
    PIT_StopTimer(PIT, EVTSTREAM_PIT_CHANNEL);
    PIT_DisableInterrupts(PIT, EVTSTREAM_PIT_CHANNEL,
                          kPIT_TimerInterruptEnable);
    NVIC_DisableIRQ(PIT_IRQn);
    NVIC_ClearPendingIRQ(PIT_IRQn);
    PIT_ClearStatusFlags(PIT, EVTSTREAM_PIT_CHANNEL, kPIT_TimerFlag);

    evtstream_state.running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_evtstream_stop_obj, py_evtstream_stop);

// evtstream.stats()
//
// Returns a snapshot of the counter dict. In step 2, windows_sent,
// sequence, and last_window_us are live (PIT-driven); the rest stay
// zero until later steps wire them.
static mp_obj_t py_evtstream_stats(void) {
    mp_obj_t d = mp_obj_new_dict(9);
    #define STORE(key, val) \
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_##key), mp_obj_new_int_from_uint(val))
    STORE(windows_sent,             evtstream_state.windows_sent);
    STORE(events_sent,              evtstream_state.events_sent);
    STORE(usb_drops,                evtstream_state.usb_drops);
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
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_evtstream)        },
    { MP_ROM_QSTR(MP_QSTR_start),    MP_ROM_PTR(&py_evtstream_start_obj)   },
    { MP_ROM_QSTR(MP_QSTR_stop),     MP_ROM_PTR(&py_evtstream_stop_obj)    },
    { MP_ROM_QSTR(MP_QSTR_stats),    MP_ROM_PTR(&py_evtstream_stats_obj)   },
};
static MP_DEFINE_CONST_DICT(evtstream_module_globals, evtstream_module_globals_table);

const mp_obj_module_t evtstream_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &evtstream_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_evtstream, evtstream_module);

#endif // MICROPY_PY_EVTSTREAM
