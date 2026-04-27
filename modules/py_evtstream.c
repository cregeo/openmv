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
 * STEP 1: SKELETON ONLY. start/stop/stats are wired to a `running` flag and
 * a zero-filled stats dict. No CSI, no PIT, no USB, no DMA. The point of
 * this commit is to verify the module loads in MicroPython on RT1060 and
 * that the build wiring (Makefile + board .mk + module table) is correct
 * end-to-end. Subsequent steps fill in real behaviour per DESIGN.md §11a.
 */
#include "omv_boardconfig.h"

#if MICROPY_PY_EVTSTREAM

#include <stdbool.h>
#include <stdint.h>
#include "py/runtime.h"
#include "py/objdict.h"
#include "py/mperrno.h"

// Module state. All zero-initialised in BSS until start() touches it.
// Keep this struct DTCM-resident even in production — it's tiny (~40 B
// here, ~128 B with all the runtime fields added in later steps), so it
// fits in the 1.4 KB free DTCM measured per DESIGN.md §3a.
static struct {
    bool running;
    uint32_t window_us;
    uint32_t max_events_per_window;

    // Stats counters. Production wiring in later steps; for now they
    // stay zero and stats() just reflects whatever start() recorded.
    uint32_t windows_sent;
    uint32_t events_sent;
    uint32_t usb_drops;
    uint32_t window_truncated_total;
    uint32_t ring_lost_events_total;
    uint32_t csi_dma_underruns;
    uint32_t last_window_event_count;
    uint32_t last_window_us;
    uint32_t sequence;
} evtstream_state;

// evtstream.start(window_us=1000, max_events_per_window=2048)
//
// Validates args, takes EBUSY if already running, otherwise records the
// requested parameters and sets the running flag. No hardware interaction
// in this step.
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
    evtstream_state.running = true;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_evtstream_start_obj, 0, py_evtstream_start);

// evtstream.stop()
//
// Clears the running flag. Silent no-op if not running. Refusing-from-IRQ
// (per DESIGN.md §7 stop() semantics) deferred to step 2 when the PIT ISR
// actually exists; in step 1 there are no ISRs that could call this.
static mp_obj_t py_evtstream_stop(void) {
    if (!evtstream_state.running) {
        return mp_const_none;
    }
    evtstream_state.running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_evtstream_stop_obj, py_evtstream_stop);

// evtstream.stats()
//
// Returns a snapshot of the counter dict. In step 1 every value is zero
// except `last_window_us` (which records what start() last accepted) so
// the dict shape is exercised end-to-end without misleading the caller
// into thinking real counters are live.
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
