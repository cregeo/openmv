# Exploration Report — OpenMV `master` (commit 999dd99, 2026-03-25)

This is a survey of the existing GenX320 event path in the OpenMV firmware, written
in preparation for the `evtstream` native module (Task 2). Before describing the
code, here are the surprises that affect the original task brief.

## Surprises (read this first)

1. **Repo layout has no `src/` prefix.** The brief references `src/omv/...`. In the
   current `master`, top-level dirs are `boards/`, `common/`, `drivers/`, `lib/`,
   `modules/`, `ports/`, etc. Every path in the brief needs that prefix stripped.

2. **The board target is `OPENMV_RT1060`, not `OPENMV_RT1062`.** RT1060 is the board
   model; the chip on it is the i.MX **RT1062DVJ6A** (defined in
   `boards/OPENMV_RT1060/omv_boardconfig.mk:1`). Build with `TARGET=OPENMV_RT1060`.

3. **`OMV_GENX320_EHC_ENABLE` does not exist.** The only build-time GenX320 flag is
   `OMV_GENX320_ENABLE`. "EHC" appears inside the driver but as register names
   (`EHC_INTEGRATION_PERIOD`, `EHC_DIFF3D_N_BITS_SIZE`) — it is a sensor-internal
   block ("Event Histogram Controller") used by **histogram mode**, not a build
   toggle. See "EHC clarified" below.

4. **GenX320 is already enabled** for `OPENMV_RT1060` in
   `boards/OPENMV_RT1060/omv_boardconfig.mk:14` (`OMV_GENX320_ENABLE=1`). It is
   also enabled for `OPENMV4P` and `OPENMV_N6`.

5. **Build verification is blocked on this Jetson host.** `make sdk` 404s on
   `openmv-sdk-1.1.0-linux-aarch64.tar.gz` — the prebuilt SDK is not published for
   aarch64. The Jetson Orin Nano is aarch64. See "Build status" at the end.

6. **The existing event readout is frame-driven, not stream-driven.** Each call to
   the `READ_EVENTS` IOCTL triggers one full DMA frame via the same
   `omv_csi_snapshot()` path used for normal images, then the post-process
   callback decodes that frame into events. There is no continuous DMA ring; each
   IOCTL is one-shot. This shapes Task 2: "fixed 1 ms cadence" can either come
   from the sensor's EHC output rate, or from a new MCU-side timer + double-
   buffered DMA. The current path, called from a 1 ms timer, would not give
   tight ±50 µs cadence because each call blocks on a hardware EOF interrupt.

## Repository layout

```
openmv-fork/
├── boards/OPENMV_RT1060/      # board config (.h headers, .mk Make flags)
│   ├── omv_boardconfig.h      # peripherals, memory map, pins
│   ├── omv_boardconfig.mk     # build toggles (OMV_GENX320_ENABLE=1)
│   ├── omv_pins.h             # pin assignments
│   └── manifest.py            # frozen Python modules
├── common/                    # cross-port HAL
│   ├── omv_csi.{c,h}          # generic CSI layer + IOCTL enum
│   ├── omv_i2c.h, omv_gpio.h, omv_spi.h
│   └── ...
├── drivers/
│   ├── sensors/genx320.c      # GenX320 driver (the relevant one)
│   ├── sensors/sensor_config.h
│   ├── genx320/               # vendor-supplied init sequences + headers
│   │   ├── include/evt_2_0.h          # EVT2.0 bit-packing macros
│   │   ├── include/psee_genx320.h
│   │   ├── include/genx320_all_pub_registers.h
│   │   └── src/genx320_issd_*.c       # init sequences (cpi_evt, cpi_histo)
│   └── drivers.mk             # OMV_GENX320_ENABLE → CFLAGS, source list
├── modules/
│   ├── py_csi.c               # legacy MicroPython CSI binding
│   ├── py_csi_ng.c            # current "next-gen" CSI binding (use this)
│   └── py_image.c
├── ports/mimxrt/              # i.MX RT1062 port
│   ├── main.c
│   ├── omv_csi.c              # port-specific CSI (DMA setup for the iMX CSI)
│   ├── omv_i2c.c, omv_spi.c, omv_gpio.c
│   ├── mimxrt_hal.{c,h}
│   ├── mimxrt.ld.S            # linker script template
│   ├── omv_mpconfigport.h
│   └── omv_portconfig.{h,mk}
├── lib/
│   ├── imlib/imlib.h          # ec_event_t struct lives here
│   ├── micropython/           # submodule
│   ├── mimxrt/MIMXRT1062/     # NXP HAL/SDK for the chip
│   └── tinyusb/               # submodule
├── docs/firmware.md           # build instructions
├── Makefile
└── README.md
```

## End-to-end event path

The flow from a user's MicroPython script to a decoded event is:

```
MicroPython script
  │
  ├─ csi.ioctl(IOCTL_GENX320_SET_MODE, MODE_EVENT)
  │     └─→ modules/py_csi_ng.c:1221  (py_csi_ioctl, SET_MODE case)
  │           └─→ common/omv_csi.c omv_csi_ioctl()
  │                 └─→ drivers/sensors/genx320.c:478  (ioctl, SET_MODE case)
  │                       └─→ set_active_mode(csi, MODE_EVENT, FRAMESIZE_CUSTOM)
  │                             ├─ swap genx->issd to dcmi_evt init sequence
  │                             ├─ swap csi->post_process to post_process_event
  │                             ├─ psee_sensor_init() → I2C reg writes
  │                             └─ EDF_CONTROL = 0  // EVT2.0 wire format
  │
  └─ count = csi.ioctl(IOCTL_GENX320_READ_EVENTS, events_ndarray)
        └─→ modules/py_csi_ng.c:1229  (py_csi_ioctl, READ_EVENTS case)
              ├─ validates ndarray: dtype uint16, shape (N, 6) where N matches
              │  framebuffer geometry, dense
              └─→ omv_csi_ioctl(csi, READ_EVENTS, array->array)
                    └─→ drivers/sensors/genx320.c:523  (ioctl, READ_EVENTS case)
                          ├─ genx->events = array  // stash pointer for callback
                          └─→ omv_csi_snapshot(csi, &image, 0)
                                ├─ ports/mimxrt/omv_csi.c arms CSI DMA
                                ├─ blocks on CSI EOF interrupt
                                ├─ DMA delivers raw EVT2.0 32-bit words into the
                                │  framebuffer
                                └─→ csi->post_process(csi, image, flags)
                                      = post_process_event()  // genx320.c:635
                                          decodes EVT2.0 words into ec_event_t[],
                                          returns valid_count
        ← count of valid events written into ndarray
```

Key consequences:

- **Each `READ_EVENTS` IOCTL is one full frame's worth of DMA**, blocking on the
  CSI peripheral's EOF interrupt. The cadence of the existing path is therefore
  the sensor's output frame rate (set by `EHC_INTEGRATION_PERIOD` in HISTO mode,
  or by the EVT2.0 packet-time configuration at `set_active_mode` time).
- **Events are decoded by `post_process_event` running in the snapshot return
  path**, not in an ISR. The callback parses 32-bit EVT2.0 words and emits
  `ec_event_t` records for `TD_LOW`/`TD_HIGH` (pixel polarity) and `EXT_TRIGGER`
  events; `EV_TIME_HIGH` words update the high bits of a 64-bit timestamp
  accumulator (`genx->event_time_us`).

## File-by-file summary

### `boards/OPENMV_RT1060/omv_boardconfig.h`

The board's static configuration: chip name (`IMXRT1060`), memory map (DTCM 384 K,
ITCM split 32+32 K, OCRAM 512 K + 64 K, DRAM 32 M, flash 16 M), framebuffer/heap
sizes, and pin definitions. Notable:

- `OMV_CSI_BASE = CSI`, `OMV_CSI_DMA = DMA0`, two DMA channels reserved for CSI
  starting at channel 4 (`OMV_CSI_DMA_CHANNEL_START`, `..._COUNT`).
- `OMV_DMA_MEMORY = DTCM` — small DMA scratch lives in DTCM.
- USB IRQ: `USB_OTG1_IRQn` with EHCI controller. (This is the host of our CDC.)
- Enabled non-GenX sensors: OV5640, OV7725, MT9M114, MT9V0XX, Lepton, PAG7920,
  PAJ6100, FROGEYE2020. The GenX320 enable flag is **not** in this `.h` file —
  it is in the parallel `.mk` file.

### `boards/OPENMV_RT1060/omv_boardconfig.mk`

Build-time toggles consumed by `Makefile`:

```make
MCU=MIMXRT1062DVJ6A
PORT=mimxrt
OMV_GENX320_ENABLE=1
MICROPY_PY_CSI = 1
MICROPY_PY_CSI_NG = 1
MICROPY_PY_ULAB = 1
...
```

Both the legacy `py_csi` and the next-gen `py_csi_ng` modules are compiled in;
ULAB is on (the `READ_EVENTS` IOCTL takes a `ulab.ndarray`).

### `drivers/sensors/genx320.c`

The Prophesee GenX320 driver. ~810 lines. Key elements:

| Symbol | Line | What it does |
|---|---|---|
| `genx_state_t` | 79 | Per-instance state: contrast/brightness, `event_time_us` accumulator, current `issd` (init seq), `mode`, AFK/STC handles, `events` buffer pointer used by the post-process callback. |
| `reset()` | 94 | Forces histogram mode on power-up (`set_active_mode(MODE_HISTO)`). |
| `set_pixformat()` / `set_framesize()` / `set_framerate()` | 149/153/168 | Mode-aware shims. In EVENT mode, `set_framerate` is rejected (`return -1`) — frame rate is meaningless. |
| `ioctl()` | ~300–680 | Dispatches GenX320 IOCTLs (SET_BIASES, SET_BIAS, SET_AFK, SET_STC, SET_MODE, READ_EVENTS, CALIBRATE). |
| `post_process_event()` | 635 | Callback set when in EVENT mode. Walks the 32-bit EVT2.0 words in the framebuffer, emits `ec_event_t` for pixel/trigger events, returns valid count. |
| `post_process_histo()` | (also defined) | Callback for HISTO mode — produces the grayscale histogram image. |
| `set_active_mode()` | 680 | Tears down/rebuilds the sensor for a new mode: stops, switches `issd` (init sequence), swaps the post-process callback, writes `TOP_CHICKEN` (mode override), runs init, sets `EDF_CONTROL=0` for EVT2.0, configures CPI packet sizes. **In HISTO mode** it also configures the EHC block (`psee_ehc_init` + `psee_ehc_activate(EHC_ALGO_DIFF3D, ...)`). |

### EHC clarified

"EHC" = Event Histogram Controller — a Prophesee on-sensor block that integrates
events into a difference image over a time window (`EHC_INTEGRATION_PERIOD`) and
streams the result as a frame. It is **only used in `MODE_HISTO`**. Names found
in the tree:

- `EHC_INTEGRATION_PERIOD`, `EHC_HandleTypeDef`, `psee_ehc_init`,
  `psee_ehc_activate`, `EHC_ALGO_DIFF3D`, `EHC_DIFF3D_N_BITS_SIZE`,
  `EHC_WITHOUT_PADDING` — all in `drivers/sensors/genx320.c` and
  `drivers/genx320/include/`.
- There is **no `OMV_GENX320_EHC_ENABLE`** anywhere. The histogram-vs-event
  decision is per-call via the `OMV_CSI_IOCTL_GENX320_SET_MODE` IOCTL with
  `OMV_CSI_GENX320_MODE_HISTO` or `OMV_CSI_GENX320_MODE_EVENT`.

For Task 2 we need `MODE_EVENT`. The brief's instruction "if your module needs
raw event mode, set it via the sensor driver at `start()` time and restore on
`stop()`" maps to: at `evtstream.start()` issue
`OMV_CSI_IOCTL_GENX320_SET_MODE → MODE_EVENT`, and on `stop()` restore the prior
mode.

### `ports/mimxrt/` (port-specific code)

Lean — only ten files. Of interest:

- `main.c` — port entry point, sets up clocks, MicroPython, USB, CSI.
- `omv_csi.c` — the DMA driver for the i.MX CSI peripheral. It is what
  `omv_csi_snapshot()` ultimately calls. **This is the file Task 2 will need to
  understand for non-blocking, double-buffered streaming**, since today's
  snapshot is blocking on a single buffer.
- `mimxrt_hal.c/h` — clocks, FlexRAM partition (`OMV_FLEXRAM_CONFIG`), pin mux.
- `mimxrt.ld.S` — linker template; `OMV_RAMFUNC_MEMORY = ITCM2` is the
  attribute we'd want for any RAM-resident hot-path code.
- `omv_mpconfigport.h` — MicroPython config (which builtins are compiled in).
- `omv_portconfig.h/mk` — port-level toggles.

### MicroPython CSI bindings (`modules/py_csi.c`, `modules/py_csi_ng.c`)

`py_csi.c` is the legacy `sensor` module; `py_csi_ng.c` is the new `csi`
class-based module. **The brief's `csi.ioctl(IOCTL_GENX320_READ_EVENTS, ndarray)`
matches the next-gen one** at `modules/py_csi_ng.c:1229`. It validates that the
argument is a `ulab.ndarray` of `dtype=uint16` and shape `(N, EC_EVENT_SIZE=6)`
where `N` matches the current framebuffer geometry, then forwards the buffer
pointer through `omv_csi_ioctl()` to the driver.

### `ec_event_t` (`lib/imlib/imlib.h:380`)

```c
typedef struct ec_event {
    uint16_t type;     // EC_PIX_OFF/ON, EC_EXT_TRIGGER_*, EC_RST_TRIGGER_*
    uint16_t ts_s;     // timestamp seconds
    uint16_t ts_ms;    // milliseconds within second (0..999)
    uint16_t ts_us;    // microseconds within ms (0..999)
    uint16_t x;        // 0..319
    uint16_t y;        // 0..319
} ec_event_t;          // 12 bytes
#define EC_EVENT_SIZE 6 // uint16 elements
```

The split-timestamp layout is awkward but lets every field stay 16-bit. Task 2's
on-wire packet should probably **not** keep this shape — the brief specifies
8-byte events. A natural packed form is `(type:1, x:9, y:9, ts_us_within_window:13)`
fitting into 32 bits, plus 32 bits of header/padding, giving 8 bytes per event.
We'll finalize this in DESIGN.md.

## Build status

`make sdk` (which fetches the prebuilt OpenMV toolchain bundle) **fails** on
this Jetson host:

```
$ bash -c "source tools/ci.sh && ci_install_sdk"
Installing OpenMV SDK 1.1.0 to /home/cregeo/openmv-sdk-1.1.0...
--2026-04-25 09:19:44--  https://download.openmv.io/sdk/openmv-sdk-1.1.0-linux-aarch64.tar.gz
HTTP request sent, awaiting response... 404 Not Found
```

The OpenMV SDK 1.1.0 is published for `linux-x86_64` and `darwin-arm64`, but
**not `linux-aarch64`** — the Jetson Orin Nano architecture. The official build
dirs at `https://download.openmv.io/sdk/` confirm this.

Three workarounds, all pending user choice:

1. **Docker build.** `cd docker && make TARGET=OPENMV_RT1060` — uses the
   project's Dockerfile, which itself runs `ci_install_sdk` inside an
   x86_64 container. Requires an x86_64 emulation layer or a different host.
2. **Manual ARM toolchain.** `apt install gcc-arm-none-eabi` then patch the
   Makefile or `PATH` so it finds the system toolchain instead of expecting
   `${SDK_DIR}/make` and `${SDK_DIR}/python/bin`. Lower friction but not the
   blessed path; some build steps may rely on tools bundled in the SDK
   (`mkromfs`, custom `python`, etc.).
3. **Build elsewhere.** Use an x86_64 Linux host or GitHub Actions (the repo's
   `.github/workflows/firmware.yml` does this and uploads firmware artifacts on
   every push to a fork's master).

Submodules are also still uninitialized (the source clone was shallow with
detached submodule state). They need:

```bash
git submodule update --init --depth=1 --no-single-branch
git -C lib/micropython/ submodule update --init --depth=1
```

before any build will succeed.

**I have not modified anything in the tree.** Build verification is unfinished
pending guidance on which workaround to take.

## Status of Task 1 deliverables

| Item | Status |
|---|---|
| Clone repo into `./openmv-fork/` | ✅ Done (locally cloned from existing `/home/cregeo/openmv` checkout, remote re-pointed at `https://github.com/openmv/openmv.git`). |
| Verify RT1062 build | ⏸ Blocked — SDK has no aarch64 prebuilt; awaiting choice of workaround. |
| Read & summarize key files | ✅ See "File-by-file summary" above. |
| Locate `OMV_GENX320_EHC_ENABLE` | ✅ Symbol does not exist; "EHC clarified" explains what it actually maps to. |
| Write `EXPLORATION.md` | ✅ This document. |

Open questions for you before proceeding to Task 2:

1. Which build workaround do you prefer (Docker / system toolchain / x86_64
   host)? The design itself doesn't need a working build, but Task 3 will.
2. The repo root is `openmv-fork/`. New module files for Task 3 will land at
   `modules/py_evtstream.c` (no `src/omv/` prefix in this layout). OK?
3. The 8-byte event format mentioned in the brief implies dropping the
   `ec_event_t` schema for the wire packet. Confirm before DESIGN.md fixes it.
