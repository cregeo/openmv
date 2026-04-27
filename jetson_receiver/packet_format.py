"""evtstream wire format definitions.

These match the OpenMV-side firmware wire format spec'd in DESIGN.md §2:
20-byte header followed by N x 8-byte events. Both halves little-endian
on RT1062 (firmware-side) and aarch64 (Jetson-side); no byte-swapping
is required as long as both sides use struct format strings starting
with '<'.

Keep this module a pure description -- no I/O, no parsing logic. The
receiver imports the constants and dtypes from here so any future wire-
format change has exactly one source of truth.
"""
import struct

import numpy as np


# --- header ---------------------------------------------------------------

EVT_PACKET_MAGIC = 0xE7E7E7E7

# Header flags. Bits 2..15 reserved.
EVT_FLAG_TRUNCATED = 1 << 0  # event_count was clamped to max_events_per_window
EVT_FLAG_USB_RETRY = 1 << 1  # informational; previous window's packet was dropped

# struct format for the 20-byte header. Field order matches DESIGN.md §2.
HEADER_FMT = "<IIHHHHI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 20, "wire-format drift in HEADER_FMT"

HEADER_FIELDS = (
    "magic",
    "window_start_us",
    "window_duration_us",
    "event_count",
    "flags",
    "reserved",
    "sequence",
)


def unpack_header(buf20):
    """Decode a 20-byte header into a dict. buf20 must be exactly 20 bytes."""
    return dict(zip(HEADER_FIELDS, struct.unpack(HEADER_FMT, buf20)))


# --- events ---------------------------------------------------------------

# struct format for a single 8-byte wire event. Same layout as
# evt_wire_event_t in modules/py_evtstream.c.
EVENT_FMT = "<HHHBB"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
assert EVENT_SIZE == 8, "wire-format drift in EVENT_FMT"

# numpy structured dtype matching the wire layout. The receiver decodes
# the payload via np.frombuffer(payload, dtype=EVENT_DTYPE) for speed --
# struct.unpack-per-event is too slow at 480 events/window x 1 kHz.
EVENT_DTYPE = np.dtype([
    ("t_us",     "<u2"),
    ("x",        "<u2"),
    ("y",        "<u2"),
    ("polarity", "u1"),
    ("flags",    "u1"),
])
assert EVENT_DTYPE.itemsize == EVENT_SIZE, "EVENT_DTYPE / EVENT_FMT mismatch"


# --- magic-bytes for resync ---------------------------------------------

# Pre-packed magic bytes for fast .find() in the resync path.
MAGIC_BYTES = struct.pack("<I", EVT_PACKET_MAGIC)
