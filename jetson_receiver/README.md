# evtstream Jetson-side receiver

Reads the binary event-stream packets emitted by the OpenMV RT1062
firmware (the `evtstream` MicroPython native module on the
`evtstream-task3-validate` branch of the firmware fork) and decodes
them into numpy arrays.

The wire format is fixed by `DESIGN.md §2` on the firmware fork:
20-byte header + N x 8-byte event payload, little-endian throughout.
`packet_format.py` is the single source of truth on the Jetson side
and asserts wire-format constraints at import time.

## Layout

```
jetson_receiver/
|-- packet_format.py         # wire-format struct + numpy dtype
|-- evtstream_receiver.py    # main reader, save + cadence modes
|-- test_cadence.py          # PASS/FAIL gate (task 5)
+-- README.md                # this file
```

Nothing here depends on the OpenMV firmware tree at runtime; the
matching firmware is identified by the wire format alone.

## Install

Tested on a Jetson Orin Nano running L4T 36.x (Ubuntu 22.04, Python
3.10). Works on any Linux host with USB CDC support.

```bash
cd jetson_receiver
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial numpy
```

## USB permissions

The OpenMV camera enumerates as `/dev/ttyACM0` (or higher if other
ACM devices are already attached). One-shot:

```bash
sudo chmod 666 /dev/ttyACM0
```

Or persistent (preferred):

```bash
sudo usermod -a -G dialout $USER
# log out / back in for the group change to take effect
```

## Usage

### Save mode (default)

Records every packet's events to a `.npz` file. Run until Ctrl+C:

```bash
python3 evtstream_receiver.py --save eyetrack_session.npz
```

The saved file contains:

| key | dtype | shape | meaning |
|---|---|---|---|
| `events` | structured (`t_us`/`x`/`y`/`polarity`/`flags`) | (N,) | wire events |
| `abs_us` | int64 | (N,) | absolute MCU microseconds (`window_start_us + t_us`) |
| `bytes_read` | int64 | () | total bytes read from serial |
| `packets` | int64 | () | total packets parsed |
| `resync_count` | int64 | () | number of magic-byte resyncs |

Load:

```python
import numpy as np
data = np.load("eyetrack_session.npz")
events = data["events"]      # structured array
abs_us = data["abs_us"]      # absolute time per event
```

### Cadence-measurement mode

Prints periodic stats on MCU window timing, Jetson arrival timing,
resync events, and sequence gaps. Doesn't save events.

```bash
python3 evtstream_receiver.py --measure-cadence --duration 10
```

The first `--drain-seconds` (default 1.0 s) of received data is
discarded before stats start. This flushes the OS USB stack and the
OpenMV CDC TX FIFO of bytes buffered before the test started --
those carry `window_start_us` values from a much earlier wall-clock
moment and would otherwise poison the inter-arrival distribution.

Inter-arrival samples are only taken across packets whose sequence
numbers are consecutive (`(last_seq + 1) mod 2^32`). Any gap counts
into `sequence_gaps` and the corresponding interval is skipped via
`skipped_intervals` rather than turning the cross-gap delta into a
nonsense sample.

Output ends with a `[CADENCE]` block of `key=value` markers that
are easy to grep. Example:

```
[CADENCE] packets=10000
[CADENCE] events=4527
[CADENCE] sequence_gaps=0
[CADENCE] resync_count=0
[CADENCE] mcu_interval_mean_us=1000.000
[CADENCE] mcu_interval_std_us=4.21
[CADENCE] mcu_interval_p99_us=1024.0
[CADENCE] jetson_interval_mean_us=1000.001
[CADENCE] jetson_interval_std_us=18.7
[CADENCE] jetson_interval_p99_us=1098.0
```

Long-running monitor:

```bash
python3 evtstream_receiver.py --measure-cadence --duration 0 \
    --summary-interval 5
```

### PASS/FAIL gate (task 5)

Wraps the cadence-mode receiver and asserts the original task-1
cadence target: MCU window inter-arrival std <= 50 us, resync_count
== 0 over a 10-second run.

```bash
python3 test_cadence.py
```

Exit code 0 = PASS, 1 = FAIL. The full receiver output is echoed so
a failing run shows where the numbers diverged from the budget.

Tunable thresholds:

```bash
python3 test_cadence.py --max-mcu-std-us 25
python3 test_cadence.py --max-resyncs 1
python3 test_cadence.py --duration 30
```

## Camera-side prerequisites

This receiver expects firmware from the `evtstream-task3-validate`
branch of the OpenMV firmware fork to be running. On the camera:

```python
import csi
import evtstream

csi0 = csi.CSI(cid=csi.GENX320)
csi0.reset()
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, 4096)

evtstream.start(window_us=1000, max_events_per_window=480)
```

Then run the receiver on the Jetson side. Stop with `evtstream.stop()`
on the camera and Ctrl+C on the Jetson.

See firmware-side `DESIGN.md §11b` for a known limitation around
rapid start/stop cycles in v1; production single-cycle use is the
verified happy path.
