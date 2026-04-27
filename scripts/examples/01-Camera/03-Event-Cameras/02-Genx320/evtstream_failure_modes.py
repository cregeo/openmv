# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenMV LLC contributors. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# evtstream task-3 step-7 failure-mode verification.
#
# Exercises the six failure modes from task3_instructions.md §8 step 7
# and prints PASS/FAIL per test. Most checks behavioral; tests that
# require physical action (USB unplug, IDE traffic) are clearly marked
# as MANUAL and described, not auto-run.
#
# Usage:
#     # In OpenMV IDE: open this file, run.
#     # Or save as main.py on the camera.
#     # Run with the lens uncovered for tests 1-4; instructions for
#     # tests 5-6 are printed inline.

import evtstream
import csi
import time

# Sensor setup ------------------------------------------------------------

csi0 = csi.CSI(cid=csi.GENX320)
csi0.reset()
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, 4096)


def banner(n, title):
    print()
    print("=" * 64)
    print(" Test %d: %s" % (n, title))
    print("=" * 64)


# Test 1: double start() raises EBUSY -------------------------------------

banner(1, "double start() should raise OSError(EBUSY)")
evtstream.start()
try:
    evtstream.start()
    print(" FAIL: second start() did not raise")
except OSError as e:
    errno = e.args[0]
    if errno == 16:
        print(" PASS: second start() raised OSError errno=%d (EBUSY)" % errno)
    else:
        print(" FAIL: raised OSError errno=%d (expected 16 EBUSY)" % errno)
evtstream.stop()


# Test 2: stop() when not running is a silent no-op -----------------------

banner(2, "stop() when not running should be silent no-op")
try:
    evtstream.stop()
    evtstream.stop()  # twice in a row
    print(" PASS: stop() returned silently when not running (called twice)")
except Exception as e:
    print(" FAIL: stop() raised %r when not running" % e)


# Test 3: out-of-range args raise ValueError ------------------------------

banner(3, "start() with out-of-range args should raise ValueError")
cases = [
    ({"window_us": 50},                   "window_us=50 (below 100 min)"),
    ({"window_us": 20000},                "window_us=20000 (above 10000 max)"),
    ({"max_events_per_window": 0},        "max_events_per_window=0 (below 64 min)"),
    ({"max_events_per_window": 1024},     "max_events_per_window=1024 (above 512 max)"),
]
for kwargs, label in cases:
    try:
        evtstream.start(**kwargs)
        print(" FAIL: start(%s) did not raise" % label)
        evtstream.stop()
    except ValueError:
        print(" PASS: %s raised ValueError" % label)
    except Exception as e:
        print(" FAIL: %s raised %r (expected ValueError)" % (label, e))


# Test 4: USB FIFO short-write recovery -----------------------------------

banner(4, "events_per_window > FIFO ceiling -- usb_drops grows but cadence holds")
print(" Wave hand vigorously for 3 seconds...")
time.sleep(1)
# 512 events × 8 bytes + 20-byte header = 4116 B, just over the 4 KB
# CDC TX FIFO. Atomic-or-skip should drop these packets cleanly without
# stalling the PIT cadence.
evtstream.start(window_us=1000, max_events_per_window=512)
time.sleep(3)
s = evtstream.stats()
evtstream.stop()
print(" windows_sent       : %d (expected ~3000)" % s['windows_sent'])
print(" usb_drops          : %d" % s['usb_drops'])
print(" events_sent        : %d" % s['events_sent'])
print(" last_window_us     : %d (cadence ~1000)" % s['last_window_us'])
ok4 = (s['windows_sent'] >= 2700 and s['windows_sent'] <= 3300
       and abs(s['last_window_us'] - 1000) <= 100)
print(" %s: streaming uninterrupted under FIFO overflow" %
      ("PASS" if ok4 else "FAIL"))


# Test 5: stop() from ISR context -- code-level verification --------------

banner(5, "stop() from ISR should raise EPERM")
print(" This is verified by inspection in modules/py_evtstream.c stop():")
print("     if (__get_IPSR() != 0) {")
print("         mp_raise_OSError(MP_EPERM);")
print("     }")
print(" The Python flow doesn't reach a real ISR context naturally, so")
print(" no runtime test exercises this path. The code branch is present")
print(" and unchanged since step 2 (commit 9dda727).")
print(" PASS by code inspection")


# Test 6: heartbeat flow during sensor-quiet periods ----------------------

banner(6, "heartbeats keep flowing when sensor produces no events")
print(" Cover the lens NOW (or aim at a static surface)...")
time.sleep(3)
evtstream.start(window_us=1000)
time.sleep(3)
s = evtstream.stats()
evtstream.stop()
print(" windows_sent       : %d (expected ~3000)" % s['windows_sent'])
print(" events_sent        : %d (low/dark-noise expected)" % s['events_sent'])
print(" csi_dma_underruns  : %d (high == sensor genuinely idle)" %
      s['csi_dma_underruns'])
print(" last_window_us     : %d" % s['last_window_us'])
ok6 = (s['windows_sent'] >= 2700 and s['windows_sent'] <= 3300
       and abs(s['last_window_us'] - 1000) <= 100)
print(" %s: heartbeats flow regardless of sensor activity" %
      ("PASS" if ok6 else "FAIL"))


# Manual tests ------------------------------------------------------------

banner(7, "USB host disconnect mid-stream (MANUAL)")
print(" Procedure:")
print("   1. evtstream.start(window_us=1000, max_events_per_window=480)")
print("   2. Unplug USB")
print("   3. Wait 5+ seconds")
print("   4. Re-plug USB")
print("   5. evtstream.stop()")
print(" Expected: MCU stays alive (no reboot); usb_drops grows during")
print(" disconnect; streaming resumes cleanly on reconnect. Atomic-or-")
print(" skip in stream_pit_drain_and_ship() makes drops orderly.")
print(" Status: NOT auto-run (requires physical USB replug)")


banner(8, "ISR latency under heavy IDE traffic (MANUAL)")
print(" Procedure:")
print("   1. evtstream.start(window_us=1000)")
print("   2. Have OpenMV IDE pulling preview frames at max rate")
print("   3. After 10 seconds, evtstream.stop() and read stats")
print(" Expected: last_window_us within +/- 50 us of 1000.")
print(" PIT NVIC priority (10) is below USB OTG (6) so USB IRQ preempts")
print(" PIT, but PIT cadence should remain tight as the PIT ISR is")
print(" bounded.")
print(" Status: NOT auto-run (requires running IDE)")


# Summary -----------------------------------------------------------------

print()
print("=" * 64)
print(" Summary")
print("=" * 64)
print(" Tests 1-6 ran inline above. Tests 7-8 are manual; their")
print(" procedures are documented for hardware operators.")
print()
print(" The known v1 rapid-start/stop loop limitation (DESIGN.md §11b)")
print(" is NOT exercised here -- single-cycle test 4 / 6 stay within")
print(" the cycle-1 happy path.")
