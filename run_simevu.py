#!/usr/bin/env python3
"""
run_simevu.py -- drive the ICU console through a simevu test and capture
                 the log, so the operator does not have to type commands
                 against a 12-second staleness clock.

WHY THIS EXISTS
    Bench test 1 was run by hand and the track went DEGRADED twice and
    EXPIRED once, purely because typing is slower than EVU_GRACE_MS. That
    is the grace logic behaving exactly as designed, applied to a vehicle
    that had stopped being typed at -- the same observation the comment
    above gwSimEvuRun() in ICU_EVU.ino already records.

    'simevu <lane> run <n> <sec>' solves the stepping. It does not solve
    getting several lanes started close enough together to contend, or
    capturing a clean log around the run. This does both.

USAGE
    pip install pyserial
    python run_simevu.py --port COM7 --test 1
    python run_simevu.py --port COM7 --test 2
    python run_simevu.py --port COM7 --test 3
    python run_simevu.py --port COM7 --test 4     divergence / U-turn
    python run_simevu.py --port COM7 --test 5     implausible jump
    python run_simevu.py --port COM7 --test 6     speed floor
    python run_simevu.py --port COM7 --test 7     emergency flag off
    python run_simevu.py --port COM7 --test all   all seven, ~13 min

    Close the Arduino Serial Monitor first -- only one program can hold
    the port.

    Find the port with:  python -m serial.tools.list_ports

OUTPUT
    simevu_test<N>_<timestamp>.log in the working directory, containing
    everything the ICU printed, with the commands sent marked '>>>'.

NOTE
    This only sends console commands. It changes nothing on the ICU and
    is not part of the firmware. Safe to run against a live bench.
"""

import argparse
import sys
import time
import threading
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed.  Run:  pip install pyserial")


# ---------------------------------------------------------------------
# TEST DEFINITIONS
#
# Each entry is (delay_before_seconds, command_or_None).
# A None command is a pure wait -- used to let a full 30 s health block
# print, which is where [PIPE] and [PERF] come from.
# ---------------------------------------------------------------------

TESTS = {
    1: {
        "name": "single lane baseline",
        "why":  "ETA must fall monotonically; demand must cap at PREPARE "
                "because the geofence is bypassed on every node.",
        "steps": [
            (0,  "simevu 1 clear"),
            (3,  "simevu 1 approach 500 40"),
            (2,  "simevu 1 run 10 2"),
            (25, None),          # let the run finish
            (32, None),          # one full health block
            (0,  "simevu 1 clear"),
            (5,  None),
        ],
    },

    2: {
        "name": "two lane contention",
        "why":  "Lane 2 opens 200 m closer and should win arbitration. "
                "Watch hand= on [PERF]: two lanes alerting doubles the "
                "ERC TX burst that loop() has to push out.",
        "steps": [
            (0,  "simevu 1 clear"),
            (1,  "simevu 2 clear"),
            (5,  "simevu 1 approach 500 40"),
            (1,  "simevu 2 approach 300 40"),
            (1,  "simevu 1 run 12 2"),
            (0,  "simevu 2 run 12 2"),   # fired immediately after, so both contend
            (30, None),
            (32, None),
            (0,  "simevu 1 clear"),
            (1,  "simevu 2 clear"),
            (5,  None),
        ],
    },

    3: {
        "name": "three lane contention + starvation bound",
        "why":  "All three approaches demanding at once. The 120 s "
                "starvation timer must stop any lane being locked out, "
                "and no preempt-of-preempt should occur.",
        "steps": [
            (0,  "simevu 1 clear"),
            (1,  "simevu 2 clear"),
            (1,  "simevu 3 clear"),
            (5,  "simevu 1 approach 500 40"),
            (1,  "simevu 2 approach 300 40"),
            (1,  "simevu 3 approach 400 40"),
            (1,  "simevu 1 run 15 2"),
            (0,  "simevu 2 run 15 2"),
            (0,  "simevu 3 run 15 2"),
            (35, None),
            (32, None),
            (32, None),          # second health block, to see starvation act
            (0,  "simevu 1 clear"),
            (1,  "simevu 2 clear"),
            (1,  "simevu 3 clear"),
            (5,  None),
        ],
    },
    # -----------------------------------------------------------------
    # FAULT INJECTION -- the paths that decide NOT to give green.
    #
    # Everything in tests 1-3 is a well-behaved vehicle approaching
    # normally. These four are the safety rejections, and they matter
    # more than the happy path: a missed ambulance is bad, but holding
    # green for a vehicle that U-turned away puts cross traffic into the
    # intersection.
    #
    # Each one uses 'run' to keep the track ACTIVE across the injection,
    # because a track that goes DEGRADED mid-test proves nothing about
    # the check being tested -- it only proves the 12 s grace timer works,
    # which tests 1-3 already established the hard way.
    # -----------------------------------------------------------------

    4: {
        "name": "divergence / U-turn (S2-09)",
        "why":  "Vehicle turns away mid-approach. DIVERGE_CONFIRM_COUNT "
                "consecutive growing fixes are required before the ICU "
                "believes it -- one bad fix must NOT release the signal. "
                "Expect: track -> DIVERGED, then release.",
        "steps": [
            (0,  "simevu 1 clear"),
            (3,  "simevu 1 approach 400 40"),
            (1,  "simevu 1 run 6 2"),
            (13, "simevu 1 diverge"),     # mid-approach, track still ACTIVE
            (1,  "simevu 1 run 8 2"),     # growing fixes to confirm
            (20, None),
            (32, None),
            (0,  "simevu 1 clear"),
            (5,  None),
        ],
    },

    5: {
        "name": "implausible position jump (S2-05)",
        "why":  "A correctly SIGNED packet whose CONTENTS are impossible. "
                "Authentication must pass and the plausibility check must "
                "reject it anyway -- that distinction is the whole point. "
                "Expect: jump rejected, track stays at its last good "
                "position, and NORMAL stepping resumes afterwards.",
        "steps": [
            (0,  "simevu 1 clear"),
            (3,  "simevu 1 approach 400 40"),
            (1,  "simevu 1 run 6 2"),
            (13, "simevu 1 teleport 3000"),
            (2,  "simevu 1 run 6 2"),     # must still work after rejection
            (16, None),
            (32, None),
            (0,  "simevu 1 clear"),
            (5,  None),
        ],
    },

    6: {
        "name": "speed floor -- vehicle stops mid-approach",
        "why":  "Stopped in traffic. ETA must not run to infinity or "
                "bounce; V_FLOOR should hold it to something an officer "
                "can read. README section 8 records ETA-from-instantaneous-"
                "speed bouncing on the console as a real past defect.",
        "steps": [
            (0,  "simevu 1 clear"),
            (3,  "simevu 1 approach 400 40"),
            (1,  "simevu 1 run 6 2"),
            (13, "simevu 1 speed 0"),
            (1,  "simevu 1 run 8 2"),     # watch ETA while stationary
            (20, None),
            (32, None),
            (0,  "simevu 1 speed 40"),    # resume, ETA must recover sanely
            (1,  "simevu 1 run 6 2"),
            (14, None),
            (0,  "simevu 1 clear"),
            (5,  None),
        ],
    },

    7: {
        "name": "emergency flag cleared mid-approach (S2-08)",
        "why":  "Crew switches the lights off -- no longer an emergency "
                "run. The demand must drop promptly rather than ride out "
                "the hold timer. Then flag back on: it must re-acquire.",
        "steps": [
            (0,  "simevu 1 clear"),
            (3,  "simevu 1 approach 400 40"),
            (1,  "simevu 1 run 6 2"),
            (13, "simevu 1 off"),
            (1,  "simevu 1 run 4 2"),
            (12, "simevu 1 on"),
            (1,  "simevu 1 run 4 2"),
            (12, None),
            (32, None),
            (0,  "simevu 1 clear"),
            (5,  None),
        ],
    },
}


def run_test(port, baud, n, logf):
    t = TESTS[n]
    total = sum(d for d, _ in t["steps"])

    banner = (f"\n{'='*70}\n"
              f"TEST {n} -- {t['name']}\n"
              f"{t['why']}\n"
              f"approx {total} s\n"
              f"{'='*70}\n")
    print(banner)
    logf.write(banner)

    ser = serial.Serial(port, baud, timeout=0.2)
    stop = threading.Event()

    def reader():
        # Read continuously so nothing the ICU prints is missed while we
        # are sleeping between commands.
        buf = b""
        while not stop.is_set():
            try:
                data = ser.read(4096)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "replace").rstrip("\r")
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                out = f"{stamp}  {s}"
                print(out)
                logf.write(out + "\n")
                logf.flush()

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    # Let the port settle and catch any partial line already in flight.
    time.sleep(1.5)

    try:
        for delay, cmd in t["steps"]:
            if delay:
                time.sleep(delay)
            if cmd is None:
                continue
            stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            marker = f"{stamp}  >>> {cmd}"
            print(marker)
            logf.write(marker + "\n")
            logf.flush()
            ser.write((cmd + "\n").encode())
            ser.flush()
    except KeyboardInterrupt:
        print("\ninterrupted -- clearing all lanes")
        for lane in (1, 2, 3):
            ser.write(f"simevu {lane} clear\n".encode())
            time.sleep(0.3)
    finally:
        time.sleep(2)
        stop.set()
        th.join(timeout=2)
        ser.close()

    print(f"\nTEST {n} done.\n")
    logf.write(f"\nTEST {n} done.\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--test", default="1", help="1-7, or all")
    args = ap.parse_args()

    which = [1, 2, 3, 4, 5, 6, 7] if args.test == "all" else [int(args.test)]
    for n in which:
        if n not in TESTS:
            sys.exit(f"no test {n}")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"simevu_test{args.test}_{ts}.log"

    with open(name, "w", encoding="utf-8") as logf:
        for i, n in enumerate(which):
            if i:
                # Let the previous test's tracks fully release before the
                # next one opens new ones, so the logs do not overlap.
                print("\n--- 20 s gap before next test ---\n")
                logf.write("\n--- 20 s gap before next test ---\n")
                time.sleep(20)
            run_test(args.port, args.baud, n, logf)

    print(f"\nLog written to {name}")
    print("Upload that file.")


if __name__ == "__main__":
    main()
