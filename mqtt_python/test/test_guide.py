#!/usr/bin/env python3

# Standalone field test for the hole-guiding module (sguide.py).
# Runs the full guide state machine on the real robot with verbose
# per-tick logging so you can tune detection and drive parameters.
#
# Assumes the ball is already captured (arm is up, gate is closed).
#
# Usage:
#   python3 test/test_guide.py -i <robot-ip> [options]
#
# Options:
#   --commit-radius N      hole radius (px) that triggers centering step  [default: 30]
#   --center-threshold F   max |steeringError| to consider hole centered before deposit  [default: 0.05]
#   --commit-duration F    seconds to drive straight before depositing  [default: 0.4]
#   --commit-speed F       forward speed during deposit run in m/s  [default: 0.12]
#   --align-threshold F    max |steeringError| to consider hole centred during approach  [default: 0.15]
#   --drive-speed F        forward speed during approach in m/s  [default: 0.12]
#   --turn-gain F          steeringError → turn rate multiplier  [default: 1.0]
#   --wiggle-rate F        turn rate during deposit wiggle in rad/s  [default: 0.5]
#   --wiggle-duration F    seconds per half-swing during wiggle  [default: 0.5]
#
# Keys:
#   Ctrl+C — stop motors, disable servo, exit

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from uservice import service
from sguide import guide


def parse_args():
    parser = argparse.ArgumentParser(description='Test hole guiding')
    parser.add_argument('-i', '--hostIP', default='localhost', help='Robot IP address')
    parser.add_argument('--commit-radius',    type=int,   default=None)
    parser.add_argument('--center-threshold', type=float, default=None)
    parser.add_argument('--commit-duration',  type=float, default=None)
    parser.add_argument('--commit-speed',     type=float, default=None)
    parser.add_argument('--align-threshold',  type=float, default=None)
    parser.add_argument('--drive-speed',      type=float, default=None)
    parser.add_argument('--turn-gain',        type=float, default=None)
    parser.add_argument('--wiggle-rate',      type=float, default=None)
    parser.add_argument('--wiggle-duration',  type=float, default=None)
    args, remaining = parser.parse_known_args()
    import sys
    sys.argv = [sys.argv[0]] + remaining
    return args


def print_params():
    print("% Active parameters:")
    print(f"%   commit_radius    = {guide.commit_radius}")
    print(f"%   center_threshold = {guide.center_threshold}")
    print(f"%   commit_duration  = {guide.commit_duration}")
    print(f"%   commit_speed     = {guide.commit_speed}")
    print(f"%   align_threshold  = {guide.align_threshold}")
    print(f"%   drive_speed      = {guide.drive_speed}")
    print(f"%   turn_gain        = {guide.turn_gain}")
    print(f"%   pos_open         = {guide.pos_open}")
    print(f"%   pos_closed       = {guide.pos_closed}")
    print(f"%   gate_open        = {guide.gate_open}")
    print(f"%   gate_closed      = {guide.gate_closed}")
    print(f"%   servo_speed      = {guide.servo_speed}")
    print(f"%   wiggle_rate      = {guide.wiggle_rate}")
    print(f"%   wiggle_duration  = {guide.wiggle_duration}")


def safe_exit():
    print("% Stopping motors ...")
    try:
        service.send("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.05)
        #service.send("robobot/cmd/T0", f"servo {collect.servo_id} {collect.arm_up} {collect.servo_speed}")
        service.send("robobot/cmd/T0", f"servo {collect.servo_id} 3000 0")
    except Exception:
        pass
    try:
        service.terminate()
    except Exception:
        pass


def main():
    args = parse_args()

    if args.commit_radius    is not None: guide.commit_radius    = args.commit_radius
    if args.center_threshold is not None: guide.center_threshold = args.center_threshold
    if args.commit_duration  is not None: guide.commit_duration  = args.commit_duration
    if args.commit_speed     is not None: guide.commit_speed     = args.commit_speed
    if args.align_threshold  is not None: guide.align_threshold  = args.align_threshold
    if args.drive_speed      is not None: guide.drive_speed      = args.drive_speed
    if args.turn_gain        is not None: guide.turn_gain        = args.turn_gain
    if args.wiggle_rate      is not None: guide.wiggle_rate      = args.wiggle_rate
    if args.wiggle_duration  is not None: guide.wiggle_duration  = args.wiggle_duration

    guide.verbose = True

    print(f"% Connecting to robot at {args.hostIP} ...")
    service.setup(args.hostIP)

    print_params()
    print("% Starting guide — Ctrl+C to abort\n")

    t_start = time.time()
    result  = False

    try:
        result = guide.guide()
    except KeyboardInterrupt:
        print("\n% Interrupted")
    finally:
        elapsed = time.time() - t_start
        safe_exit()

    print(f"\n% Result: {'SUCCESS — ball deposited' if result else 'FAILED / interrupted'}")
    print(f"% Time elapsed: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
