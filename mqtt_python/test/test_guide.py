#!/usr/bin/env python3

# Standalone field test for the hole-guiding module (sguide.py).
# Runs the full guide state machine on the real robot with verbose
# per-tick logging so you can tune detection and drive parameters.
#
# Assumes the ball is already captured (arm is down). The arm is confirmed
# in the closed position at the start of the run.
#
# Usage:
#   python3 test/test_guide.py <robot-ip> [options]
#
# Options:
#   --close-radius N      hole radius (px) that triggers deposit nudge  [default: 60]
#   --nudge-duration F    seconds to drive forward into hole  [default: 0.8]
#   --align-threshold F   max |steeringError| to consider hole centred  [default: 0.15]
#   --drive-speed F       forward speed in m/s  [default: 0.12]
#   --turn-gain F         steeringError → turn rate multiplier  [default: 1.0]
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
    parser.add_argument('host', help='Robot IP address')
    parser.add_argument('--close-radius',    type=int,   default=None)
    parser.add_argument('--nudge-duration',  type=float, default=None)
    parser.add_argument('--align-threshold', type=float, default=None)
    parser.add_argument('--drive-speed',     type=float, default=None)
    parser.add_argument('--turn-gain',       type=float, default=None)
    return parser.parse_args()


def print_params():
    print("% Active parameters:")
    print(f"%   close_radius    = {guide.close_radius}")
    print(f"%   nudge_duration  = {guide.nudge_duration}")
    print(f"%   nudge_speed     = {guide.nudge_speed}")
    print(f"%   align_threshold = {guide.align_threshold}")
    print(f"%   drive_speed     = {guide.drive_speed}")
    print(f"%   turn_gain       = {guide.turn_gain}")
    print(f"%   pos_closed      = {guide.pos_closed}")
    print(f"%   servo_speed     = {guide.servo_speed}")


def safe_exit():
    print("% Stopping motors and disabling servo ...")
    try:
        service.send("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.05)
        service.send("robobot/cmd/T0", f"servo {guide.servo_id} 3000 0")
    except Exception:
        pass
    try:
        service.terminate()
    except Exception:
        pass


def main():
    args = parse_args()

    if args.close_radius    is not None: guide.close_radius    = args.close_radius
    if args.nudge_duration  is not None: guide.nudge_duration  = args.nudge_duration
    if args.align_threshold is not None: guide.align_threshold = args.align_threshold
    if args.drive_speed     is not None: guide.drive_speed     = args.drive_speed
    if args.turn_gain       is not None: guide.turn_gain       = args.turn_gain

    guide.verbose = True

    print(f"% Connecting to robot at {args.host} ...")
    service.setup(args.host)

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
