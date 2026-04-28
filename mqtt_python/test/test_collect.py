#!/usr/bin/env python3

# Standalone field test for the ball collection module (scollect.py).
# Runs the full collect state machine on the real robot with verbose
# per-tick logging so you can tune detection and drive parameters.
#
# Usage:
#   python3 test/test_collect.py <robot-ip> [options]
#
# Options:
#   --commit-radius N      ball radius (px) that triggers centering step  [default: 50]
#   --center-threshold F   max |steeringError| to consider ball centered before capture  [default: 0.05]
#   --commit-duration F    seconds to drive straight before closing arm  [default: 0.4]
#   --commit-speed F       forward speed during capture run in m/s  [default: 0.15]
#   --align-threshold F    max |steeringError| to consider ball centred during approach  [default: 0.05]
#   --drive-speed F        forward speed during approach in m/s  [default: 0.15]
#   --turn-gain F          steeringError → turn rate multiplier  [default: 1.0]
#
# Keys:
#   Ctrl+C — stop motors, disable servo, exit

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from uservice import service
from scollect import collect


def parse_args():
    parser = argparse.ArgumentParser(description='Test ball collection')
    parser.add_argument('-i', '--hostIP', default='localhost', help='Robot IP address')
    parser.add_argument('--commit-radius',    type=int,   default=None)
    parser.add_argument('--center-threshold', type=float, default=None)
    parser.add_argument('--commit-duration',  type=float, default=None)
    parser.add_argument('--commit-speed',     type=float, default=None)
    parser.add_argument('--align-threshold',  type=float, default=None)
    parser.add_argument('--drive-speed',      type=float, default=None)
    parser.add_argument('--turn-gain',        type=float, default=None)
    args, remaining = parser.parse_known_args()
    # Restore only unrecognised args so uservice.setup() can parse its own flags
    import sys
    sys.argv = [sys.argv[0]] + remaining
    return args


def print_params():
    print("% Active parameters:")
    print(f"%   commit_radius    = {collect.commit_radius}")
    print(f"%   center_threshold = {collect.center_threshold}")
    print(f"%   commit_duration  = {collect.commit_duration}")
    print(f"%   commit_speed     = {collect.commit_speed}")
    print(f"%   align_threshold  = {collect.align_threshold}")
    print(f"%   drive_speed      = {collect.drive_speed}")
    print(f"%   turn_gain        = {collect.turn_gain}")
    print(f"%   pos_open         = {collect.pos_open}")
    print(f"%   pos_closed       = {collect.pos_closed}")
    print(f"%   servo_speed      = {collect.servo_speed}")


def safe_exit():
    print("% Stopping motors and disabling servo ...")
    try:
        service.send("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.05)
        service.send("robobot/cmd/T0", f"servo {collect.servo_id} 3000 0")
    except Exception:
        pass
    try:
        service.terminate()
    except Exception:
        pass


def main():
    args = parse_args()

    # Apply any CLI overrides before setup so they show in the param summary
    if args.commit_radius    is not None: collect.commit_radius    = args.commit_radius
    if args.center_threshold is not None: collect.center_threshold = args.center_threshold
    if args.commit_duration  is not None: collect.commit_duration  = args.commit_duration
    if args.commit_speed     is not None: collect.commit_speed     = args.commit_speed
    if args.align_threshold  is not None: collect.align_threshold  = args.align_threshold
    if args.drive_speed      is not None: collect.drive_speed      = args.drive_speed
    if args.turn_gain        is not None: collect.turn_gain        = args.turn_gain

    collect.verbose = True

    print(f"% Connecting to robot at {args.hostIP} ...")
    service.setup(args.hostIP)

    print_params()
    print("% Starting collection — Ctrl+C to abort\n")

    t_start = time.time()
    result  = False

    try:
        result = collect.collect()
    except KeyboardInterrupt:
        print("\n% Interrupted")
    finally:
        elapsed = time.time() - t_start
        safe_exit()

    print(f"\n% Result: {'SUCCESS — ball captured' if result else 'FAILED / interrupted'}")
    print(f"% Time elapsed: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
