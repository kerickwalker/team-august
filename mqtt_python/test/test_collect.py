#!/usr/bin/env python3

# Standalone field test for the ball collection module (scollect.py).
# Runs the full collect state machine on the real robot with verbose
# per-tick logging so you can tune detection and drive parameters.
#
# Usage:
#   python3 test/test_collect.py <robot-ip> [options]
#
# Options:
#   --close-radius N      ball radius (px) that triggers arm drop  [default: 40]
#   --align-threshold F   max |steeringError| to consider ball centred  [default: 0.15]
#   --drive-speed F       forward speed in m/s  [default: 0.15]
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
from scollect import collect


def parse_args():
    parser = argparse.ArgumentParser(description='Test ball collection')
    parser.add_argument('host', help='Robot IP address')
    parser.add_argument('--close-radius',    type=int,   default=None)
    parser.add_argument('--align-threshold', type=float, default=None)
    parser.add_argument('--drive-speed',     type=float, default=None)
    parser.add_argument('--turn-gain',       type=float, default=None)
    return parser.parse_args()


def print_params():
    print("% Active parameters:")
    print(f"%   close_radius    = {collect.close_radius}")
    print(f"%   align_threshold = {collect.align_threshold}")
    print(f"%   drive_speed     = {collect.drive_speed}")
    print(f"%   turn_gain       = {collect.turn_gain}")
    print(f"%   pos_open        = {collect.pos_open}")
    print(f"%   pos_closed      = {collect.pos_closed}")
    print(f"%   servo_speed     = {collect.servo_speed}")


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
    if args.close_radius    is not None: collect.close_radius    = args.close_radius
    if args.align_threshold is not None: collect.align_threshold = args.align_threshold
    if args.drive_speed     is not None: collect.drive_speed     = args.drive_speed
    if args.turn_gain       is not None: collect.turn_gain       = args.turn_gain

    collect.verbose = True

    print(f"% Connecting to robot at {args.host} ...")
    service.setup(args.host)

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
