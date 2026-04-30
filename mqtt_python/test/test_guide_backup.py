#!/usr/bin/env python3

# Standalone field test for the odometry-based backup deposit (sguide.py).
# Drives a fixed distance then lowers the arm, opens the gate, and wiggles.
#
# Assumes the ball is already captured (arm is up, gate is closed).
#
# Usage:
#   python3 test/test_guide_backup.py -i <robot-ip> [options]
#
# Options:
#   --backup-distance F    metres to drive before depositing  [default: 0.30]
#   --backup-speed F       forward speed during approach in m/s  [default: 0.10]
#   --wiggle-rate F        turn rate during deposit wiggle in rad/s  [default: 0.5]
#   --wiggle-duration F    seconds per half-swing during wiggle  [default: 0.5]
#
# Keys:
#   Ctrl+C — stop motors and exit

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from uservice import service
from sguide import guide


def parse_args():
    parser = argparse.ArgumentParser(description='Test odometry backup deposit')
    parser.add_argument('-i', '--hostIP',         default='localhost', help='Robot IP address')
    parser.add_argument('--backup-distance', type=float, default=None)
    parser.add_argument('--backup-speed',    type=float, default=None)
    parser.add_argument('--wiggle-rate',     type=float, default=None)
    parser.add_argument('--wiggle-duration', type=float, default=None)
    parser.add_argument('--rotate-deg',      type=float, default=None)
    args, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + remaining
    return args


def print_params():
    print("% Active parameters:")
    print(f"%   backup_distance    = {guide.backup_distance} m")
    print(f"%   backup_drive_speed = {guide.backup_drive_speed} m/s")
    print(f"%   wiggle_rate        = {guide.wiggle_rate} rad/s")
    print(f"%   wiggle_duration    = {guide.wiggle_duration} s")
    print(f"%   backup_rotate_deg  = {guide.backup_rotate_deg}°")
    print(f"%   pos_closed         = {guide.pos_closed}  (arm down)")
    print(f"%   gate_open          = {guide.gate_open}")


def safe_exit():
    print("% Stopping motors ...")
    try:
        service.send("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.05)
    except Exception:
        pass
    try:
        service.terminate()
    except Exception:
        pass


def main():
    args = parse_args()

    if args.backup_distance is not None: guide.backup_distance    = args.backup_distance
    if args.backup_speed    is not None: guide.backup_drive_speed = args.backup_speed
    if args.wiggle_rate     is not None: guide.wiggle_rate        = args.wiggle_rate
    if args.wiggle_duration is not None: guide.wiggle_duration    = args.wiggle_duration
    if args.rotate_deg      is not None: guide.backup_rotate_deg  = args.rotate_deg

    guide.verbose = True

    print(f"% Connecting to robot at {args.hostIP} ...")
    service.setup(args.hostIP)

    print_params()
    print("% Starting backup guide — Ctrl+C to abort\n")

    t_start = time.time()
    result  = False

    try:
        result = guide.guide_backup()
    except KeyboardInterrupt:
        print("\n% Interrupted")
    finally:
        elapsed = time.time() - t_start
        safe_exit()

    print(f"\n% Result: {'SUCCESS — ball deposited' if result else 'FAILED / interrupted'}")
    print(f"% Distance driven: see tripB in verbose log above")
    print(f"% Time elapsed: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
