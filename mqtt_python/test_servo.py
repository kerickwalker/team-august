#!/usr/bin/env python3

# Arm servo smoke test: same bring-up pattern as mqtt-final-mission.py
# (teensy_interface subprocess + MQTT service.setup), then cycles the arm
# using the same positions as scollect.py (friend's collection module).
#
# Intended to run on the robot with Mosquitto on localhost — no -i or -n needed.
# Ends with the arm held at -900 (up); no torque-release jump to 3000.
#
# Usage:
#   python3 test_servo.py
#   python3 test_servo.py --pause 2 --hold-down 6 --verbose
#   python3 test_servo.py --servo3 --servo3-min -400 --servo3-max 400
#   (--servo3 runs only servo 3; servos 1/2 are not moved.)

import argparse
import sys
import time

from setproctitle import setproctitle

from sedge import edge
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface

# MQTT broker on the same machine as this script (typical on-robot setup)
MQTT_HOST = "localhost"

# Same defaults as scollect.SCollect
SERVO_ID = 1
SERVO_MIRROR_ID = 2
POS_OPEN = -475
POS_CLOSED = 480
SERVO_SPEED = 100

# Third servo (gripper / extra): tune --servo3-min / --servo3-max on your hardware
SERVO3_ID = 3
SERVO3_MIN_DEFAULT = -500
SERVO3_MAX_DEFAULT = 500


def servo_move(position):
    service.send("robobot/cmd/T0", f"servo {SERVO_ID} {position} {SERVO_SPEED}")
    service.send("robobot/cmd/T0", f"servo {SERVO_MIRROR_ID} {-position} {SERVO_SPEED}")


def run_cycle(pause_s, hold_down_s):
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    print(f"% Arm UP    (servo {SERVO_ID} {POS_OPEN} {SERVO_SPEED}, "
          f"servo {SERVO_MIRROR_ID} {-POS_OPEN} {SERVO_SPEED})")
    servo_move(POS_OPEN)
    time.sleep(pause_s)
    print(f"% Arm DOWN  (servo {SERVO_ID} {POS_CLOSED} {SERVO_SPEED}, "
          f"servo {SERVO_MIRROR_ID} {-POS_CLOSED} {SERVO_SPEED}) — holding {hold_down_s:.1f}s")
    servo_move(POS_CLOSED)
    time.sleep(hold_down_s)
    print(f"% Arm UP    (servo {SERVO_ID} {POS_OPEN} {SERVO_SPEED}, "
          f"servo {SERVO_MIRROR_ID} {-POS_OPEN} {SERVO_SPEED}) — final park")
    servo_move(POS_OPEN)
    time.sleep(pause_s)


def servo3_move(position, speed):
    service.send("robobot/cmd/T0", f"servo {SERVO3_ID} {position} {speed}")


def run_servo3_cycle(dwell_s, pos_min, pos_max, speed):
    """Hold min then max for dwell_s each (open/close style check). Ends at pos_min."""
    print(f"% Servo {SERVO3_ID} min ({pos_min}) — {dwell_s:.1f}s")
    servo3_move(pos_min, speed)
    time.sleep(dwell_s)
    print(f"% Servo {SERVO3_ID} max ({pos_max}) — {dwell_s:.1f}s")
    servo3_move(pos_max, speed)
    time.sleep(dwell_s)
    print(f"% Servo {SERVO3_ID} park at min ({pos_min})")
    servo3_move(pos_min, speed)
    time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(
        description="Arm servo smoke test (localhost MQTT, start immediately).",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=2.0,
        help="Seconds after initial up and after final up (default: 2)",
    )
    parser.add_argument(
        "--hold-down",
        type=float,
        default=5.0,
        dest="hold_down",
        metavar="SEC",
        help="Seconds to stay in the down position (default: 5)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print MQTT diagnostics (disable silent mode)",
    )
    parser.add_argument(
        "--servo3",
        action="store_true",
        help="Only cycle servo 3 between min and max (does not move servos 1/2)",
    )
    parser.add_argument(
        "--servo3-min",
        type=int,
        default=SERVO3_MIN_DEFAULT,
        dest="servo3_min",
        metavar="POS",
        help=f"Servo 3 'open'/min position (default: {SERVO3_MIN_DEFAULT})",
    )
    parser.add_argument(
        "--servo3-max",
        type=int,
        default=SERVO3_MAX_DEFAULT,
        dest="servo3_max",
        metavar="POS",
        help=f"Servo 3 'close'/max position (default: {SERVO3_MAX_DEFAULT})",
    )
    parser.add_argument(
        "--servo3-dwell",
        type=float,
        default=5.0,
        dest="servo3_dwell",
        metavar="SEC",
        help="Seconds to hold at each servo 3 endpoint (default: 5)",
    )
    parser.add_argument(
        "--servo3-speed",
        type=int,
        default=SERVO_SPEED,
        dest="servo3_speed",
        metavar="S",
        help=f"Servo 3 move speed (default: {SERVO_SPEED})",
    )
    cli = parser.parse_args()

    # uservice.setup() calls parse_args(); clear argv so only script name remains
    sys.argv = [sys.argv[0]]

    setproctitle("test_servo")
    start_teensy_interface()
    service.setup(MQTT_HOST)

    if not service.connected:
        print("% MQTT not connected — exiting")
        stop_teensy_interface()
        sys.exit(1)

    service.args.now = True
    if not cli.verbose:
        service.args.silent = True

    try:
        if cli.servo3:
            run_servo3_cycle(
                cli.servo3_dwell,
                cli.servo3_min,
                cli.servo3_max,
                cli.servo3_speed,
            )
        else:
            run_cycle(cli.pause, cli.hold_down)
    except KeyboardInterrupt:
        print("\n% Ctrl+C — stopping")
        service.stop = True
    finally:
        if cli.servo3:
            print("% Stopping drive; servo 3 at min (park)")
        else:
            print("% Stopping drive; arm at -900 (up)")
        try:
            service.send("robobot/cmd/ti", "rc 0 0")
            time.sleep(0.05)
            if cli.servo3:
                servo3_move(cli.servo3_min, cli.servo3_speed)
            else:
                servo_move(POS_OPEN)
            edge.lineControl(0)
        except Exception:
            pass

    service.terminate()
    stop_teensy_interface()
    if cli.verbose:
        print("% test_servo: done")


if __name__ == "__main__":
    main()
