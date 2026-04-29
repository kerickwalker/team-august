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
        run_cycle(cli.pause, cli.hold_down)
    except KeyboardInterrupt:
        print("\n% Ctrl+C — parking arm up")
        service.stop = True
    finally:
        print("% Stopping drive; arm at -900 (up)")
        try:
            service.send("robobot/cmd/ti", "rc 0 0")
            time.sleep(0.05)
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
