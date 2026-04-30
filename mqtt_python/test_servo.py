#!/usr/bin/env python3

# Arm servo smoke test: same bring-up pattern as mqtt-final-mission.py
# (teensy_interface subprocess + MQTT service.setup), then cycles servos 1/2
# together. Servo 3 is only tested when --servo3 is provided.
#
# Intended to run on the robot with Mosquitto on localhost - no -i or -n needed.
#
# Usage:
#   python3 test_servo.py
#   python3 test_servo.py --pause 2 --hold-down 6 --verbose
#   python3 test_servo.py --servo12-min -475 --servo12-max 260 --arm-speed 70
#   python3 test_servo.py --servo3 --servo3-min -400 --servo3-max 400

import argparse
import sys
import time

try:
    from setproctitle import setproctitle
except ImportError:
    def setproctitle(_title):
        return None

# MQTT broker on the same machine as this script (typical on-robot setup)
MQTT_HOST = "localhost"

# Safer defaults for bench testing (reduce slam risk)
SERVO_ID = 1
SERVO_MIRROR_ID = 2
POS_OPEN = -475
POS_CLOSED = 260
SERVO_SPEED = 70

# Third servo (gripper / extra): tune --servo3-min / --servo3-max on your hardware
SERVO3_ID = 3
SERVO3_MIN_DEFAULT = -500
SERVO3_MAX_DEFAULT = 500


def servo_move(position, speed):
    service.send("robobot/cmd/T0", f"servo {SERVO_ID} {position} {speed}")
    service.send("robobot/cmd/T0", f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def run_cycle(pause_s, hold_down_s, open_pos, closed_pos, speed):
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    print(f"% Arm UP    (servo {SERVO_ID} {open_pos} {speed}, "
          f"servo {SERVO_MIRROR_ID} {-open_pos} {speed})")
    servo_move(open_pos, speed)
    time.sleep(pause_s)
    print(f"% Arm DOWN  (servo {SERVO_ID} {closed_pos} {speed}, "
          f"servo {SERVO_MIRROR_ID} {-closed_pos} {speed}) - holding {hold_down_s:.1f}s")
    servo_move(closed_pos, speed)
    time.sleep(hold_down_s)
    print(f"% Arm UP    (servo {SERVO_ID} {open_pos} {speed}, "
          f"servo {SERVO_MIRROR_ID} {-open_pos} {speed}) - final park")
    servo_move(open_pos, speed)
    time.sleep(pause_s)


def servo3_move(position, speed):
    service.send("robobot/cmd/T0", f"servo {SERVO3_ID} {position} {speed}")


def run_servo3_cycle(dwell_s, pos_min, pos_max, speed):
    """Hold min then max for dwell_s each. Ends at pos_min."""
    print(f"% Servo {SERVO3_ID} min ({pos_min}) - {dwell_s:.1f}s")
    servo3_move(pos_min, speed)
    time.sleep(dwell_s)
    print(f"% Servo {SERVO3_ID} max ({pos_max}) - {dwell_s:.1f}s")
    servo3_move(pos_max, speed)
    time.sleep(dwell_s)
    print(f"% Servo {SERVO3_ID} park at min ({pos_min})")
    servo3_move(pos_min, speed)
    time.sleep(0.05)


def main():
    global edge, service, start_teensy_interface, stop_teensy_interface

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
        "--servo12-min",
        "--open-pos",
        type=int,
        default=POS_OPEN,
        dest="servo12_min",
        metavar="POS",
        help=f"Servo 1 open/up position; servo 2 uses the mirrored value (default: {POS_OPEN})",
    )
    parser.add_argument(
        "--servo12-max",
        "--closed-pos",
        type=int,
        default=POS_CLOSED,
        dest="servo12_max",
        metavar="POS",
        help=f"Servo 1 closed/down position; servo 2 uses the mirrored value (default: {POS_CLOSED})",
    )
    parser.add_argument(
        "--arm-speed",
        type=int,
        default=SERVO_SPEED,
        metavar="S",
        help=f"Servo speed for servos 1/2 (default: {SERVO_SPEED})",
    )
    parser.add_argument(
        "--servo3",
        action="store_true",
        help="Also cycle servo 3 between min and max after testing servos 1/2",
    )
    parser.add_argument(
        "--servo3-min",
        type=int,
        default=SERVO3_MIN_DEFAULT,
        dest="servo3_min",
        metavar="POS",
        help=f"Servo 3 minimum position (default: {SERVO3_MIN_DEFAULT})",
    )
    parser.add_argument(
        "--servo3-max",
        type=int,
        default=SERVO3_MAX_DEFAULT,
        dest="servo3_max",
        metavar="POS",
        help=f"Servo 3 maximum position (default: {SERVO3_MAX_DEFAULT})",
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

    from sedge import edge
    from uservice import service
    from uteensy import start_teensy_interface, stop_teensy_interface

    # uservice.setup() calls parse_args(); clear argv so only script name remains
    sys.argv = [sys.argv[0]]

    setproctitle("test_servo")
    start_teensy_interface()
    service.setup(MQTT_HOST)

    if not service.connected:
        print("% MQTT not connected - exiting")
        stop_teensy_interface()
        sys.exit(1)

    service.args.now = True
    if not cli.verbose:
        service.args.silent = True

    if cli.servo12_max > 320:
        print(f"% Refusing servo12-max={cli.servo12_max}: too aggressive for safe test (max 320).")
        print("% Use <= 320 for testing to avoid arm slam.")
        stop_teensy_interface()
        sys.exit(2)
    if cli.arm_speed > 120:
        print(f"% Refusing arm-speed={cli.arm_speed}: too aggressive for safe test (max 120).")
        stop_teensy_interface()
        sys.exit(2)

    try:
        run_cycle(
            cli.pause,
            cli.hold_down,
            cli.servo12_min,
            cli.servo12_max,
            cli.arm_speed,
        )
        if cli.servo3:
            run_servo3_cycle(
                cli.servo3_dwell,
                cli.servo3_min,
                cli.servo3_max,
                cli.servo3_speed,
            )
    except KeyboardInterrupt:
        print("\n% Ctrl+C - parking servos")
        service.stop = True
    finally:
        print("% Stopping drive; arm parked up")
        try:
            service.send("robobot/cmd/ti", "rc 0 0")
            time.sleep(0.05)
            servo_move(cli.servo12_min, cli.arm_speed)
            if cli.servo3:
                servo3_move(cli.servo3_min, cli.servo3_speed)
            edge.lineControl(0)
        except Exception:
            pass

    service.terminate()
    stop_teensy_interface()
    if cli.verbose:
        print("% test_servo: done")


if __name__ == "__main__":
    main()
