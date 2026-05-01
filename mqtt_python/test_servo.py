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
#   python3 test_servo.py --servo12-min -400 --servo12-max 200 --arm-speed 70
#   python3 test_servo.py --servo1 --servo12-min -100 --servo12-max 100
#   python3 test_servo.py --servo2 --servo12-min -100 --servo12-max 100
#   python3 test_servo.py --servo1 --set-pos -400 --arm-speed 1000
#   python3 test_servo.py --servo3 --set-pos 0 --servo3-speed 1000
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

# Defaults for calibration/testing
SERVO_ID = 1
SERVO_MIRROR_ID = 2
POS_OPEN = -400
POS_CLOSED = 200
SERVO_SPEED = 70

# Arm servo limits after recalibration:
# Servo 1 up position is -400; decreasing it makes it go higher / closer to body.
# Servo 2 is mirrored, so servo 1 -400 means servo 2 400.
# Lowest arm position is servo 1 200 and servo 2 -200.
SERVO1_MIN = -400
SERVO1_MAX = 200
SERVO2_MIN = -200
SERVO2_MAX = 400

# Third servo (gripper / extra): tune --servo3-min / --servo3-max on your hardware
SERVO3_ID = 3
SERVO3_MIN_DEFAULT = -500
SERVO3_MAX_DEFAULT = 500


def servo_move(position, speed, servo_mode):
    if servo_mode in ("both", "servo1"):
        service.send("robobot/cmd/T0", f"servo {SERVO_ID} {position} {speed}")
    if servo_mode in ("both", "servo2"):
        service.send("robobot/cmd/T0", f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def servo_set_literal(position, speed, servo_mode):
    if servo_mode == "servo1":
        service.send("robobot/cmd/T0", f"servo {SERVO_ID} {position} {speed}")
    elif servo_mode == "servo2":
        service.send("robobot/cmd/T0", f"servo {SERVO_MIRROR_ID} {position} {speed}")
    elif servo_mode == "servo3":
        service.send("robobot/cmd/T0", f"servo {SERVO3_ID} {position} {speed}")
    else:
        service.send("robobot/cmd/T0", f"servo {SERVO_ID} {position} {speed}")
        service.send("robobot/cmd/T0", f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def literal_move_label(position, speed, servo_mode):
    if servo_mode == "servo1":
        return f"servo {SERVO_ID} {position} {speed}"
    if servo_mode == "servo2":
        return f"servo {SERVO_MIRROR_ID} {position} {speed}"
    if servo_mode == "servo3":
        return f"servo {SERVO3_ID} {position} {speed}"
    return (f"servo {SERVO_ID} {position} {speed}, "
            f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def arm_move_label(position, speed, servo_mode):
    if servo_mode == "servo1":
        return f"servo {SERVO_ID} {position} {speed}"
    if servo_mode == "servo2":
        return f"servo {SERVO_MIRROR_ID} {-position} {speed}"
    return (f"servo {SERVO_ID} {position} {speed}, "
            f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def run_cycle(pause_s, hold_down_s, open_pos, closed_pos, speed, servo_mode):
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    print(f"% Arm UP    ({arm_move_label(open_pos, speed, servo_mode)})")
    servo_move(open_pos, speed, servo_mode)
    time.sleep(pause_s)
    print(f"% Arm DOWN  ({arm_move_label(closed_pos, speed, servo_mode)}) - holding {hold_down_s:.1f}s")
    servo_move(closed_pos, speed, servo_mode)
    time.sleep(hold_down_s)
    print(f"% Arm UP    ({arm_move_label(open_pos, speed, servo_mode)}) - final park")
    servo_move(open_pos, speed, servo_mode)
    time.sleep(pause_s)


def run_single_position(position, speed, servo_mode, hold_s):
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    print(f"% Set position ({literal_move_label(position, speed, servo_mode)})")
    servo_set_literal(position, speed, servo_mode)
    time.sleep(hold_s)


def validate_position(name, position, pos_min, pos_max):
    if position < pos_min or position > pos_max:
        print(f"% Refusing {name}={position}: allowed range is {pos_min}..{pos_max}.")
        return False
    return True


def validate_arm_cycle(cli, servo_mode):
    if servo_mode in ("both", "servo1"):
        ok_min = validate_position("servo12-min", cli.servo12_min, SERVO1_MIN, SERVO1_MAX)
        ok_max = validate_position("servo12-max", cli.servo12_max, SERVO1_MIN, SERVO1_MAX)
        return ok_min and ok_max
    if servo_mode == "servo2":
        servo2_min = -cli.servo12_min
        servo2_max = -cli.servo12_max
        ok_min = validate_position("mirrored servo2 min", servo2_min, SERVO2_MIN, SERVO2_MAX)
        ok_max = validate_position("mirrored servo2 max", servo2_max, SERVO2_MIN, SERVO2_MAX)
        return ok_min and ok_max
    return True


def validate_single_position(position, servo_mode):
    if servo_mode in ("both", "servo1"):
        return validate_position("servo1 set-pos", position, SERVO1_MIN, SERVO1_MAX)
    if servo_mode == "servo2":
        return validate_position("servo2 set-pos", position, SERVO2_MIN, SERVO2_MAX)
    return True


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
        epilog=(
            "Calibration note: servo1 up position is -400 and decreasing it "
            "makes it go higher / closer to body. Servo2 is mirrored: servo1 "
            "-400 equals servo2 400, and servo1 200 equals servo2 -200."
        ),
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
    servo_mode = parser.add_mutually_exclusive_group()
    servo_mode.add_argument(
        "--servo1",
        action="store_true",
        help="Only cycle servo 1 during the arm test",
    )
    servo_mode.add_argument(
        "--servo2",
        action="store_true",
        help="Only cycle servo 2 during the arm test; positions are mirrored like the paired test",
    )
    parser.add_argument(
        "--servo12-min",
        "--open-pos",
        type=int,
        default=POS_OPEN,
        dest="servo12_min",
        metavar="POS",
        help=f"Servo 1 up position; servo 2 uses the mirrored value (default: {POS_OPEN})",
    )
    parser.add_argument(
        "--servo12-max",
        "--closed-pos",
        type=int,
        default=POS_CLOSED,
        dest="servo12_max",
        metavar="POS",
        help=f"Servo 1 down position; servo 2 uses the mirrored value (default: {POS_CLOSED})",
    )
    parser.add_argument(
        "--arm-speed",
        type=int,
        default=SERVO_SPEED,
        metavar="S",
        help=f"Servo speed for servos 1/2 (default: {SERVO_SPEED})",
    )
    parser.add_argument(
        "--set-pos",
        type=int,
        default=None,
        metavar="POS",
        help="Send one literal position command and exit; with --servo2/--servo3 this is not mirrored",
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

    servo_mode = "both"
    if cli.servo1:
        servo_mode = "servo1"
    elif cli.servo2:
        servo_mode = "servo2"
    elif cli.servo3 and cli.set_pos is not None:
        servo_mode = "servo3"

    if cli.set_pos is not None:
        speed = cli.servo3_speed if servo_mode == "servo3" else cli.arm_speed
        if not validate_single_position(cli.set_pos, servo_mode):
            sys.exit(2)
    else:
        speed = cli.arm_speed
        if not validate_arm_cycle(cli, servo_mode):
            sys.exit(2)

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

    try:
        if cli.set_pos is not None:
            run_single_position(
                cli.set_pos,
                speed,
                servo_mode,
                cli.pause,
            )
        else:
            run_cycle(
                cli.pause,
                cli.hold_down,
                cli.servo12_min,
                cli.servo12_max,
                cli.arm_speed,
                servo_mode,
            )
        if cli.servo3 and cli.set_pos is None:
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
        if cli.set_pos is None:
            print("% Stopping drive; arm parked up")
        else:
            print("% Stopping drive; leaving servo at requested position")
        try:
            service.send("robobot/cmd/ti", "rc 0 0")
            time.sleep(0.05)
            if cli.set_pos is None:
                servo_move(cli.servo12_min, cli.arm_speed, servo_mode)
            if cli.servo3 and cli.set_pos is None:
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
