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
#   python3 test_servo.py --servo3 --set-pos 0 --servo3-speed 1000
#   python3 test_servo.py --servo3 --servo3-min -400 --servo3-max 400
#   python3 test_servo.py --kill
#   python3 test_servo.py --servo3 --kill

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
POS_OPEN = -400 # -475
POS_CLOSED = 260 # 260
SERVO_SPEED = 100 # 70
SERVO_KILL_POSITION = 10000
SERVO_KILL_SPEED = 0

# Arm servo limits after recalibration:
# Servo 1 up position is -400; decreasing it makes it go higher / closer to body.
# Servo 2 is mirrored, so servo 1 -400 means servo 2 400.
# Lowest arm position is servo 1 200 and servo 2 -200.
SERVO1_MIN = -400
SERVO1_MAX = 260
SERVO2_MIN = -260
SERVO2_MAX = 400

# Third servo (gripper / extra): tune --servo3-min / --servo3-max on your hardware
SERVO3_ID = 3
SERVO3_MIN_DEFAULT = -500
SERVO3_MAX_DEFAULT = 500


def send_servo_command(servo_id, position, speed):
    command = f"servo {servo_id} {position} {speed}"
    print(f"% Sending {command}")
    service.send("robobot/cmd/T0", command)


def servo_move(position, speed):
    send_servo_command(SERVO_ID, position, speed)
    send_servo_command(SERVO_MIRROR_ID, -position, speed)


def servo_kill(servo_id):
    send_servo_command(servo_id, SERVO_KILL_POSITION, SERVO_KILL_SPEED)


def kill_arm_servos():
    servo_kill(SERVO_ID)
    servo_kill(SERVO_MIRROR_ID)


def servo_set_literal(position, speed, servo_mode):
    if servo_mode == "servo3":
        send_servo_command(SERVO3_ID, position, speed)
    else:
        servo_move(position, speed)


def literal_move_label(position, speed, servo_mode):
    if servo_mode == "servo3":
        return f"servo {SERVO3_ID} {position} {speed}"
    return (f"servo {SERVO_ID} {position} {speed}, "
            f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def arm_move_label(position, speed, servo_mode):
    return (f"servo {SERVO_ID} {position} {speed}, "
            f"servo {SERVO_MIRROR_ID} {-position} {speed}")


def run_cycle(pause_s, hold_down_s, open_pos, closed_pos, speed, servo_mode):
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    print(f"% Arm UP    ({arm_move_label(open_pos, speed, servo_mode)})")
    servo_move(open_pos, speed)
    time.sleep(pause_s)
    print(f"% Arm DOWN  ({arm_move_label(closed_pos, speed, servo_mode)}) - holding {hold_down_s:.1f}s")
    servo_move(closed_pos, speed)
    time.sleep(hold_down_s)
    print(f"% Arm UP    ({arm_move_label(open_pos, speed, servo_mode)}) - final park")
    servo_move(open_pos, speed)
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
    ok_min = validate_position("servo12-min", cli.servo12_min, SERVO1_MIN, SERVO1_MAX)
    ok_max = validate_position("servo12-max", cli.servo12_max, SERVO1_MIN, SERVO1_MAX)
    return ok_min and ok_max


def validate_single_position(position, servo_mode):
    if servo_mode == "both":
        return validate_position("servo1 set-pos", position, SERVO1_MIN, SERVO1_MAX)
    return True


def refuse_independent_linked_servo(cli):
    if cli.servo1 or cli.servo2:
        requested = "--servo1" if cli.servo1 else "--servo2"
        print(f"% Refusing {requested}: servo 1 and servo 2 are mechanically linked.")
        print("% Move them together with the default arm test or --set-pos without --servo1/--servo2.")
        return True
    return False


def servo3_move(position, speed):
    send_servo_command(SERVO3_ID, position, speed)


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
        help="Refuse and exit: servo 1 is linked to servo 2",
    )
    servo_mode.add_argument(
        "--servo2",
        action="store_true",
        help="Refuse and exit: servo 2 is linked to servo 1",
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
        help="Send one position command and exit; with --servo3 this targets servo 3",
    )
    parser.add_argument(
        "--kill",
        action="store_true",
        help="Disable PWM power and exit; default kills linked servos 1+2, with --servo3 kills servo 3",
    )
    parser.add_argument(
        "--servo3",
        action="store_true",
        help="Cycle servo 3 between min and max, or target servo 3 with --set-pos/--kill",
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

    if refuse_independent_linked_servo(cli):
        sys.exit(2)

    servo_mode = "both"
    if cli.servo3:
        servo_mode = "servo3"

    if cli.kill:
        speed = SERVO_KILL_SPEED
    elif cli.set_pos is not None:
        speed = cli.servo3_speed if servo_mode == "servo3" else cli.arm_speed
        if not validate_single_position(cli.set_pos, servo_mode):
            sys.exit(2)
    elif servo_mode == "both":
        speed = cli.arm_speed
        if not validate_arm_cycle(cli, servo_mode):
            sys.exit(2)
    else:
        speed = cli.servo3_speed

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
        if cli.kill:
            if servo_mode == "servo3":
                print(f"% Killing servo {SERVO3_ID} PWM")
                servo_kill(SERVO3_ID)
            else:
                print(f"% Killing linked servos {SERVO_ID}+{SERVO_MIRROR_ID} PWM")
                kill_arm_servos()
            time.sleep(cli.pause)
        elif cli.set_pos is not None:
            run_single_position(
                cli.set_pos,
                speed,
                servo_mode,
                cli.pause,
            )
        elif servo_mode == "servo3":
            run_servo3_cycle(
                cli.servo3_dwell,
                cli.servo3_min,
                cli.servo3_max,
                cli.servo3_speed,
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
    except KeyboardInterrupt:
        print("\n% Ctrl+C - parking servos")
        service.stop = True
    finally:
        if cli.kill:
            print("% Stopping drive; servo PWM disabled as requested")
        elif cli.set_pos is None and servo_mode == "both":
            print("% Stopping drive; arm parked up")
        elif cli.set_pos is not None:
            print("% Stopping drive; leaving servo at requested position")
        else:
            print("% Stopping drive; servo 3 parked at min")
        try:
            service.send("robobot/cmd/ti", "rc 0 0")
            time.sleep(0.05)
            if cli.set_pos is None and not cli.kill and servo_mode == "both":
                servo_move(cli.servo12_min, cli.arm_speed)
            if cli.set_pos is None and not cli.kill and servo_mode == "servo3":
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
