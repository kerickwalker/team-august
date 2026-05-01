#!/usr/bin/env python3

import math
import sys
import time as t
from pathlib import Path

try:
    from setproctitle import setproctitle
except ImportError:
    def setproctitle(_title):
        return None

OTHER_MQTT_DIR = Path(__file__).resolve().parent / "other-mqtt"
if str(OTHER_MQTT_DIR) not in sys.path:
    sys.path.insert(0, str(OTHER_MQTT_DIR))

from spose import pose
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface


def stop_robot():
    service.send("robobot/cmd/ti", "rc 0.0 0.0")


def wait_settled(timeout_s=2.0):
    deadline = t.time() + max(0.0, timeout_s)
    while not service.stop and t.time() < deadline:
        if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
            break
        t.sleep(0.02)


def drive_distance(distance_m, speed_mps, timeout_s):
    target = abs(distance_m)
    signed_speed = abs(speed_mps) if distance_m >= 0 else -abs(speed_mps)
    pose.tripBreset()
    t.sleep(0.05)
    print(f"% step 1: odometry drive {distance_m:.3f} m at {signed_speed:.3f} m/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc {signed_speed:.3f} 0.0")
    while not service.stop:
        if abs(pose.tripB) >= target:
            break
        if pose.tripBtimePassed() >= timeout_s:
            print(f"% step 1 timeout after {timeout_s:.2f} s")
            break
        t.sleep(0.02)
    stop_robot()
    wait_settled()
    print(f"# step 1 done: tripB={pose.tripB:.3f} m, time={pose.tripBtimePassed():.2f} s")


def drive_for_time(duration_s, speed_mps):
    pose.tripBreset()
    t.sleep(0.05)
    print(f"% step 2: timed drive {duration_s:.3f} s at {speed_mps:.3f} m/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc {speed_mps:.3f} 0.0")
    start = t.time()
    while not service.stop:
        if t.time() - start >= duration_s:
            break
        t.sleep(0.02)
    stop_robot()
    wait_settled()
    print(f"# step 2 done: tripB={pose.tripB:.3f} m, time={pose.tripBtimePassed():.2f} s")


def turn_degrees(degrees, direction, turn_rate, timeout_s):
    sign = 1.0 if direction == "left" else -1.0
    target_rad = math.radians(abs(degrees))
    signed_rate = sign * abs(turn_rate)
    pose.tripBreset()
    t.sleep(0.05)
    print(f"% step 3: odometry turn {degrees:.1f} deg {direction} at {abs(turn_rate):.3f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc 0.0 {signed_rate:.3f}")
    while not service.stop:
        if abs(pose.tripBh) >= target_rad:
            break
        if pose.tripBtimePassed() >= timeout_s:
            print(f"% step 3 timeout after {timeout_s:.2f} s")
            break
        t.sleep(0.02)
    stop_robot()
    wait_settled()
    print(f"# step 3 done: tripBh={math.degrees(pose.tripBh):.1f} deg, time={pose.tripBtimePassed():.2f} s")


def run_sequence(args):
    print("% odometry test sequence:")
    print(f"%   1) drive {args.distance:.3f} m at {args.distance_speed:.3f} m/s")
    print(f"%   2) drive {args.forward_time:.3f} s at {args.time_speed:.3f} m/s")
    print(f"%   3) turn {args.turn_deg:.1f} deg {args.turn_dir} at {args.turn_rate:.3f} rad/s")
    drive_distance(args.distance, args.distance_speed, args.distance_timeout)
    drive_for_time(args.forward_time, args.time_speed)
    turn_degrees(args.turn_deg, args.turn_dir, args.turn_rate, args.turn_timeout)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    print("% odometry test done")


if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running - terminating")
    else:
        setproctitle("test-odometry")
        service.parser.add_argument("--distance", type=float, default=1.0,
                                    help="Step 1 odometry distance in meters.")
        service.parser.add_argument("--distance-speed", type=float, default=0.20,
                                    help="Step 1 forward speed in m/s.")
        service.parser.add_argument("--distance-timeout", type=float, default=10.0,
                                    help="Step 1 safety timeout in seconds.")
        service.parser.add_argument("--forward-time", type=float, default=2.0,
                                    help="Step 2 timed forward drive duration in seconds.")
        service.parser.add_argument("--time-speed", type=float, default=0.20,
                                    help="Step 2 forward speed in m/s.")
        service.parser.add_argument("--turn-deg", type=float, default=90.0,
                                    help="Step 3 odometry turn angle in degrees.")
        service.parser.add_argument("--turn-dir", choices=("left", "right"), default="left",
                                    help="Step 3 turn direction.")
        service.parser.add_argument("--turn-rate", type=float, default=0.50,
                                    help="Step 3 angular speed in rad/s.")
        service.parser.add_argument("--turn-timeout", type=float, default=6.0,
                                    help="Step 3 safety timeout in seconds.")

        start_teensy_interface()
        service.setup("localhost")
        try:
            if service.connected:
                service.args.now = True
                run_sequence(service.args)
        except KeyboardInterrupt:
            print("\n% Ctrl+C -> aborting odometry test")
            service.stop = True
        finally:
            stop_robot()
            service.send("robobot/cmd/T0", "leds 16 0 0 0")
            service.terminate()
            stop_teensy_interface()
