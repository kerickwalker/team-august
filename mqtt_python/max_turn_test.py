#!/usr/bin/env python3

"""Constant turn-rate test for Robobot.

This script sends a constant in-place turn command:
  rc 0.0 <turn_speed>

Use it to compare numeric turn-speed values (for example around 5.0) and
observe actual measured turnrate/angle from odometry.
"""

import time as t
import sys
from pathlib import Path
from setproctitle import setproctitle

OTHER_MQTT_DIR = Path(__file__).resolve().parent / "other-mqtt"
if str(OTHER_MQTT_DIR) not in sys.path:
    sys.path.insert(0, str(OTHER_MQTT_DIR))

from spose import pose
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface


def run_constant_turn(turn_speed, duration_s, sample_dt=0.1):
    """Run in-place turn at constant angular speed for a fixed duration."""
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    pose.tripBreset()
    t.sleep(0.05)

    print(f"% constant turn start: w={turn_speed:.3f} rad/s for {duration_s:.2f} s")
    service.send("robobot/cmd/ti", f"rc 0.0 {turn_speed:.3f}")

    next_print = 0.0
    while not service.stop:
        elapsed = pose.tripBtimePassed()
        if elapsed >= duration_s:
            break
        if elapsed >= next_print:
            measured_w = pose.turnrate()
            measured_deg = pose.tripBh * 180.0 / 3.141592653589793
            print(
                f"# t={elapsed:5.2f}s  cmd_w={turn_speed:6.3f}  "
                f"meas_w={measured_w:6.3f} rad/s  heading={measured_deg:7.2f} deg"
            )
            next_print += max(0.02, sample_dt)
        t.sleep(0.01)

    service.send("robobot/cmd/ti", "rc 0.0 0.0")

    # Wait briefly for the robot to settle.
    settle_deadline = t.time() + 2.0
    while not service.stop and t.time() < settle_deadline:
        if abs(pose.turnrate()) < 0.02 and abs(pose.velocity()) < 0.01:
            break
        t.sleep(0.02)

    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    turned_deg = pose.tripBh * 180.0 / 3.141592653589793
    print(
        f"% constant turn done: turned={turned_deg:.2f} deg in "
        f"{pose.tripBtimePassed():.2f} s"
    )


if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running - terminating")
    else:
        setproctitle("mqtt-client")

        service.parser.add_argument(
            "--turn-speed",
            type=float,
            default=5.0,
            help="Angular command w for 'rc 0.0 w' (rad/s). Negative = right turn.",
        )
        service.parser.add_argument(
            "--duration",
            type=float,
            default=2.0,
            help="How long to hold the constant turn command (seconds).",
        )
        service.parser.add_argument(
            "--sample-dt",
            type=float,
            default=0.1,
            help="Print interval for odometry diagnostics (seconds).",
        )

        start_teensy_interface()
        service.setup("localhost")
        try:
            if service.connected:
                service.args.now = True
                run_constant_turn(
                    turn_speed=float(service.args.turn_speed),
                    duration_s=max(0.1, float(service.args.duration)),
                    sample_dt=max(0.02, float(service.args.sample_dt)),
                )
        except KeyboardInterrupt:
            print("\n% Ctrl+C -> aborting max turn test")
            service.stop = True
        finally:
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.send("robobot/cmd/T0", "leds 16 0 0 0")
            service.terminate()
            stop_teensy_interface()
