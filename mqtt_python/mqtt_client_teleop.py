# mqtt_client_teleop.py — Keyboard teleoperation + waypoint capture
#
# Run with:
#   python3 mqtt_client_teleop.py
#
# Prerequisites:
#   - teensy_interface must be running
#   - Kalman filter node must be publishing on robobot/kalman/state
#
# Controls:
#   w / s        Forward / Backward   (0.15 m/s)
#   a / d        Turn left / right    (0.5 rad/s, in place)
#   Space        Stop
#   c            Capture current pose as waypoint
#   q / Ctrl-C   Quit and save waypoints to teleop_waypoints.csv
#
# Coordinate conventions (saved CSV):
#   x            Forward   (Kalman frame, unchanged)
#   y            Right     (negated from Kalman left-positive → matches trajectory.csv)
#   heading      CCW+      (Kalman frame, unchanged — matches trajectory.csv convention)
#   cumulative_dist  metres between consecutive captured waypoints (cumulative)
#
# NOTE: These are sparse waypoints captured manually.  They are NOT a dense
#       trajectory suitable for direct Pure Pursuit tracking.  Use them as
#       reference points to design or spline-interpolate a trajectory.csv.

import csv
import json
import math
import platform
import select
import sys
import time as t
import tty
import termios
from datetime import datetime
from setproctitle import setproctitle

from paho.mqtt import client as mqtt_client

from skalman import kalman
from spose import pose
from uservice import service

# ── Teleop speeds ────────────────────────────────────────────────────────────
LINVEL   = 0.15   # m/s  — forward / backward speed
TURNRATE = 0.5    # rad/s — turn-in-place rate

# ── Output filename ──────────────────────────────────────────────────────────
OUTPUT_CSV = "teleop_waypoints.csv"


# ════════════════════════════════════════════════════════════════════════════
# STeleop — dedicated MQTT subscriber for Kalman pose
# Mirrors the spath_follow.py pattern exactly.
# ════════════════════════════════════════════════════════════════════════════
class STeleop:
    def __init__(self):
        self.x             = 0.0
        self.y             = 0.0
        self.heading       = 0.0
        self.speed         = 0.0
        self.pose_received = False
        self._client       = None

    def setup(self):
        if platform.system() == "Windows":
            self._client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
        else:
            try:
                self._client = mqtt_client.Client(client_id="teleop-in")
            except Exception:
                self._client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)

        self._client.on_message = self._on_kalman_message
        try:
            self._client.connect("localhost", 1883)
            self._client.loop_start()
            self._client.subscribe("robobot/kalman/state", qos=1)
            print("% STeleop: subscribed to robobot/kalman/state")
        except Exception as e:
            print(f"% STeleop: WARNING — could not connect to broker: {e}")

    def _on_kalman_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            if "x" in data:
                s = data["x"]
                self.x       = s.get("x", 0.0)
                self.y       = s.get("y", 0.0)
                self.heading = s.get("yaw", 0.0)
                self.speed   = s.get("velocity", 0.0)
            elif "position" in data:
                self.x       = data["position"].get("x", 0.0)
                self.y       = data["position"].get("y", 0.0)
                self.heading = data.get("orientation", {}).get("yaw", 0.0)
                self.speed   = data.get("velocity", {}).get("linear", 0.0)
            self.pose_received = True
        except Exception as e:
            print(f"% STeleop: error parsing Kalman message: {e}")

    def terminate(self):
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass


teleop = STeleop()


# ════════════════════════════════════════════════════════════════════════════
# Keyboard helpers
# ════════════════════════════════════════════════════════════════════════════
def get_key():
    """Return a pending keypress (lowercase) or None — non-blocking."""
    if select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        return ch.lower() if ch else None
    return None


BANNER = """\
┌─────────────────────────────────────────────────┐
│        TELEOPERATION + WAYPOINT CAPTURE         │
│                                                 │
│  w / s       Forward / Backward (0.15 m/s)      │
│  a / d       Turn left / right  (0.5 rad/s)     │
│  Space       Stop                               │
│  c           Capture waypoint                   │
│  q / Ctrl-C  Quit and save waypoints            │
└─────────────────────────────────────────────────┘"""


# ════════════════════════════════════════════════════════════════════════════
# Waypoint helpers
# ════════════════════════════════════════════════════════════════════════════
def capture_waypoint(waypoints: list) -> None:
    """Append current Kalman pose to waypoints list (trajectory CSV convention)."""
    x_save   =  teleop.x
    y_save   = -teleop.y        # Kalman +y=left → CSV +y=right
    hdg_save =  teleop.heading  # CCW+ unchanged

    if waypoints:
        prev = waypoints[-1]
        dx   = x_save - prev[0]
        dy   = y_save - prev[1]
        cum  = prev[3] + math.sqrt(dx * dx + dy * dy)
    else:
        cum = 0.0

    waypoints.append([x_save, y_save, hdg_save, cum])
    print(f"\r% [WP {len(waypoints):3d}]  "
          f"x={x_save:.3f} m  y={y_save:.3f} m  "
          f"hdg={hdg_save:.3f} rad  cum={cum:.3f} m")


def save_waypoints(waypoints: list) -> None:
    if not waypoints:
        print("% No waypoints captured — nothing saved.")
        return
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "heading", "cumulative_dist"])
        writer.writerows(waypoints)
    print(f"% Saved {len(waypoints)} waypoints → {OUTPUT_CSV}")
    print(f"%   Total path length: {waypoints[-1][3]:.3f} m")
    print(f"%   NOTE: sparse waypoints — NOT a dense trajectory.")
    print(f"%         Use as reference to design trajectory.csv.")


# ════════════════════════════════════════════════════════════════════════════
# Main teleoperation drive loop
# ════════════════════════════════════════════════════════════════════════════
def drive_teleop():
    waypoints   = []
    last_print  = datetime.now()
    last_cmd    = ""
    state       = 0

    while not service.stop:
        # ── State 0: initialise ──────────────────────────────────────────
        if state == 0:
            print(BANNER)
            for i in range(3, 0, -1):
                print(f"  Starting in {i} ...", end="\r", flush=True)
                t.sleep(1.0)
            print(" " * 20, end="\r")  # clear countdown line
            pose.tripBreset()
            kalman.reset()
            service.send("robobot/cmd/T0", "leds 16 100 50 0")  # amber = teleop
            while not teleop.pose_received and not service.stop:
                t.sleep(0.1)
            print("Ready — script is now ready to be used to capture waypoints.")
            state = 1

        # ── State 1: control loop ────────────────────────────────────────
        elif state == 1:
            # Periodic pose print (every 0.5 s)
            now = datetime.now()
            if (now - last_print).total_seconds() >= 1.0:
                x_d   =  teleop.x
                y_d   = -teleop.y        # display: positive = right
                hdg_d = -teleop.heading  # display: positive = right turn
                wp_n  = len(waypoints)
                print(f"\r% x={x_d:7.3f} m  y={y_d:7.3f} m  "
                      f"hdg={hdg_d:6.3f} rad  "
                      f"wps={wp_n:3d}  [{last_cmd}]   ", end="", flush=True)
                last_print = now

            # Keyboard → rc command
            key = get_key()
            if key == "w":
                service.send("robobot/cmd/ti", f"rc  {LINVEL:.3f}  0.000")
                last_cmd = "FWD"
            elif key == "s":
                service.send("robobot/cmd/ti", f"rc -{LINVEL:.3f}  0.000")
                last_cmd = "BWD"
            elif key == "a":
                service.send("robobot/cmd/ti", f"rc  0.000  {TURNRATE:.3f}")
                last_cmd = "LEFT"
            elif key == "d":
                service.send("robobot/cmd/ti", f"rc  0.000 -{TURNRATE:.3f}")
                last_cmd = "RIGHT"
            elif key == " ":
                service.send("robobot/cmd/ti", "rc  0.000  0.000")
                last_cmd = "STOP"
            elif key == "c":
                if teleop.pose_received:
                    capture_waypoint(waypoints)
                else:
                    print("\r% No pose yet — cannot capture waypoint.")
            elif key in ("q", "\x03"):  # q or Ctrl-C
                state = 99

            t.sleep(0.02)  # ~50 Hz

        # ── State 99: shutdown ───────────────────────────────────────────
        else:
            break

    # Stop robot regardless of how we exited
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    print()  # newline after inline pose print
    save_waypoints(waypoints)


# ════════════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client already running — terminating")
    else:
        setproctitle("mqtt-client")
        # Suppress uservice verbose output — teleop has its own clean display
        if "-s" not in sys.argv and "--silent" not in sys.argv:
            sys.argv.append("-s")
        service.setup("localhost")
        teleop.setup()

        if service.connected:
            # Put terminal in raw mode so keystrokes arrive immediately
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                drive_teleop()
            except KeyboardInterrupt:
                pass
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")

        teleop.terminate()
        service.terminate()
    print("% Teleop terminated")
