#!/usr/bin/env python3
"""Minimal creep tester — raw paho MQTT, no uservice/srobot.

Bypasses the srobot heartbeat watchdog so the script keeps running even if
teensy_interface is silent. Listens to robobot/kalman/state for pose feedback
and publishes 'rc <v> <w>' on robobot/cmd/ti.

Note: motors still need teensy_interface running on the Pi to actually move.
This script just lets you send commands and observe whatever pose comes back.

Usage:
    python3 -m control.tasks.creep_test_raw [robot-ip]
"""
from __future__ import annotations
import json
import sys
import time as t

from paho.mqtt import client as mqtt_client

DEFAULT_IP = "10.197.219.117"


class State:
    x = 0.0
    y = 0.0
    yaw = 0.0
    last_msg_time = 0.0


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        State.x = data["position"]["x"]
        State.y = data["position"]["y"]
        State.yaw = data["orientation"]["yaw"]
        State.last_msg_time = t.time()
    except Exception:
        pass


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP

    if hasattr(mqtt_client, "CallbackAPIVersion"):
        client = mqtt_client.Client(
            client_id="creep-test-raw",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        client = mqtt_client.Client(client_id="creep-test-raw")
    client.on_message = on_message
    client.connect(ip, 1883, 60)
    client.subscribe("robobot/kalman/state")
    client.loop_start()
    t.sleep(1.0)

    age = t.time() - State.last_msg_time if State.last_msg_time else float("inf")
    print(f"\n[creep-raw] connected to {ip}")
    print(f"[creep-raw] kalman pose: x={State.x:.2f} y={State.y:.2f} yaw_deg={State.yaw*57.3:.1f}  (msg age {age:.1f}s)")
    print(f"[creep-raw] format: <dist_m> [speed_m/s]   'q' to quit\n")

    try:
        while True:
            try:
                line = input("dist speed > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line or line.lower() in ("q", "quit", "exit"):
                break

            parts = line.split()
            try:
                dist = float(parts[0])
                speed = float(parts[1]) if len(parts) > 1 else 0.05
            except (ValueError, IndexError):
                print("  ! enter: <dist_m> [speed_m/s]   e.g. 0.08 0.05")
                continue

            if abs(dist) > 0.5 or abs(speed) > 0.2:
                print("  ! safety cap: |dist|<=0.5 m, |speed|<=0.2 m/s")
                continue

            x0, y0, h0 = State.x, State.y, State.yaw
            print(f"  start: x={x0:+.3f} y={y0:+.3f} yaw_deg={h0*57.3:+.1f}")

            t0 = t.time()
            timeout_s = abs(dist) / max(abs(speed), 0.01) * 3.0 + 3.0
            last_log = t0

            try:
                while True:
                    client.publish("robobot/cmd/ti", f"rc {speed:.3f} 0.000")
                    t.sleep(0.05)
                    dx, dy = State.x - x0, State.y - y0
                    travelled = (dx * dx + dy * dy) ** 0.5
                    now = t.time()
                    if travelled >= abs(dist):
                        print(f"  reached  travelled={travelled:.3f}")
                        break
                    if now - t0 > timeout_s:
                        print(f"  ! timeout {timeout_s:.1f}s — travelled={travelled:.3f}")
                        break
                    if now - last_log > 0.5:
                        msg_age = now - State.last_msg_time
                        print(f"  ... travel={travelled:.3f}  x={State.x:+.3f} y={State.y:+.3f}  pose_age={msg_age:.2f}s")
                        last_log = now
            except KeyboardInterrupt:
                print("  ^C — stopping")

            client.publish("robobot/cmd/ti", "rc 0.000 0.000")
            t.sleep(0.3)
            print(f"  end:   x={State.x:+.3f} y={State.y:+.3f} yaw_deg={State.yaw*57.3:+.1f}")
            print(f"  delta: dx={State.x-x0:+.3f} dy={State.y-y0:+.3f} dyaw={(State.yaw-h0)*57.3:+.1f}\n")
    finally:
        client.publish("robobot/cmd/ti", "rc 0.000 0.000")
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("[creep-raw] done.")


if __name__ == "__main__":
    main()
