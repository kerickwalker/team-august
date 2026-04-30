#!/usr/bin/env python3

# Simple diagnostic script for checking whether servo 3 commands are reaching the robot.
#
# Usage:
#   python3 servo3_diag.py <broker-host> [--port PORT] [--servo-id ID] [--speed SPEED]
#
# Example:
#   python3 servo3_diag.py 10.197.217.81 --servo-id 3 --speed 0

import argparse
import sys
import time

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("Error: paho-mqtt is required. Install with 'pip install paho-mqtt'.")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Servo 3 MQTT diagnostic tool")
    parser.add_argument("host", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--servo-id", type=int, default=3, help="Servo channel to test")
    parser.add_argument("--speed", type=int, default=0, help="Servo speed (0=full speed)")
    parser.add_argument("--topic", default="robobot/cmd/T0", help="Command topic")
    parser.add_argument("--subscribe", default="robobot/#", help="MQTT topic to subscribe for replies")
    parser.add_argument("--manual", action="store_true", help="Enter manual command mode after connecting")
    return parser.parse_args()


def make_client(client_id):
    try:
        return mqtt_client.Client(client_id)
    except Exception:
        return mqtt_client.Client()


def main():
    args = parse_args()
    client = make_client("servo3-diag")

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"% Connected to MQTT broker {args.host}:{args.port}")
            client.subscribe(args.subscribe)
            print(f"% Subscribed to '{args.subscribe}'")
        else:
            print(f"Error: MQTT connection failed with rc={rc}")
            sys.exit(1)

    def on_message(client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="replace")
        print(f"< {msg.topic} {payload}")

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.host, args.port)
    except Exception as exc:
        print(f"Error: could not connect to {args.host}:{args.port} — {exc}")
        sys.exit(1)

    client.loop_start()
    time.sleep(0.5)

    def publish(cmd):
        print(f"> {args.topic} {cmd}")
        client.publish(args.topic, cmd)
        time.sleep(1.0)

    if args.manual:
        print("% Manual mode. Type raw servo commands or 'quit' to exit.")
        print("% Example: servo 3 0 0")
        while True:
            try:
                raw = input("servo3> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not raw:
                continue
            if raw.lower() in ("quit", "exit", "q"):
                break
            if raw.startswith("servo "):
                publish(raw)
            else:
                print("% Expected raw command format: servo <id> <position> <speed>")
        client.loop_stop()
        client.disconnect()
        return

    print("% Sending diagnostic servo commands. Watch for any response messages.")
    publish(f"servo {args.servo_id} 0 {args.speed}")
    publish(f"servo {args.servo_id} 900 {args.speed}")
    publish(f"servo {args.servo_id} -900 {args.speed}")
    publish(f"servo {args.servo_id} 0 {args.speed}")

    print("% Waiting 5 seconds for any replies...")
    time.sleep(5)
    client.loop_stop()
    client.disconnect()
    print("% Done.")


if __name__ == "__main__":
    main()
