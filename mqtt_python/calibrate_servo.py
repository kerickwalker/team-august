#!/usr/bin/env python3

# Interactive servo sweep tool for finding arm positions.
#
# Connects directly to the robot's MQTT broker and sends servo commands
# so you can observe arm movement and record the positions for sservo.py.
#
# Usage:
#   python3 calibrate_servo.py <robot-ip>
#   python3 calibrate_servo.py 10.197.217.81
#
# At the prompt, type servo commands or shortcuts:
#   servo <id> <position> <speed>   — send a raw servo command
#   <position>                      — resend last id/speed with new position
#   open   <position>               — record the "open/scoop" position
#   carry  <position>               — record the "carry" position
#   putt   <position>               — record the "putt/release" position
#   id     <n>                      — change the default servo id
#   speed  <n>                      — change the default speed
#   show                            — print currently recorded positions
#   save                            — print values to copy into sservo.py
#   q / quit                        — exit

import sys
import time
import platform
from paho.mqtt import client as mqtt_client

TOPIC = "robobot/cmd/T0"
PORT  = 1883

# Recorded positions — fill these in as you sweep
positions = {
    "open":  None,   # arm lowered to scoop up ball
    "carry": None,   # arm raised to carry ball
    "putt":  None,   # arm releases ball into hole
}


def connect(host):
    try:
        if platform.system() == "Windows":
            c = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
        else:
            try:
                c = mqtt_client.Client("servo-calibrate")
            except Exception:
                c = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
    except Exception:
        c = mqtt_client.Client()

    def on_connect(client, userdata, flags, rc, *args):
        if rc == 0:
            print(f"% Connected to MQTT broker at {host}:{PORT}")
        else:
            print(f"% Connection failed (rc={rc})")
            sys.exit(1)

    c.on_connect = on_connect
    try:
        c.connect(host, PORT)
    except Exception as e:
        print(f"% Could not connect to {host}:{PORT} — {e}")
        sys.exit(1)
    c.loop_start()
    time.sleep(0.5)   # give connection time to complete
    return c


def send_servo(client, servo_id, position, speed):
    cmd = f"servo {servo_id} {position} {speed}"
    client.publish(TOPIC, cmd)
    print(f"  → sent: {cmd}")


def print_positions():
    print("\n  Recorded positions:")
    for name, val in positions.items():
        print(f"    {name:6s} = {val}")
    print()


def print_save(servo_id, speed):
    print("\n--- Copy these values into sservo.py ---")
    print(f"    servo_id      = {servo_id}")
    print(f"    servo_speed   = {speed}")
    for name, val in positions.items():
        print(f"    pos_{name:6s}  = {val}")
    print("----------------------------------------\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 calibrate_servo.py <robot-ip>")
        sys.exit(1)

    host = sys.argv[1]
    client = connect(host)

    # Defaults — adjust id once you know the right channel
    servo_id    = 1
    servo_speed = 200

    print(f"\n% Servo sweep tool — sending to topic '{TOPIC}'")
    print(f"% Default: id={servo_id}  speed={servo_speed}")
    print("% Commands:")
    print("%   servo <id> <pos> <speed>  — raw command")
    print("%   <pos>                     — move to position (uses current id/speed)")
    print("%   open/carry/putt <pos>     — move and record named position")
    print("%   id <n>  /  speed <n>      — change defaults")
    print("%   show  /  save  /  q\n")

    while True:
        try:
            raw = input("servo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            break

        elif cmd == "servo" and len(parts) == 4:
            try:
                sid   = int(parts[1])
                pos   = int(parts[2])
                spd   = int(parts[3])
                servo_id    = sid
                servo_speed = spd
                send_servo(client, sid, pos, spd)
            except ValueError:
                print("  ! Usage: servo <id> <position> <speed>")

        elif cmd in ("open", "carry", "putt") and len(parts) == 2:
            try:
                pos = int(parts[1])
                positions[cmd] = pos
                send_servo(client, servo_id, pos, servo_speed)
                print(f"  ✓ Recorded '{cmd}' = {pos}")
            except ValueError:
                print(f"  ! Usage: {cmd} <position>")

        elif cmd == "id" and len(parts) == 2:
            try:
                servo_id = int(parts[1])
                print(f"  ✓ Servo id set to {servo_id}")
            except ValueError:
                print("  ! Usage: id <n>")

        elif cmd == "speed" and len(parts) == 2:
            try:
                servo_speed = int(parts[1])
                print(f"  ✓ Speed set to {servo_speed}")
            except ValueError:
                print("  ! Usage: speed <n>")

        elif cmd == "show":
            print_positions()

        elif cmd == "save":
            print_save(servo_id, servo_speed)

        else:
            # Try interpreting a bare integer as a position shortcut
            try:
                pos = int(raw)
                send_servo(client, servo_id, pos, servo_speed)
            except ValueError:
                print(f"  ! Unknown command: '{raw}'")

    client.loop_stop()
    client.disconnect()
    print("% Disconnected.")
    if any(v is not None for v in positions.values()):
        print_save(servo_id, servo_speed)


if __name__ == "__main__":
    main()
