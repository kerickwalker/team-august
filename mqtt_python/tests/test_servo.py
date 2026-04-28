#!/usr/bin/env python3
"""Quick servo tester. Usage: python3 test_servo.py [robot-ip]
At prompt: servo <id> <position> <speed>  or just  <position>  (uses last id/speed)
Type 'q' to quit."""
import sys, time
import paho.mqtt.client as mqtt

host  = sys.argv[1] if len(sys.argv) > 1 else "10.197.219.117"
sid   = 1
speed = 300

c = mqtt.Client()
c.connect(host, 1883)
c.loop_start()
time.sleep(0.3)
print(f"Connected to {host}. Default: id={sid} speed={speed}")
print("Commands: servo <id> <pos> <speed>  |  <pos>  |  q")

while True:
    try:
        raw = input("servo> ").strip()
    except (EOFError, KeyboardInterrupt):
        break
    if raw in ("q", "quit"):
        break
    parts = raw.split()
    if not parts:
        continue
    try:
        if parts[0] == "servo" and len(parts) == 4:
            sid, pos, speed = int(parts[1]), int(parts[2]), int(parts[3])
        else:
            pos = int(parts[0])
        cmd = f"servo {sid} {pos} {speed}"
        c.publish("robobot/cmd/T0", cmd)
        print(f"  → {cmd}")
    except ValueError:
        print("  ! geçersiz komut")

c.loop_stop()
c.disconnect()
