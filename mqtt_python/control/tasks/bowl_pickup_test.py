#!/usr/bin/env python3
"""Bowl pickup sequence test — creep + arm down + (shake) + arm up.

Assumes the robot is already at the star base, pointed toward the bowl.
Drives forward a small distance using time*speed (no pose feedback needed),
lowers the arm over the bowl, optionally yaw-shakes, then raises the arm.

Run from mqtt_python/:
    python3 -m control.tasks.bowl_pickup_test [robot-ip]
"""
from __future__ import annotations
import sys
import time as t

from paho.mqtt import client as mqtt_client

DEFAULT_IP        = "10.197.219.117"

# Travel — robot is at star base, bowl is forward (-Y in field frame).
APPROACH_DIST_M   = 0.30    # how far to creep onto the bowl  (~6 s @ 0.05 m/s)
APPROACH_SPEED    = 0.05    # m/s — slow

# Arm (servo 1)
SERVO_ID          = 1
ARM_DOWN_PWM      = 500
ARM_UP_PWM        = -700
SERVO_SPEED       = 100
ARM_SETTLE_S      = 1.5     # wait after each arm command
ARM_RELEASE_PWM   = 9999    # |pos|>1024 disables servo on Teensy → silences buzzing

# Optional shake to tip the bowl
SHAKE_ENABLED     = True
SHAKE_W           = 1.5     # rad/s
SHAKE_PERIOD_S    = 0.3
SHAKE_CYCLES      = 4

# Hold the bowl pose for this many seconds with arm down
BOWL_HOLD_S       = 1.0

# How often to resend rc command while driving (motors need fresh commands)
RC_RESEND_S       = 0.05


def make_client(ip: str) -> mqtt_client.Client:
    if hasattr(mqtt_client, "CallbackAPIVersion"):
        c = mqtt_client.Client(
            client_id="bowl-pickup-test",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        c = mqtt_client.Client(client_id="bowl-pickup-test")
    c.connect(ip, 1883, 60)
    c.loop_start()
    return c


def drive_for(client: mqtt_client.Client, v: float, w: float, duration_s: float) -> None:
    t_end = t.time() + duration_s
    while t.time() < t_end:
        client.publish("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")
        t.sleep(RC_RESEND_S)
    client.publish("robobot/cmd/ti", "rc 0.000 0.000")


def stop(client: mqtt_client.Client) -> None:
    client.publish("robobot/cmd/ti", "rc 0.000 0.000")


def servo(client: mqtt_client.Client, sid: int, pos: int, speed: int = SERVO_SPEED) -> None:
    client.publish("robobot/cmd/T0", f"servo {sid} {pos} {speed}")


def confirm(prompt: str) -> bool:
    ans = input(f"{prompt} [y/N] > ").strip().lower()
    return ans in ("y", "yes")


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    client = make_client(ip)
    t.sleep(0.6)

    print(f"\n[bowl] connected to {ip}")
    print(f"[bowl] plan:")
    print(f"  1. creep forward  {APPROACH_DIST_M:.2f} m  @ {APPROACH_SPEED:.2f} m/s")
    print(f"  2. arm down       (servo {SERVO_ID} -> {ARM_DOWN_PWM})")
    print(f"  3. shake          (yaw +/-{SHAKE_W} rad/s, {SHAKE_CYCLES} cycles)" if SHAKE_ENABLED
          else "  3. (shake disabled)")
    print(f"  4. hold           {BOWL_HOLD_S:.1f}s")
    print(f"  5. arm up         (servo {SERVO_ID} -> {ARM_UP_PWM})")
    print(f"  Ctrl+C any time = emergency stop\n")

    if not confirm("Start sequence?"):
        print("[bowl] aborted before start.")
        client.loop_stop(); client.disconnect()
        return

    try:
        # 1. creep
        duration = APPROACH_DIST_M / APPROACH_SPEED
        print(f"[1/5] creep forward {APPROACH_DIST_M:.2f}m for {duration:.1f}s ...")
        drive_for(client, APPROACH_SPEED, 0.0, duration)
        t.sleep(0.3)

        # 2. arm down
        print(f"[2/5] arm DOWN (pwm {ARM_DOWN_PWM}) ...")
        servo(client, SERVO_ID, ARM_DOWN_PWM)
        t.sleep(ARM_SETTLE_S)

        # 3. shake
        if SHAKE_ENABLED:
            print(f"[3/5] shake x{SHAKE_CYCLES} ...")
            for _ in range(SHAKE_CYCLES):
                drive_for(client, 0.0, +SHAKE_W, SHAKE_PERIOD_S)
                drive_for(client, 0.0, -SHAKE_W, SHAKE_PERIOD_S)
            stop(client)
        else:
            print("[3/5] shake skipped")

        # 4. hold
        print(f"[4/5] hold {BOWL_HOLD_S:.1f}s with arm down ...")
        t.sleep(BOWL_HOLD_S)

        # 5. arm up
        print(f"[5/5] arm UP (pwm {ARM_UP_PWM}) ...")
        servo(client, SERVO_ID, ARM_UP_PWM)
        t.sleep(ARM_SETTLE_S)
        print(f"      release servo (pwm {ARM_RELEASE_PWM}) — silences buzzing")
        servo(client, SERVO_ID, ARM_RELEASE_PWM)

        print("\n[bowl] sequence complete.")

    except KeyboardInterrupt:
        print("\n[bowl] ^C — emergency stop, raising arm.")
        stop(client)
        servo(client, SERVO_ID, ARM_UP_PWM)
        t.sleep(0.5)
    finally:
        stop(client)
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("[bowl] done.")


if __name__ == "__main__":
    main()
