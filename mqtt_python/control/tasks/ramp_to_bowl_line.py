#!/usr/bin/env python3
"""Ramp-exit → forward → left → find-line → left sequence.

Robot starts where the short ramp lets out. Plan:
    1. forward F1 metres (off the ramp area)
    2. yaw left  Y1 rad   (face roughly toward the line)
    3. creep forward, watching the floor IR line sensor;
       stop as soon as the white line is detected
    4. yaw left  Y2 rad   (align onto the line)

All motion uses raw paho MQTT — no uservice/srobot heartbeat dependency.
Line sensor source:
    topic robobot/drive/T0/livn  -> "<ts> s1 s2 ... s8"  (0..1000 normalized).
    line "valid" when peak sensor >= LINE_VALID_THRESHOLD.

Each phase asks for confirmation so you can tune step-by-step.

Run from mqtt_python/:
    python3 -m control.tasks.ramp_to_bowl_line [robot-ip] [--from N]
    python3 -m control.tasks.ramp_to_bowl_line 10.197.219.117 --from 5
        → starts at phase 5 (skips 1..4)
"""
from __future__ import annotations
import json
import sys
import time as t

from paho.mqtt import client as mqtt_client

DEFAULT_IP = "10.197.219.117"

# ---------------------------------------------------------------------------
# FIELD_TUNE — tweak these on the day, every value is "small and slow first".
# ---------------------------------------------------------------------------
F1_DIST_M       = 0.78      # phase 1: forward off ramp  (~4.3 s @ 0.18 m/s)
F1_SPEED        = 0.18      # m/s

# Arm/tray servo — keep raised while driving so it doesn't drop the bowl
SERVO_ARM       = 1
ARM_UP_PWM      = -800      # tray raised, but not slammed against stop (cooler)
SERVO_SPEED_HOLD = 150      # gentler hold — was 300 and overheating
ARM_PRESET_S    = 1.5       # let the servo fully reach UP before driving

Y1_RAD          = 1.57      # phase 2: yaw left ~90°  (positive = CCW = left)
Y1_RATE         = 0.6       # rad/s

F3_SPEED        = 0.06      # phase 3: creep speed while looking for the line
F3_MAX_DIST_M   = 0.80      # safety cap — stop after this much travel even without line
F3_TIMEOUT_S    = 15.0
F3_OVERSHOOT_M  = 0.02      # creep this much further AFTER line detected, so the
                            # turn pivots over the line (otherwise we knock the bowl)
F3_WARMUP_M     = 0.06      # ignore line detection for the first N metres — robot
                            # ends phase 2 already on/near a line and would trigger
                            # immediately. Drive blindly past it first.

Y2_RAD          = -1.57     # phase 4: yaw RIGHT after line is found  (negative = CW)
Y2_RATE         = 0.6       # rad/s

F5_DIST_M       = 0.40      # phase 5: gentle approach onto the bowl
F5_SPEED        = 0.05      # m/s — very slow so we don't slam the bowl
F5_RAMP_S       = 0.6       # smooth accel/decel — extra cushion for the cargo

# Phase 6: place the bowl down (legacy — drop arm onto floor)
ARM_DOWN_PWM    = 500       # tray fully lowered onto floor (calibrated saha)
ARM_PLACE_SPEED = 120       # servo speed while lowering — slow & smooth
ARM_PLACE_S     = 2.0       # wait for the arm to reach DOWN
ARM_RELEASE_PWM = 9999      # |pos|>1024 disables the servo (silences buzzing)

# Phase 7: lift the arm with cargo (after balls are picked up)
P7_ARM_LIFT_S   = 1.5       # let the arm fully reach UP before driving

# Phase 8: short reverse from the dispenser (just clear the bowl)
F8_DIST_M       = 0.25
F8_SPEED        = 0.10

# Phase 9: yaw RIGHT 90° (toward the C-side of the field)
Y9_RAD          = -1.57     # -90° CW (was +1.57 left — wrong direction)
Y9_RATE         = 0.6

# Phase 10: short forward step ("az bir şey gitsin") to clear the line
F10_DIST_M      = 0.80
F10_SPEED       = 0.12

# Phase 11: yaw LEFT 90° onto the C-bound straight
Y11_RAD         = 2.22      # +127° CCW (was 2.15 — needed a tiny bit more)
Y11_RATE        = 0.6

# Phase 12: vision-guided dock against the Zone C ArUcos.
# Tunable parameters live in aruco_dock.py (TARGET_IDS, GRIPPER_DISTANCE, …).

# Phase 13: dump the balls (handled by aruco_dock.drop_balls)

LINE_VALID_THRESHOLD = 500  # peak sensor value to call the line "seen"
RC_RESEND_S          = 0.05

# ---------------------------------------------------------------------------
# State updated by MQTT callbacks
# ---------------------------------------------------------------------------
class S:
    yaw           = None    # rad, from kalman state
    pose_seen     = False
    line_high     = 0       # peak of the 8 IR sensors (0..1000)
    line_valid    = False
    line_seen_t   = 0.0


def on_connect(client, userdata, flags, rc):
    client.subscribe("robobot/kalman/state")
    client.subscribe("robobot/drive/T0/livn")
    # Ask Teensy to publish livn @ 100 ms.
    client.publish("robobot/cmd/T0", "sub livn 10")


def on_message(client, userdata, msg):
    try:
        if msg.topic == "robobot/kalman/state":
            data = json.loads(msg.payload.decode())
            S.yaw = data["orientation"]["yaw"]
            S.pose_seen = True
        elif msg.topic.endswith("/livn"):
            parts = msg.payload.decode().split()
            if len(parts) >= 9:
                vals = [int(p) for p in parts[1:9]]
                S.line_high = max(vals)
                S.line_valid = S.line_high >= LINE_VALID_THRESHOLD
                if S.line_valid:
                    S.line_seen_t = t.time()
    except Exception:
        pass


def make_client(ip: str) -> mqtt_client.Client:
    if hasattr(mqtt_client, "CallbackAPIVersion"):
        c = mqtt_client.Client(
            client_id="ramp-to-bowl",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        c = mqtt_client.Client(client_id="ramp-to-bowl")
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(ip, 1883, 60)
    c.loop_start()
    return c


def stop(c): c.publish("robobot/cmd/ti", "rc 0.000 0.000")


def drive_for(c, v: float, w: float, duration_s: float, ramp_s: float = 0.0):
    """Drive at (v, w) for duration_s. With ramp_s>0, smoothly ramp v at start
    AND end so cargo doesn't slosh. Yaw rate w isn't ramped (turns are short)."""
    t_start = t.time()
    t_end = t_start + duration_s
    while True:
        now = t.time()
        if now >= t_end:
            break
        if ramp_s > 0:
            ramp_in  = min(1.0, (now - t_start) / ramp_s)
            ramp_out = min(1.0, (t_end - now)  / ramp_s)
            scale = min(ramp_in, ramp_out)
        else:
            scale = 1.0
        c.publish("robobot/cmd/ti", f"rc {v*scale:.3f} {w:.3f}")
        t.sleep(RC_RESEND_S)
    stop(c)


def yaw_to_delta(c, delta_rad: float, rate: float):
    """Turn by delta_rad. Uses kalman yaw if available; else falls back to time."""
    direction = 1.0 if delta_rad >= 0 else -1.0
    target_w  = direction * abs(rate)

    if S.pose_seen and S.yaw is not None:
        yaw_start = S.yaw
        accumulated = 0.0
        prev = yaw_start
        timeout = abs(delta_rad) / abs(rate) * 2.5 + 2.0
        t0 = t.time()
        while True:
            c.publish("robobot/cmd/ti", f"rc 0.000 {target_w:.3f}")
            t.sleep(RC_RESEND_S)
            if S.yaw is not None:
                d = S.yaw - prev
                # unwrap ±π
                if d > 3.14159: d -= 6.28318
                if d < -3.14159: d += 6.28318
                accumulated += d
                prev = S.yaw
            if direction > 0 and accumulated >= delta_rad:
                break
            if direction < 0 and accumulated <= delta_rad:
                break
            if t.time() - t0 > timeout:
                print(f"      ! yaw timeout — accumulated {accumulated:+.2f} of {delta_rad:+.2f}")
                break
        stop(c)
    else:
        duration = abs(delta_rad) / abs(rate)
        print(f"      (no pose feedback — open-loop yaw for {duration:.2f}s)")
        drive_for(c, 0.0, target_w, duration)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] > ").strip().lower() in ("y", "yes")


def parse_args() -> tuple[str, int]:
    ip = DEFAULT_IP
    start_phase = 1
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a in ("--from", "-f") and i + 1 < len(args):
            start_phase = int(args[i + 1])
            i += 2
        elif a.startswith("--from="):
            start_phase = int(a.split("=", 1)[1])
            i += 1
        elif not a.startswith("-"):
            ip = a
            i += 1
        else:
            print(f"  ! unknown arg: {a}")
            i += 1
    return ip, start_phase


def main() -> None:
    ip, start_phase = parse_args()
    client = make_client(ip)
    t.sleep(1.0)

    print(f"\n[ramp] connected to {ip}  start_phase={start_phase}")
    print(f"[ramp] pose_seen={S.pose_seen}  yaw={S.yaw}  line_high={S.line_high}")
    print(f"[ramp] plan:")
    print(f"  1. forward {F1_DIST_M:.2f} m @ {F1_SPEED:.2f} m/s  ({F1_DIST_M/F1_SPEED:.1f} s)")
    print(f"  2. yaw left  {Y1_RAD:+.2f} rad  ({Y1_RAD*57.3:+.0f}°) @ {Y1_RATE} rad/s")
    print(f"  3. creep @ {F3_SPEED:.2f} m/s until line detected (high≥{LINE_VALID_THRESHOLD})")
    print(f"     safety cap {F3_MAX_DIST_M:.2f} m or {F3_TIMEOUT_S:.0f}s")
    print(f"  4. yaw       {Y2_RAD:+.2f} rad  ({Y2_RAD*57.3:+.0f}°) @ {Y2_RATE} rad/s")
    print(f"  5. gentle approach {F5_DIST_M:.2f} m @ {F5_SPEED:.2f} m/s "
          f"({F5_DIST_M/F5_SPEED:.1f} s, ramp {F5_RAMP_S}s)")
    print(f"  6. lower tray onto floor (servo {SERVO_ARM}={ARM_DOWN_PWM} @ {ARM_PLACE_SPEED}), then release")
    print(f"  7. arm UP with cargo (servo {SERVO_ARM}={ARM_UP_PWM})  [after balls picked up]")
    print(f"  8. short reverse  {F8_DIST_M:.2f} m @ {F8_SPEED:.2f} m/s")
    print(f"  9. yaw {Y9_RAD:+.2f} rad ({Y9_RAD*57.3:+.0f}°) — right")
    print(f" 10. short forward {F10_DIST_M:.2f} m @ {F10_SPEED:.2f} m/s")
    print(f" 11. yaw {Y11_RAD:+.2f} rad ({Y11_RAD*57.3:+.0f}°) — left, onto C straight")
    print(f" 12. ArUco dock against Zone C markers (closed-loop)")
    print(f" 13. dump balls (arm DOWN, then release)")
    print(f"  Each phase asks before running.  Ctrl+C = e-stop.\n")

    try:
        # Hold the arm/tray raised the whole time so cargo doesn't drop on bumps.
        print(f"[arm] holding tray UP (servo {SERVO_ARM}={ARM_UP_PWM}, speed {SERVO_SPEED_HOLD})")
        client.publish("robobot/cmd/T0", f"servo {SERVO_ARM} {ARM_UP_PWM} {SERVO_SPEED_HOLD}")
        t.sleep(ARM_PRESET_S)

        # Phase 1 — ramp accel/decel so the bowl doesn't slosh out
        if start_phase <= 1 and confirm("[1/13] forward off ramp?"):
            drive_for(client, F1_SPEED, 0.0, F1_DIST_M / F1_SPEED, ramp_s=0.5)
            t.sleep(0.3)

        # Phase 2
        if start_phase <= 2 and confirm("[2/13] yaw left?"):
            yaw_to_delta(client, Y1_RAD, Y1_RATE)
            t.sleep(0.3)

        # Phase 3 — creep while watching line
        if start_phase <= 3 and confirm("[3/13] creep until line detected?"):
            t0 = t.time()
            duration_cap = F3_MAX_DIST_M / F3_SPEED
            warmup_s = F3_WARMUP_M / F3_SPEED
            line_found = False
            print(f"      warmup {F3_WARMUP_M*100:.0f} cm ({warmup_s:.2f}s) — ignoring line")
            print(f"      then creeping; watching livn (current high={S.line_high})")
            while True:
                client.publish("robobot/cmd/ti", f"rc {F3_SPEED:.3f} 0.000")
                t.sleep(RC_RESEND_S)
                now = t.time()
                elapsed = now - t0
                if elapsed >= warmup_s and S.line_valid:
                    print(f"      LINE DETECTED  high={S.line_high}  after {elapsed:.2f}s")
                    line_found = True
                    break
                if elapsed > duration_cap or elapsed > F3_TIMEOUT_S:
                    print(f"      ! safety cap hit at {elapsed:.2f}s — high={S.line_high}")
                    break
            # creep a bit further so the wheels straddle the line before turning
            if line_found and F3_OVERSHOOT_M > 0:
                extra_s = F3_OVERSHOOT_M / F3_SPEED
                print(f"      overshoot {F3_OVERSHOOT_M*100:.0f} cm  ({extra_s:.2f}s)")
                drive_for(client, F3_SPEED, 0.0, extra_s)
            stop(client)
            t.sleep(0.3)

        # Phase 4
        if start_phase <= 4 and confirm("[4/13] yaw onto line?"):
            yaw_to_delta(client, Y2_RAD, Y2_RATE)
            t.sleep(0.3)

        # Phase 5 — gentle approach onto bowl
        if start_phase <= 5 and confirm("[5/13] gentle approach to bowl?"):
            drive_for(client, F5_SPEED, 0.0, F5_DIST_M / F5_SPEED, ramp_s=F5_RAMP_S)
            t.sleep(0.3)

        # Phase 6 — lower tray, place bowl on floor, release servo
        if start_phase <= 6 and confirm("[6/13] place bowl on floor (arm down)?"):
            print(f"      arm DOWN (servo {SERVO_ARM}={ARM_DOWN_PWM} @ {ARM_PLACE_SPEED})")
            client.publish("robobot/cmd/T0", f"servo {SERVO_ARM} {ARM_DOWN_PWM} {ARM_PLACE_SPEED}")
            t.sleep(ARM_PLACE_S)
            print(f"      release servo (pwm {ARM_RELEASE_PWM}) — silences buzzing")
            client.publish("robobot/cmd/T0", f"servo {SERVO_ARM} {ARM_RELEASE_PWM} {ARM_PLACE_SPEED}")
            t.sleep(0.3)

        # Phase 7 — arm UP with cargo (after balls have been picked up)
        if start_phase <= 7 and confirm("[7/13] arm UP with cargo (lock balls inside)?"):
            print(f"      arm UP (servo {SERVO_ARM}={ARM_UP_PWM} @ {SERVO_SPEED_HOLD})")
            client.publish("robobot/cmd/T0", f"servo {SERVO_ARM} {ARM_UP_PWM} {SERVO_SPEED_HOLD}")
            t.sleep(P7_ARM_LIFT_S)

        # Phase 8 — short reverse from dispenser
        if start_phase <= 8 and confirm("[8/13] short reverse from dispenser?"):
            drive_for(client, -F8_SPEED, 0.0, F8_DIST_M / F8_SPEED, ramp_s=0.4)
            t.sleep(0.3)

        # Phase 9 — yaw RIGHT (toward C-side of the field)
        if start_phase <= 9 and confirm("[9/13] yaw RIGHT?"):
            yaw_to_delta(client, Y9_RAD, Y9_RATE)
            t.sleep(0.3)

        # Phase 10 — short forward to clear the line
        if start_phase <= 10 and confirm("[10/13] short forward?"):
            drive_for(client, F10_SPEED, 0.0, F10_DIST_M / F10_SPEED, ramp_s=0.4)
            t.sleep(0.3)

        # Phase 11 — yaw LEFT onto the C-bound straight
        if start_phase <= 11 and confirm("[11/13] yaw LEFT onto C straight?"):
            yaw_to_delta(client, Y11_RAD, Y11_RATE)
            t.sleep(0.3)

        # Phase 12 — vision-guided dock to Zone C ArUcos (closed-loop)
        if start_phase <= 12 and confirm("[12/13] ArUco dock to Zone C?"):
            from control.tasks.aruco_dock import run_dock
            ok = run_dock(client, ip)
            if not ok:
                print("      ! dock failed — markers not visible. Aim and retry.")
            t.sleep(0.3)

        # Phase 13 — dump the balls
        if start_phase <= 13 and confirm("[13/13] dump balls (arm down)?"):
            from control.tasks.aruco_dock import drop_balls
            drop_balls(client)

        print("\n[ramp] sequence complete.")

    except KeyboardInterrupt:
        print("\n[ramp] ^C — emergency stop.")
        stop(client)
    finally:
        stop(client)
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("[ramp] done.")


if __name__ == "__main__":
    main()
