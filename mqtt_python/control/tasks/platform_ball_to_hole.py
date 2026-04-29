from __future__ import annotations
import json
import math
import sys
import threading
import time as t

import cv2
import numpy as np
from paho.mqtt import client as mqtt_client


DEFAULT_IP = "10.197.219.117"

SERVO_ARM           = 1
SERVO_GATE          = 3
GATE_HARDWARE_OK    = False            

ARM_UP_PWM          = -700
ARM_DOWN_PWM        =  500
GATE_OPEN_PWM       =  900
GATE_CLOSE_PWM      = -900
ARM_RELEASE_PWM     = 9999

SERVO_SPEED_HOLD    = 200
SERVO_SPEED_MOVE    = 150
ARM_SETTLE_S        = 1.5

RC_RESEND_S         = 0.05
SCAN_YAW_RATE       = 0.45            
APPROACH_SPEED      = 0.15             
COMMIT_SPEED        = 0.10             
BALL_COMMIT_DURATION_S = 1.0           
HOLE_COMMIT_DURATION_S = 1.0           

ENTRY_FORWARD_DIST_M = 0.20
ENTRY_FORWARD_SPEED  = 0.10
YAW_RATE             = 0.6             
BALL_TURN_RAD        = math.radians(90.0)    
HOLE_TURN_RAD        = math.radians(180.0)   

BALL_HSV_LOWER      = np.array([0,   150, 100], dtype=np.uint8)
BALL_HSV_UPPER      = np.array([57,  255, 255], dtype=np.uint8)
BALL_MIN_AREA       = 50
BALL_MIN_RADIUS     = 8
BALL_MAX_RADIUS     = 80
BALL_MIN_CIRC       = 0.55
BALL_COMMIT_RADIUS  = 65               
BALL_COMMIT_CLIP    = 0.45             

HOLE_BLUR           = 5
HOLE_CANNY_LO       = 30
HOLE_CANNY_HI       = 80
HOLE_MIN_AREA       = 500
HOLE_MAX_AREA       = 40000
HOLE_MAX_ASPECT     = 4.0
HOLE_MIN_CIRC       = 0.15
HOLE_ROI_FRACTION   = 0.5              
HOLE_COMMIT_RADIUS  = 35             
HOLE_PERSISTENCE_N  = 5              

APPROACH_TURN_GAIN  = 1.0              
APPROACH_ALIGN_TOL  = 0.15             
CENTER_TOL          = 0.05             

BALL_LOST_BAILOUT_S = 6.0
HOLE_LOST_BAILOUT_S = 10.0
SEARCH_TIMEOUT_S    = 25.0

WIGGLE_OMEGA        = 0.5
WIGGLE_HALF_S       = 0.5
WIGGLE_CYCLES       = 2

ARM_HOLD_PERIOD_S   = 0.4
ARM_HOLD_HEARTBEAT_N = 13

STREAM_URL_FMT      = "http://{ip}:7123/stream.mjpg"

class S:
    yaw       = None
    pose_seen = False


def on_connect(client, userdata, flags, rc):
    client.subscribe("robobot/kalman/state")


def on_message(client, userdata, msg):
    try:
        if msg.topic == "robobot/kalman/state":
            data = json.loads(msg.payload.decode())
            S.yaw = data["orientation"]["yaw"]
            S.pose_seen = True
    except Exception:
        pass


def make_client(ip: str) -> mqtt_client.Client:
    if hasattr(mqtt_client, "CallbackAPIVersion"):
        c = mqtt_client.Client(
            client_id="platform-ball",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        c = mqtt_client.Client(client_id="platform-ball")
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(ip, 1883, 60)
    c.loop_start()
    return c


def stop(c): c.publish("robobot/cmd/ti", "rc 0.000 0.000")


def drive_rc(c, v: float, w: float):
    c.publish("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")


def drive_for(c, v: float, w: float, duration_s: float, ramp_s: float = 0.0):
    t_start = t.time()
    t_end = t_start + duration_s
    while True:
        now = t.time()
        if now >= t_end:
            break
        if ramp_s > 0:
            scale = min(1.0, (now - t_start) / ramp_s, (t_end - now) / ramp_s)
        else:
            scale = 1.0
        drive_rc(c, v * scale, w)
        t.sleep(RC_RESEND_S)
    stop(c)


def yaw_to_delta(c, delta_rad: float, rate: float):
    """In-place yaw by delta_rad. Closed-loop on Kalman yaw if available,
    open-loop fallback otherwise."""
    direction = 1.0 if delta_rad >= 0 else -1.0
    target_w  = direction * abs(rate)
    if S.pose_seen and S.yaw is not None:
        accumulated = 0.0
        prev = S.yaw
        timeout = abs(delta_rad) / abs(rate) * 2.5 + 2.0
        t0 = t.time()
        while True:
            drive_rc(c, 0.0, target_w)
            t.sleep(RC_RESEND_S)
            if S.yaw is not None:
                d = S.yaw - prev
                if d > math.pi:  d -= 2*math.pi
                if d < -math.pi: d += 2*math.pi
                accumulated += d
                prev = S.yaw
            if direction > 0 and accumulated >= delta_rad: break
            if direction < 0 and accumulated <= delta_rad: break
            if t.time() - t0 > timeout:
                print(f"      ! yaw timeout — accumulated {accumulated:+.2f} of {delta_rad:+.2f}")
                break
        stop(c)
    else:
        duration = abs(delta_rad) / abs(rate)
        print(f"      (no pose feedback — open-loop yaw for {duration:.2f}s)")
        drive_for(c, 0.0, target_w, duration)


def shake_in_place(c, omega: float, half_period_s: float, cycles: int):
    for _ in range(cycles):
        drive_rc(c, 0.0,  omega)
        t.sleep(half_period_s)
        drive_rc(c, 0.0, -omega)
        t.sleep(half_period_s)
    stop(c)


def open_stream(ip: str):
    url = STREAM_URL_FMT.format(ip=ip)
    print(f"      [stream] opening {url}")
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"      ! cannot open stream {url}")
        return None
    return cap


def detect_ball(frame):
    h, w = frame.shape[:2]
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_HSV_LOWER, BALL_HSV_UPPER)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kern, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_r = 0
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < BALL_MIN_AREA: continue
        per = cv2.arcLength(cnt, True)
        if per == 0: continue
        circ = 4 * math.pi * area / (per ** 2)
        if circ < BALL_MIN_CIRC: continue
        (cx, cy), r = cv2.minEnclosingCircle(cnt)
        r = int(round(r))
        if r < BALL_MIN_RADIUS or r > BALL_MAX_RADIUS: continue
        if r > best_r:
            best_r = r
            best = (int(round(cx)), int(round(cy)), r)

    if best is None:
        return (False, 0, 0, 0, 0.0, h, w)
    cx, cy, r = best
    bottom = cy + r
    clip = max(0.0, min(1.0, (bottom - h) / (2.0 * r))) if bottom > h and r > 0 else 0.0
    return (True, cx, cy, r, clip, h, w)


def detect_hole(frame):
    h, w = frame.shape[:2]
    roi_y = int(h * (1.0 - HOLE_ROI_FRACTION))
    roi   = frame[roi_y:, :]
    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (HOLE_BLUR | 1, HOLE_BLUR | 1), 0)
    edges = cv2.Canny(blur, HOLE_CANNY_LO, HOLE_CANNY_HI)
    kern  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kern, iterations=2)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_score = -1.0
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < HOLE_MIN_AREA or area > HOLE_MAX_AREA: continue
        if len(cnt) < 5: continue
        per = cv2.arcLength(cnt, True)
        if per == 0: continue
        circ = 4 * math.pi * area / (per * per)
        if circ < HOLE_MIN_CIRC: continue
        ellipse = cv2.fitEllipse(cnt)
        (ex, ey), (d1, d2), _ = ellipse
        minor = min(d1, d2) / 2.0
        major = max(d1, d2) / 2.0
        if minor == 0 or (major / minor) > HOLE_MAX_ASPECT: continue
        if circ > best_score:
            best_score = circ
            best = (int(ex), int(ey) + roi_y, int((minor + major) / 2.0))
    if best is None:
        return (False, 0, 0, 0, h, w)
    return (True, *best, h, w)


def steering_error_px(cx: int, frame_w: int) -> float:
    return (cx - frame_w / 2.0) / (frame_w / 2.0)


def grab_frame(cap):
    ok, frame = cap.read()
    if not ok or frame is None:
        return None
    return frame


class ArmHolder:
    def __init__(self, client, sid: int, period_s: float = ARM_HOLD_PERIOD_S):
        self._client = client
        self._sid = sid
        self._period_s = period_s
        self._target_pwm = None
        self._target_speed = SERVO_SPEED_HOLD
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = None

    def hold(self, pwm: int, speed: int = SERVO_SPEED_HOLD) -> None:
        with self._lock:
            self._target_pwm = pwm
            self._target_speed = speed
        self._client.publish("robobot/cmd/T0", f"servo {self._sid} {pwm} {speed}")

    def release(self) -> None:
        with self._lock:
            self._target_pwm = None
        self._client.publish("robobot/cmd/T0",
                             f"servo {self._sid} {ARM_RELEASE_PWM} {SERVO_SPEED_HOLD}")

    def start(self) -> None:
        if self._thread is not None: return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _loop(self) -> None:
        tick = 0
        while not self._stop_evt.is_set():
            with self._lock:
                pwm = self._target_pwm
                speed = self._target_speed
            if pwm is not None:
                effective = pwm if (tick % 2 == 0) else pwm + 1
                self._client.publish("robobot/cmd/T0",
                                     f"servo {self._sid} {effective} {speed}")
                tick += 1
                if tick % ARM_HOLD_HEARTBEAT_N == 0:
                    print(f"[arm-hold] tick {tick}  pwm≈{pwm}  speed={speed}")
            t.sleep(self._period_s)


def gate_publish(client, pwm: int) -> None:
    if GATE_HARDWARE_OK:
        client.publish("robobot/cmd/T0", f"servo {SERVO_GATE} {pwm} {SERVO_SPEED_MOVE}")
    else:
        verb = "OPEN" if pwm > 0 else "CLOSE"
        print(f"      >>> MANUAL: {verb} the gate by hand (servo {SERVO_GATE} hardware off) <<<")


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
            start_phase = int(args[i + 1]); i += 2
        elif a.startswith("--from="):
            start_phase = int(a.split("=", 1)[1]); i += 1
        elif not a.startswith("-"):
            ip = a; i += 1
        else:
            print(f"  ! unknown arg: {a}"); i += 1
    return ip, start_phase


def search_and_approach_ball(client, cap) -> bool:
    last_seen = None
    t_search_start = t.time()
    while True:
        frame = grab_frame(cap)
        if frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, clip, h, w = detect_ball(frame)

        if not found:
            if last_seen is None:
                if t.time() - t_search_start > SEARCH_TIMEOUT_S:
                    stop(client)
                    print("      [ball] search timeout — bail")
                    return False
                drive_rc(client, 0.0, SCAN_YAW_RATE)
            elif t.time() - last_seen > BALL_LOST_BAILOUT_S:
                stop(client)
                print(f"      [ball] lost > {BALL_LOST_BAILOUT_S:.0f}s — bail")
                return False
            else:
                drive_rc(client, 0.0, 0.0)
            t.sleep(0.05); continue

        last_seen = t.time()
        err = steering_error_px(cx, w)
        close = (r >= BALL_COMMIT_RADIUS) or (clip >= BALL_COMMIT_CLIP)
        print(f"      [ball] r={r}px  err={err:+.2f}  clip={clip:.2f}  close={close}")
        if close:
            stop(client)
            return True

        if abs(err) > APPROACH_ALIGN_TOL:
            drive_rc(client, APPROACH_SPEED * 0.4,
                     max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        else:
            drive_rc(client, APPROACH_SPEED,
                     max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        t.sleep(0.05)


def center_on_ball(client, cap) -> bool:
    last_seen = t.time()
    while True:
        frame = grab_frame(cap)
        if frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, clip, h, w = detect_ball(frame)
        if not found:
            if t.time() - last_seen > BALL_LOST_BAILOUT_S:
                stop(client)
                print("      [ball-center] lost — bail")
                return False
            drive_rc(client, 0.0, 0.0)
            t.sleep(0.05); continue
        last_seen = t.time()
        err = steering_error_px(cx, w)
        print(f"      [ball-center] r={r}px  err={err:+.3f}")
        if abs(err) <= CENTER_TOL:
            stop(client)
            return True
        drive_rc(client, 0.0,
                 max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        t.sleep(0.05)


def search_and_approach_hole(client, cap) -> bool:
    last_seen = None
    t_search_start = t.time()
    miss = 0
    last_cx, last_w, last_r = 0, 0, 0
    while True:
        frame = grab_frame(cap)
        if frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, h, w = detect_hole(frame)

        if found:
            miss = 0
            last_seen = t.time()
            last_cx, last_w, last_r = cx, w, r
        else:
            miss += 1
            if miss < HOLE_PERSISTENCE_N and last_seen is not None:
                cx, w, r = last_cx, last_w, last_r
                found = True

        if not found:
            if last_seen is None:
                if t.time() - t_search_start > SEARCH_TIMEOUT_S:
                    stop(client)
                    print("      [hole] search timeout — bail")
                    return False
                drive_rc(client, 0.0, SCAN_YAW_RATE)
            elif t.time() - last_seen > HOLE_LOST_BAILOUT_S:
                stop(client)
                print(f"      [hole] lost > {HOLE_LOST_BAILOUT_S:.0f}s — bail")
                return False
            else:
                drive_rc(client, 0.0, 0.0)
            t.sleep(0.05); continue

        err = steering_error_px(cx, w)
        print(f"      [hole] r={r}px  err={err:+.2f}  miss={miss}")
        if r >= HOLE_COMMIT_RADIUS:
            stop(client)
            return True

        if abs(err) > APPROACH_ALIGN_TOL:
            drive_rc(client, APPROACH_SPEED * 0.4,
                     max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        else:
            drive_rc(client, APPROACH_SPEED,
                     max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        t.sleep(0.05)


def center_on_hole(client, cap) -> bool:
    last_seen = t.time()
    miss = 0
    while True:
        frame = grab_frame(cap)
        if frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, h, w = detect_hole(frame)
        if not found:
            miss += 1
            if miss > HOLE_PERSISTENCE_N and t.time() - last_seen > HOLE_LOST_BAILOUT_S:
                stop(client)
                print("      [hole-center] lost — bail")
                return False
            drive_rc(client, 0.0, 0.0)
            t.sleep(0.05); continue
        miss = 0
        last_seen = t.time()
        err = steering_error_px(cx, w)
        print(f"      [hole-center] r={r}px  err={err:+.3f}")
        if abs(err) <= CENTER_TOL:
            stop(client)
            return True
        drive_rc(client, 0.0,
                 max(-1.0, min(1.0, -APPROACH_TURN_GAIN * err)))
        t.sleep(0.05)


def blind_commit(client, duration_s: float) -> None:
    print(f"      blind commit: {COMMIT_SPEED:.2f} m/s for {duration_s:.2f}s")
    drive_for(client, COMMIT_SPEED, 0.0, duration_s, ramp_s=0.15)


def main() -> None:
    ip, start_phase = parse_args()
    client = make_client(ip)
    t.sleep(1.0)

    print(f"\n[platform] connected to {ip}  start_phase={start_phase}")
    print(f"[platform] gate hardware: {'ENABLED' if GATE_HARDWARE_OK else 'DISABLED — manual'}")
    print(f"[platform] plan:")
    print(f"   1. arm UP, gate OPEN — ready")
    print(f"   2. entry creep   ({ENTRY_FORWARD_DIST_M:.2f} m onto platform)")
    print(f"   3. turn LEFT     ({math.degrees(BALL_TURN_RAD):+.0f}°) to face ball")
    print(f"   4. search ball (rotate)")
    print(f"   5. approach ball  (commit r={BALL_COMMIT_RADIUS}px or clip={BALL_COMMIT_CLIP:.2f})")
    print(f"   6. center ball   (|err| <= {CENTER_TOL:.2f})")
    print(f"   7. blind commit  ({BALL_COMMIT_DURATION_S:.2f}s @ {COMMIT_SPEED:.2f} m/s)")
    print(f"   8. capture       (arm DOWN -> gate CLOSE -> arm UP)")
    print(f"   9. turn ~180°    ({math.degrees(HOLE_TURN_RAD):+.0f}°) to face hole")
    print(f"  10. search hole")
    print(f"  11. approach hole (commit r={HOLE_COMMIT_RADIUS}px)")
    print(f"  12. center hole")
    print(f"  13. blind commit")
    print(f"  14. drop         (arm DOWN -> gate OPEN -> wiggle -> arm UP)")
    print(f"  Each phase asks before running.  Ctrl+C = e-stop.\n")

    holder = ArmHolder(client, SERVO_ARM)
    holder.start()
    print(f"[arm] starting holder at UP (pwm {ARM_UP_PWM})")
    holder.hold(ARM_UP_PWM)
    t.sleep(ARM_SETTLE_S)

    cap = None

    try:
        if start_phase <= 1 and confirm("[1/14] ready pose: arm UP + gate OPEN?"):
            holder.hold(ARM_UP_PWM)
            t.sleep(ARM_SETTLE_S)
            print(f"      gate OPEN  (servo {SERVO_GATE}={GATE_OPEN_PWM})")
            gate_publish(client, GATE_OPEN_PWM)
            t.sleep(0.5)

        if start_phase <= 2 and confirm(
                f"[2/14] creep forward {ENTRY_FORWARD_DIST_M:.2f} m onto platform?"):
            drive_for(client, ENTRY_FORWARD_SPEED, 0.0,
                      ENTRY_FORWARD_DIST_M / ENTRY_FORWARD_SPEED, ramp_s=0.2)
            t.sleep(0.3)

        if start_phase <= 3 and confirm(
                f"[3/14] yaw {math.degrees(BALL_TURN_RAD):+.0f}° to face ball?"):
            yaw_to_delta(client, BALL_TURN_RAD, YAW_RATE)
            t.sleep(0.3)

        cap = open_stream(ip)
        if cap is None:
            print("      ! no camera stream — abort")
            return

        if start_phase <= 4 and confirm("[4/14] search + approach ball?"):
            if not search_and_approach_ball(client, cap):
                print("      ! ball approach failed; retry phase 4 or abort.")

        if start_phase <= 6 and confirm("[6/14] center on ball (yaw-only)?"):
            if not center_on_ball(client, cap):
                print("      ! center failed; retry or abort.")

        if start_phase <= 7 and confirm("[7/14] blind commit forward?"):
            blind_commit(client, BALL_COMMIT_DURATION_S)
            t.sleep(0.2)

        if start_phase <= 8 and confirm("[8/14] capture: arm DOWN -> gate CLOSE -> arm UP?"):
            print(f"      arm DOWN (pwm {ARM_DOWN_PWM})")
            holder.hold(ARM_DOWN_PWM, SERVO_SPEED_MOVE)
            t.sleep(ARM_SETTLE_S)
            print(f"      gate CLOSE  (servo {SERVO_GATE}={GATE_CLOSE_PWM})")
            gate_publish(client, GATE_CLOSE_PWM)
            t.sleep(0.8)
            print(f"      arm UP (pwm {ARM_UP_PWM}) — ball locked")
            holder.hold(ARM_UP_PWM)
            t.sleep(ARM_SETTLE_S)

        if start_phase <= 9 and confirm(
                f"[9/14] yaw {math.degrees(HOLE_TURN_RAD):+.0f}° to face hole?"):
            yaw_to_delta(client, HOLE_TURN_RAD, YAW_RATE)
            t.sleep(0.3)

        if start_phase <= 10 and confirm("[10/14] search + approach hole?"):
            if not search_and_approach_hole(client, cap):
                print("      ! hole approach failed; retry or abort.")

        if start_phase <= 12 and confirm("[12/14] center on hole?"):
            if not center_on_hole(client, cap):
                print("      ! center failed; retry or abort.")

        if start_phase <= 13 and confirm("[13/14] blind commit forward?"):
            blind_commit(client, HOLE_COMMIT_DURATION_S)
            t.sleep(0.2)

        if start_phase <= 14 and confirm("[14/14] drop ball (arm DOWN, gate OPEN, wiggle, arm UP)?"):
            print(f"      arm DOWN (pwm {ARM_DOWN_PWM})")
            holder.hold(ARM_DOWN_PWM, SERVO_SPEED_MOVE)
            t.sleep(ARM_SETTLE_S)
            print(f"      gate OPEN  (servo {SERVO_GATE}={GATE_OPEN_PWM}) — release")
            gate_publish(client, GATE_OPEN_PWM)
            t.sleep(0.8)
            print(f"      wiggle x{WIGGLE_CYCLES} to seat ball")
            shake_in_place(client, WIGGLE_OMEGA, WIGGLE_HALF_S, WIGGLE_CYCLES)
            t.sleep(0.3)
            print(f"      arm UP (pwm {ARM_UP_PWM}) — done")
            holder.hold(ARM_UP_PWM)
            t.sleep(ARM_SETTLE_S)

        print("\n[platform] sequence complete.")

    except KeyboardInterrupt:
        print("\n[platform] ^C — emergency stop.")
        stop(client)
    finally:
        holder.stop()
        stop(client)
        if cap is not None:
            try: cap.release()
            except Exception: pass
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("[platform] done.")


if __name__ == "__main__":
    main()
