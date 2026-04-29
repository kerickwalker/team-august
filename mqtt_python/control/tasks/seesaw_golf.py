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
ARM_MID_PWM         = -200            
GATE_OPEN_PWM       =  900             
GATE_CLOSE_PWM      = -900             

ARM_RELEASE_PWM     = 9999             

SERVO_SPEED_HOLD    = 200
SERVO_SPEED_MOVE    = 150
ARM_SETTLE_S        = 1.5

RC_RESEND_S         = 0.05
PLATFORM_SPEED      = 0.20             
SEESAW_MOUNT_SPEED  = 0.10             
EXIT_SEARCH_SPEED   = 0.08             
RAMP_LINE_SPEED     = 0.18             

YAW_RATE            = 0.6              
P3_MOUNT_DIST_M     = 0.30             
                                       
P3_MOUNT_RAMP_S     = 0.5
P5_BALL_LOST_BAILOUT_S = 6.0           

P7_OFF_SEESAW_DIST_M = 0.50            
P7_OFF_SEESAW_SPEED  = 0.10

P9_FIND_LINE_MAX_M   = 0.80
P9_FIND_LINE_TIMEOUT_S = 12.0

P10_TURN_RIGHT_RAD   = -math.radians(90.0)

P12_HOLE_SCAN_W      = 0.35            
P12_HOLE_LOST_TIMEOUT_S = 10.0

LINE_VALID_THRESHOLD = 500             
LINE_FOLLOW_KP       = 1.4             
LINE_FOLLOW_MAX_W    = 1.2

JUNCTION_SENSOR_COUNT  = 5
JUNCTION_STABLE_FRAMES = 4

BALL_HSV_LOWER       = np.array([0,   150, 100], dtype=np.uint8)
BALL_HSV_UPPER       = np.array([57,  255, 255], dtype=np.uint8)
BALL_MIN_AREA        = 50
BALL_MIN_RADIUS      = 8
BALL_MAX_RADIUS      = 80
BALL_MIN_CIRC        = 0.55
BALL_CAPTURE_RADIUS  = 50              
BALL_CAPTURE_CLIP    = 0.30            

HOLE_BLUR            = 5
HOLE_CANNY_LO        = 30
HOLE_CANNY_HI        = 80
HOLE_MIN_AREA        = 500
HOLE_MAX_AREA        = 40000
HOLE_MAX_ASPECT      = 4.0
HOLE_MIN_CIRC        = 0.15
HOLE_ROI_FRACTION    = 0.5             
HOLE_CAPTURE_RADIUS  = 60              

VISION_TURN_GAIN     = 1.0             
VISION_FORWARD_SPEED = 0.07           
VISION_ALIGN_TOL     = 0.05            

ARM_HOLD_PERIOD_S    = 0.4
ARM_HOLD_HEARTBEAT_N = 13              

STREAM_URL_FMT       = "http://{ip}:7123/stream.mjpg"


class S:
    yaw          = None
    pose_seen    = False
    line_high    = 0
    line_valid   = False
    line_vals    = [0]*8     
    line_seen_t  = 0.0


def on_connect(client, userdata, flags, rc):
    client.subscribe("robobot/kalman/state")
    client.subscribe("robobot/drive/T0/livn")
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
                S.line_vals  = vals
                S.line_high  = max(vals)
                S.line_valid = S.line_high >= LINE_VALID_THRESHOLD
                if S.line_valid:
                    S.line_seen_t = t.time()
    except Exception:
        pass


def make_client(ip: str) -> mqtt_client.Client:
    if hasattr(mqtt_client, "CallbackAPIVersion"):
        c = mqtt_client.Client(
            client_id="seesaw-golf",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        c = mqtt_client.Client(client_id="seesaw-golf")
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(ip, 1883, 60)
    c.loop_start()
    return c

def stop(c): c.publish("robobot/cmd/ti", "rc 0.000 0.000")


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
        c.publish("robobot/cmd/ti", f"rc {v*scale:.3f} {w:.3f}")
        t.sleep(RC_RESEND_S)
    stop(c)


def yaw_to_delta(c, delta_rad: float, rate: float):
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
        c.publish("robobot/cmd/ti", f"rc 0.000 {omega:.3f}")
        t.sleep(half_period_s)
        c.publish("robobot/cmd/ti", f"rc 0.000 {-omega:.3f}")
        t.sleep(half_period_s)
    stop(c)


def line_position() -> float:
    vals = S.line_vals
    total = sum(vals)
    if total < LINE_VALID_THRESHOLD:
        return 0.0
    weighted = sum(i * v for i, v in enumerate(vals))
    centre   = weighted / total          # 0..7
    return (centre - 3.5) / 3.5          # to [-1, 1]


def count_high_sensors() -> int:
    return sum(1 for v in S.line_vals if v >= LINE_VALID_THRESHOLD)


def line_follow_until(c, until_fn, base_speed: float, timeout_s: float,
                      label: str = "line-follow") -> bool:
    print(f"      [{label}] following @ {base_speed:.2f} m/s, timeout {timeout_s:.1f}s")
    t0 = t.time()
    last_log = 0.0
    while t.time() - t0 < timeout_s:
        if until_fn():
            stop(c)
            print(f"      [{label}] until_fn fired after {t.time()-t0:.2f}s")
            return True
        err = line_position()
        w   = max(-LINE_FOLLOW_MAX_W, min(LINE_FOLLOW_MAX_W, -LINE_FOLLOW_KP * err))
        if not S.line_valid:
            w = 0.0
        c.publish("robobot/cmd/ti", f"rc {base_speed:.3f} {w:.3f}")
        t.sleep(RC_RESEND_S)
        now = t.time()
        if now - last_log > 1.0:
            last_log = now
            print(f"      [{label}] high={S.line_high:4d}  err={err:+.2f}  "
                  f"w={w:+.2f}  count={count_high_sensors()}")
    stop(c)
    print(f"      [{label}] timeout — bailed")
    return False

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

def approach_ball(client, cap) -> bool:
    last_seen = t.time()
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, clip, h, w = detect_ball(frame)
        if found:
            last_seen = t.time()
            err = steering_error_px(cx, w)
            close = (r >= BALL_CAPTURE_RADIUS) or (clip >= BALL_CAPTURE_CLIP)
            print(f"      [ball] r={r}px  err={err:+.2f}  clip={clip:.2f}  close={close}")
            if close:
                stop(client)
                return True
            v = VISION_FORWARD_SPEED
            if abs(err) > 0.20:
                v *= 0.4
            wcmd = max(-1.0, min(1.0, -VISION_TURN_GAIN * err))
            client.publish("robobot/cmd/ti", f"rc {v:.3f} {wcmd:.3f}")
        else:
            if t.time() - last_seen > P5_BALL_LOST_BAILOUT_S:
                stop(client)
                print(f"      [ball] lost for >{P5_BALL_LOST_BAILOUT_S:.0f}s — bail")
                return False
            client.publish("robobot/cmd/ti", "rc 0.000 0.000")
        t.sleep(0.05)


def approach_hole(client, cap, holder: ArmHolder) -> bool:
    last_seen = None
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            t.sleep(0.05); continue
        found, cx, cy, r, h, w = detect_hole(frame)
        if found:
            last_seen = t.time()
            err = steering_error_px(cx, w)
            print(f"      [hole] r={r}px  err={err:+.2f}")
            if r >= HOLE_CAPTURE_RADIUS and abs(err) < VISION_ALIGN_TOL:
                stop(client)
                return True
            v = VISION_FORWARD_SPEED * (0.5 if r >= HOLE_CAPTURE_RADIUS else 1.0)
            wcmd = max(-1.0, min(1.0, -VISION_TURN_GAIN * err))
            client.publish("robobot/cmd/ti", f"rc {v:.3f} {wcmd:.3f}")
        else:
            if last_seen is None:
                # Never seen — scan
                client.publish("robobot/cmd/ti", f"rc 0.000 {P12_HOLE_SCAN_W:.3f}")
            elif t.time() - last_seen > P12_HOLE_LOST_TIMEOUT_S:
                stop(client)
                print(f"      [hole] lost too long — bail")
                return False
            else:
                client.publish("robobot/cmd/ti", "rc 0.000 0.000")
        t.sleep(0.05)

def main() -> None:
    ip, start_phase = parse_args()
    client = make_client(ip)
    t.sleep(1.0)

    print(f"\n[seesaw] connected to {ip}  start_phase={start_phase}")
    print(f"[seesaw] gate hardware: {'ENABLED' if GATE_HARDWARE_OK else 'DISABLED — manual'}")
    print(f"[seesaw] plan:")
    print(f"  1. line follow on long platform until junction")
    print(f"  2. (junction confirmed; nothing to do — proceed)")
    print(f"  3. turn onto seesaw branch (FIELD_TUNE direction)")
    print(f"  4. mount seesaw  arm→MID  speed {SEESAW_MOUNT_SPEED:.2f} m/s  ~{P3_MOUNT_DIST_M:.2f} m")
    print(f"  5. slow cross + watch for ball; capture when r≥{BALL_CAPTURE_RADIUS}px")
    print(f"  6. arm DOWN → gate CLOSE → arm UP")
    print(f"  7. drive off seesaw {P7_OFF_SEESAW_DIST_M:.2f} m")
    print(f"  8. find line forward (no line at exit)")
    print(f"  9. turn right {math.degrees(P10_TURN_RIGHT_RAD):+.0f}°")
    print(f" 10. line follow; SKIP first junction")
    print(f" 11. take SECOND junction (up the short ramp); follow up")
    print(f" 12. search + approach hole")
    print(f" 13. drop: arm DOWN → gate OPEN → wiggle → arm UP")
    print(f"  Each phase asks before running.  Ctrl+C = e-stop.\n")

    holder = ArmHolder(client, SERVO_ARM)
    holder.start()
    print(f"[arm] starting holder at UP (pwm {ARM_UP_PWM})")
    holder.hold(ARM_UP_PWM)
    t.sleep(ARM_SETTLE_S)

    cap = None  

    try:
        if start_phase <= 1 and confirm("[1/13] line follow on long platform until junction?"):
            consecutive = 0
            def at_junction():
                nonlocal consecutive
                if count_high_sensors() >= JUNCTION_SENSOR_COUNT:
                    consecutive += 1
                else:
                    consecutive = 0
                return consecutive >= JUNCTION_STABLE_FRAMES
            line_follow_until(client, at_junction,
                              base_speed=PLATFORM_SPEED, timeout_s=30.0,
                              label="P1-platform")
            t.sleep(0.3)

        if start_phase <= 2 and confirm("[2/13] junction reached — continue?"):
            print("      (no motion — operator inspection step)")

        if start_phase <= 3 and confirm("[3/13] turn onto seesaw branch?"):
            seesaw_branch_rad = -math.radians(90.0)
            print(f"      yaw {math.degrees(seesaw_branch_rad):+.0f}° onto branch")
            yaw_to_delta(client, seesaw_branch_rad, YAW_RATE)
            t.sleep(0.3)

        if start_phase <= 4 and confirm("[4/13] mount seesaw (arm to MID, slow forward)?"):
            print(f"      arm to MID (pwm {ARM_MID_PWM}) — shifts CoM forward for balance")
            holder.hold(ARM_MID_PWM)
            t.sleep(ARM_SETTLE_S)
            print(f"      creep forward {P3_MOUNT_DIST_M:.2f} m @ {SEESAW_MOUNT_SPEED:.2f} m/s")
            drive_for(client, SEESAW_MOUNT_SPEED, 0.0,
                      P3_MOUNT_DIST_M / SEESAW_MOUNT_SPEED,
                      ramp_s=P3_MOUNT_RAMP_S)
            t.sleep(0.3)

        if start_phase <= 5 and confirm("[5/13] cross seesaw + approach ball with vision?"):
            cap = open_stream(ip)
            if cap is None:
                print("      ! no camera stream — abort")
                return
            ok = approach_ball(client, cap)
            if not ok:
                print("      ! ball approach failed; you can retry phase 5 or abort.")

        if start_phase <= 6 and confirm("[6/13] capture: arm DOWN → gate CLOSE → arm UP?"):
            print(f"      arm DOWN (pwm {ARM_DOWN_PWM}) — tray over ball")
            holder.hold(ARM_DOWN_PWM, SERVO_SPEED_MOVE)
            t.sleep(ARM_SETTLE_S)
            print(f"      gate CLOSE  (servo {SERVO_GATE}={GATE_CLOSE_PWM})")
            gate_publish(client, GATE_CLOSE_PWM)
            t.sleep(0.8)
            print(f"      arm UP (pwm {ARM_UP_PWM}) — ball locked in cage")
            holder.hold(ARM_UP_PWM)
            t.sleep(ARM_SETTLE_S)

        if start_phase <= 7 and confirm("[7/13] drive off seesaw?"):
            drive_for(client, P7_OFF_SEESAW_SPEED, 0.0,
                      P7_OFF_SEESAW_DIST_M / P7_OFF_SEESAW_SPEED,
                      ramp_s=0.4)
            t.sleep(0.3)

        if start_phase <= 8 and confirm("[8/13] creep forward until line found?"):
            t0 = t.time()
            duration_cap = P9_FIND_LINE_MAX_M / EXIT_SEARCH_SPEED
            line_found = False
            while True:
                client.publish("robobot/cmd/ti", f"rc {EXIT_SEARCH_SPEED:.3f} 0.000")
                t.sleep(RC_RESEND_S)
                elapsed = t.time() - t0
                if S.line_valid:
                    print(f"      LINE DETECTED  high={S.line_high}  after {elapsed:.2f}s")
                    line_found = True
                    break
                if elapsed > duration_cap or elapsed > P9_FIND_LINE_TIMEOUT_S:
                    print(f"      ! safety cap hit at {elapsed:.2f}s — high={S.line_high}")
                    break
            stop(client)
            t.sleep(0.3)

        if start_phase <= 9 and confirm("[9/13] turn right onto line?"):
            yaw_to_delta(client, P10_TURN_RIGHT_RAD, YAW_RATE)
            t.sleep(0.3)

        if start_phase <= 10 and confirm("[10/13] line follow until first junction (will skip it)?"):
            consecutive = 0
            def at_junction_p10():
                nonlocal consecutive
                if count_high_sensors() >= JUNCTION_SENSOR_COUNT:
                    consecutive += 1
                else:
                    consecutive = 0
                return consecutive >= JUNCTION_STABLE_FRAMES
            line_follow_until(client, at_junction_p10,
                              base_speed=RAMP_LINE_SPEED, timeout_s=20.0,
                              label="P10-pre-stairs")
            print("      crossing past first junction (drive 0.20 m forward)")
            drive_for(client, RAMP_LINE_SPEED, 0.0, 0.20 / RAMP_LINE_SPEED, ramp_s=0.2)
            t.sleep(0.3)

        if start_phase <= 11 and confirm("[11/13] line follow to ramp junction + up the ramp?"):
            consecutive = 0
            def at_junction_p11():
                nonlocal consecutive
                if count_high_sensors() >= JUNCTION_SENSOR_COUNT:
                    consecutive += 1
                else:
                    consecutive = 0
                return consecutive >= JUNCTION_STABLE_FRAMES
            line_follow_until(client, at_junction_p11,
                              base_speed=RAMP_LINE_SPEED, timeout_s=20.0,
                              label="P11-find-ramp")
            ramp_branch_rad = math.radians(90.0)
            print(f"      yaw {math.degrees(ramp_branch_rad):+.0f}° onto ramp branch")
            yaw_to_delta(client, ramp_branch_rad, YAW_RATE)
            line_follow_until(client, lambda: not S.line_valid,
                              base_speed=RAMP_LINE_SPEED * 0.8, timeout_s=15.0,
                              label="P11-ramp-up")
            t.sleep(0.3)

        if start_phase <= 12 and confirm("[12/13] search and approach hole?"):
            if cap is None:
                cap = open_stream(ip)
                if cap is None:
                    print("      ! no camera — abort")
                    return
            ok = approach_hole(client, cap, holder)
            if not ok:
                print("      ! hole approach failed; you can retry phase 12.")

        if start_phase <= 13 and confirm("[13/13] drop ball (arm DOWN, gate OPEN, wiggle, arm UP)?"):
            print(f"      arm DOWN (pwm {ARM_DOWN_PWM}) — tray over hole")
            holder.hold(ARM_DOWN_PWM, SERVO_SPEED_MOVE)
            t.sleep(ARM_SETTLE_S)
            print(f"      gate OPEN  (servo {SERVO_GATE}={GATE_OPEN_PWM}) — release ball")
            gate_publish(client, GATE_OPEN_PWM)
            t.sleep(0.8)
            print("      wiggle to seat the ball")
            shake_in_place(client, omega=0.6, half_period_s=0.20, cycles=3)
            t.sleep(0.3)
            print(f"      arm UP (pwm {ARM_UP_PWM}) — done")
            holder.hold(ARM_UP_PWM)
            t.sleep(ARM_SETTLE_S)

        print("\n[seesaw] sequence complete.")

    except KeyboardInterrupt:
        print("\n[seesaw] ^C — emergency stop.")
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
        print("[seesaw] done.")


if __name__ == "__main__":
    main()
