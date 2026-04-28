#!/usr/bin/env python3
"""ArUco-based closed-loop docking to a marker pair, then drop the balls.

The ball-delivery final approach: instead of an open-loop drive of a fixed
distance, this looks at the target ArUco markers (Zone C: IDs 14 and 15),
estimates their range and bearing, drives most of the way, re-detects to
correct any drift, and finishes a fixed gripper offset from the marker face.

Designed to be called from another script (`run_dock(client, ip)`,
`drop_balls(client)`) or run standalone for tuning:

    python3 -m control.tasks.aruco_dock [robot-ip]
"""
from __future__ import annotations
import math
import os
import sys
import time as t

import cv2
import numpy as np
from paho.mqtt import client as mqtt_client

# ---------------------------------------------------------------------------
# CONFIG — tune for the day
# ---------------------------------------------------------------------------
DEFAULT_IP        = "10.197.219.117"
STREAM_URL_FMT    = "http://{ip}:7123/stream.mjpg"

CALIB_PATH        = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "calibration", "camera_params.npz",
)

ARUCO_DICT        = cv2.aruco.DICT_4X4_50
TARGET_IDS        = {14, 15}              # Zone C markers
MARKER_SIZE_M     = 0.10                  # 10 cm physical size

GRIPPER_DISTANCE  = 0.30                  # final stop distance to marker face
SAFETY_BUFFER     = 0.20                  # leave this much for the correction pass

DRIVE_SPEED       = 0.15                  # m/s
TURN_RATE         = 0.6                   # rad/s
RC_RESEND_S       = 0.05

DETECT_TIMEOUT_S  = 6.0
MIN_DETECTIONS    = 3                     # consecutive stable frames before locking

# Drop sequence (servo IDs match ramp_to_bowl_line.py)
SERVO_ARM         = 1
ARM_DOWN_PWM      = 500
ARM_PLACE_SPEED   = 120
ARM_RELEASE_PWM   = 9999                  # |pos|>1024 disables the servo
DUMP_HOLD_S       = 1.5


# ---------------------------------------------------------------------------
# Camera + detector setup
# ---------------------------------------------------------------------------
def load_camera_params(path: str):
    if not os.path.isfile(path):
        print(f"  ! calibration file not found: {path}")
        return None, None
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def make_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshConstant = 10
    params.minMarkerPerimeterRate = 0.01
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def marker_object_points(size_m: float) -> np.ndarray:
    """Square marker on Z=0 plane, +X right, +Y up, in marker frame."""
    s = size_m / 2.0
    return np.array([
        [-s,  s, 0.0],   # top-left
        [ s,  s, 0.0],   # top-right
        [ s, -s, 0.0],   # bottom-right
        [-s, -s, 0.0],   # bottom-left
    ], dtype=np.float32)


def detect_target(cap, detector, camera_matrix, dist_coeffs):
    ok, frame = cap.read()
    if not ok or frame is None:
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return frame, {}

    obj_pts = marker_object_points(MARKER_SIZE_M)
    found = {}
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        mid = int(marker_id)
        if mid not in TARGET_IDS:
            continue
        img_pts = marker_corners.reshape(-1, 2).astype(np.float32)
        ok_pnp, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if ok_pnp:
            found[mid] = tvec.flatten()
    return frame, found


def target_geometry(tvecs: dict):
    """Centroid of detected markers → (distance_m, bearing_rad).

    Bearing: 0 = straight ahead (camera +Z), positive = target on right.
    """
    if not tvecs:
        return None
    points = np.stack(list(tvecs.values()))
    centroid = points.mean(axis=0)
    x_cam, _, z_cam = centroid
    distance = float(math.hypot(x_cam, z_cam))
    bearing  = float(math.atan2(x_cam, z_cam))
    return distance, bearing


# ---------------------------------------------------------------------------
# Robot motion (raw MQTT, no uservice)
# ---------------------------------------------------------------------------
def make_mqtt(ip: str) -> mqtt_client.Client:
    if hasattr(mqtt_client, "CallbackAPIVersion"):
        c = mqtt_client.Client(
            client_id="aruco-dock",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
        )
    else:
        c = mqtt_client.Client(client_id="aruco-dock")
    c.connect(ip, 1883, 60)
    c.loop_start()
    return c


def stop(client): client.publish("robobot/cmd/ti", "rc 0.000 0.000")


def drive_for(client, v: float, w: float, duration_s: float, ramp_s: float = 0.4):
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
        client.publish("robobot/cmd/ti", f"rc {v*scale:.3f} {w:.3f}")
        t.sleep(RC_RESEND_S)
    stop(client)


def turn_by(client, delta_rad: float):
    if abs(delta_rad) < math.radians(2.0):
        return
    direction = 1.0 if delta_rad > 0 else -1.0
    duration = abs(delta_rad) / TURN_RATE
    drive_for(client, 0.0, direction * TURN_RATE, duration, ramp_s=0.0)


def servo(client, sid: int, pos: int, speed: int = ARM_PLACE_SPEED):
    client.publish("robobot/cmd/T0", f"servo {sid} {pos} {speed}")


# ---------------------------------------------------------------------------
# Detection helpers
# ---------------------------------------------------------------------------
def lock_target(cap, detector, K, D, label: str):
    """Spend up to DETECT_TIMEOUT_S looking for TARGET_IDS. Need
    MIN_DETECTIONS consecutive frames before returning the average."""
    print(f"  [detect:{label}] looking for ArUco IDs {sorted(TARGET_IDS)} ...")
    samples = []
    last_ids = ()
    t_end = t.time() + DETECT_TIMEOUT_S
    while t.time() < t_end:
        _, tvecs = detect_target(cap, detector, K, D)
        if tvecs is None:
            print(f"  [detect:{label}] stream read failed; retrying")
            t.sleep(0.1)
            continue
        geom = target_geometry(tvecs)
        if geom is None:
            samples.clear()
            t.sleep(0.05)
            continue
        samples.append(geom)
        last_ids = tuple(sorted(tvecs.keys()))
        if len(samples) >= MIN_DETECTIONS:
            recent = samples[-MIN_DETECTIONS:]
            d_avg = sum(g[0] for g in recent) / len(recent)
            b_avg = sum(g[1] for g in recent) / len(recent)
            print(f"  [detect:{label}] LOCK  ids={list(last_ids)}  "
                  f"distance={d_avg:.2f} m  bearing={math.degrees(b_avg):+.1f}°")
            return d_avg, b_avg
    print(f"  [detect:{label}] timed out — no stable detection")
    return None


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def drop_balls(client) -> None:
    """Lower the tray to drop the cargo, then release the servo."""
    print(f"  [drop] arm DOWN (pwm {ARM_DOWN_PWM} @ {ARM_PLACE_SPEED})")
    servo(client, SERVO_ARM, ARM_DOWN_PWM, ARM_PLACE_SPEED)
    t.sleep(DUMP_HOLD_S)
    print(f"  [drop] release servo (pwm {ARM_RELEASE_PWM})")
    servo(client, SERVO_ARM, ARM_RELEASE_PWM, ARM_PLACE_SPEED)
    t.sleep(0.3)


def run_dock(client, ip: str) -> bool:
    """Closed-loop docking against TARGET_IDS using the robot's MJPEG stream.

    Stops GRIPPER_DISTANCE from the marker face. Does NOT drop the balls
    (call drop_balls() separately). Returns True on success.
    """
    K, D = load_camera_params(CALIB_PATH)
    if K is None:
        return False

    detector = make_detector()
    stream_url = STREAM_URL_FMT.format(ip=ip)
    print(f"  [dock] opening stream {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"  ! cannot open stream {stream_url}")
        return False

    try:
        first = lock_target(cap, detector, K, D, "first")
        if first is None:
            print("  [dock] aborting — no markers visible at start")
            return False
        d1, b1 = first

        if abs(b1) > math.radians(2.0):
            print(f"  [dock] yaw to face target  delta={math.degrees(-b1):+.1f}°")
            turn_by(client, -b1)
            t.sleep(0.5)

        drive1_m = max(0.0, d1 - GRIPPER_DISTANCE - SAFETY_BUFFER)
        if drive1_m > 0.05:
            duration = drive1_m / DRIVE_SPEED
            print(f"  [dock] coarse drive  dist={drive1_m:.2f} m  "
                  f"({duration:.1f} s @ {DRIVE_SPEED:.2f} m/s)")
            drive_for(client, DRIVE_SPEED, 0.0, duration, ramp_s=0.5)
        else:
            print(f"  [dock] already close enough for correction (d1={d1:.2f})")

        t.sleep(0.5)

        second = lock_target(cap, detector, K, D, "second")
        if second is None:
            print("  [dock] WARNING — lost markers on second look; continuing blind")
            d2, b2 = d1 - drive1_m, 0.0
        else:
            d2, b2 = second

        if abs(b2) > math.radians(2.0):
            print(f"  [dock] correction yaw  delta={math.degrees(-b2):+.1f}°")
            turn_by(client, -b2)
            t.sleep(0.3)

        drive2_m = max(0.0, d2 - GRIPPER_DISTANCE)
        if drive2_m > 0.02:
            duration = drive2_m / DRIVE_SPEED
            print(f"  [dock] final drive  dist={drive2_m:.2f} m  "
                  f"({duration:.1f} s @ {DRIVE_SPEED:.2f} m/s)")
            drive_for(client, DRIVE_SPEED, 0.0, duration, ramp_s=0.4)
        t.sleep(0.5)
        return True

    finally:
        cap.release()


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------
def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    print(f"[dock] connecting to robot {ip}")
    client = make_mqtt(ip)
    t.sleep(0.5)
    try:
        ok = run_dock(client, ip)
        if ok:
            drop_balls(client)
            print("\n[dock] sequence complete.")
        else:
            print("\n[dock] dock failed; not dropping.")
    except KeyboardInterrupt:
        print("\n[dock] ^C — emergency stop")
    finally:
        stop(client)
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("[dock] done.")


if __name__ == "__main__":
    main()
