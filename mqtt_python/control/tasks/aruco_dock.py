from __future__ import annotations
import math
import os
import sys
import time as t

import cv2
import numpy as np
from paho.mqtt import client as mqtt_client


DEFAULT_IP        = "10.197.219.117"
STREAM_URL_FMT    = "http://{ip}:7123/stream.mjpg"

CALIB_PATH        = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "calibration", "camera_params.npz",
)

ARUCO_DICT        = cv2.aruco.DICT_4X4_50
TARGET_IDS        = {14, 15}              
PRIMARY_ID        = 15                   
                                         
                                          
                                        
MARKER_SIZE_M     = 0.10                  

GRIPPER_DISTANCE  = 0.70                  
                                          
                                          
                                         
                                          
                                         
SAFETY_BUFFER     = 0.20                 

DRIVE_SPEED       = 0.15                  # m/s
TURN_RATE         = 0.6                   # rad/s
RC_RESEND_S       = 0.05

DETECT_TIMEOUT_S  = 6.0
MIN_DETECTIONS    = 3                     

DOCK_MAX_V        = 0.15
DOCK_MAX_W        = 0.6
DOCK_KV           = 0.6                  
DOCK_KW           = 1.6                   
DOCK_RANGE_TOL_M  = 0.025                 
DOCK_BEAR_TOL_RAD = math.radians(2.0)     
DOCK_TIMEOUT_S    = 25.0
LOST_FRAMES_BAIL  = 25                    
DOCK_DEBUG_PERIOD = 0.4                   

SERVO_ARM         = 1
ARM_UP_PWM        = -800                  
ARM_DOWN_PWM      = 500                   
ARM_DROP_PWM      = 200                   
                                         
ARM_PLACE_SPEED   = 120
ARM_DROP_HOLD_S   = 1.0                   

SHAKE_CYCLES        = 4
SHAKE_OMEGA_RAD_S   = 0.6
SHAKE_HALF_PERIOD_S = 0.18
SHAKE_PAUSE_S       = 0.6                 

ARM_LIFT_S          = 1.5                 

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
    if not tvecs:
        return None
    if PRIMARY_ID in tvecs:
        chosen_id = PRIMARY_ID
    else:
        chosen_id = next(iter(tvecs))      
    x_cam, _, z_cam = tvecs[chosen_id]
    distance = float(math.hypot(x_cam, z_cam))
    bearing  = float(math.atan2(x_cam, z_cam))
    return distance, bearing, int(chosen_id)

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


def lock_target(cap, detector, K, D, label: str):
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
        last_chosen = geom[2]
        if len(samples) >= MIN_DETECTIONS:
            recent = samples[-MIN_DETECTIONS:]
            d_avg = sum(g[0] for g in recent) / len(recent)
            b_avg = sum(g[1] for g in recent) / len(recent)
            print(f"  [detect:{label}] LOCK on id={last_chosen}  "
                  f"(visible={list(last_ids)})  "
                  f"distance={d_avg:.2f} m  bearing={math.degrees(b_avg):+.1f}°")
            return d_avg, b_avg
    print(f"  [detect:{label}] timed out — no stable detection")
    return None

def _shake_tray(client) -> None:
    """Yaw left/right a few times so any balls stuck in the tray roll out."""
    for _ in range(SHAKE_CYCLES):
        client.publish("robobot/cmd/ti", f"rc 0.000 {SHAKE_OMEGA_RAD_S:.3f}")
        t.sleep(SHAKE_HALF_PERIOD_S)
        client.publish("robobot/cmd/ti", f"rc 0.000 {-SHAKE_OMEGA_RAD_S:.3f}")
        t.sleep(SHAKE_HALF_PERIOD_S)
    stop(client)
    t.sleep(0.2)


def drop_balls(client) -> None:
    print(f"  [drop] arm to drop height  (pwm {ARM_DROP_PWM} @ {ARM_PLACE_SPEED})")
    servo(client, SERVO_ARM, ARM_DROP_PWM, ARM_PLACE_SPEED)
    t.sleep(ARM_DROP_HOLD_S)
    print("  [drop] >>> MANUAL: open hazne slider by hand, then wait <<<")
    t.sleep(SHAKE_PAUSE_S)

    print("  [drop] shake #1")
    _shake_tray(client)
    t.sleep(SHAKE_PAUSE_S)

    print("  [drop] shake #2")
    _shake_tray(client)
    t.sleep(SHAKE_PAUSE_S)

    print(f"  [drop] arm UP  (pwm {ARM_UP_PWM} @ {ARM_PLACE_SPEED})")
    servo(client, SERVO_ARM, ARM_UP_PWM, ARM_PLACE_SPEED)
    t.sleep(ARM_LIFT_S)
    print("  [drop] mission complete.")


def run_dock(client, ip: str) -> bool:
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

    print(f"  [dock] target  range={GRIPPER_DISTANCE:.2f} m  "
          f"prefer_id={PRIMARY_ID}  tol={DOCK_RANGE_TOL_M*100:.1f} cm / "
          f"{math.degrees(DOCK_BEAR_TOL_RAD):.1f}°")

    t_start = t.time()
    lost_frames = 0
    last_debug = 0.0
    last_geom = None

    try:
        while True:
            if t.time() - t_start > DOCK_TIMEOUT_S:
                print(f"  [dock] TIMEOUT after {DOCK_TIMEOUT_S:.0f}s — last "
                      f"geom={last_geom}")
                stop(client)
                return False

            _, tvecs = detect_target(cap, detector, K, D)
            geom = target_geometry(tvecs) if tvecs else None

            if geom is None:
                lost_frames += 1
                if lost_frames > LOST_FRAMES_BAIL:
                    print(f"  [dock] markers lost for {lost_frames} frames; "
                          f"giving up. last={last_geom}")
                    stop(client)
                    return False
                client.publish("robobot/cmd/ti", "rc 0.000 0.000")
                t.sleep(0.05)
                continue

            lost_frames = 0
            last_geom = geom
            rng, bearing, chosen = geom
            range_err = rng - GRIPPER_DISTANCE  

            if abs(range_err) < DOCK_RANGE_TOL_M and abs(bearing) < DOCK_BEAR_TOL_RAD:
                stop(client)
                print(f"  [dock] DOCKED  id={chosen}  range={rng:.2f} m  "
                      f"bearing={math.degrees(bearing):+.1f}°")
                return True

            if range_err <= 0.0:
                v = 0.0
            else:
                v = min(DOCK_MAX_V, DOCK_KV * range_err)

            w = max(-DOCK_MAX_W, min(DOCK_MAX_W, DOCK_KW * bearing))

            if abs(bearing) > math.radians(20.0):
                v = 0.0
            elif abs(bearing) > math.radians(10.0):
                v *= 0.4

            client.publish("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

            now = t.time()
            if now - last_debug > DOCK_DEBUG_PERIOD:
                last_debug = now
                print(f"  [dock] id={chosen}  rng={rng:.2f}m  "
                      f"bear={math.degrees(bearing):+5.1f}°  "
                      f"v={v:.2f}  w={w:+.2f}")

            t.sleep(0.05)

    finally:
        cap.release()
        stop(client)


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
