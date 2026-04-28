#!/usr/bin/env python3
"""
live_perception_overlay.py
=============================================================================
Live perception pipeline: line extraction, ArUco detection, and EKF feed.

This script is field-map aware. Position correction is handled through
slocalize.py, while svision_pose.py produces the vision pose update sent to
the Kalman filter.

Annotated frames can also be exposed as an MJPEG stream for the web dashboard.
=============================================================================
"""

from __future__ import annotations

import argparse
import http.server
import socketserver
import sys
import threading
import time
import math
import json as _json
from datetime import datetime
from typing import Optional

import cv2
import numpy as np

from perception.camera.sline import vision
from perception.camera.saruco import aruco_detector
from sensors.fusion.smeasurement import build_measurement
from sensors.spose import pose
from sensors.fusion.skalman import kalman
from util.uservice import service
from sensors.fusion.slocalize import localizer
from perception.pose.svision_pose import vision_pose
from perception.pose.slandmark_match import landmark_matcher


# ---------------------------------------------------------------------------
# Annotated-frame MJPEG server
# ---------------------------------------------------------------------------
# When --mjpeg-port > 0, the latest annotated overlay is exposed at:
#   http://<host>:<port>/stream.mjpg
#
# This lets the web dashboard subscribe to the annotated view without requiring
# an OpenCV preview window. Frames are shared through one lock-protected JPEG
# buffer to avoid copying large arrays for every request.

_mjpeg_lock = threading.Lock()
_mjpeg_latest_jpeg: Optional[bytes] = None


class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_response(404)
            self.end_headers()
            return

        boundary = "frame"
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={boundary}",
        )
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()

        try:
            while True:
                with _mjpeg_lock:
                    jpg = _mjpeg_latest_jpeg
                if jpg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(0.03)  # cap the stream near 30 fps
        except Exception:
            pass

    def log_message(self, fmt, *args):  # keep stdout clean
        return


class _ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def _start_mjpeg_server(port: int) -> None:
    srv = _ThreadingHTTPServer(("0.0.0.0", port), _MJPEGHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(
        f"% MJPEG annotated stream on http://localhost:{port}/stream.mjpg",
        flush=True,
    )


# ---------------------------------------------------------------------------
# CSV LOG
# ---------------------------------------------------------------------------
# The _v2 filename is intentional because the column set changed.
# New fields include sigma, confidence, landmark, and fix-mode columns.
# Keeping a new file avoids mixing different CSV schemas in one log.
_log_file = open("vision_kalman_log_v2.csv", "a", encoding="utf-8")

if _log_file.tell() == 0:
    _log_file.write(
        "t_epoch,t_str,"
        "prior_x,prior_y,prior_yaw,"
        "vision_valid,vision_src,vision_x,vision_y,vision_z,vision_yaw,"
        "sigma_xy_m,sigma_yaw_rad,aruco_confidence,"
        "aruco_fix_mode,n_aruco_used,aruco_ids,"
        "landmark_name,"
        "line_valid,aruco_n_detected\n"
    )
    _log_file.flush()


def _log_vision_row(px, py, yaw, vp_out, result, aruco_detections):
    """Append one diagnostic row to the vision/Kalman CSV log.

    `vp_out` is the dict returned by SVisionPose.update(). Invalid fixes may
    not contain all fields, so missing values are written as blanks or "nan"
    to keep the CSV rectangular and easy to load.
    """
    t_epoch = time.time()
    t_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    line_valid = bool(result.get("line_valid", False)) if result else False
    aruco_n_detected = len(aruco_detections) if aruco_detections else 0

    def _fmt(v, prec=4):
        if v is None:
            return ""
        if isinstance(v, float) and (v != v):   # NaN
            return "nan"
        try:
            return f"{float(v):.{prec}f}"
        except (TypeError, ValueError):
            return str(v)

    if vp_out and vp_out.get("valid"):
        vp_ids = vp_out.get("aruco_ids") or []
        ids_str = "|".join(str(i) for i in vp_ids)   # pipe separator keeps CSV commas safe
        _log_file.write(
            f"{t_epoch:.3f},{t_str},"
            f"{px:.3f},{py:.3f},{yaw:.3f},"
            f"1,{vp_out.get('source','unknown')},"
            f"{_fmt(vp_out.get('x'),3)},{_fmt(vp_out.get('y'),3)},"
            f"{_fmt(vp_out.get('z'),3)},{_fmt(vp_out.get('yaw'),3)},"
            f"{_fmt(vp_out.get('sigma_xy_m'),5)},"
            f"{_fmt(vp_out.get('sigma_yaw_rad'),5)},"
            f"{_fmt(vp_out.get('aruco_confidence'),3)},"
            f"{vp_out.get('aruco_fix_mode') or ''},"
            f"{vp_out.get('n_aruco_used') or ''},"
            f"{ids_str},"
            f"{vp_out.get('landmark') or ''},"
            f"{line_valid},{aruco_n_detected}\n"
        )
    else:
        _log_file.write(
            f"{t_epoch:.3f},{t_str},"
            f"{px:.3f},{py:.3f},{yaw:.3f},"
            f"0,none,nan,nan,nan,nan,"
            f",,,"          # sigma_xy, sigma_yaw, confidence
            f",,,"          # fix_mode, n_used, ids
            f","            # landmark_name
            f"{line_valid},{aruco_n_detected}\n"
        )

    _log_file.flush()


# ---------------------------------------------------------------------------
# MQTT Kalman state cache
# ---------------------------------------------------------------------------
# This script can run as a separate process, so it cannot directly read the
# Kalman object in memory. The latest Kalman state is cached from MQTT.
_kalman_mqtt: dict = {"x": None, "y": None, "yaw": None, "pitch": None}

# Predicted state before measurement update, based on encoder + IMU only.
# Vision should use this as the prior to avoid feeding back its own correction.
_kalman_pred_mqtt: dict = {"x": None, "y": None, "yaw": None, "pitch": None}

# Initial pose used before Kalman data arrives.
_start_pose_cache = {"x": 4.775, "y": 0.235, "yaw": 1.5708, "pitch": 0.0}

# Last reliable pose used as fallback when Kalman is unavailable or not settled yet.
_pose_seed_cache = {"x": 4.775, "y": 0.235, "yaw": 1.5708, "pitch": 0.0}


_kalman_msg_count = 0
_kalman_last_logged_pos = None


def _on_kalman_state_msg(payload):
    """Process a Kalman state payload.

    The payload may arrive either as a string/bytes object from the in-process
    listener or as a paho MQTT message object from the standalone callback path.
    Both formats are handled here.
    """
    global _kalman_mqtt, _kalman_pred_mqtt
    global _kalman_msg_count, _kalman_last_logged_pos
    try:
        if isinstance(payload, (bytes, str)):
            raw = payload if isinstance(payload, str) else payload.decode()
        else:
            raw = payload.payload.decode()
        d = _json.loads(raw)

        # Format 1: SKalman.publish_state()
        if "position" in d and "orientation" in d:
            _kalman_mqtt["x"] = float(d["position"]["x"])
            _kalman_mqtt["y"] = float(d["position"]["y"])
            _kalman_mqtt["yaw"] = float(d["orientation"]["yaw"])
            _kalman_mqtt["pitch"] = float(d["orientation"]["pitch"])
        # Format 2: UService.publish_kalman_state()
        elif "x" in d and isinstance(d["x"], dict):
            xs = d["x"]
            _kalman_mqtt["x"] = float(xs["x"])
            _kalman_mqtt["y"] = float(xs["y"])
            _kalman_mqtt["yaw"] = float(xs["yaw"])
            _kalman_mqtt["pitch"] = float(xs["pitch"])
        else:
            print(f"% kalman state: unknown format keys={list(d.keys())}")
            return

        # One-shot diagnostic to confirm the subscription is alive.
        # After that, log only when the cached position shifts by more than
        # 5 cm, which proves the cache is updating and not frozen.
        _kalman_msg_count += 1
        if _kalman_msg_count == 1:
            print(f"% kalman state subscriber: FIRST message received  "
                  f"x={_kalman_mqtt['x']:.3f}  y={_kalman_mqtt['y']:.3f}  "
                  f"yaw={_kalman_mqtt['yaw']:.3f}")
            _kalman_last_logged_pos = (_kalman_mqtt["x"], _kalman_mqtt["y"])
        elif _kalman_last_logged_pos is not None:
            dx = _kalman_mqtt["x"] - _kalman_last_logged_pos[0]
            dy = _kalman_mqtt["y"] - _kalman_last_logged_pos[1]
            if (dx * dx + dy * dy) > 0.05 ** 2:
                print(f"% kalman cache shifted: x={_kalman_mqtt['x']:.3f}  "
                      f"y={_kalman_mqtt['y']:.3f}  yaw={_kalman_mqtt['yaw']:.3f}  "
                      f"(msg #{_kalman_msg_count})")
                _kalman_last_logged_pos = (_kalman_mqtt["x"], _kalman_mqtt["y"])

        # Vision uses the predicted state as prior, before measurement update.
        # If x_pred is missing, fall back to the freshly decoded post-update
        # state so the prior still follows the robot.
        if "x_pred" in d:
            xp = d["x_pred"]
            _kalman_pred_mqtt["x"]     = float(xp.get("x", 0))
            _kalman_pred_mqtt["y"]     = float(xp.get("y", 0))
            _kalman_pred_mqtt["yaw"]   = float(xp.get("yaw", 0))
            _kalman_pred_mqtt["pitch"] = float(xp.get("pitch", 0))
        else:
            _kalman_pred_mqtt["x"]     = _kalman_mqtt["x"]
            _kalman_pred_mqtt["y"]     = _kalman_mqtt["y"]
            _kalman_pred_mqtt["yaw"]   = _kalman_mqtt["yaw"]
            _kalman_pred_mqtt["pitch"] = _kalman_mqtt["pitch"]
            if _kalman_msg_count <= 5 or _kalman_msg_count % 100 == 0:
                print(f"% kalman state: 'x_pred' missing  "
                      f"keys={list(d.keys())}  "
                      f"(msg #{_kalman_msg_count}; using post-update as prior)")

        # Periodic prior log so we can confirm the landmark gate sees the robot moving.
        if _kalman_msg_count == 1 or _kalman_msg_count % 50 == 0:
            print(f"% kalman prior: x={_kalman_pred_mqtt['x']:.3f}  "
                  f"y={_kalman_pred_mqtt['y']:.3f}  "
                  f"yaw={_kalman_pred_mqtt['yaw']:.3f}  "
                  f"(msg #{_kalman_msg_count})")

    except Exception as e:
        print(f"% kalman state parse error: {e}")


def connect_stream(stream_url: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def _send_vision_pose(x, y, z, yaw, pitch, source='unknown',
                       sigma_xy=None, sigma_yaw=None,
                       n_used=None, confidence=None,
                       landmark=None):
    """Publish a vision_pose update.

    Wire format:
        TIMESTAMP X Y Z YAW PITCH SOURCE [KEY=VALUE ...]

    The positional fields stay fixed for legacy decoders. Optional KEY=VALUE
    tokens carry measurement-noise metadata from SVisionPose. Consumers that
    do not recognise a key can safely ignore it.
    """
    msg = f"{time.time():.6f} {x:.4f} {y:.4f} {z:.4f} {yaw:.4f} {pitch:.4f} {source}"
    extras = []
    if sigma_xy  is not None: extras.append(f"sxy={sigma_xy:.5f}")
    if sigma_yaw is not None: extras.append(f"syaw={sigma_yaw:.5f}")
    if n_used    is not None: extras.append(f"n={int(n_used)}")
    if confidence is not None: extras.append(f"conf={confidence:.3f}")
    if landmark  is not None: extras.append(f"landmark={landmark}")
    if extras:
        msg = msg + " " + " ".join(extras)
    service.clientOut.publish("robobot/drive/T0/vision_pose", msg)


def _current_pitch_imu():
    """Return the IMU pitch estimate, or 0.0 if it is unavailable."""
    try:
        from sensors.spose import pose
        if pose.poseCnt > 0 and len(pose.pose) > 3:
            return float(pose.pose[3])
    except Exception:
        pass
    return 0.0


# ---------------------------------------------------------------------------
# MINI MAP
# ---------------------------------------------------------------------------
_FIELD_W = 7.0
_FIELD_H = 6.0


def draw_mini_map(frame: np.ndarray, vision_state: dict,
                  map_size: int = 220) -> np.ndarray:
    """
    Draw a small top-down map in the bottom-right corner.

    The map shows the robot position, tape layout, gates, start/goal area,
    and a few field landmarks for quick visual debugging.
    """
    h, w = frame.shape[:2]
    margin = 10
    x0 = w - map_size - margin
    y0 = h - map_size - margin

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + map_size, y0 + map_size), (20, 20, 20), -1)
    frame = cv2.addWeighted(overlay, 0.75, frame, 0.25, 0)

    cv2.rectangle(frame, (x0 + 2, y0 + 2), (x0 + map_size - 2, y0 + map_size - 2), (0, 180, 255), 1)

    def world_to_map(wx, wy):
        mx = int(x0 + (wx / _FIELD_W) * map_size)
        my = int(y0 + map_size - (wy / _FIELD_H) * map_size)
        return mx, my

    try:
        from field_map.field_map_2026 import FIELD

        for tl in FIELD.tape_lines:
            pts = tl.waypoints
            for i in range(len(pts) - 1):
                p1 = world_to_map(pts[i].x, pts[i].y)
                p2 = world_to_map(pts[i + 1].x, pts[i + 1].y)
                cv2.line(frame, p1, p2, (180, 180, 180), 1)

        for np_ in FIELD.nav_paths:
            for seg in np_.segments:
                if hasattr(seg, 'waypoints') and len(seg.waypoints) >= 2:
                    pts = seg.waypoints
                    for i in range(len(pts) - 1):
                        p1 = world_to_map(pts[i].x, pts[i].y)
                        p2 = world_to_map(pts[i + 1].x, pts[i + 1].y)
                        cv2.line(frame, p1, p2, (120, 120, 120), 1)
                elif hasattr(seg, 'sample'):
                    xs, ys, _ = seg.sample(n=30)
                    pts_map = [world_to_map(x, y) for x, y in zip(xs, ys)]
                    for i in range(len(pts_map) - 1):
                        cv2.line(frame, pts_map[i], pts_map[i + 1], (120, 120, 120), 1)

        for g in FIELD.all_gates():
            gp = world_to_map(g.center.x, g.center.y)
            color = (0, 255, 255) if g.has_satellite else (0, 200, 200)
            cv2.circle(frame, gp, 3, color, -1)

        sa = FIELD.start_area
        cv2.rectangle(
            frame,
            world_to_map(sa.origin.x, sa.origin.y),
            world_to_map(sa.origin.x + sa.width, sa.origin.y + sa.depth),
            (255, 200, 0),
            1,
        )

        gp_world = FIELD.goal_point
        cv2.rectangle(
            frame,
            world_to_map(gp_world.x - 0.09, 0),
            world_to_map(gp_world.x + 0.09, 0.18),
            (255, 100, 0),
            1,
        )

        sc = FIELD.sorting_center
        abcd_d = 0.60 / math.sqrt(2)
        abcd_pts_map = [
            world_to_map(sc.center.x, sc.center.y - abcd_d),
            world_to_map(sc.center.x + abcd_d, sc.center.y),
            world_to_map(sc.center.x, sc.center.y + abcd_d),
            world_to_map(sc.center.x - abcd_d, sc.center.y),
        ]
        for i in range(4):
            cv2.line(frame, abcd_pts_map[i], abcd_pts_map[(i + 1) % 4], (0, 180, 100), 1)

        rc_map = world_to_map(FIELD.roundabout.center.x, FIELD.roundabout.center.y)
        r_px = int(FIELD.roundabout.radius / _FIELD_W * map_size)
        cv2.circle(frame, rc_map, r_px, (150, 150, 150), 1)

    except Exception as e:
        print(f"% draw_mini_map warning: {e}")

    if vision_state.get("valid"):
        rx, ry = vision_state.get("px", 0), vision_state.get("py", 0)
        ryaw = vision_state.get("yaw", 0)
        rmap = world_to_map(rx, ry)

        cv2.circle(frame, rmap, 6, (0, 255, 0), -1)
        cv2.circle(frame, rmap, 6, (255, 255, 255), 1)

        arrow_len = 14
        ax = int(rmap[0] + arrow_len * math.cos(ryaw))
        ay = int(rmap[1] - arrow_len * math.sin(ryaw))
        cv2.arrowedLine(frame, rmap, (ax, ay), (0, 255, 0), 2, tipLength=0.4)

        zone = vision_state.get("zone", "")
        if zone:
            cv2.putText(frame, zone, (x0 + 4, y0 + map_size - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)

    cv2.putText(frame, "MAP", (x0 + map_size // 2 - 14, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    return frame


# ---------------------------------------------------------------------------
# PANEL
# ---------------------------------------------------------------------------
def draw_panel(frame: np.ndarray, result: dict, aruco_detections: list,
               vision_state: dict) -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]

    panel_w = min(580, w - 20)
    panel_h = 420
    x0, y0 = 10, 10

    overlay = vis.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    vis = cv2.addWeighted(overlay, 0.42, vis, 0.58, 0)

    def put(line_idx, text, color=(220, 220, 220), scale=0.68, thick=1):
        y = y0 + 28 + line_idx * 22
        cv2.putText(vis, text, (x0 + 12, y),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

    fork_evidence = int(result.get("fork_evidence", 0))
    fork_cand = bool(result.get("fork_candidate", False))
    fork_conf = bool(result.get("fork_confirmed", False))
    active_b = int(result.get("active_branch", 0))
    branches = result.get("branches", [{}, {}])
    line_valid = bool(result.get("line_valid", False))

    put(0, "Live Perception Overlay", (0, 255, 255), 0.78, 2)

    if fork_conf:
        fork_text = f"FORK CONFIRMED  evidence={fork_evidence}/8  active: B{active_b}"
        fork_color = (0, 60, 255)
    elif fork_cand:
        fork_text = f"fork candidate  evidence={fork_evidence}/8  active: B{active_b}"
        fork_color = (0, 165, 255)
    else:
        fork_text = f"no fork  evidence={fork_evidence}/8"
        fork_color = (140, 140, 140)
    put(1, fork_text, fork_color)

    put(2, f"line_valid : {line_valid}", (0, 220, 0) if line_valid else (0, 80, 220))

    offset = float(result.get("line_offset", 0.0))
    heading = float(result.get("line_heading", 0.0))
    curvature = float(result.get("line_curvature", 0.0))
    pts = result.get("line_points_robot", [])

    put(3, f"points_count   : {len(pts)}")

    if line_valid:
        put(4, f"line_offset    : {offset:+.3f} m", (0, 220, 0))
        put(5, f"line_heading   : {heading:+.3f} rad  ({math.degrees(heading):+.1f} deg)", (0, 220, 0))
        put(6, f"line_curvature : {curvature:+.3f} 1/m", (0, 220, 0))
        near_pt = pts[0] if len(pts) >= 1 else None
        far_pt = pts[-1] if len(pts) >= 1 else None
        if near_pt:
            put(7, f"nearest_point  : X={near_pt[0]:.3f}  Y={near_pt[1]:+.3f}", (200, 200, 120))
        if far_pt:
            put(8, f"farthest_point : X={far_pt[0]:.3f}  Y={far_pt[1]:+.3f}", (200, 200, 120))
    else:
        for row, lbl in enumerate(["line_offset", "line_heading", "line_curvature"], start=4):
            put(row, f"{lbl:<15}: ---", (0, 80, 220))

    if fork_cand and len(branches) > 1:
        b1 = branches[1]
        b1_color = (0, 220, 220)
        if b1.get("valid"):
            put(10, f"B1 offset  : {b1['offset']:+.3f} m", b1_color)
            put(11, f"B1 heading : {math.degrees(b1['heading']):+.1f} deg", b1_color)
        else:
            put(10, "B1: not valid", b1_color)

    if aruco_detections:
        put(12, f"ArUco: {len(aruco_detections)} marker(s)", (200, 200, 0))
        for idx, d in enumerate(aruco_detections[:2]):
            put(13 + idx,
                f"  ID={d['id']}  r={d['range']:.2f}m  b={math.degrees(d['bearing']):+.1f}deg",
                (200, 200, 0))
    else:
        put(12, "ArUco: none", (100, 100, 100))

    vs = vision_state
    if vs["valid"]:
        put(15, f"EKF <- px={vs['px']:.3f}  py={vs['py']:.3f}  pz={vs['pz']:.3f}", (0, 220, 0))
        put(16, f"EKF <- yaw={vs['yaw']:+.3f}  src={vs.get('source','?')}", (0, 220, 0))
        put(17, f"zone={vs.get('zone','?')}  tape={vs.get('nearest_tape','?')}", (0, 180, 220))
    else:
        put(15, "EKF feed: waiting for valid line", (0, 80, 220))

    put(18, "L=branch 0  R=branch 1  Q=quit   (tune thresholds: test_sline_tuner.py)",
        (200, 200, 200), 0.55, 1)

    # Mini map is not drawn here because the web dashboard already has a 3D field map.
    return vis


# ---------------------------------------------------------------------------
# PERCEPTION THREAD (headless)
# ---------------------------------------------------------------------------
def perception_thread(params_path: str = '../calibration/camera_params.npz',
                      stream_url: str = None,
                      show_window: bool = False):
    global _start_pose_cache, _pose_seed_cache

    if stream_url is None:
        stream_url = f"http://{service.host}:7123/stream.mjpg"

    vision.setup(params_path, debug=False)
    if not vision.ready:
        print("% live_perception_overlay: vision setup failed")
        return

    aruco_detector.setup(params_path, debug=False)
    localizer.setup()
    vision_pose.setup()
    if not landmark_matcher.setup():
        print("% live_perception_overlay: landmark matcher inactive (no field landmarks loaded)")

    start = localizer.set_start_pose()
    if start:
        service.send(
            "robobot/kalman/cmd",
            f"reset {start['x']:.4f} {start['y']:.4f} {start['z']:.4f} 0 0 {start['yaw']:.4f} 0"
        )
        _start_pose_cache = {
            "x": float(start["x"]),
            "y": float(start["y"]),
            "yaw": float(start["yaw"]),
            "pitch": 0.0,
        }
        _pose_seed_cache = {
            "x": float(start["x"]),
            "y": float(start["y"]),
            "yaw": float(start["yaw"]),
            "pitch": 0.0,
        }
        print(f"% SLocalize: start pose set --> x={start['x']:.3f} y={start['y']:.3f}")

    cap = connect_stream(stream_url)
    if not cap.isOpened():
        print(f"% live_perception_overlay: could not open {stream_url}")
        return

    print("% live_perception_overlay: perception thread running")

    reconnect_attempts = 0

    while not service.stop:
        ret, frame = cap.read()
        if not ret:
            reconnect_attempts += 1
            cap.release()
            time.sleep(min(1.0 * reconnect_attempts, 3.0))
            cap = connect_stream(stream_url)
            if reconnect_attempts >= 5 and not cap.isOpened():
                break
            continue
        reconnect_attempts = 0

        result = vision.process(frame)
        aruco_detections = aruco_detector.process(frame)

        if _kalman_mqtt["x"] is not None:
            px = float(_kalman_mqtt["x"])
            py = float(_kalman_mqtt["y"])
            yaw = float(_kalman_mqtt["yaw"])
            pitch = float(_kalman_mqtt["pitch"])
        else:
            px = float(_pose_seed_cache["x"])
            py = float(_pose_seed_cache["y"])
            yaw = float(_pose_seed_cache["yaw"])
            pitch = float(_pose_seed_cache["pitch"])

        meas = build_measurement(result, (px, py, yaw))
        if meas.get('valid') and meas.get('world_line_points'):
            result = dict(result)
            result['world_line_points'] = meas['world_line_points']

        correction = localizer.update(
            line_result=result,
            aruco_detections=aruco_detections,
            px=px, py=py, yaw=yaw, pitch=pitch
        )

        # Use the predicted state as the vision prior.
        # If it is unavailable, fall back to the post-update Kalman estimate.
        if _kalman_pred_mqtt["x"] is not None:
            px_prior = float(_kalman_pred_mqtt["x"])
            py_prior = float(_kalman_pred_mqtt["y"])
            yaw_prior = float(_kalman_pred_mqtt["yaw"])
        else:
            px_prior, py_prior, yaw_prior = px, py, yaw

        # Try to match the current line signature against known map landmarks.
        # svision_pose uses this only if no ArUco fix is available.
        lm_fix = landmark_matcher.process(
            line_result=result,
            kalman_prior=(px_prior, py_prior, yaw_prior),
        )

        # Build the full vision pose output for this frame.
        vp_out = vision_pose.update(
            aruco_detections=aruco_detections,
            line_result=result,
            px_kalman=px_prior, py_kalman=py_prior, yaw_kalman=yaw_prior,
            pitch_imu=_current_pitch_imu(),
            landmark_fix=lm_fix,
        )

        _log_vision_row(px, py, yaw, vp_out, result, aruco_detections)

        if vp_out['valid']:
            _send_vision_pose(
                vp_out['x'], vp_out['y'], vp_out['z'],
                vp_out['yaw'], vp_out['pitch'],
                source=vp_out.get('source', 'unknown'),
                sigma_xy=vp_out.get('sigma_xy_m'),
                sigma_yaw=vp_out.get('sigma_yaw_rad'),
                n_used=vp_out.get('n_aruco_used'),
                confidence=vp_out.get('aruco_confidence'),
                landmark=vp_out.get('landmark'),
            )
            _pose_seed_cache = {
                "x":     float(vp_out["x"]),
                "y":     float(vp_out["y"]),
                "yaw":   float(vp_out["yaw"]),
                "pitch": float(vp_out["pitch"]),
            }

    cap.release()
    print("% live_perception_overlay: perception thread stopped")


# ---------------------------------------------------------------------------
# MAIN (windowed)
# ---------------------------------------------------------------------------
def main():
    global _start_pose_cache, _pose_seed_cache

    parser = argparse.ArgumentParser(description="Live perception + EKF feed")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=7123)
    parser.add_argument("--path", default="/stream.mjpg")
    parser.add_argument("--params", default="../calibration/camera_params.npz")
    parser.add_argument("--window", default="Live Perception Overlay")
    parser.add_argument("--print-terminal", action="store_true")
    parser.add_argument(
        "--mjpeg-port",
        type=int,
        default=0,
        help="If >0, expose annotated frames as MJPEG at /stream.mjpg on this port",
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="Suppress the OpenCV preview window (use when streaming via --mjpeg-port)",
    )
    args, _ = parser.parse_known_args()

    if args.mjpeg_port > 0:
        _start_mjpeg_server(args.mjpeg_port)

    # Disable the local Kalman instance in this process. The real Kalman state
    # arrives through service callbacks, so this script should not run its own
    # active Kalman loop.
    try:
        from sensors.fusion import skalman as _skalman_mod
        _skalman_mod.kalman.enabled = False
    except (ImportError, AttributeError):
        pass

    service.setup(args.host)

    try:
        _skalman_mod.kalman.enabled = False
    except (NameError, AttributeError):
        pass

    # Register an in-process listener for Kalman state updates.
    # uservice.run() owns the actual MQTT subscription.
    service.add_kalman_state_listener(_on_kalman_state_msg)

    vision.setup(args.params, debug=True)
    if not vision.ready:
        print("% ERROR: vision setup failed")
        _log_file.close()
        sys.exit(1)

    aruco_detector.setup(args.params, debug=False)
    localizer.setup()
    vision_pose.setup()
    if not landmark_matcher.setup():
        print("% live_perception_overlay: landmark matcher inactive (no field landmarks loaded)")

    start = localizer.set_start_pose()
    if start:
        service.send(
            "robobot/kalman/cmd",
            f"reset {start['x']:.4f} {start['y']:.4f} {start['z']:.4f} 0 0 {start['yaw']:.4f} 0"
        )
        _start_pose_cache = {
            "x": float(start["x"]),
            "y": float(start["y"]),
            "yaw": float(start["yaw"]),
            "pitch": 0.0,
        }
        _pose_seed_cache = {
            "x": float(start["x"]),
            "y": float(start["y"]),
            "yaw": float(start["yaw"]),
            "pitch": 0.0,
        }
        print(f"% SLocalize: start pose set --> x={start['x']:.3f} y={start['y']:.3f}")

    stream_url = f"http://{args.host}:{args.port}{args.path}"
    print(f"% Connecting to {stream_url} ...")

    cap = connect_stream(stream_url)
    if not cap.isOpened():
        print("% ERROR: Could not open camera stream.")
        _log_file.close()
        sys.exit(1)

    print("% Stream opened. No drive commands sent.")

    reconnect_attempts = 0
    last_print_t = 0.0

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                reconnect_attempts += 1
                print(f"% Stream lost ({reconnect_attempts}/5), reconnecting...")
                cap.release()
                time.sleep(min(1.0 * reconnect_attempts, 3.0))
                cap = connect_stream(stream_url)
                if reconnect_attempts >= 5 and not cap.isOpened():
                    print("% ERROR: Too many reconnect failures.")
                    break
                continue

            reconnect_attempts = 0

            result = vision.process(frame)
            aruco_detections = aruco_detector.process(frame)

            if _kalman_mqtt["x"] is not None:
                px = float(_kalman_mqtt["x"])
                py = float(_kalman_mqtt["y"])
                yaw = float(_kalman_mqtt["yaw"])
                pitch = float(_kalman_mqtt["pitch"])
            else:
                px = float(_pose_seed_cache["x"])
                py = float(_pose_seed_cache["y"])
                yaw = float(_pose_seed_cache["yaw"])
                pitch = float(_pose_seed_cache["pitch"])

            if args.print_terminal:
                # Print both the post-update pose and the pre-update prior.
                # Large differences here usually explain landmark gate misses
                # or stale-prior behaviour.
                pred_x = _kalman_pred_mqtt['x']
                if pred_x is not None:
                    pred_str = (f"pred=({pred_x:.3f},"
                                f"{_kalman_pred_mqtt['y']:.3f},"
                                f"{_kalman_pred_mqtt['yaw']:.3f})")
                else:
                    pred_str = "pred=None"
                print(f"% pose source: kalman={_kalman_mqtt['x'] is not None} "
                      f"used=({px:.3f},{py:.3f},{yaw:.3f})  {pred_str}")

            meas = build_measurement(result, (px, py, yaw))
            if meas.get('valid') and meas.get('world_line_points'):
                result = dict(result)
                result['world_line_points'] = meas['world_line_points']

            vision_state = {
                "valid": False,
                "px": px,
                "py": py,
                "pz": 0.0,
                "yaw": yaw,
                "pitch": pitch,
                "velocity": 0.0,
                "w": 0.0,
                "source": "",
                "zone": "",
                "nearest_tape": "",
            }

            correction = localizer.update(
                line_result=result,
                aruco_detections=aruco_detections,
                px=px, py=py, yaw=yaw, pitch=pitch
            )

            # Use the predicted state as the vision prior.
            # If it is unavailable, fall back to the post-update estimate.
            if _kalman_pred_mqtt["x"] is not None:
                px_prior = float(_kalman_pred_mqtt["x"])
                py_prior = float(_kalman_pred_mqtt["y"])
                yaw_prior = float(_kalman_pred_mqtt["yaw"])
            else:
                px_prior, py_prior, yaw_prior = px, py, yaw

            lm_fix = landmark_matcher.process(
                line_result=result,
                kalman_prior=(px_prior, py_prior, yaw_prior),
            )

            # Build the full vision pose output for this frame.
            vp_out = vision_pose.update(
                aruco_detections=aruco_detections,
                line_result=result,
                px_kalman=px_prior, py_kalman=py_prior, yaw_kalman=yaw_prior,
                pitch_imu=_current_pitch_imu(),
                landmark_fix=lm_fix,
            )

            _log_vision_row(px, py, yaw, vp_out, result, aruco_detections)

            if vp_out['valid']:
                _send_vision_pose(
                    vp_out['x'], vp_out['y'], vp_out['z'],
                    vp_out['yaw'], vp_out['pitch'],
                    source=vp_out.get('source', 'unknown'),
                    sigma_xy=vp_out.get('sigma_xy_m'),
                    sigma_yaw=vp_out.get('sigma_yaw_rad'),
                    n_used=vp_out.get('n_aruco_used'),
                    confidence=vp_out.get('aruco_confidence'),
                )
                _pose_seed_cache = {
                    "x":     float(vp_out["x"]),
                    "y":     float(vp_out["y"]),
                    "yaw":   float(vp_out["yaw"]),
                    "pitch": float(vp_out["pitch"]),
                }
                if args.print_terminal:
                    dx = vp_out['x'] - px
                    dy = vp_out['y'] - py
                    dyaw = vp_out['yaw'] - yaw
                    print(
                        f"% VP valid src={vp_out['source']} "
                        f"in=({px:.3f},{py:.3f},{yaw:.3f}) "
                        f"out=({vp_out['x']:.3f},{vp_out['y']:.3f},{vp_out['z']:.3f},{vp_out['yaw']:.3f})"
                    )
                    print(f"% DIFF dx={dx:+.4f} dy={dy:+.4f} dyaw={dyaw:+.4f} "
                          f"vel={vp_out['velocity']:+.3f} omega={vp_out['omega']:+.3f}")
            else:
                if args.print_terminal:
                    tape_name = ''
                    ctx_dbg = (correction.get('context') if correction else None) \
                              or localizer.get_context(px, py, yaw)
                    if ctx_dbg and ctx_dbg.get('nearest_tape'):
                        tape_name = ctx_dbg['nearest_tape'].get('name', '')
                    print(
                        f"% NO_VP px={px:.3f} py={py:.3f} yaw={yaw:.3f} "
                        f"zone={ctx_dbg.get('zone','') if ctx_dbg else ''} tape={tape_name}"
                    )

            # The overlay tracks the Kalman pose. Context fields come from
            # localizer/vision so the panel can show zone, tape, and source.
            ctx = (correction.get('context') if correction else None) \
                  or (localizer.get_context(vp_out['x'], vp_out['y'], vp_out['yaw'])
                      if vp_out['valid'] else localizer.get_context(px, py, yaw))
            vision_state = {
                "valid":        True,
                "px":           px,
                "py":           py,
                "pz":           (ctx or {}).get('pz', 0.0),
                "yaw":          yaw,
                "pitch":        pitch,
                "velocity":     vp_out.get('velocity', 0.0),
                "w":            vp_out.get('omega', 0.0),
                "source":       vp_out.get('source', '') if vp_out['valid'] else '',
                "zone":         (ctx or {}).get('zone', ''),
                "nearest_tape": (ctx.get('nearest_tape') or {}).get('name', '')
                                  if ctx and ctx.get('nearest_tape') else '',
            }

            base = vision.debug_frame if vision.debug_frame is not None else frame
            vis = draw_panel(base, result, aruco_detections, vision_state)

            if args.print_terminal:
                now_t = time.time()
                if now_t - last_print_t > 0.5:
                    kalman_src = "kalman" if _kalman_mqtt["x"] is not None else "seed"
                    print(
                        f"% map: px={vision_state['px']:.3f} py={vision_state['py']:.3f} "
                        f"yaw={vision_state['yaw']:.3f} src={kalman_src} "
                        f"zone={vision_state['zone']} vp={vision_state['source'] or 'none'}"
                    )
                    last_print_t = now_t

            if args.mjpeg_port > 0:
                ok_enc, buf = cv2.imencode(
                    ".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 75]
                )
                if ok_enc:
                    global _mjpeg_latest_jpeg
                    with _mjpeg_lock:
                        _mjpeg_latest_jpeg = buf.tobytes()

            # Headless mode skips the OpenCV preview. Keyboard controls are
            # unavailable in this mode, so exit with Ctrl+C.
            if args.no_window:
                time.sleep(0.001)
            else:
                cv2.imshow(args.window, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord("l"):
                    vision.set_active_branch(0)
                elif key == ord("r"):
                    vision.set_active_branch(1)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        _log_file.close()


if __name__ == "__main__":
    main()