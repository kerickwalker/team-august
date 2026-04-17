#!/usr/bin/env python3
"""
live_perception_overlay.py
=============================================================================
Live perception pipeline - line extraction + ArUco + EKF feed.
Field map aware: position correction is performed via slocalize.py.
Mini map is displayed in the bottom-right corner.
=============================================================================
"""

from __future__ import annotations

import argparse
import sys
import time
import math
import json as _json
from datetime import datetime

import cv2
import numpy as np

from sline import vision
from saruco import aruco_detector
from smeasurement import build_measurement
from spose import pose
from skalman import kalman
from uservice import service
from slocalize import localizer


# ---------------------------------------------------------------------------
# CSV LOG
# ---------------------------------------------------------------------------
_log_file = open("vision_kalman_log.csv", "a", encoding="utf-8")

if _log_file.tell() == 0:
    _log_file.write(
        "t_epoch,t_str,"
        "prior_x,prior_y,prior_yaw,"
        "vision_x,vision_y,vision_z,vision_yaw,vision_src,"
        "line_valid,aruco_n\n"
    )
    _log_file.flush()


def _log_vision_row(px, py, yaw, correction, result, aruco_detections):
    t_epoch = time.time()
    t_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    line_valid = bool(result.get("line_valid", False))
    aruco_n = len(aruco_detections)

    if correction and correction.get("valid", False):
        _log_file.write(
            f"{t_epoch:.3f},{t_str},"
            f"{px:.3f},{py:.3f},{yaw:.3f},"
            f"{correction['x']:.3f},{correction['y']:.3f},{correction['z']:.3f},{correction['yaw']:.3f},"
            f"{correction['source']},"
            f"{line_valid},{aruco_n}\n"
        )
    else:
        _log_file.write(
            f"{t_epoch:.3f},{t_str},"
            f"{px:.3f},{py:.3f},{yaw:.3f},"
            f"nan,nan,nan,nan,"
            f"none,"
            f"{line_valid},{aruco_n}\n"
        )

    _log_file.flush()


# ---------------------------------------------------------------------------
# MQTT Kalman state cache (this script runs as a separate process - cannot read directly)
# ---------------------------------------------------------------------------
_kalman_mqtt: dict = {"x": None, "y": None, "yaw": None, "pitch": None}

# Initial pose cache
_start_pose_cache = {"x": 4.775, "y": 0.235, "yaw": 1.5708, "pitch": 0.0}

# Last reliable pose used when Kalman is unavailable or not yet settled
_pose_seed_cache = {"x": 4.775, "y": 0.235, "yaw": 1.5708, "pitch": 0.0}


def _on_kalman_state_msg(client, userdata, msg):
    global _kalman_mqtt
    try:
        d = _json.loads(msg.payload.decode())

        # Format 1: SKalman.publish_state()
        if "position" in d and "orientation" in d:
            _kalman_mqtt["x"] = float(d["position"]["x"])
            _kalman_mqtt["y"] = float(d["position"]["y"])
            _kalman_mqtt["yaw"] = float(d["orientation"]["yaw"])
            _kalman_mqtt["pitch"] = float(d["orientation"]["pitch"])
            return

        # Format 2: UService.publish_kalman_state()
        if "x" in d and isinstance(d["x"], dict):
            xs = d["x"]
            _kalman_mqtt["x"] = float(xs["x"])
            _kalman_mqtt["y"] = float(xs["y"])
            _kalman_mqtt["yaw"] = float(xs["yaw"])
            _kalman_mqtt["pitch"] = float(xs["pitch"])
            return

        print(f"% kalman state: unknown format keys={list(d.keys())}")

    except Exception as e:
        print(f"% kalman state parse error: {e}")


def connect_stream(stream_url: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def _send_vision_pose(x, y, z, yaw, pitch):
    msg = f"{time.time():.6f} {x:.4f} {y:.4f} {z:.4f} {yaw:.4f} {pitch:.4f}"
    service.clientOut.publish("robobot/drive/T0/vision_pose", msg)


# ---------------------------------------------------------------------------
# MINI MAP
# ---------------------------------------------------------------------------
_FIELD_W = 7.0
_FIELD_H = 6.0


def draw_mini_map(frame: np.ndarray, vision_state: dict,
                  map_size: int = 220) -> np.ndarray:
    """
    Draws a mini top-down map in the bottom-right corner.
    Shows robot position, nearest tape, and nearest gate.
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
        from field_map_2026 import FIELD

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

    put(18, "L=B0  R=B1  V/B=brightness  N/M=saturation  Q=quit", (200, 200, 200), 0.55, 1)

    vis = draw_mini_map(vis, vision_state)
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

        _log_vision_row(px, py, yaw, correction, result, aruco_detections)

        if correction['valid']:
            _send_vision_pose(
                correction['x'], correction['y'], correction['z'],
                correction['yaw'], correction['pitch'],
            )
            _pose_seed_cache = {
                "x": float(correction["x"]),
                "y": float(correction["y"]),
                "yaw": float(correction["yaw"]),
                "pitch": float(correction["pitch"]),
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
    args, _ = parser.parse_known_args()

    import skalman, sgpio, srobot, sir, spose, simu, scam, sedge
    skalman.kalman.enabled = False
    service.setup(args.host)
    skalman.kalman.enabled = False

    service.client.subscribe("robobot/kalman/state")
    service.client.message_callback_add("robobot/kalman/state", _on_kalman_state_msg)

    vision.setup(args.params, debug=True)
    if not vision.ready:
        print("% ERROR: vision setup failed")
        _log_file.close()
        sys.exit(1)

    aruco_detector.setup(args.params, debug=False)
    localizer.setup()

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
                print(f"% pose source: kalman={_kalman_mqtt['x'] is not None} "
                      f"used=({px:.3f},{py:.3f},{yaw:.3f})")

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

            _log_vision_row(px, py, yaw, correction, result, aruco_detections)

            if correction['valid']:
                _send_vision_pose(
                    correction['x'], correction['y'], correction['z'],
                    correction['yaw'], correction['pitch'],
                )

                _pose_seed_cache = {
                    "x": float(correction["x"]),
                    "y": float(correction["y"]),
                    "yaw": float(correction["yaw"]),
                    "pitch": float(correction["pitch"]),
                }

                ctx = correction['context']
                vision_state = {
                    "valid": True,
                    "px": correction['x'],
                    "py": correction['y'],
                    "pz": correction['z'],
                    "yaw": correction['yaw'],
                    "pitch": correction['pitch'],
                    "velocity": pose.velocity() if pose.poseCnt > 0 else 0.0,
                    "w": pose.turnrate() if pose.poseCnt > 0 else 0.0,
                    "source": correction['source'],
                    "zone": ctx.get('zone', ''),
                    "nearest_tape": ctx['nearest_tape']['name'] if ctx.get('nearest_tape') else '',
                }

                if args.print_terminal:
                    dx = correction['x'] - px
                    dy = correction['y'] - py
                    dyaw = correction['yaw'] - yaw
                    print(
                        f"% CORR valid src={correction['source']} "
                        f"in=({px:.3f},{py:.3f},{yaw:.3f}) "
                        f"out=({correction['x']:.3f},{correction['y']:.3f},{correction['z']:.3f},{correction['yaw']:.3f})"
                    )
                    print(f"% DIFF dx={dx:+.4f} dy={dy:+.4f} dyaw={dyaw:+.4f}")

            else:
                vision_state["valid"] = True
                ctx = correction.get('context') or localizer.get_context(px, py, yaw)
                if ctx:
                    vision_state["zone"] = ctx.get('zone', '')
                    if ctx.get('nearest_tape'):
                        vision_state["nearest_tape"] = ctx['nearest_tape'].get('name', '')

                if args.print_terminal:
                    tape_name = ''
                    if ctx and ctx.get('nearest_tape'):
                        tape_name = ctx['nearest_tape'].get('name', '')
                    print(
                        f"% NO_CORR px={px:.3f} py={py:.3f} yaw={yaw:.3f} "
                        f"zone={ctx.get('zone','') if ctx else ''} tape={tape_name}"
                    )

            base = vision.debug_frame if vision.debug_frame is not None else frame
            vis = draw_panel(base, result, aruco_detections, vision_state)

            if args.print_terminal:
                now_t = time.time()
                if now_t - last_print_t > 0.5:
                    print(
                        f"% vision_state: px={vision_state['px']:.3f} "
                        f"py={vision_state['py']:.3f} zone={vision_state['zone']} "
                        f"src={vision_state['source']}"
                    )
                    last_print_t = now_t

            cv2.imshow(args.window, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            elif key == ord("l"):
                vision.set_active_branch(0)
            elif key == ord("r"):
                vision.set_active_branch(1)
            elif key == ord("v"):
                vision.update_threshold(white_v_min=min(255, vision.white_v_min + 5))
            elif key == ord("b"):
                vision.update_threshold(white_v_min=max(0, vision.white_v_min - 5))
            elif key == ord("n"):
                vision.update_threshold(white_s_max=min(255, vision.white_s_max + 5))
            elif key == ord("m"):
                vision.update_threshold(white_s_max=max(0, vision.white_s_max - 5))

    finally:
        cap.release()
        cv2.destroyAllWindows()
        _log_file.close()


if __name__ == "__main__":
    main()