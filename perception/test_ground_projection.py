#!/usr/bin/env python3
"""
test_ground_projection.py
=============================================================================
Live visual test for sground.py.

Usage:
    python3 vision/test_ground_projection.py --host 10.197.216.254

Controls:
    +  / -       tilt  +/- 0.5°
    W  / S       height +/- 1 cm
    A  / D       roll  +/- 0.5°
    R            reset all to camera_extrinsics.py values
    P            print current extrinsics (copy-paste into camera_extrinsics.py)
    Q / ESC      quit

    * Left click   project that exact pixel -> prints X,Y to terminal + shows on screen
"""

import cv2
import numpy as np
import argparse
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from perception.sground import ground

GRID_POINTS = [
    (0.5,  1.0,  "near-C"),
    (0.5,  0.75, "mid-C"),
    (0.5,  0.5,  "far-C"),
    (0.25, 1.0,  "near-L"),
    (0.75, 1.0,  "near-R"),
    (0.25, 0.75, "mid-L"),
    (0.75, 0.75, "mid-R"),
]

# bgr
COL_OK      = (0,   220,   0)
COL_FAIL    = (0,    60, 220)
COL_OVERLAY = (200, 200,   0)
COL_GRID    = (120, 120, 120)
COL_CLICK   = (0,  200, 255)   # mouse click
# -----------------------------------------------------------------------------

# Shared state for mouse callback
_click_state = {"u": None, "v": None, "X": None, "Y": None, "ok": False}


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        ok, X, Y = ground.pixel_to_ground(x, y)
        _click_state.update({"u": x, "v": y, "X": X, "Y": Y, "ok": ok})
        if ok:
            print(f"% CLICK  pixel=({x},{y})  ->  X={X:.3f}m  Y={Y:+.3f}m  "
                  f"dist={np.hypot(X,Y):.3f}m")
        else:
            print(f"% CLICK  pixel=({x},{y})  ->  [no ground intersection]")


def draw_overlay(frame, grid_results):
    vis = frame.copy()
    h, w = vis.shape[:2]

    # Approximate horizon line
    cv2.line(vis, (0, h//2), (w, h//2), COL_GRID, 1)
    cv2.putText(vis, "horizon (approx)", (4, h//2 - 6),
                cv2.FONT_HERSHEY_PLAIN, 0.9, COL_GRID, 1)

    # Fixed grid points
    for u, v, ok, X, Y, label in grid_results:
        u_i, v_i = int(u), int(v)
        color = COL_OK if ok else COL_FAIL
        cv2.drawMarker(vis, (u_i, v_i), color,
                       markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
        txt = (f"{label} X={X:.2f}m Y={Y:+.2f}m") if ok else f"{label} [no hit]"
        tx = u_i + 10 if u_i + 190 < w else u_i - 190
        cv2.putText(vis, txt, (tx, v_i - 6),
                    cv2.FONT_HERSHEY_PLAIN, 0.95, color, 1)

    # Mouse click result
    if _click_state["u"] is not None:
        cu, cv_ = _click_state["u"], _click_state["v"]
        cv2.drawMarker(vis, (cu, cv_), COL_CLICK,
                       markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
        if _click_state["ok"]:
            ctxt = (f"CLICK X={_click_state['X']:.3f}m  "
                    f"Y={_click_state['Y']:+.3f}m")
        else:
            ctxt = "CLICK [no ground intersection]"
        cv2.putText(vis, ctxt, (cu + 12, cv_ - 8),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, COL_CLICK, 1)

    # HUD
    hud = [
        f"tilt  : {ground.camera_tilt:.1f} deg   ( +/- )",
        f"height: {ground.camera_height*100:.1f} cm    ( W/S )",
        f"roll  : {ground.camera_roll:.1f} deg   ( A/D )  <- fix if centre-Y drifts",
        f"R = reset   P = print   Q = quit   click = measure pixel",
    ]
    for i, line in enumerate(hud):
        cv2.putText(vis, line, (8, 20 + i * 18),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, COL_OVERLAY, 1)

    return vis


def project_grid(img_w, img_h):
    results = []
    for col_f, row_f, label in GRID_POINTS:
        u = col_f * img_w
        v = row_f * (img_h - 1)
        ok, X, Y = ground.pixel_to_ground(u, v)
        results.append((u, v, ok, X, Y, label))
    return results


def print_extrinsics():
    print("\n-- Current extrinsics (copy into camera_extrinsics.py) --")
    print(f"CAMERA_HEIGHT_M   = {ground.camera_height:.4f}")
    print(f"CAMERA_TILT_DEG   = {ground.camera_tilt:.2f}")
    print(f"CAMERA_ROLL_DEG   = {ground.camera_roll:.2f}")
    print(f"CAMERA_X_OFFSET_M = {ground.x_offset:.4f}")
    print(f"CAMERA_Y_OFFSET_M = {ground.y_offset:.4f}")
    print("-" * 52)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host",   default="localhost")
    parser.add_argument("--port",   type=int, default=7123)
    parser.add_argument("--params", default="calibration/camera_params.npz")
    args = parser.parse_args()

    ground.setup(args.params)
    if not ground.ready:
        print("% Setup failed")
        sys.exit(1)

    default_tilt   = ground.camera_tilt
    default_height = ground.camera_height
    default_roll   = ground.camera_roll

    stream_url = f"http://{args.host}:{args.port}/stream.mjpg"
    print(f"% Connecting to {stream_url} ...")
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"% ERROR: Could not open stream at {stream_url}")
        sys.exit(1)

    print("% Stream opened.  Click anywhere on the image to measure that pixel.")
    print_extrinsics()

    cv2.namedWindow("Ground Projection Test")
    cv2.setMouseCallback("Ground Projection Test", mouse_callback)

    reconnect_attempts = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            reconnect_attempts += 1
            print(f"% Stream lost ({reconnect_attempts}/5) reconnecting...")
            cap.release()
            if reconnect_attempts >= 5:
                print("% Too many failures.")
                break
            time.sleep(1.0 * reconnect_attempts)
            cap = cv2.VideoCapture(stream_url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            continue
        reconnect_attempts = 0

        img_h, img_w = frame.shape[:2]
        results = project_grid(img_w, img_h)
        vis = draw_overlay(frame, results)

        cv2.imshow("Ground Projection Test", vis)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key in (ord('+'), ord('=')):
            ground.update_extrinsics(camera_tilt=ground.camera_tilt + 0.5)
        elif key == ord('-'):
            ground.update_extrinsics(camera_tilt=max(0.0, ground.camera_tilt - 0.5))
        elif key == ord('w'):
            ground.update_extrinsics(camera_height=ground.camera_height + 0.01)
        elif key == ord('s'):
            ground.update_extrinsics(camera_height=max(0.01, ground.camera_height - 0.01))
        elif key == ord('a'):
            ground.update_extrinsics(camera_roll=ground.camera_roll - 0.5)
        elif key == ord('d'):
            ground.update_extrinsics(camera_roll=ground.camera_roll + 0.5)
        elif key == ord('r'):
            ground.update_extrinsics(camera_tilt=default_tilt,
                                     camera_height=default_height,
                                     camera_roll=default_roll)
            print("% Reset to defaults.")
        elif key == ord('p'):
            print_extrinsics()

    cap.release()
    cv2.destroyAllWindows()
    print("% Test finished.")
    print_extrinsics()


if __name__ == "__main__":
    main()