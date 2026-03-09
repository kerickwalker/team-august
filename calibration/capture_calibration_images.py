"""
capture_calibration_images.py
==============================
Grabs calibration images from the robot camera stream.

Usage:
    python3 calibration/capture_calibration_images.py --host 10.197.218.17

Controls:
    SPACE  -> save current frame (if sharp enough)
    D      -> toggle blur overlay
    Q/ESC  -> quit

Output: calibration/images/calib_YYYYMMDD_HHMMSS_NNN.jpg
"""

import cv2
import numpy as np
import os
import argparse
import time
from datetime import datetime

# ─── CHECKERBOARD CONFIG ──────────────────────────────────────────────────────
BOARD_W = 7
BOARD_H = 10

# Sharpness threshold- frames below this are rejected as blurry
BLUR_THRESHOLD = 80.0

TARGET_COUNT = 30
# ─────────────────────────────────────────────────────────────────────────────


def laplacian_variance(gray: np.ndarray) -> float:
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def draw_overlay(frame: np.ndarray, sharpness: float, count: int,
                 show_blur: bool, corners_found: bool) -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]

    # sharpness bar on the left edge
    bar_h = int(min(sharpness / 200.0, 1.0) * (h - 40))
    bar_color = (0, 220, 0) if sharpness >= BLUR_THRESHOLD else (0, 60, 220)
    cv2.rectangle(vis, (8, h - 20 - bar_h), (22, h - 20), bar_color, -1)
    cv2.rectangle(vis, (8, 20), (22, h - 20), (180, 180, 180), 1)

    # Status text
    sharp_label = f"Sharp: {sharpness:.0f}" if sharpness >= BLUR_THRESHOLD \
                  else f"BLURRY ({sharpness:.0f})"
    sharp_color = (0, 220, 0) if sharpness >= BLUR_THRESHOLD else (0, 60, 220)
    cv2.putText(vis, sharp_label, (30, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, sharp_color, 2)
    cv2.putText(vis, f"Saved: {count}/{TARGET_COUNT}", (30, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # Corner detection indicator
    if corners_found:
        cv2.putText(vis, "CORNERS OK  [SPACE to save]", (30, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    else:
        cv2.putText(vis, "No corners detected", (30, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)

    if show_blur:
        lap = cv2.Laplacian(frame, cv2.CV_64F)
        lap_norm = cv2.normalize(np.abs(lap), None, 0, 255,
                                 cv2.NORM_MINMAX).astype(np.uint8)
        lap_color = cv2.applyColorMap(lap_norm, cv2.COLORMAP_JET)
        vis = cv2.addWeighted(vis, 0.6, lap_color, 0.4, 0)

    return vis


def main():
    parser = argparse.ArgumentParser(description="Capture calibration images from robot camera")
    parser.add_argument("--host", default="localhost",
                        help="Robot IP address (default: localhost)")
    parser.add_argument("--port", type=int, default=7123,
                        help="Camera stream port (default: 7123)")
    parser.add_argument("--out", default="calibration/images",
                        help="Output directory for captured images")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    stream_url = f"http://{args.host}:{args.port}/stream.mjpg"
    print(f"% Connecting to {stream_url} ...")
    cap = cv2.VideoCapture(stream_url)

    if not cap.isOpened():
        print("couldn't open stream, is the robot on?")
        return

     # always grab the latest frame, not a stale one
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"Board: {BOARD_W}x{BOARD_H}  |  Target: {TARGET_COUNT} images")
    print("SPACE = save  D = blur overlay  Q = quit")
    print(f"% Target: {TARGET_COUNT} images  |  Blur threshold: {BLUR_THRESHOLD}")

    board_size = (BOARD_W, BOARD_H)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    count = 0
    show_blur = False
    last_saved_frame_idx = -10  # prevent accidental double-saves
    reconnect_attempts = 0
    MAX_RECONNECT = 5  # give up after this many consecutive failures

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            reconnect_attempts += 1
            print(f"% Stream lost (attempt {reconnect_attempts}/{MAX_RECONNECT}) — reconnecting...")
            cap.release()
            if reconnect_attempts >= MAX_RECONNECT:
                print("% Too many reconnect failures, giving up")
                break
            time.sleep(1.0 * reconnect_attempts) 
            cap = cv2.VideoCapture(stream_url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                print(f"% Reconnect attempt {reconnect_attempts} failed — stream not reachable.")
            continue
        reconnect_attempts = 0  # reset on successful read

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharpness = laplacian_variance(gray)

        cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                    cv2.CALIB_CB_NORMALIZE_IMAGE)
        corners_found, corners = cv2.findChessboardCorners(
            gray, board_size, cb_flags
        )

        vis = draw_overlay(frame, sharpness, count, show_blur, corners_found)

        if corners_found:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(vis, board_size, corners_refined, True)

        cv2.imshow("Calibration Capture", vis)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):  # Q or ESC
            break

        elif key == ord('d'):
            show_blur = not show_blur

        elif key == ord(' ') and frame_idx != last_saved_frame_idx:
            if sharpness < BLUR_THRESHOLD:
                print(f"% Rejected (too blurry: {sharpness:.1f} < {BLUR_THRESHOLD})")
            elif not corners_found:
                print("% Rejected (no corners found - adjust angle/distance)")
            else:
                # Include microseconds to prevent filename collision if SPACE is
                # pressed faster than 1-second resolution (e.g. burst captures).
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                fname = os.path.join(args.out, f"calib_{ts}_{count:03d}.jpg")

                cv2.imwrite(fname, frame)
                count += 1
                last_saved_frame_idx = frame_idx

                print(f"% Saved [{count:02d}/{TARGET_COUNT}] → {fname}  "
                    f"(sharpness={sharpness:.1f})")

                if count >= TARGET_COUNT:
                    print("% Reached target image count — you can stop or continue.")

        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"% Done. Captured {count} images in '{args.out}'")
    if count < 15:
        print("% WARNING: < 15 images. Calibration quality may be poor.")
        print("%   Recommended: 20–40 images from varied angles and distances.")


if __name__ == "__main__":
    main()