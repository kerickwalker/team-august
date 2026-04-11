#!/usr/bin/env python3

# Offline calibration utility for shole.py (golf hole detection).
#
# Two modes:
#   Capture mode  — connect to robot stream, grab a single raw frame, save it,
#                   then drop into calibration mode on that frame.
#   Offline mode  — load a previously saved frame and calibrate immediately.
#
# Usage:
#   python3 calibrate_hole.py <robot-ip>          # capture + calibrate
#   python3 calibrate_hole.py <path-to-image>     # offline calibration only
#
# Windows:
#   "Source + detection"  — original frame with fitted ellipse drawn on top
#   "CLAHE + mask"        — contrast-enhanced image and dark-region mask side by side
#   "Controls"            — trackbars
#
# Keys:
#   s  — print final parameter values to copy into shole.py
#   q  — quit

import sys
import os
import cv2 as cv
import numpy as np
from datetime import datetime


WINDOW_SRC  = "Source + detection  (s=save values  q=quit)"
WINDOW_MASK = "CLAHE enhanced  |  Dark mask"
WINDOW_CTRL = "Controls"


def nothing(_):
    pass


def build_controls(clahe_clip_x10, clahe_tile, dark_thresh,
                   min_area, max_area, max_aspect_x10, roi_pct):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 440, 380)
    cv.createTrackbar("CLAHE clip (x10)",   WINDOW_CTRL, clahe_clip_x10, 100,  nothing)
    cv.createTrackbar("CLAHE tile",         WINDOW_CTRL, clahe_tile,     32,   nothing)
    cv.createTrackbar("Dark threshold",     WINDOW_CTRL, dark_thresh,    255,  nothing)
    cv.createTrackbar("Min area (px^2)",    WINDOW_CTRL, min_area,       5000, nothing)
    cv.createTrackbar("Max area (x100px^2)",WINDOW_CTRL, max_area,       1000, nothing)
    cv.createTrackbar("Max aspect (x10)",   WINDOW_CTRL, max_aspect_x10, 100,  nothing)
    cv.createTrackbar("ROI % from bottom",  WINDOW_CTRL, roi_pct,        100,  nothing)


def read_controls():
    clip_x10      = max(cv.getTrackbarPos("CLAHE clip (x10)",    WINDOW_CTRL), 1)
    tile          = max(cv.getTrackbarPos("CLAHE tile",          WINDOW_CTRL), 2)
    dark_thresh   = cv.getTrackbarPos("Dark threshold",          WINDOW_CTRL)
    min_area      = cv.getTrackbarPos("Min area (px^2)",         WINDOW_CTRL)
    max_area_x100 = max(cv.getTrackbarPos("Max area (x100px^2)", WINDOW_CTRL), 1)
    aspect_x10    = max(cv.getTrackbarPos("Max aspect (x10)",    WINDOW_CTRL), 10)
    roi_pct       = max(cv.getTrackbarPos("ROI % from bottom",   WINDOW_CTRL), 1)
    return (clip_x10 / 10.0, tile, dark_thresh,
            min_area, max_area_x100 * 100,
            aspect_x10 / 10.0, roi_pct / 100.0)


def detect_and_draw(img, clahe_clip, clahe_tile, dark_thresh,
                    min_area, max_area, max_aspect, roi_fraction):
    h, w = img.shape[:2]
    roi_y = int(h * (1.0 - roi_fraction))
    roi   = img[roi_y:, :]

    # CLAHE contrast enhancement
    gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    clahe   = cv.createCLAHE(clipLimit=clahe_clip,
                               tileGridSize=(clahe_tile, clahe_tile))
    enhanced = clahe.apply(gray)
    blurred  = cv.GaussianBlur(enhanced, (5, 5), 1)

    # Dark region mask
    _, mask = cv.threshold(blurred, dark_thresh, 255, cv.THRESH_BINARY_INV)
    kernel  = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask    = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
    mask    = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

    # Contour detection + ellipse fitting
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    overlay      = img.copy()
    best_area    = 0
    best_ellipse = None
    grey         = (160, 160, 160)

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        if len(cnt) < 5:
            continue
        ellipse = cv.fitEllipse(cnt)
        (ex, ey), (d1, d2), ang = ellipse
        minor_r = min(d1, d2) / 2.0
        major_r = max(d1, d2) / 2.0
        if minor_r == 0 or (major_r / minor_r) > max_aspect:
            continue
        # Draw all passing candidates in dim grey
        center = (int(ex), int(ey) + roi_y)
        axes   = (int(minor_r), int(major_r))
        cv.ellipse(overlay, center, axes, ang, 0, 360, (80, 80, 80), 1)
        if area > best_area:
            best_area    = area
            best_ellipse = (ellipse, roi_y)

    # Draw ROI boundary
    cv.line(overlay, (0, roi_y), (w, roi_y), (60, 60, 60), 1)
    cv.line(overlay, (w // 2, 0), (w // 2, h), (180, 180, 180), 1)

    if best_ellipse is not None:
        ellipse, offset = best_ellipse
        (ex, ey), (d1, d2), ang = ellipse
        minor_r = int(min(d1, d2) / 2.0)
        major_r = int(max(d1, d2) / 2.0)
        cx = int(ex)
        cy = int(ey) + offset
        cv.ellipse(overlay, (cx, cy), (minor_r, major_r), ang, 0, 360, grey, 2)
        cv.circle(overlay, (cx, cy), 3, grey, -1)
        avg_r = (minor_r + major_r) // 2
        err   = (cx - w / 2.0) / (w / 2.0)
        label = f"Hole: r={avg_r}px  err={err:+.2f}  aspect={major_r/max(minor_r,1):.1f}"
        cv.putText(overlay, label, (cx + major_r + 4, cy),
                   cv.FONT_HERSHEY_PLAIN, 1.1, grey, 2)
    else:
        cv.putText(overlay, "No hole detected", (10, 50),
                   cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), 2)

    # Side-by-side debug view: CLAHE enhanced | mask (pad to full-frame height)
    pad_top   = roi_y
    enhanced3 = cv.cvtColor(enhanced, cv.COLOR_GRAY2BGR)
    mask3     = cv.cvtColor(mask,     cv.COLOR_GRAY2BGR)
    black_top = np.zeros((pad_top, w, 3), dtype=np.uint8)
    debug     = np.hstack([
        np.vstack([black_top, enhanced3]),
        np.vstack([black_top, mask3]),
    ])

    return overlay, debug


def print_values(clahe_clip, clahe_tile, dark_thresh,
                 min_area, max_area, max_aspect, roi_fraction):
    print("\n--- Copy these values into shole.py ---")
    print(f"    clahe_clip       = {clahe_clip}")
    print(f"    clahe_tile       = {clahe_tile}")
    print(f"    dark_threshold   = {dark_thresh}")
    print(f"    min_area         = {min_area}")
    print(f"    max_area         = {max_area}")
    print(f"    max_aspect_ratio = {max_aspect}")
    print(f"    roi_fraction     = {roi_fraction}")
    print("---------------------------------------\n")


def capture_frame(host):
    url = f"http://{host}:7123/stream.mjpg"
    print(f"% Connecting to {url} ...")
    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% Error: could not open stream at {url}")
        sys.exit(1)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("% Error: failed to read frame from stream")
        sys.exit(1)
    ts        = datetime.now().strftime('%Y_%b_%d_%H%M%S')
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             f"calibrate_hole_{ts}.jpg")
    cv.imwrite(save_path, frame)
    print(f"% Captured frame saved to: {save_path}")
    return frame


def calibrate(img):
    max_dim = 900
    h, w = img.shape[:2]
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        img   = cv.resize(img, (int(w * scale), int(h * scale)))

    # Seed trackbars with shole.py defaults
    build_controls(
        clahe_clip_x10=30,   # 3.0
        clahe_tile=8,
        dark_thresh=80,
        min_area=400,
        max_area=400,        # 400 * 100 = 40000 px²
        max_aspect_x10=40,   # 4.0
        roi_pct=60,
    )

    print("Adjust trackbars until the hole is clearly outlined in the source window.")
    print("  Step 1 — use CLAHE clip + Dark threshold until the hole shows as a white")
    print("           blob in the mask panel (right half of the debug window).")
    print("  Step 2 — use Min/Max area to filter out noise blobs.")
    print("  Step 3 — use Max aspect to reject elongated false positives.")
    print("  s = print values  |  q = quit")

    while True:
        (clahe_clip, clahe_tile, dark_thresh,
         min_area, max_area, max_aspect, roi_frac) = read_controls()

        overlay, debug = detect_and_draw(img, clahe_clip, clahe_tile, dark_thresh,
                                         min_area, max_area, max_aspect, roi_frac)

        cv.imshow(WINDOW_SRC,  overlay)
        cv.imshow(WINDOW_MASK, debug)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(clahe_clip, clahe_tile, dark_thresh,
                         min_area, max_area, max_aspect, roi_frac)

    cv.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 calibrate_hole.py <robot-ip>      # capture frame then calibrate")
        print("  python3 calibrate_hole.py <image-file>    # calibrate from saved image")
        sys.exit(1)

    arg = sys.argv[1]
    if os.path.isfile(arg):
        img = cv.imread(arg)
        if img is None:
            print(f"% Error: could not read '{arg}'")
            sys.exit(1)
        print(f"% Loaded image: {arg}")
    else:
        img = capture_frame(arg)

    calibrate(img)


if __name__ == "__main__":
    main()
