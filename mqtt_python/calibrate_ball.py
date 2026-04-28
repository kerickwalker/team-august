#!/usr/bin/env python3

# Offline calibration utility for sball.py (orange golf ball detection).
#
# Two modes:
#   Capture mode  — connect to robot stream, grab a single raw frame, save it,
#                   then drop into calibration mode on that frame.
#   Offline mode  — load a previously saved frame and calibrate immediately.
#
# Uses the same contour-fitting pipeline as sball.py so every parameter
# you tune here maps directly to sball.py with no translation.
#
# Usage:
#   python3 calibrate_ball.py <robot-ip>          # capture + calibrate
#   python3 calibrate_ball.py <path-to-image>     # offline calibration only
#
# Windows:
#   "Source + detection"  — original frame with detected circle drawn on top
#   "Colour mask"         — binary mask after HSV filter + morphology
#   "Controls"            — trackbars
#
# Keys:
#   s  — print final parameter values to copy into sball.py
#   q  — quit

import sys
import os
import cv2 as cv
import numpy as np
from datetime import datetime


WINDOW_SRC  = "Source + detection  (s=save values  q=quit)"
WINDOW_MASK = "Colour mask"
WINDOW_CTRL = "Controls"


def nothing(_):
    pass


def build_controls(h_lo, h_hi, s_lo, s_hi, v_lo, v_hi,
                   min_area, min_radius, max_radius, min_circ_x100):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 420, 380)
    cv.createTrackbar("H low",              WINDOW_CTRL, h_lo,          179, nothing)
    cv.createTrackbar("H high",             WINDOW_CTRL, h_hi,          179, nothing)
    cv.createTrackbar("S low",              WINDOW_CTRL, s_lo,          255, nothing)
    cv.createTrackbar("S high",             WINDOW_CTRL, s_hi,          255, nothing)
    cv.createTrackbar("V low",              WINDOW_CTRL, v_lo,          255, nothing)
    cv.createTrackbar("V high",             WINDOW_CTRL, v_hi,          255, nothing)
    cv.createTrackbar("Min area (px^2)",    WINDOW_CTRL, min_area,     5000, nothing)
    cv.createTrackbar("Min radius (px)",    WINDOW_CTRL, min_radius,    200, nothing)
    cv.createTrackbar("Max radius (px)",    WINDOW_CTRL, max_radius,    400, nothing)
    cv.createTrackbar("Min circ (x100)",    WINDOW_CTRL, min_circ_x100, 100, nothing)


def read_controls():
    h_lo          = cv.getTrackbarPos("H low",           WINDOW_CTRL)
    h_hi          = cv.getTrackbarPos("H high",          WINDOW_CTRL)
    s_lo          = cv.getTrackbarPos("S low",           WINDOW_CTRL)
    s_hi          = cv.getTrackbarPos("S high",          WINDOW_CTRL)
    v_lo          = cv.getTrackbarPos("V low",           WINDOW_CTRL)
    v_hi          = cv.getTrackbarPos("V high",          WINDOW_CTRL)
    min_area      = cv.getTrackbarPos("Min area (px^2)", WINDOW_CTRL)
    min_radius    = cv.getTrackbarPos("Min radius (px)", WINDOW_CTRL)
    max_radius    = max(cv.getTrackbarPos("Max radius (px)", WINDOW_CTRL), min_radius + 1)
    min_circ_x100 = cv.getTrackbarPos("Min circ (x100)", WINDOW_CTRL)
    lower = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
    upper = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
    return lower, upper, min_area, min_radius, max_radius, min_circ_x100 / 100.0


def detect_and_draw(img, lower, upper, min_area, min_radius, max_radius, min_circularity):
    h, w = img.shape[:2]

    hsv  = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    overlay  = img.copy()
    orange   = (0, 100, 255)
    grey     = (160, 160, 160)
    best_cx, best_cy, best_r = 0, 0, 0
    best_circ = 0.0
    found = False

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < min_area:
            continue
        perimeter = cv.arcLength(cnt, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < min_circularity:
            continue
        (cx, cy), r = cv.minEnclosingCircle(cnt)
        r = int(round(r))
        if r < min_radius or r > max_radius:
            continue
        # Draw all passing candidates in grey
        cv.circle(overlay, (int(round(cx)), int(round(cy))), r, grey, 1)
        cv.putText(overlay, f"{circularity:.2f}",
                   (int(round(cx)) + r + 2, int(round(cy))),
                   cv.FONT_HERSHEY_PLAIN, 0.9, grey, 1)
        # Keep the largest (closest) ball
        if r > best_r:
            best_cx, best_cy, best_r = int(round(cx)), int(round(cy)), r
            best_circ = circularity
            found = True

    cv.line(overlay, (w // 2, 0), (w // 2, h), (180, 180, 180), 1)

    if found:
        cv.circle(overlay, (best_cx, best_cy), best_r, orange, 2)
        cv.circle(overlay, (best_cx, best_cy), 3, orange, -1)
        err   = (best_cx - w / 2.0) / (w / 2.0)
        label = f"Ball: r={best_r}px  err={err:+.2f}  circ={best_circ:.2f}"
        cv.putText(overlay, label, (best_cx + best_r + 4, best_cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, 2)
    else:
        cv.putText(overlay, "No ball detected", (10, 30),
                   cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), 2)

    return overlay, mask


def print_values(lower, upper, min_area, min_radius, max_radius, min_circularity):
    print("\n--- Copy these values into sball.py ---")
    print(f"    hsv_lower       = np.array({lower.tolist()}, dtype=np.uint8)")
    print(f"    hsv_upper       = np.array({upper.tolist()}, dtype=np.uint8)")
    print(f"    min_area        = {min_area}")
    print(f"    min_radius      = {min_radius}")
    print(f"    max_radius      = {max_radius}")
    print(f"    min_circularity = {min_circularity}")
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
                             f"calibrate_ball_{ts}.jpg")
    cv.imwrite(save_path, frame)
    print(f"% Captured frame saved to: {save_path}")
    return frame


def calibrate(img):
    h, w = img.shape[:2]
    print(f"% Image resolution: {w}x{h} (no resize — thresholds match sball.py directly)")

    # Seed trackbars with sball.py defaults
    build_controls(
        h_lo=0,   h_hi=20,
        s_lo=150, s_hi=255,
        v_lo=100, v_hi=255,
        min_area=50,
        min_radius=8,
        max_radius=80,
        min_circ_x100=55,
    )

    print("Adjust trackbars until the ball shows as a clean white blob in the mask window.")
    print("  Step 1 — tune H low/high to match the orange hue of the ball.")
    print("  Step 2 — raise S low to exclude low-saturation background.")
    print("  Step 3 — adjust Min area / Min radius to filter noise blobs.")
    print("  s = print values  |  q = quit")

    while True:
        lower, upper, min_area, min_radius, max_radius, min_circ = read_controls()
        overlay, mask = detect_and_draw(img, lower, upper,
                                        min_area, min_radius, max_radius, min_circ)

        cv.imshow(WINDOW_SRC,  overlay)
        cv.imshow(WINDOW_MASK, mask)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(lower, upper, min_area, min_radius, max_radius, min_circ)

    cv.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 calibrate_ball.py <robot-ip>      # capture frame then calibrate")
        print("  python3 calibrate_ball.py <image-file>    # calibrate from saved image")
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
