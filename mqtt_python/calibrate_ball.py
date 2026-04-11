#!/usr/bin/env python3

# Offline calibration utility for sball.py (orange golf ball detection).
#
# Two modes:
#   Capture mode  — connect to robot stream, grab a single raw frame, save it,
#                   then drop into calibration mode on that frame.
#   Offline mode  — load a previously saved frame and calibrate immediately.
#
# Usage:
#   python3 calibrate_ball.py <robot-ip>          # capture + calibrate
#   python3 calibrate_ball.py <path-to-image>     # offline calibration only
#
# Windows:
#   "Source + detection"  — original frame with detected circle drawn on top
#   "Colour mask"         — binary mask after HSV filter (white = kept)
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
                   dp_x10, min_dist, param1, param2, min_radius, max_radius):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 420, 420)
    # HSV thresholds
    cv.createTrackbar("H low",             WINDOW_CTRL, h_lo,      179, nothing)
    cv.createTrackbar("H high",            WINDOW_CTRL, h_hi,      179, nothing)
    cv.createTrackbar("S low",             WINDOW_CTRL, s_lo,      255, nothing)
    cv.createTrackbar("S high",            WINDOW_CTRL, s_hi,      255, nothing)
    cv.createTrackbar("V low",             WINDOW_CTRL, v_lo,      255, nothing)
    cv.createTrackbar("V high",            WINDOW_CTRL, v_hi,      255, nothing)
    # Hough parameters
    cv.createTrackbar("Hough dp (x10)",    WINDOW_CTRL, dp_x10,    30,  nothing)
    cv.createTrackbar("Min dist (px)",     WINDOW_CTRL, min_dist,  300, nothing)
    cv.createTrackbar("Param1 (Canny hi)", WINDOW_CTRL, param1,    300, nothing)
    cv.createTrackbar("Param2 (accum)",    WINDOW_CTRL, param2,    100, nothing)
    cv.createTrackbar("Min radius (px)",   WINDOW_CTRL, min_radius, 200, nothing)
    cv.createTrackbar("Max radius (px)",   WINDOW_CTRL, max_radius, 400, nothing)


def read_controls():
    h_lo       = cv.getTrackbarPos("H low",             WINDOW_CTRL)
    h_hi       = cv.getTrackbarPos("H high",            WINDOW_CTRL)
    s_lo       = cv.getTrackbarPos("S low",             WINDOW_CTRL)
    s_hi       = cv.getTrackbarPos("S high",            WINDOW_CTRL)
    v_lo       = cv.getTrackbarPos("V low",             WINDOW_CTRL)
    v_hi       = cv.getTrackbarPos("V high",            WINDOW_CTRL)
    dp_x10     = max(cv.getTrackbarPos("Hough dp (x10)",    WINDOW_CTRL), 1)
    min_dist   = max(cv.getTrackbarPos("Min dist (px)",     WINDOW_CTRL), 1)
    param1     = max(cv.getTrackbarPos("Param1 (Canny hi)", WINDOW_CTRL), 1)
    param2     = max(cv.getTrackbarPos("Param2 (accum)",    WINDOW_CTRL), 1)
    min_radius = cv.getTrackbarPos("Min radius (px)",   WINDOW_CTRL)
    max_radius = max(cv.getTrackbarPos("Max radius (px)",   WINDOW_CTRL), min_radius + 1)
    lower = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
    upper = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
    return lower, upper, dp_x10 / 10.0, min_dist, param1, param2, min_radius, max_radius


def detect_and_draw(img, lower, upper, dp, min_dist, param1, param2,
                    min_radius, max_radius):
    hsv  = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

    blurred = cv.GaussianBlur(mask, (9, 9), 2)

    overlay = img.copy()
    h, w = img.shape[:2]

    circles = cv.HoughCircles(
        blurred,
        cv.HOUGH_GRADIENT,
        dp=dp,
        minDist=min_dist,
        param1=param1,
        param2=param2,
        minRadius=min_radius,
        maxRadius=max_radius,
    )

    orange = (0, 100, 255)
    if circles is not None:
        circles = np.round(circles[0]).astype(int)
        for i, (cx, cy, r) in enumerate(circles):
            color = orange if i == 0 else (80, 80, 180)
            cv.circle(overlay, (cx, cy), r, color, 2)
            cv.circle(overlay, (cx, cy), 3, color, -1)
        cx, cy, r = circles[0]
        label = f"Ball: r={r}px  err={((cx - w/2) / (w/2)):+.2f}"
        cv.putText(overlay, label, (cx + r + 4, cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, 2)
    else:
        cv.putText(overlay, "No ball detected", (10, 30),
                   cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), 2)

    cv.line(overlay, (w // 2, 0), (w // 2, h), (180, 180, 180), 1)

    return overlay, mask


def print_values(lower, upper, dp, min_dist, param1, param2, min_radius, max_radius):
    print("\n--- Copy these values into sball.py ---")
    print(f"    hsv_lower        = np.array({lower.tolist()}, dtype=np.uint8)")
    print(f"    hsv_upper        = np.array({upper.tolist()}, dtype=np.uint8)")
    print(f"    hough_dp         = {dp}")
    print(f"    hough_min_dist   = {min_dist}")
    print(f"    hough_param1     = {param1}")
    print(f"    hough_param2     = {param2}")
    print(f"    hough_min_radius = {min_radius}")
    print(f"    hough_max_radius = {max_radius}")
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

    ts = datetime.now().strftime('%Y_%b_%d_%H%M%S')
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             f"calibrate_ball_{ts}.jpg")
    cv.imwrite(save_path, frame)
    print(f"% Captured frame saved to: {save_path}")
    return frame


def calibrate(img):
    max_dim = 900
    h, w = img.shape[:2]
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        img = cv.resize(img, (int(w * scale), int(h * scale)))

    # Seed with sball.py defaults — bright orange (lower hue, high saturation)
    build_controls(
        h_lo=0,   h_hi=20,
        s_lo=150, s_hi=255,
        v_lo=100, v_hi=255,
        dp_x10=12, min_dist=50, param1=80, param2=18,
        min_radius=8, max_radius=80,
    )

    print("Step 1: Adjust H/S/V sliders until only the ball is white in the mask window.")
    print("Step 2: Adjust Hough sliders until a single circle locks onto the ball.")
    print("  Tip: lower Param2 to detect more circles, raise to reduce false positives.")
    print("  s = print values to terminal")
    print("  q = quit")

    while True:
        lower, upper, dp, min_dist, param1, param2, min_radius, max_radius = read_controls()
        overlay, mask = detect_and_draw(img, lower, upper, dp, min_dist,
                                        param1, param2, min_radius, max_radius)

        cv.imshow(WINDOW_SRC,  overlay)
        cv.imshow(WINDOW_MASK, mask)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(lower, upper, dp, min_dist, param1, param2,
                         min_radius, max_radius)

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
