#!/usr/bin/env python3

# Offline calibration utility for sball.py (white golf ball detection).
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
#   "Brightness mask"     — binary mask after threshold (white = kept)
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
WINDOW_MASK = "Brightness mask"
WINDOW_CTRL = "Controls"


def nothing(_):
    pass


def build_controls(brightness, dp_x10, min_dist, param1, param2,
                   min_radius, max_radius):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 420, 340)
    cv.createTrackbar("Brightness thresh", WINDOW_CTRL, brightness, 255, nothing)
    # dp stored as dp*10 so we get one decimal place via integer trackbar
    cv.createTrackbar("Hough dp (x10)",   WINDOW_CTRL, dp_x10,     30,  nothing)
    cv.createTrackbar("Min dist (px)",     WINDOW_CTRL, min_dist,   300, nothing)
    cv.createTrackbar("Param1 (Canny hi)", WINDOW_CTRL, param1,     300, nothing)
    cv.createTrackbar("Param2 (accum)",    WINDOW_CTRL, param2,     100, nothing)
    cv.createTrackbar("Min radius (px)",   WINDOW_CTRL, min_radius, 200, nothing)
    cv.createTrackbar("Max radius (px)",   WINDOW_CTRL, max_radius, 400, nothing)


def read_controls():
    brightness  = cv.getTrackbarPos("Brightness thresh", WINDOW_CTRL)
    dp_x10      = max(cv.getTrackbarPos("Hough dp (x10)",   WINDOW_CTRL), 1)
    min_dist    = max(cv.getTrackbarPos("Min dist (px)",     WINDOW_CTRL), 1)
    param1      = max(cv.getTrackbarPos("Param1 (Canny hi)", WINDOW_CTRL), 1)
    param2      = max(cv.getTrackbarPos("Param2 (accum)",    WINDOW_CTRL), 1)
    min_radius  = cv.getTrackbarPos("Min radius (px)",   WINDOW_CTRL)
    max_radius  = max(cv.getTrackbarPos("Max radius (px)",   WINDOW_CTRL), min_radius + 1)
    return brightness, dp_x10 / 10.0, min_dist, param1, param2, min_radius, max_radius


def detect_and_draw(img, brightness, dp, min_dist, param1, param2,
                    min_radius, max_radius):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, mask = cv.threshold(gray, brightness, 255, cv.THRESH_BINARY)
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

    if circles is not None:
        circles = np.round(circles[0]).astype(int)
        # Draw all candidates in dim white, best (first) in bright white
        for i, (cx, cy, r) in enumerate(circles):
            color = (255, 255, 255) if i == 0 else (120, 120, 120)
            cv.circle(overlay, (cx, cy), r,  color, 2)
            cv.circle(overlay, (cx, cy), 3,  color, -1)
        cx, cy, r = circles[0]
        label = f"Ball: r={r}px  err={((cx - w/2) / (w/2)):+.2f}"
        cv.putText(overlay, label, (cx + r + 4, cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
    else:
        cv.putText(overlay, "No ball detected", (10, 30),
                   cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), 2)

    # Centre reference line
    cv.line(overlay, (w // 2, 0), (w // 2, h), (180, 180, 180), 1)

    return overlay, mask


def print_values(brightness, dp, min_dist, param1, param2, min_radius, max_radius):
    print("\n--- Copy these values into sball.py ---")
    print(f"    brightness_threshold = {brightness}")
    print(f"    hough_dp             = {dp}")
    print(f"    hough_min_dist       = {min_dist}")
    print(f"    hough_param1         = {param1}")
    print(f"    hough_param2         = {param2}")
    print(f"    hough_min_radius     = {min_radius}")
    print(f"    hough_max_radius     = {max_radius}")
    print("---------------------------------------\n")


def capture_frame(host):
    """Connect to the robot stream, grab one frame, save it, return the image."""
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
    # Resize large images so they fit on screen comfortably
    max_dim = 900
    h, w = img.shape[:2]
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        img = cv.resize(img, (int(w * scale), int(h * scale)))

    # Seed trackbars with sball.py defaults
    build_controls(
        brightness=180,
        dp_x10=12,       # 1.2 * 10
        min_dist=50,
        param1=80,
        param2=18,
        min_radius=8,
        max_radius=80,
    )

    print("Adjust trackbars until the ball is circled in the source window.")
    print("  Tip: lower Param2 (accum) to detect more circles, raise to reduce false positives.")
    print("  s = print values to terminal")
    print("  q = quit")

    while True:
        brightness, dp, min_dist, param1, param2, min_radius, max_radius = read_controls()
        overlay, mask = detect_and_draw(img, brightness, dp, min_dist,
                                        param1, param2, min_radius, max_radius)

        cv.imshow(WINDOW_SRC,  overlay)
        cv.imshow(WINDOW_MASK, mask)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(brightness, dp, min_dist, param1, param2,
                         min_radius, max_radius)

    cv.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 calibrate_ball.py <robot-ip>      # capture frame then calibrate")
        print("  python3 calibrate_ball.py <image-file>    # calibrate from saved image")
        sys.exit(1)

    arg = sys.argv[1]

    # Decide: IP address (capture mode) or file path (offline mode)
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
