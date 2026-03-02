#!/usr/bin/env python3

# Offline HSV threshold calibration utility for sgate.py.
#
# Usage:
#   python3 calibrate_hsv.py <image_file>
#
# Use trackbars to isolate the gate colour in the mask window, then press:
#   s  — print the final HSV values to copy into sgate.py
#   q  — quit

import sys
import cv2 as cv
import numpy as np


WINDOW_SRC  = "Source image  (q=quit  s=save)"
WINDOW_MASK = "Colour mask"
WINDOW_CTRL = "HSV controls"


def nothing(_):
    pass


def build_controls(h_lo, h_hi, s_lo, s_hi, v_lo, v_hi):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 400, 300)
    cv.createTrackbar("H low",  WINDOW_CTRL, h_lo, 179, nothing)
    cv.createTrackbar("H high", WINDOW_CTRL, h_hi, 179, nothing)
    cv.createTrackbar("S low",  WINDOW_CTRL, s_lo, 255, nothing)
    cv.createTrackbar("S high", WINDOW_CTRL, s_hi, 255, nothing)
    cv.createTrackbar("V low",  WINDOW_CTRL, v_lo, 255, nothing)
    cv.createTrackbar("V high", WINDOW_CTRL, v_hi, 255, nothing)


def read_controls():
    h_lo = cv.getTrackbarPos("H low",  WINDOW_CTRL)
    h_hi = cv.getTrackbarPos("H high", WINDOW_CTRL)
    s_lo = cv.getTrackbarPos("S low",  WINDOW_CTRL)
    s_hi = cv.getTrackbarPos("S high", WINDOW_CTRL)
    v_lo = cv.getTrackbarPos("V low",  WINDOW_CTRL)
    v_hi = cv.getTrackbarPos("V high", WINDOW_CTRL)
    return (np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
            np.array([h_hi, s_hi, v_hi], dtype=np.uint8))


def print_values(lower, upper):
    print("\n--- Copy these values into sgate.py ---")
    print(f"    hsv_lower = np.array({lower.tolist()}, dtype=np.uint8)")
    print(f"    hsv_upper = np.array({upper.tolist()}, dtype=np.uint8)")
    print("---------------------------------------\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 calibrate_hsv.py <image_file>")
        sys.exit(1)

    img = cv.imread(sys.argv[1])
    if img is None:
        print(f"Error: could not read '{sys.argv[1]}'")
        sys.exit(1)

    # Resize large images so they fit on screen comfortably
    max_dim = 800
    h, w = img.shape[:2]
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        img = cv.resize(img, (int(w * scale), int(h * scale)))

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Seed trackbars with sgate.py defaults for orange
    build_controls(h_lo=3, h_hi=30, s_lo=120, s_hi=255, v_lo=80, v_hi=255)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

    print("Adjust trackbars until only the gate is white in the mask window.")
    print("  s = print values to terminal")
    print("  q = quit")

    while True:
        lower, upper = read_controls()

        mask = cv.inRange(hsv, lower, upper)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

        # Colour the masked region on a copy of the source for easy comparison
        overlay = img.copy()
        overlay[mask == 0] = (overlay[mask == 0] * 0.3).astype(np.uint8)

        cv.imshow(WINDOW_SRC,  overlay)
        cv.imshow(WINDOW_MASK, mask)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(lower, upper)

    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
