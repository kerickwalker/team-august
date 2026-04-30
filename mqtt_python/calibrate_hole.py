#!/usr/bin/env python3

# Offline calibration utility for shole.py (golf hole detection).
# Uses Canny edge detection + ellipse fitting — no colour filtering,
# so results are robust across lighting changes and white-balance shifts.
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
#   "Canny edges"         — edge image used for detection
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shole import hole as _hole


WINDOW_SRC   = "Source + detection  (s=save values  q=quit)"
WINDOW_EDGES = "Canny edges"
WINDOW_CTRL  = "Controls"


def nothing(_):
    pass


def build_controls(blur, canny_lo, canny_hi, min_area, max_area,
                   max_aspect_x10, min_circ_x100, roi_pct):
    cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_CTRL, 440, 460)
    cv.createTrackbar("Blur (odd)",          WINDOW_CTRL, blur,           15,   nothing)
    cv.createTrackbar("Canny low",           WINDOW_CTRL, canny_lo,       300,  nothing)
    cv.createTrackbar("Canny high",          WINDOW_CTRL, canny_hi,       300,  nothing)
    cv.createTrackbar("Min area (px^2)",     WINDOW_CTRL, min_area,       5000, nothing)
    cv.createTrackbar("Max area (x100px^2)", WINDOW_CTRL, max_area,       1000, nothing)
    cv.createTrackbar("Max aspect (x10)",    WINDOW_CTRL, max_aspect_x10, 100,  nothing)
    cv.createTrackbar("Min circ (x100)",     WINDOW_CTRL, min_circ_x100,  100,  nothing)
    cv.createTrackbar("ROI % from bottom",   WINDOW_CTRL, roi_pct,        100,  nothing)


def read_controls():
    blur_raw      = cv.getTrackbarPos("Blur (odd)",          WINDOW_CTRL)
    blur          = max(blur_raw | 1, 1)          # force odd, minimum 1
    canny_lo      = cv.getTrackbarPos("Canny low",           WINDOW_CTRL)
    canny_hi      = cv.getTrackbarPos("Canny high",          WINDOW_CTRL)
    min_area      = cv.getTrackbarPos("Min area (px^2)",     WINDOW_CTRL)
    max_area_x100 = max(cv.getTrackbarPos("Max area (x100px^2)", WINDOW_CTRL), 1)
    aspect_x10    = max(cv.getTrackbarPos("Max aspect (x10)",    WINDOW_CTRL), 10)
    circ_x100     = cv.getTrackbarPos("Min circ (x100)",    WINDOW_CTRL)
    roi_pct       = max(cv.getTrackbarPos("ROI % from bottom", WINDOW_CTRL), 1)
    return (blur, canny_lo, canny_hi, min_area, max_area_x100 * 100,
            aspect_x10 / 10.0, circ_x100 / 100.0, roi_pct / 100.0)


def detect_and_draw(img, blur, canny_lo, canny_hi, min_area, max_area,
                    max_aspect, min_circularity, roi_fraction):
    h, w = img.shape[:2]
    roi_y = int(h * (1.0 - roi_fraction))
    roi   = img[roi_y:, :]

    # Edge map: grayscale → blur → Canny → close gaps in the ellipse boundary
    gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (blur, blur), 0)
    edges   = cv.Canny(blurred, canny_lo, canny_hi)
    kernel  = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    closed  = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    overlay      = img.copy()
    best_score   = -1.0
    best_ellipse = None
    grey         = (160, 160, 160)

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        if len(cnt) < 5:
            continue
        perimeter = cv.arcLength(cnt, closed=True)
        if perimeter == 0:
            continue
        circularity = (4 * np.pi * area) / (perimeter * perimeter)
        if circularity < min_circularity:
            continue
        ellipse = cv.fitEllipse(cnt)
        (ex, ey), (d1, d2), ang = ellipse
        minor_r = min(d1, d2) / 2.0
        major_r = max(d1, d2) / 2.0
        if minor_r == 0 or (major_r / minor_r) > max_aspect:
            continue
        center = (int(ex), int(ey) + roi_y)
        axes   = (int(minor_r), int(major_r))
        # Draw all passing candidates in dim grey
        cv.ellipse(overlay, center, axes, ang, 0, 360, (80, 80, 80), 1)
        cv.putText(overlay, f"{circularity:.2f}", (center[0] + int(major_r) + 2, center[1]),
                   cv.FONT_HERSHEY_PLAIN, 0.9, (80, 80, 80), 1)
        if circularity > best_score:
            best_score   = circularity
            best_ellipse = (ellipse, roi_y)

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
        label = (f"Hole: r={avg_r}px  err={err:+.2f}  "
                 f"aspect={major_r/max(minor_r,1):.1f}  circ={best_score:.2f}")
        cv.putText(overlay, label, (cx + major_r + 4, cy),
                   cv.FONT_HERSHEY_PLAIN, 1.1, grey, 2)
    else:
        cv.putText(overlay, "No hole detected", (10, 50),
                   cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), 2)

    # Pad Canny edges back to full frame height for display
    pad_top = roi_y
    edges3  = cv.cvtColor(closed, cv.COLOR_GRAY2BGR)
    black   = np.zeros((pad_top, w, 3), dtype=np.uint8)
    debug   = np.vstack([black, edges3])

    return overlay, debug, best_ellipse is not None


def print_values(blur, canny_lo, canny_hi, min_area, max_area,
                 max_aspect, min_circularity, roi_fraction):
    print("\n--- Copy these values into shole.py ---")
    print(f"    blur_size        = {blur}")
    print(f"    canny_lo         = {canny_lo}")
    print(f"    canny_hi         = {canny_hi}")
    print(f"    min_area         = {min_area}")
    print(f"    max_area         = {max_area}")
    print(f"    max_aspect_ratio = {max_aspect}")
    print(f"    min_circularity  = {min_circularity}")
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
    h, w = img.shape[:2]
    print(f"% Image resolution: {w}x{h} (no resize — thresholds match shole.py directly)")

    # Seed trackbars from the current values in shole.py
    build_controls(
        blur=_hole.blur_size,
        canny_lo=_hole.canny_lo,
        canny_hi=_hole.canny_hi,
        min_area=_hole.min_area,
        max_area=_hole.max_area // 100,
        max_aspect_x10=int(_hole.max_aspect_ratio * 10),
        min_circ_x100=int(_hole.min_circularity * 100),
        roi_pct=int(_hole.roi_fraction * 100),
    )

    print("Canny edges window shows what the detector sees.")
    print("  Step 1 — raise Canny high until the hole boundary appears as a clear arc.")
    print("  Step 2 — tune Canny low (usually ~1/3 of high) to fill in weak edges.")
    print("  Step 3 — adjust Blur to smooth noise without erasing the hole edge.")
    print("  Step 4 — tune Min/Max area to isolate the hole blob size.")
    print("  Step 5 — raise Max aspect if the ellipse is being rejected as too elongated.")
    print("  Step 6 — raise Min circ to reject non-ellipse-shaped noise, if needed.")
    print("  s = print values  |  q = quit")

    while True:
        (blur, canny_lo, canny_hi,
         min_area, max_area,
         max_aspect, min_circ, roi_frac) = read_controls()

        overlay, debug, _ = detect_and_draw(img, blur, canny_lo, canny_hi,
                                            min_area, max_area,
                                            max_aspect, min_circ, roi_frac)

        cv.imshow(WINDOW_SRC,   overlay)
        cv.imshow(WINDOW_EDGES, debug)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(blur, canny_lo, canny_hi, min_area, max_area,
                         max_aspect, min_circ, roi_frac)

    cv.destroyAllWindows()


def load_images_from_dir(dirpath):
    exts = ('.jpg', '.jpeg', '.png')
    paths = sorted(
        p for p in (os.path.join(dirpath, f) for f in os.listdir(dirpath))
        if os.path.isfile(p) and p.lower().endswith(exts)
    )
    images = []
    for p in paths:
        img = cv.imread(p)
        if img is not None:
            images.append((os.path.basename(p), img))
    print(f"% Loaded {len(images)} images from {dirpath}")
    return images


def calibrate_batch(named_images):
    THUMB_W = 160
    THUMB_H = 120
    BORDER  = 3
    COLS    = 8
    n       = len(named_images)
    ROWS    = (n + COLS - 1) // COLS
    GRID_W  = COLS * THUMB_W
    GRID_H  = ROWS * THUMB_H

    WINDOW_GRID = "Batch  (green=detected  red=missed  s=save values  q=quit)"
    cv.namedWindow(WINDOW_GRID, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_GRID, GRID_W, GRID_H)

    build_controls(
        blur=_hole.blur_size,
        canny_lo=_hole.canny_lo,
        canny_hi=_hole.canny_hi,
        min_area=_hole.min_area,
        max_area=_hole.max_area // 100,
        max_aspect_x10=int(_hole.max_aspect_ratio * 10),
        min_circ_x100=int(_hole.min_circularity * 100),
        roi_pct=int(_hole.roi_fraction * 100),
    )

    print(f"% Batch mode: {n} images")
    print("  s = print values  |  q = quit")

    while True:
        (blur, canny_lo, canny_hi,
         min_area, max_area,
         max_aspect, min_circ, roi_frac) = read_controls()

        grid      = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
        n_detected = 0

        for i, (name, img) in enumerate(named_images):
            overlay, _, detected = detect_and_draw(
                img, blur, canny_lo, canny_hi,
                min_area, max_area, max_aspect, min_circ, roi_frac)

            thumb = cv.resize(overlay, (THUMB_W, THUMB_H))
            color = (0, 200, 0) if detected else (0, 0, 200)
            cv.rectangle(thumb, (0, 0), (THUMB_W - 1, THUMB_H - 1), color, BORDER)

            # Filename label in bottom-left of thumbnail
            cv.putText(thumb, name[:18], (2, THUMB_H - 4),
                       cv.FONT_HERSHEY_PLAIN, 0.6, color, 1)

            row, col = divmod(i, COLS)
            y1, x1 = row * THUMB_H, col * THUMB_W
            grid[y1:y1 + THUMB_H, x1:x1 + THUMB_W] = thumb

            if detected:
                n_detected += 1

        cv.setWindowTitle(WINDOW_GRID,
                          f"Batch  {n_detected}/{n} detected  "
                          f"(green=detected  red=missed  s=save  q=quit)")
        cv.imshow(WINDOW_GRID, grid)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print_values(blur, canny_lo, canny_hi, min_area, max_area,
                         max_aspect, min_circ, roi_frac)

    cv.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 calibrate_hole.py <robot-ip>      # capture frame then calibrate")
        print("  python3 calibrate_hole.py <image-file>    # calibrate from saved image")
        print("  python3 calibrate_hole.py <directory>     # batch calibrate across all images")
        sys.exit(1)

    arg = sys.argv[1]
    if os.path.isdir(arg):
        named_images = load_images_from_dir(arg)
        if not named_images:
            print(f"% Error: no images found in '{arg}'")
            sys.exit(1)
        calibrate_batch(named_images)
    elif os.path.isfile(arg):
        img = cv.imread(arg)
        if img is None:
            print(f"% Error: could not read '{arg}'")
            sys.exit(1)
        print(f"% Loaded image: {arg}")
        calibrate(img)
    else:
        calibrate(capture_frame(arg))


if __name__ == "__main__":
    main()
