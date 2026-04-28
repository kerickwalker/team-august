#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2025 by DTU
#*   jcan@dtu.dk
#*
#* The MIT License (MIT)  https://mit-license.org/
#***************************************************************************/

# Golf hole detection using Canny edge detection + ellipse fitting.
# No colour filtering — robust to lighting changes and white-balance shifts.
# Tune parameters with calibrate_hole.py.

import cv2 as cv
import numpy as np
from datetime import datetime


class SHole:

    # --- Edge detection parameters ---
    # Tune with calibrate_hole.py; copy printed values here.
    blur_size        = 5      # Gaussian blur kernel size (must be odd)
    canny_lo         = 30     # Canny lower threshold
    canny_hi         = 80     # Canny upper threshold

    # --- Contour size filter ---
    min_area         = 500    # px²
    max_area         = 40000  # px²

    # --- Ellipse shape filter ---
    max_aspect_ratio = 4.0    # major/minor axis ratio; allows elongated ellipses from angled view
    min_circularity  = 0.15   # 4π·area/perimeter² — rejects very non-ellipse-like contours

    # --- Ground ROI ---
    roi_fraction     = 0.5    # search only the bottom fraction of the frame

    # --- Detection persistence ---
    persistence_frames = 5    # keep last known position for this many consecutive misses

    # --- Detection results (updated each detect() call) ---
    detected      = False
    cx            = 0         # pixel x of hole centre (full-frame coordinates)
    cy            = 0         # pixel y of hole centre (full-frame coordinates)
    radius        = 0         # average semi-axis length in px
    axes          = (0, 0)    # (semi-minor, semi-major) in px
    angle         = 0.0       # ellipse rotation angle in degrees
    detectedTime  = datetime.now()
    detCnt        = 0

    # --- Internal ---
    _img_w        = 0
    _img_h        = 0
    _miss_count   = 0

    ##########################################################

    def setup(self):
        print("% SHole:: ready (Canny edge pipeline)")
        print(f"%         blur_size={self.blur_size}  "
              f"canny_lo={self.canny_lo}  canny_hi={self.canny_hi}")
        print(f"%         min_area={self.min_area}  max_area={self.max_area}")
        print(f"%         max_aspect_ratio={self.max_aspect_ratio}  "
              f"min_circularity={self.min_circularity}  "
              f"roi_fraction={self.roi_fraction}")

    ##########################################################

    def detect(self, img):
        """Detect a golf hole in a BGR image using Canny edge detection and
        ellipse fitting. Updates detected, cx, cy, radius, axes, angle.
        Returns True if a hole was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h

        # 1. Restrict to ground ROI
        roi_y = int(h * (1.0 - self.roi_fraction))
        roi   = img[roi_y:, :]

        # 2. Edge map: grayscale → blur → Canny → close gaps in ellipse boundary
        gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (self.blur_size, self.blur_size), 0)
        edges   = cv.Canny(blurred, self.canny_lo, self.canny_hi)
        kernel  = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        closed  = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel, iterations=2)

        # 3. Find contours of closed edge regions
        contours, _ = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 4. Filter by area, circularity, and ellipse shape.
        #    Score by circularity so the most ellipse-like blob wins.
        best_score   = -1.0
        best_ellipse = None

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            if len(cnt) < 5:
                continue

            perimeter = cv.arcLength(cnt, closed=True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue

            ellipse = cv.fitEllipse(cnt)
            (ex, ey), (d1, d2), ang = ellipse
            minor_r = min(d1, d2) / 2.0
            major_r = max(d1, d2) / 2.0

            if minor_r == 0:
                continue
            if (major_r / minor_r) > self.max_aspect_ratio:
                continue

            if circularity > best_score:
                best_score   = circularity
                best_ellipse = ellipse

        if best_ellipse is not None:
            (ex, ey), (d1, d2), ang = best_ellipse
            minor_r = min(d1, d2) / 2.0
            major_r = max(d1, d2) / 2.0
            self.cx           = int(ex)
            self.cy           = int(ey) + roi_y
            self.axes         = (int(minor_r), int(major_r))
            self.radius       = int((minor_r + major_r) / 2.0)
            self.angle        = ang
            self.detected     = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
            self._miss_count  = 0
        else:
            self._miss_count += 1
            if self._miss_count >= self.persistence_frames:
                self.detected = False
            # else: keep detected=True with last known cx/cy/radius

        return self.detected

    ##########################################################

    def steeringError(self):
        """Horizontal offset of the hole centre from the image centre,
        normalised to [-1, 1]. Negative = hole is left, positive = hole is right.
        Returns 0 if no hole detected or image width unknown."""
        if not self.detected or self._img_w == 0:
            return 0.0
        return (self.cx - self._img_w / 2.0) / (self._img_w / 2.0)

    ##########################################################

    def isClose(self, radius_threshold=60):
        """Returns True when the hole's average apparent radius exceeds
        radius_threshold px, indicating the robot is near the hole."""
        return self.detected and self.radius >= radius_threshold

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        grey  = (160, 160, 160)
        roi_y = int(self._img_h * (1.0 - self.roi_fraction))
        cv.line(img, (0, roi_y), (self._img_w, roi_y), (60, 60, 60), 1)

        if not self.detected:
            cv.putText(img, "Hole: not found", (10, 70),
                       cv.FONT_HERSHEY_PLAIN, 1.4, grey, thickness=2)
            return

        cv.ellipse(img,
                   (self.cx, self.cy),
                   self.axes,
                   self.angle,
                   0, 360,
                   grey, thickness=2)
        cv.circle(img, (self.cx, self.cy), 3, grey, thickness=-1)
        cv.line(img, (self._img_w // 2, 0), (self._img_w // 2, self._img_h),
                (200, 200, 200), thickness=1)
        err   = self.steeringError()
        label = f"Hole: r={self.radius}px  err={err:+.2f}"
        cv.putText(img, label, (self.cx + self.axes[1] + 4, self.cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, grey, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SHole:: terminated")


# singleton instance
hole = SHole()
