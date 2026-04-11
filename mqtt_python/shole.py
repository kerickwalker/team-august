#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2025 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the "Software"), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

import cv2 as cv
import numpy as np
from datetime import datetime


class SHole:

    # --- CLAHE (local contrast enhancement) ---
    # Amplifies the subtle brightness difference between the hole and the track
    # before thresholding. Higher clip_limit = more aggressive enhancement.
    clahe_clip      = 3.0    # contrast limit per tile (1.0 = off, 2-4 typical)
    clahe_tile      = 8      # tile grid size (NxN pixels per tile)

    # --- Dark region threshold ---
    # After CLAHE, pixels below this value are treated as the hole interior.
    # Tune with calibrate_hole.py — depends on track brightness after CLAHE.
    dark_threshold  = 80     # 0-255

    # --- Contour size filter ---
    # Reject blobs that are clearly too small (noise) or too large (shadows,
    # track edges). Units are pixels^2 of contour area.
    min_area        = 400    # px²
    max_area        = 40000  # px²

    # --- Ellipse shape filter ---
    # Rejects elongated shapes that can't plausibly be a round hole in perspective.
    # Ratio of major to minor axis; 1.0 = perfect circle, higher = more elongated.
    max_aspect_ratio = 3.0

    # --- Circularity filter ---
    # 4π·area/perimeter² — 1.0 = perfect circle, lower = more irregular.
    # Drops sharply for jagged/non-convex shapes even if their bounding ellipse
    # looks reasonable. Raise this to be more strict (0.6–0.8 is a useful range).
    min_circularity  = 0.55

    # --- Ground ROI ---
    # Only search the bottom fraction of the frame where the ground is visible.
    roi_fraction    = 0.6    # bottom 60% of the frame

    # --- Detection results (updated each detect() call) ---
    detected      = False
    cx            = 0        # pixel x of hole centre (full-frame coordinates)
    cy            = 0        # pixel y of hole centre (full-frame coordinates)
    radius        = 0        # average semi-axis length in px (for isClose())
    axes          = (0, 0)   # (semi-minor, semi-major) in px
    angle         = 0.0      # ellipse rotation angle in degrees
    detectedTime  = datetime.now()
    detCnt        = 0

    # --- Internal ---
    _img_w        = 0
    _img_h        = 0

    ##########################################################

    def setup(self):
        print("% SHole:: ready")
        print(f"%         clahe_clip={self.clahe_clip}  clahe_tile={self.clahe_tile}")
        print(f"%         dark_threshold={self.dark_threshold}")
        print(f"%         min_area={self.min_area}  max_area={self.max_area}")
        print(f"%         max_aspect_ratio={self.max_aspect_ratio}  "
              f"min_circularity={self.min_circularity}  "
              f"roi_fraction={self.roi_fraction}")

    ##########################################################

    def detect(self, img):
        """Detect a golf hole in a BGR image using CLAHE contrast enhancement,
        dark-region thresholding, contour detection, and ellipse fitting.
        The hole appears as a slightly darker oval on the track surface.
        Updates detected, cx, cy, radius, axes, angle.
        Returns True if a hole was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h

        # 1. Restrict to ground ROI — hole is always in the lower part of the frame
        roi_y = int(h * (1.0 - self.roi_fraction))
        roi   = img[roi_y:, :]

        # 2. Grayscale + CLAHE to amplify local contrast between hole and track
        gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        clahe   = cv.createCLAHE(clipLimit=self.clahe_clip,
                                  tileGridSize=(self.clahe_tile, self.clahe_tile))
        enhanced = clahe.apply(gray)

        # 3. Blur to suppress noise before thresholding
        blurred = cv.GaussianBlur(enhanced, (5, 5), 1)

        # 4. Isolate dark regions (the hole interior is darker than the track)
        _, mask = cv.threshold(blurred, self.dark_threshold, 255, cv.THRESH_BINARY_INV)

        # 5. Morphological cleanup — close gaps in the hole rim, remove specks
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask   = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
        mask   = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

        # 6. Find contours of dark blobs
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 7. Filter contours by area, circularity, and ellipse shape.
        #    Score by circularity so the most circle-like blob wins.
        best_score   = -1.0
        best_ellipse = None

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            if len(cnt) < 5:          # fitEllipse requires at least 5 points
                continue

            perimeter = cv.arcLength(cnt, closed=True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue            # too irregular — not a round hole

            ellipse = cv.fitEllipse(cnt)
            (ex, ey), (d1, d2), ang = ellipse
            minor_r = min(d1, d2) / 2.0
            major_r = max(d1, d2) / 2.0

            if minor_r == 0:
                continue
            if (major_r / minor_r) > self.max_aspect_ratio:
                continue            # too elongated — not a round hole

            if circularity > best_score:
                best_score   = circularity
                best_ellipse = ellipse

        if best_ellipse is not None:
            (ex, ey), (d1, d2), ang = best_ellipse
            minor_r = min(d1, d2) / 2.0
            major_r = max(d1, d2) / 2.0
            self.cx           = int(ex)
            self.cy           = int(ey) + roi_y   # convert back to full-frame coords
            self.axes         = (int(minor_r), int(major_r))
            self.radius       = int((minor_r + major_r) / 2.0)
            self.angle        = ang
            self.detected     = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
        else:
            self.detected = False

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
        grey = (160, 160, 160)

        # Draw the ROI boundary so it's clear what area is being searched
        roi_y = int(self._img_h * (1.0 - self.roi_fraction))
        cv.line(img, (0, roi_y), (self._img_w, roi_y), (60, 60, 60), 1)

        if not self.detected:
            cv.putText(img, "Hole: not found", (10, 70),
                       cv.FONT_HERSHEY_PLAIN, 1.4, grey, thickness=2)
            return

        # Draw the fitted ellipse
        cv.ellipse(img,
                   (self.cx, self.cy),
                   self.axes,          # (semi-minor, semi-major)
                   self.angle,
                   0, 360,
                   grey, thickness=2)
        # Centre dot
        cv.circle(img, (self.cx, self.cy), 3, grey, thickness=-1)
        # Image centre reference line
        cv.line(img, (self._img_w // 2, 0), (self._img_w // 2, self._img_h),
                (200, 200, 200), thickness=1)
        # Status text
        err = self.steeringError()
        label = f"Hole: r={self.radius}px  err={err:+.2f}"
        cv.putText(img, label, (self.cx + self.axes[1] + 4, self.cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, grey, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SHole:: terminated")


# singleton instance — mirrors pattern used by sball, sgate, svline, etc.
hole = SHole()
