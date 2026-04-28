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


class SBall:

    # --- HSV colour filter for isolating the orange ball ---
    # Bright orange sits at lower hue values (closer to red) with high saturation.
    # Tune with calibrate_ball.py if the ball shade differs from these defaults.
    hsv_lower = np.array([0,  150, 100], dtype=np.uint8)   # H, S, V
    hsv_upper       = np.array([57, 255, 255], dtype=np.uint8)

    # --- Contour filter parameters ---
    min_radius      = 8     # px — ignore blobs smaller than this
    max_radius      = 80    # px — ignore blobs larger than this
    min_area        = 50    # px² — minimum contour area before fitting a circle
    min_circularity = 0.55  # 4π·area/perimeter² — rejects non-round blobs (1.0 = perfect circle)

    # --- Detection results (updated each detect() call) ---
    detected      = False
    cx            = 0           # pixel x of fitted circle centre (may be below frame when clipped)
    cy            = 0           # pixel y of fitted circle centre
    radius        = 0           # pixel radius of fitted circle
    clip_fraction = 0.0         # fraction of circle below the bottom of the frame (0–1)
    detectedTime  = datetime.now()
    detCnt        = 0           # total frames where a ball was found

    # --- Internal ---
    _img_w        = 0
    _img_h        = 0

    ##########################################################

    def setup(self):
        print("% SBall:: ready")
        print(f"%         hsv_lower = {self.hsv_lower.tolist()}  "
              f"hsv_upper = {self.hsv_upper.tolist()}")
        print(f"%         min_radius = {self.min_radius}  "
              f"max_radius = {self.max_radius}  "
              f"min_circularity = {self.min_circularity}")

    ##########################################################

    def detect(self, img):
        """Detect an orange golf ball — including when partially clipped at the
        bottom of the frame — using HSV colour filtering and contour fitting.
        Updates detected, cx, cy, radius, clip_fraction.
        Returns True if a ball was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h

        # 1. HSV colour mask to isolate orange regions
        hsv  = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 2. Morphological cleanup — close small holes, remove specks
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  kernel, iterations=1)

        # 3. Find contours and fit a minimum enclosing circle to each candidate.
        #    minEnclosingCircle works on partial arcs, so it handles the ball
        #    clipping out of the bottom of the frame correctly.
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        best_cx, best_cy, best_r = 0, 0, 0
        found = False
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < self.min_area:
                continue
            perimeter = cv.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity < self.min_circularity:
                continue
            (cx, cy), r = cv.minEnclosingCircle(cnt)
            r = int(round(r))
            if r < self.min_radius or r > self.max_radius:
                continue
            # Prefer the largest circle (closest ball)
            if r > best_r:
                best_cx, best_cy, best_r = int(round(cx)), int(round(cy)), r
                found = True

        if found:
            self.cx           = best_cx
            self.cy           = best_cy
            self.radius       = best_r
            # How much of the fitted circle is below the bottom of the frame
            bottom = self.cy + self.radius
            if bottom > h and self.radius > 0:
                self.clip_fraction = min(1.0, (bottom - h) / (2.0 * self.radius))
            else:
                self.clip_fraction = 0.0
            self.detected     = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
        else:
            self.detected      = False
            self.clip_fraction = 0.0

        return self.detected

    ##########################################################

    def steeringError(self):
        """Horizontal offset of the ball centre from the image centre,
        normalised to [-1, 1]. Negative = ball is left, positive = ball is right.
        Returns 0 if no ball detected or image width unknown."""
        if not self.detected or self._img_w == 0:
            return 0.0
        return (self.cx - self._img_w / 2.0) / (self._img_w / 2.0)

    ##########################################################

    def isClose(self, radius_threshold=40, clip_threshold=0.3):
        """Returns True when the ball is close enough to capture.
        Triggers when the ball is large enough (radius >= radius_threshold) AND
        is clipping out of the bottom of the frame (clip_fraction >= clip_threshold).
        The radius guard prevents small orange specks near the frame edge from
        triggering a false capture."""
        return (self.detected
                and self.radius >= radius_threshold
                and self.clip_fraction >= clip_threshold)

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        if not self.detected:
            cv.putText(img, "Ball: not found", (10, 50),
                       cv.FONT_HERSHEY_PLAIN, 1.4, (0, 100, 255), thickness=2)
            return
        orange = (0, 100, 255)  # BGR orange
        # Draw circle only if centre is within the frame; otherwise just the arc
        if self.cy < self._img_h:
            cv.circle(img, (self.cx, self.cy), self.radius, orange, thickness=2)
            cv.circle(img, (self.cx, self.cy), 3, orange, thickness=-1)
        else:
            # Centre is off-screen — draw a horizontal arc at the bottom edge
            cv.ellipse(img, (self.cx, self._img_h - 1), (self.radius, self.radius),
                       0, 180, 360, orange, thickness=2)
        # Image centre reference line
        cv.line(img, (self._img_w // 2, 0), (self._img_w // 2, self._img_h),
                (200, 200, 200), thickness=1)
        # Status text
        err = self.steeringError()
        label = f"Ball: r={self.radius}px  err={err:+.2f}  clip={self.clip_fraction:.2f}"
        text_y = max(self.cy, 20) if self.cy < self._img_h else self._img_h - 20
        cv.putText(img, label, (max(self.cx - self.radius, 4), text_y - self.radius - 6),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SBall:: terminated")


# singleton instance — mirrors pattern used by sgate, svline, sedge, etc.
ball = SBall()
