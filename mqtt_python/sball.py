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
    hsv_upper = np.array([20, 255, 255], dtype=np.uint8)

    # --- Hough circle parameters ---
    hough_dp            = 1.2    # inverse resolution ratio (1 = full res, higher = faster)
    hough_min_dist      = 50     # minimum pixel distance between detected circle centres
    hough_param1        = 80     # Canny high threshold passed internally to HoughCircles
    hough_param2        = 18     # accumulator threshold — lower = more circles detected
    hough_min_radius    = 8      # px — smallest circle to consider
    hough_max_radius    = 80     # px — largest circle to consider

    # --- Detection results (updated each detect() call) ---
    detected      = False
    cx            = 0           # pixel x of ball centre
    cy            = 0           # pixel y of ball centre
    radius        = 0           # pixel radius of detected ball
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
        print(f"%         hough_min_radius = {self.hough_min_radius}  "
              f"hough_max_radius = {self.hough_max_radius}")

    ##########################################################

    def detect(self, img):
        """Detect an orange golf ball in a BGR image using HSV colour filtering
        and Hough circle transform.
        Updates detected, cx, cy, radius.
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

        # 3. Blur before circle detection
        blurred = cv.GaussianBlur(mask, (9, 9), 2)

        # 4. Hough circle transform
        circles = cv.HoughCircles(
            blurred,
            cv.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius,
        )

        # 5. Pick the most confident circle (first in the list, highest accumulator vote)
        if circles is not None:
            circles = np.round(circles[0]).astype(int)
            self.cx           = int(circles[0][0])
            self.cy           = int(circles[0][1])
            self.radius       = int(circles[0][2])
            self.detected     = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
        else:
            self.detected = False

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

    def isClose(self, radius_threshold=40):
        """Returns True when the ball's apparent radius exceeds radius_threshold px,
        indicating the robot is near the ball. Tune threshold to match desired
        stopping distance."""
        return self.detected and self.radius >= radius_threshold

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        if not self.detected:
            cv.putText(img, "Ball: not found", (10, 50),
                       cv.FONT_HERSHEY_PLAIN, 1.4, (0, 100, 255), thickness=2)
            return
        orange = (0, 100, 255)  # BGR orange
        # detected circle
        cv.circle(img, (self.cx, self.cy), self.radius, orange, thickness=2)
        # centre dot
        cv.circle(img, (self.cx, self.cy), 3, orange, thickness=-1)
        # image centre reference line
        cv.line(img, (self._img_w // 2, 0), (self._img_w // 2, self._img_h),
                (200, 200, 200), thickness=1)
        # status text
        err = self.steeringError()
        label = f"Ball: r={self.radius}px  err={err:+.2f}"
        cv.putText(img, label, (self.cx + self.radius + 4, self.cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SBall:: terminated")


# singleton instance — mirrors pattern used by sgate, svline, sedge, etc.
ball = SBall()
