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

    # --- Canny edge detection parameters ---
    canny_low           = 30     # lower hysteresis threshold
    canny_high          = 80     # upper hysteresis threshold

    # --- Hough circle parameters (tune if getting false positives/negatives) ---
    hough_dp            = 1.2    # inverse resolution ratio
    hough_min_dist      = 50     # minimum pixel distance between detected circle centres
    hough_param1        = 80     # Canny high threshold passed internally to HoughCircles
    hough_param2        = 20     # accumulator threshold — lower = more circles detected
    hough_min_radius    = 20     # px — hole should be larger than the ball's min radius
    hough_max_radius    = 120    # px — largest expected hole size in frame

    # --- Detection results (updated each detect() call) ---
    detected      = False
    cx            = 0            # pixel x of hole centre
    cy            = 0            # pixel y of hole centre
    radius        = 0            # pixel radius of detected hole
    detectedTime  = datetime.now()
    detCnt        = 0            # total frames where a hole was found

    # --- Internal ---
    _img_w        = 0
    _img_h        = 0

    ##########################################################

    def setup(self):
        print("% SHole:: ready")
        print(f"%         canny_low={self.canny_low}  canny_high={self.canny_high}")
        print(f"%         hough_min_radius={self.hough_min_radius}  "
              f"hough_max_radius={self.hough_max_radius}")

    ##########################################################

    def detect(self, img):
        """Detect a golf hole in a BGR image using Canny edge detection and
        Hough circle transform. The hole rim creates a circular edge gradient
        even when the interior and ground are the same dark colour.
        Updates detected, cx, cy, radius.
        Returns True if a hole was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h

        # 1. Convert to grayscale and blur to suppress noise before edge detection
        gray    = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (9, 9), 2)

        # 2. Canny edge detection to find the hole rim
        edges = cv.Canny(blurred, self.canny_low, self.canny_high)

        # 3. Hough circle transform on the edge image
        circles = cv.HoughCircles(
            edges,
            cv.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius,
        )

        # 4. Pick the most confident circle
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
        """Horizontal offset of the hole centre from the image centre,
        normalised to [-1, 1]. Negative = hole is left, positive = hole is right.
        Returns 0 if no hole detected or image width unknown."""
        if not self.detected or self._img_w == 0:
            return 0.0
        return (self.cx - self._img_w / 2.0) / (self._img_w / 2.0)

    ##########################################################

    def isClose(self, radius_threshold=60):
        """Returns True when the hole's apparent radius exceeds radius_threshold px,
        indicating the robot (and ball) is near the hole. Tune threshold to match
        desired stopping distance."""
        return self.detected and self.radius >= radius_threshold

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        if not self.detected:
            cv.putText(img, "Hole: not found", (10, 70),
                       cv.FONT_HERSHEY_PLAIN, 1.4, (100, 100, 100), thickness=2)
            return
        grey = (160, 160, 160)
        # detected circle
        cv.circle(img, (self.cx, self.cy), self.radius, grey, thickness=2)
        # centre dot
        cv.circle(img, (self.cx, self.cy), 3, grey, thickness=-1)
        # image centre reference line (only if ball module hasn't drawn it)
        cv.line(img, (self._img_w // 2, 0), (self._img_w // 2, self._img_h),
                (200, 200, 200), thickness=1)
        # status text
        err = self.steeringError()
        label = f"Hole: r={self.radius}px  err={err:+.2f}"
        cv.putText(img, label, (self.cx + self.radius + 4, self.cy),
                   cv.FONT_HERSHEY_PLAIN, 1.2, grey, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SHole:: terminated")


# singleton instance — mirrors pattern used by sball, sgate, svline, etc.
hole = SHole()
