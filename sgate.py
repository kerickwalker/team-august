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


class SGate:

    # --- HSV thresholds for orange (tune with calibrate_hsv.py) ---
    hsv_lower = np.array([5,  120, 80],  dtype=np.uint8)
    hsv_upper = np.array([25, 255, 255], dtype=np.uint8)

    # --- Contour filter parameters ---
    min_area        = 500    # px² — ignore specks smaller than this
    min_aspect      = 0.3    # width/height — reject very thin vertical slivers
    max_aspect      = 5.0    # width/height — reject very thin horizontal slivers
    min_fill        = 0.15   # contour area / bounding-rect area — rejects L-shapes etc.

    # --- Detection results (updated each detect() call) ---
    detected        = False
    gate_cx         = 0      # pixel x of gate bounding-box centre
    gate_cy         = 0      # pixel y of gate bounding-box centre
    gate_rect       = None   # (x, y, w, h) bounding rect of best contour
    gate_area       = 0      # contour area in px²
    detectedTime    = datetime.now()
    detCnt          = 0      # total frames where a gate was found

    # --- Internal ---
    _kernel         = None   # morphology kernel (created in setup)
    _img_w          = 0      # image width, set on first detect()

    ##########################################################

    def setup(self):
        # 5×5 kernel for closing small gaps in the colour mask
        self._kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        print("% SGate:: ready")
        print(f"%         HSV lower = {self.hsv_lower.tolist()}")
        print(f"%         HSV upper = {self.hsv_upper.tolist()}")

    ##########################################################

    def detect(self, img):
        """Detect the gate in a BGR image.
        Updates detected, gate_cx, gate_cy, gate_rect, gate_area.
        Returns True if a gate candidate was found."""
        h, w = img.shape[:2]
        self._img_w = w

        # 1. Colour mask in HSV
        hsv  = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 2. Morphological cleanup: close gaps then remove small noise
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self._kernel, iterations=2)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  self._kernel, iterations=1)

        # 3. Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 4. Filter and score contours, keep the best candidate
        best       = None
        best_area  = 0
        for c in contours:
            area = cv.contourArea(c)
            if area < self.min_area:
                continue
            x, y, cw, ch = cv.boundingRect(c)
            aspect = cw / ch if ch > 0 else 0
            if not (self.min_aspect <= aspect <= self.max_aspect):
                continue
            fill = area / (cw * ch) if (cw * ch) > 0 else 0
            if fill < self.min_fill:
                continue
            if area > best_area:
                best_area = area
                best      = (c, x, y, cw, ch)

        # 5. Store results
        if best is not None:
            _, x, y, cw, ch = best
            self.gate_rect   = (x, y, cw, ch)
            self.gate_cx     = x + cw // 2
            self.gate_cy     = y + ch // 2
            self.gate_area   = best_area
            self.detected    = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
        else:
            self.detected    = False
            self.gate_rect   = None
            self.gate_area   = 0

        return self.detected

    ##########################################################

    def steeringError(self):
        """Horizontal offset of the gate centre from the image centre,
        normalised to [-1, 1] across the image width.
        Negative = gate is left of centre, positive = gate is right.
        Returns 0 if no gate is detected or image width is unknown."""
        if not self.detected or self._img_w == 0:
            return 0.0
        return (self.gate_cx - self._img_w / 2.0) / (self._img_w / 2.0)

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        if not self.detected:
            cv.putText(img, "Gate: not found", (10, 30),
                       cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), thickness=2)
            return
        x, y, w, h = self.gate_rect
        # bounding box
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 165, 255), thickness=2)
        # centre crosshair
        cx, cy = self.gate_cx, self.gate_cy
        cv.drawMarker(img, (cx, cy), (0, 165, 255),
                      markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)
        # image centre reference line
        img_cx = img.shape[1] // 2
        cv.line(img, (img_cx, 0), (img_cx, img.shape[0]), (200, 200, 200), thickness=1)
        # status text
        err = self.steeringError()
        label = f"Gate: ID area={self.gate_area:.0f}px  err={err:+.2f}"
        cv.putText(img, label, (x, max(y - 8, 12)),
                   cv.FONT_HERSHEY_PLAIN, 1.2, (0, 165, 255), thickness=2)

    ##########################################################

    def terminate(self):
        print("% SGate:: terminated")


# singleton instance — mirrors pattern used by sedge, scam, sir, etc.
gate = SGate()
