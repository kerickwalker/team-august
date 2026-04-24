#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from datetime import datetime


class SGate:

    # --- HSV thresholds for orange (tune with calibrate_hsv.py) ---
    hsv_lower = np.array([3, 120, 80], dtype=np.uint8)
    hsv_upper = np.array([30, 255, 255], dtype=np.uint8)

    # --- Contour filter parameters for individual uprights ---
    min_area        = 500    # px² — ignore specks smaller than this
    min_aspect      = 0.05   # width/height — allow very thin posts
    max_aspect      = 0.8    # width/height — reject anything wider than it is tall
    min_fill        = 0.15   # contour area / bounding-rect area — rejects L-shapes etc.
    min_height_px   = 50     # px — reject short specks that pass the aspect filter

    # --- Detection results (updated each detect() call) ---
    detected        = False
    gate_cx         = 0      # pixel x of midpoint between the two uprights
    gate_cy         = 0      # pixel y of midpoint between the two uprights
    gate_width_px   = 0      # pixel distance between the two upright centres
    left_rect       = None   # (x, y, w, h) bounding rect of left upright
    right_rect      = None   # (x, y, w, h) bounding rect of right upright
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
        """Detect the gate in a BGR image by finding two orange uprights.
        Updates detected, gate_cx, gate_cy, gate_width_px, left_rect, right_rect.
        Returns True if two upright candidates were found."""
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

        # 4. Filter for upright-shaped contours (taller than wide)
        uprights = []
        for c in contours:
            area = cv.contourArea(c)
            if area < self.min_area:
                continue
            x, y, cw, ch = cv.boundingRect(c)
            if ch < self.min_height_px:
                continue
            aspect = cw / ch if ch > 0 else 0
            if not (self.min_aspect <= aspect <= self.max_aspect):
                continue
            fill = area / (cw * ch) if (cw * ch) > 0 else 0
            if fill < self.min_fill:
                continue
            uprights.append((x, y, cw, ch))

        # 5. Need at least two uprights; take the leftmost and rightmost
        if len(uprights) < 2:
            self.detected  = False
            self.left_rect = None
            self.right_rect = None
            self.gate_width_px = 0
            return False

        uprights.sort(key=lambda r: r[0])   # sort by x position
        left  = uprights[0]
        right = uprights[-1]

        # 6. Gate centre = midpoint between the two upright centres
        left_cx  = left[0]  + left[2]  // 2
        right_cx = right[0] + right[2] // 2
        left_cy  = left[1]  + left[3]  // 2
        right_cy = right[1] + right[3] // 2

        self.left_rect     = left
        self.right_rect    = right
        self.gate_cx       = (left_cx + right_cx) // 2
        self.gate_cy       = (left_cy + right_cy) // 2
        self.gate_width_px = right_cx - left_cx
        self.detected      = True
        self.detectedTime  = datetime.now()
        self.detCnt       += 1
        return True

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
        orange = (0, 165, 255)
        # bounding boxes for each upright
        for rect in (self.left_rect, self.right_rect):
            x, y, w, h = rect
            cv.rectangle(img, (x, y), (x + w, y + h), orange, thickness=2)
        # line connecting the two upright centres
        left_cx  = self.left_rect[0]  + self.left_rect[2]  // 2
        left_cy  = self.left_rect[1]  + self.left_rect[3]  // 2
        right_cx = self.right_rect[0] + self.right_rect[2] // 2
        right_cy = self.right_rect[1] + self.right_rect[3] // 2
        cv.line(img, (left_cx, left_cy), (right_cx, right_cy), orange, thickness=1)
        # gate centre crosshair
        cv.drawMarker(img, (self.gate_cx, self.gate_cy), orange,
                      markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)
        # image centre reference line
        img_cx = img.shape[1] // 2
        cv.line(img, (img_cx, 0), (img_cx, img.shape[0]), (200, 200, 200), thickness=1)
        # status text
        err = self.steeringError()
        label = f"Gate: width={self.gate_width_px}px  err={err:+.2f}"
        x = min(self.left_rect[0], self.right_rect[0])
        y = min(self.left_rect[1], self.right_rect[1])
        cv.putText(img, label, (x, max(y - 8, 12)),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SGate:: terminated")


# singleton instance — mirrors pattern used by sedge, scam, sir, etc.
gate = SGate()
