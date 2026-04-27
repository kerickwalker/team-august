#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from datetime import datetime


class SVLine:

    # --- White-ish line model from sampled line_final_*.jpg frames ---
    # Sample-derived suggestions (latest run):
    #   line_white_min_v=166, line_white_max_s=32
    #   ground_dark_max_v=92, ground_sat_min_s=18
    # White line: bright + low saturation.
    line_white_min_v = 166
    line_white_max_s = 32
    # Ground exclusion model: darker + at least mildly saturated.
    ground_dark_max_v = 92
    ground_sat_min_s = 18
    min_line_span_frac = 0.20    # selected line must span at least this fraction of image width
    avoid_bottom_edge_px = 8      # reject contours touching ROI bottom edge (common false positives)
    min_horizontal_aspect = 2.8   # require width/height >= this (favor horizontal lines)

    # --- ROI: selectable top/bottom fraction of the image ---
    roi_fraction = 0.25          # fraction of frame height used for detection
    roi_mode = "bottom"          # "bottom" or "top"

    # --- Detection results (updated each detect() call) ---
    lineValid     = False        # True if a white line was found in the ROI
    lineValidCnt  = 0            # confidence counter 0-20, mirrors sedge.lineValidCnt
    lineOffset    = 0.0          # lateral offset of line centre from image centre,
                                 # normalised to [-1, 1]. Negative = line is left,
                                 # positive = line is right.
    detectedTime  = datetime.now()
    detCnt        = 0            # total frames where a line was found

    # --- Internal ---
    _img_w        = 0
    _img_h        = 0
    _roi_y0       = 0            # top row of ROI
    _roi_y1       = 0            # bottom row of ROI
    _line_contour = None         # contour points in full-image coordinates
    _line_mask    = None         # binary mask for current ROI (for debug overlay)

    ##########################################################

    def setup(self, brightness_threshold=180, roi_fraction=0.25, roi_mode="bottom"):
        # Keep brightness_threshold arg for compatibility with existing callers.
        # It maps to line_white_min_v in the sampled HSV model.
        self.line_white_min_v = brightness_threshold
        self.roi_fraction         = roi_fraction
        self.roi_mode             = roi_mode
        print("% SVLine:: ready")
        print(f"%          line_white_min_v     = {self.line_white_min_v}")
        print(f"%          line_white_max_s     = {self.line_white_max_s}")
        print(f"%          ground_dark_max_v    = {self.ground_dark_max_v}")
        print(f"%          ground_sat_min_s     = {self.ground_sat_min_s}")
        print(f"%          min_line_span_frac   = {self.min_line_span_frac:.2f}")
        print(f"%          avoid_bottom_edge_px = {self.avoid_bottom_edge_px}")
        print(f"%          min_horizontal_aspect= {self.min_horizontal_aspect:.2f}")
        print(f"%          roi_fraction         = {self.roi_fraction}  ({self.roi_mode} {roi_fraction*100:.0f}% of frame)")

    ##########################################################

    def detect(self, img):
        """Detect the nearest white line in the bottom ROI of a BGR image.
        Updates lineValid, lineValidCnt, lineOffset.
        Returns True if a line was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h
        roi_h = max(1, int(h * self.roi_fraction))
        if self.roi_mode == "top":
            self._roi_y0 = 0
            self._roi_y1 = min(h, roi_h)
        else:
            self._roi_y0 = max(0, h - roi_h)
            self._roi_y1 = h

        # 1. Crop ROI
        roi = img[self._roi_y0:self._roi_y1, 0:w]

        # 2. White-ish mask in HSV with explicit ground-color suppression.
        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        hch, sch, vch = cv.split(hsv)
        line_mask = ((vch >= self.line_white_min_v) & (sch <= self.line_white_max_s)).astype(np.uint8) * 255
        ground_mask = ((vch <= self.ground_dark_max_v) & (sch >= self.ground_sat_min_s)).astype(np.uint8) * 255
        mask = cv.bitwise_and(line_mask, cv.bitwise_not(ground_mask))
        # Encourage a single continuous-ish line and suppress speckles.
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE,
                               cv.getStructuringElement(cv.MORPH_RECT, (13, 3)),
                               iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN,
                               cv.getStructuringElement(cv.MORPH_RECT, (3, 3)),
                               iterations=1)
        self._line_mask = mask

        # 3. Find contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 4. Choose a single best contour (continuous-ish and near center), not simply largest.
        roi_h = max(1, self._roi_y1 - self._roi_y0)
        min_area = w * h * self.roi_fraction * 0.01  # 1% of ROI area
        best = None
        best_score = -1.0
        for c in contours:
            area = cv.contourArea(c)
            if area < min_area:
                continue
            x, y, cw, ch = cv.boundingRect(c)
            if cw < int(self.min_line_span_frac * w):
                continue
            # Enforce a very horizontal contour shape when searching from afar.
            if ch <= 0 or (cw / ch) < self.min_horizontal_aspect:
                continue
            # In top-ROI mode, ignore contours attached to ROI bottom edge.
            if self.roi_mode == "top" and (y + ch) >= (roi_h - self.avoid_bottom_edge_px):
                continue
            M = cv.moments(c)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                center_dist = abs(cx - w / 2.0) / (w / 2.0)  # 0..1
                # Prefer large, centered, continuous components.
                score = area * (1.2 - 0.8 * center_dist)
                if score > best_score:
                    best_score = score
                    best = (c, M, cx)

        if best is not None:
            chosen, M, cx = best
            # Save contour in full-image coordinates so paint() can draw
            # exactly what detector treated as line.
            self._line_contour = chosen + [0, self._roi_y0]
            # Keep mask visualization focused on this single chosen line.
            one = np.zeros_like(mask)
            cv.drawContours(one, [chosen], -1, 255, thickness=cv.FILLED)
            self._line_mask = one
            self.lineOffset   = (cx - w / 2.0) / (w / 2.0)
            self.lineValid    = True
            self.detectedTime = datetime.now()
            self.detCnt      += 1
            if self.lineValidCnt < 20:
                self.lineValidCnt += 1
            return True

        # No valid line found
        self._line_contour = None
        self.lineValid = False
        if self.lineValidCnt > 0:
            self.lineValidCnt -= 1
        return False

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        h, w = img.shape[:2]
        roi_y0 = self._roi_y0 if self._img_h == h else (0 if self.roi_mode == "top" else int(h * (1.0 - self.roi_fraction)))
        roi_y1 = self._roi_y1 if self._img_h == h else (int(h * self.roi_fraction) if self.roi_mode == "top" else h)

        # ROI boundaries
        cv.line(img, (0, roi_y0), (w, roi_y0), (200, 200, 0), thickness=1)
        cv.line(img, (0, roi_y1 - 1), (w, roi_y1 - 1), (200, 200, 0), thickness=1)
        cv.putText(img, f"Line ROI ({self.roi_mode})", (4, max(12, roi_y0 + 14)),
                   cv.FONT_HERSHEY_PLAIN, 1.0, (200, 200, 0), thickness=1)

        if not self.lineValid:
            cv.putText(img, "Line: not found", (4, min(h - 8, roi_y0 + 30)),
                       cv.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 200), thickness=2)
            return

        # Draw detected contour region (what detector considers line).
        if self._line_contour is not None:
            cv.drawContours(img, [self._line_contour], -1, (0, 255, 255), 2)

        # Detected line centre marker
        cx = int((self.lineOffset + 1.0) / 2.0 * w)
        cy = roi_y0 + max(1, (roi_y1 - roi_y0) // 2)
        cv.drawMarker(img, (cx, cy), (200, 200, 0),
                      markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)

        # Image centre reference line
        cv.line(img, (w // 2, roi_y0), (w // 2, roi_y1), (200, 200, 200), thickness=1)

        # Status text
        label = f"Line: offset={self.lineOffset:+.2f}  conf={self.lineValidCnt}/20"
        cv.putText(img, label, (4, h - 8),
                   cv.FONT_HERSHEY_PLAIN, 1.2, (200, 200, 0), thickness=2)

    ##########################################################

    def terminate(self):
        print("% SVLine:: terminated")


# singleton instance — mirrors pattern used by sedge, scam, sgate, etc.
vline = SVLine()
