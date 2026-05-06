#!/usr/bin/env python3

import cv2 as cv
from datetime import datetime


class SVLine:

    # --- Threshold parameter (tune against a saved frame) ---
    brightness_threshold = 180   # 0-255; white line on black ground is high contrast

    # --- ROI: bottom fraction of the image to search for the nearest line ---
    roi_fraction = 0.25          # use the bottom 25% of the frame

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
    _roi_y        = 0            # top pixel row of the ROI (set on first detect)

    ##########################################################

    def setup(self, brightness_threshold=180, roi_fraction=0.25):
        self.brightness_threshold = brightness_threshold
        self.roi_fraction         = roi_fraction
        print("% SVLine:: ready")
        print(f"%          brightness_threshold = {self.brightness_threshold}")
        print(f"%          roi_fraction         = {self.roi_fraction}  (bottom {roi_fraction*100:.0f}% of frame)")

    ##########################################################

    def detect(self, img):
        """Detect the nearest white line in the bottom ROI of a BGR image.
        Updates lineValid, lineValidCnt, lineOffset.
        Returns True if a line was found."""
        h, w = img.shape[:2]
        self._img_w = w
        self._img_h = h
        self._roi_y = int(h * (1.0 - self.roi_fraction))

        # 1. Crop to bottom ROI (nearest ground)
        roi = img[self._roi_y:h, 0:w]

        # 2. Grayscale + brightness threshold
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, mask = cv.threshold(gray, self.brightness_threshold, 255, cv.THRESH_BINARY)

        # 3. Find contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 4. Pick the largest contour (most likely the line, not a speck)
        if contours:
            largest = max(contours, key=cv.contourArea)
            area = cv.contourArea(largest)
        else:
            area = 0

        # 5. Require a minimum area to suppress noise
        min_area = w * h * self.roi_fraction * 0.01  # 1% of ROI area
        if area >= min_area:
            M = cv.moments(largest)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                self.lineOffset   = (cx - w / 2.0) / (w / 2.0)
                self.lineValid    = True
                self.detectedTime = datetime.now()
                self.detCnt      += 1
                if self.lineValidCnt < 20:
                    self.lineValidCnt += 1
                return True

        # No valid line found
        self.lineValid = False
        if self.lineValidCnt > 0:
            self.lineValidCnt -= 1
        return False

    ##########################################################

    def paint(self, img):
        """Draw detection overlay onto img in-place."""
        h, w = img.shape[:2]
        roi_y = int(h * (1.0 - self.roi_fraction))

        # ROI boundary
        cv.line(img, (0, roi_y), (w, roi_y), (200, 200, 0), thickness=1)
        cv.putText(img, "Line ROI", (4, roi_y - 6),
                   cv.FONT_HERSHEY_PLAIN, 1.0, (200, 200, 0), thickness=1)

        if not self.lineValid:
            cv.putText(img, "Line: not found", (4, roi_y + 20),
                       cv.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 200), thickness=2)
            return

        # Detected line centre marker
        cx = int((self.lineOffset + 1.0) / 2.0 * w)
        cv.drawMarker(img, (cx, roi_y + (h - roi_y) // 2), (200, 200, 0),
                      markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)

        # Image centre reference line
        cv.line(img, (w // 2, roi_y), (w // 2, h), (200, 200, 200), thickness=1)

        # Status text
        label = f"Line: offset={self.lineOffset:+.2f}  conf={self.lineValidCnt}/20"
        cv.putText(img, label, (4, h - 8),
                   cv.FONT_HERSHEY_PLAIN, 1.2, (200, 200, 0), thickness=2)

    ##########################################################

    def terminate(self):
        print("% SVLine:: terminated")


# singleton instance — mirrors pattern used by sedge, scam, sgate, etc.
vline = SVLine()
