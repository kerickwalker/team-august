#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from datetime import datetime


class SGateOne:
    """Single orange-upright detector.

    TUNING (detection):
      - hsv_lower / hsv_upper: orange color mask range.
      - min_area: reject tiny blobs.
      - min_aspect / max_aspect: keep tall-ish shapes only.
      - min_fill: reject hollow or very irregular blobs.
      - min_height_px: reject short detections.
    """

    # --- HSV thresholds for orange (same defaults as sgate.py) ---
    hsv_lower = np.array([3, 120, 80], dtype=np.uint8)
    hsv_upper = np.array([30, 255, 255], dtype=np.uint8)

    # --- Contour filter parameters for one upright ---
    min_area      = 500
    min_aspect    = 0.05
    max_aspect    = 0.8
    min_fill      = 0.15
    min_height_px = 50

    # --- Detection results (updated each detect() call) ---
    detected        = False
    bar_cx          = 0
    bar_cy          = 0
    bar_height_px   = 0
    bar_width_px    = 0
    bar_area_px     = 0
    bar_rect        = None
    bar_side_hint   = "unknown"   # "left", "right", or "unknown"
    detectedTime    = datetime.now()
    detCnt          = 0

    # --- Internal ---
    _kernel         = None
    _img_w          = 0

    ##########################################################

    def setup(self):
        self._kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        print("% SGateOne:: ready")
        print(f"%            HSV lower = {self.hsv_lower.tolist()}")
        print(f"%            HSV upper = {self.hsv_upper.tolist()}")

    ##########################################################

    def detect(self, img):
        """Detect a single orange upright in a BGR image.
        Picks the best upright candidate (largest area) and updates:
        detected, bar_cx, bar_cy, bar_height_px, bar_width_px, bar_area_px, bar_rect.
        Returns True if a candidate is found."""
        h, w = img.shape[:2]
        self._img_w = w

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.hsv_lower, self.hsv_upper)

        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self._kernel, iterations=2)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self._kernel, iterations=1)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        candidates = []
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
            candidates.append((x, y, cw, ch, area))

        if not candidates:
            self.detected = False
            self.bar_rect = None
            self.bar_area_px = 0
            self.bar_height_px = 0
            self.bar_width_px = 0
            self.bar_side_hint = "unknown"
            return False

        # Use the most dominant upright by area.
        best = max(candidates, key=lambda r: r[4])
        x, y, bw, bh, area = best
        self.bar_rect = (x, y, bw, bh)
        self.bar_width_px = bw
        self.bar_height_px = bh
        self.bar_area_px = int(area)
        self.bar_cx = x + bw // 2
        self.bar_cy = y + bh // 2
        self.bar_side_hint = "left" if self.bar_cx < (w // 2) else "right"
        self.detected = True
        self.detectedTime = datetime.now()
        self.detCnt += 1
        return True

    ##########################################################

    def steeringError(self):
        """Horizontal offset of detected bar centre from image centre in [-1, 1].
        Negative means bar is left of centre; positive means right."""
        if not self.detected or self._img_w == 0:
            return 0.0
        return (self.bar_cx - self._img_w / 2.0) / (self._img_w / 2.0)

    ##########################################################

    def paint(self, img):
        """Draw single-upright overlay on img in-place."""
        if not self.detected:
            cv.putText(img, "Gate1: not found", (10, 30),
                       cv.FONT_HERSHEY_PLAIN, 1.4, (0, 0, 200), thickness=2)
            return

        orange = (0, 165, 255)
        x, y, w, h = self.bar_rect
        cv.rectangle(img, (x, y), (x + w, y + h), orange, thickness=2)
        cv.drawMarker(img, (self.bar_cx, self.bar_cy), orange,
                      markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)

        img_cx = img.shape[1] // 2
        cv.line(img, (img_cx, 0), (img_cx, img.shape[0]), (200, 200, 200), thickness=1)

        err = self.steeringError()
        label = (
            f"Gate1: side={self.bar_side_hint} err={err:+.2f} "
            f"h={self.bar_height_px}px a={self.bar_area_px}px2"
        )
        cv.putText(img, label, (x, max(y - 8, 12)),
                   cv.FONT_HERSHEY_PLAIN, 1.2, orange, thickness=2)

    ##########################################################

    def terminate(self):
        print("% SGateOne:: terminated")


class SGateOneEntryController:
    """Minimal P-controller to align with and approach one orange upright.

    Designed for safe testing: very slow speeds, no filtering, no derivative.

    Conventions (Robobot rc command):
        v > 0 = forward
        w > 0 = left turn (CCW), w < 0 = right turn (CW)

    If the bar is right of image center (bar_cx > W/2), the robot must turn
    right -> w must be negative -> w_cmd = -kx * e_x.

    Outputs from command():
        v_cmd (m/s), w_cmd (rad/s), pose dict with:
          valid          (bool)
          lateral_error  (normalized [-1, 1], + = bar right of target)
          depth_error    ((target_w - bar_w) / target_w, + = too far)
    """

    # Desired bar position in normalized image x ([-1, +1]).
    # 0.0 = image center.
    target_x_norm = 0.0

    # Desired bar width in px at the "good entry" distance.
    target_width_px = 130.0

    # Stop forward motion when bar is at least this wide (close enough).
    stop_width_px = 110.0

    # Slow / safe command limits (override per-test if needed).
    min_v = 0.02
    max_v = 0.05
    max_w = 0.30

    # Proportional gain on normalized lateral error.
    kx = 0.6

    # Search behavior when the bar is not detected.
    search_w = 0.15

    _LOST_POSE = {
        "valid": False,
        "lateral_error": 0.0,
        "depth_error": 0.0,
    }

    def reset(self):
        # Stateless controller; nothing to reset, kept for API compatibility.
        pass

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _lateral_error(self, gate1: SGateOne) -> float:
        if gate1._img_w <= 0:
            return 0.0
        x_norm = (gate1.bar_cx - gate1._img_w / 2.0) / (gate1._img_w / 2.0)
        return x_norm - self.target_x_norm

    def command(self, gate1: SGateOne, dt: float = 0.1):
        """Return (v_cmd, w_cmd, pose) for the current detection."""
        if not gate1.detected or gate1.bar_width_px <= 0:
            # Slow rotational search; bias by last known side if known.
            w = self.search_w
            if gate1.bar_side_hint == "right":
                w = -abs(self.search_w)
            elif gate1.bar_side_hint == "left":
                w = abs(self.search_w)
            return 0.0, float(w), dict(self._LOST_POSE)

        e_x = self._lateral_error(gate1)
        bw = float(gate1.bar_width_px)
        e_w = (self.target_width_px - bw) / max(self.target_width_px, 1.0)

        # Steering: turn opposite of error sign (bar right -> turn right).
        w_cmd = self._clamp(-self.kx * e_x, -self.max_w, self.max_w)

        # Forward speed: stop when close enough; otherwise crawl forward.
        if bw >= self.stop_width_px:
            v_cmd = 0.0
        else:
            v_cmd = self._clamp(self.min_v + (self.max_v - self.min_v) * max(e_w, 0.0),
                                self.min_v, self.max_v)
            # Reduce speed when poorly aligned to avoid lateral drift.
            v_cmd *= self._clamp(1.0 - 0.5 * abs(e_x), 0.3, 1.0)

        pose = {
            "valid": True,
            "lateral_error": float(e_x),
            "depth_error": float(e_w),
        }
        return float(v_cmd), float(w_cmd), pose


# singleton instance — mirrors pattern used by sgate, svline, etc.
gate1 = SGateOne()
gate1_ctrl = SGateOneEntryController()
