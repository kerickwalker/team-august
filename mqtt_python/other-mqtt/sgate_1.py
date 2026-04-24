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
    """Small close-range controller for roundabout entry using one orange bar.

    Outputs a relative pose proxy and a suggested (v, w) command:
      - lateral_error: normalized horizontal error wrt desired bar position
      - depth_error: proxy based on bar width wrt desired target width
      - v_cmd: forward speed command (m/s)
      - w_cmd: turn-rate command (rad/s)
    """

    # =========================
    # TUNING (entry alignment)
    # =========================
    # Desired bar position in image coordinates.
    # Keep this at 0.0 to center the bar, or offset (e.g. -0.25) to keep
    # the bar left-of-center if that gives safer wheel clearance.
    # Calibrated default from field sample where entry pose was judged good.
    target_x_norm = 0.302

    # Desired bar width in px at "good entry pose".
    # Tune from real frames where entry works well.
    # Calibrated default from field sample where entry pose was judged good.
    target_width_px = 128.0

    # If bar is larger than this, treat as too close and stop.
    stop_width_px = 80.0

    # Command limits and gains.
    min_v = 0.05
    max_v = 0.18
    max_w = 0.7
    kx = 1.2       # steering from lateral error
    kd = 0.20      # damping on lateral error derivative
    kh = 0.20      # speed scaling from depth error
    search_w = 0.25

    # Smoothing and confidence settings.
    ema_alpha = 0.25
    min_seen_width_px = 12.0

    _x_filt = 0.0
    _w_filt = 0.0
    _e_x_prev = 0.0
    _has_history = False

    def reset(self):
        self._x_filt = 0.0
        self._w_filt = 0.0
        self._e_x_prev = 0.0
        self._has_history = False

    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _norm_bar_x(self, gate1: SGateOne):
        if gate1._img_w <= 0:
            return 0.0
        return (gate1.bar_cx - gate1._img_w / 2.0) / (gate1._img_w / 2.0)

    def relative_pose(self, gate1: SGateOne):
        """Return relative pose proxy and confidence from current detection."""
        if not gate1.detected or gate1.bar_width_px <= 0:
            return {
                "valid": False,
                "lateral_error": 0.0,
                "depth_error": 0.0,
                "distance_scale": 0.0,
                "confidence": 0.0,
            }

        x_norm = self._norm_bar_x(gate1)
        w_px = float(gate1.bar_width_px)
        if not self._has_history:
            self._x_filt = x_norm
            self._w_filt = w_px
            self._has_history = True
        else:
            a = self.ema_alpha
            self._x_filt = (1.0 - a) * self._x_filt + a * x_norm
            self._w_filt = (1.0 - a) * self._w_filt + a * w_px

        lateral_error = self._x_filt - self.target_x_norm
        depth_error = (self.target_width_px - self._w_filt) / max(self.target_width_px, 1.0)
        distance_scale = self.target_width_px / max(self._w_filt, 1.0)

        conf_w = self._clamp((self._w_filt - self.min_seen_width_px) / max(self.target_width_px, 1.0), 0.0, 1.0)
        conf_x = self._clamp(1.0 - abs(lateral_error), 0.0, 1.0)
        confidence = 0.5 * conf_w + 0.5 * conf_x

        return {
            "valid": True,
            "lateral_error": float(lateral_error),
            "depth_error": float(depth_error),
            "distance_scale": float(distance_scale),
            "confidence": float(confidence),
        }

    def command(self, gate1: SGateOne, dt=0.1):
        """Compute (v_cmd, w_cmd, pose_dict) from detector output."""
        pose = self.relative_pose(gate1)
        if not pose["valid"]:
            # If target is lost, rotate slowly to reacquire.
            w = self.search_w
            if gate1.bar_side_hint == "left":
                w = abs(self.search_w)
            elif gate1.bar_side_hint == "right":
                w = -abs(self.search_w)
            return 0.0, w, pose

        e_x = pose["lateral_error"]
        de_x = 0.0
        if dt > 1e-3:
            de_x = (e_x - self._e_x_prev) / dt
        self._e_x_prev = e_x

        # Steering: keep bar at target_x_norm with light derivative damping.
        w_cmd = self.kx * e_x + self.kd * de_x
        w_cmd = self._clamp(w_cmd, -self.max_w, self.max_w)

        # Speed: go slower as bar gets larger (closer).
        w = gate1.bar_width_px
        if w >= self.stop_width_px:
            v_cmd = 0.0
        else:
            v_base = self.min_v + self.kh * pose["depth_error"]
            v_cmd = self._clamp(v_base, self.min_v, self.max_v)

            # Slow down more when laterally misaligned.
            v_cmd *= self._clamp(1.0 - 0.6 * abs(e_x), 0.25, 1.0)

        return float(v_cmd), float(w_cmd), pose


# singleton instance — mirrors pattern used by sgate, svline, etc.
gate1 = SGateOne()
gate1_ctrl = SGateOneEntryController()
