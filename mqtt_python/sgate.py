#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from datetime import datetime


class SGate:

    # --- HSV thresholds for orange (tune with calibrate_hsv.py) ---
    hsv_lower = np.array([3, 120, 80], dtype=np.uint8)
    hsv_upper = np.array([30, 255, 255], dtype=np.uint8)

    # --- Contour filter parameters for individual uprights ---
    min_height_px   = 100    # px — only keep tall enough uprights
    min_gate_width_px = 100  # px — minimum centre-to-centre distance between gate uprights
    max_gate_width_px = 600  # px — wider pairs are rejected as not one gate
    merge_x_tol_px = 18      # px — max horizontal center offset to merge stacked segments
    merge_y_gap_px = 35      # px — max vertical gap between segments to merge

    # --- Detection results (updated each detect() call) ---
    detected        = False
    gate_cx         = 0      # pixel x of midpoint between the two uprights
    gate_cy         = 0      # pixel y of midpoint between the two uprights
    gate_width_px   = 0      # pixel distance between the two upright centres
    left_rect       = None   # (x, y, w, h) bounding rect of left upright
    right_rect      = None   # (x, y, w, h) bounding rect of right upright
    gates           = None   # list of candidate gates: [{"left":..,"right":..,"cx":..,"cy":..,"width_px":..}, ...]
    bars            = None   # list of accepted bar rects [(x,y,w,h), ...]
    rejected_bars   = None   # list of rejected bar dicts: {"rect":(x,y,w,h),"reason":str}
    rejected_pairs  = None   # list of rejected pair dicts: {"left":..,"right":..,"reason":str}
    reject_bar_counts = None  # dict: rejection-reason -> count (bars)
    reject_pair_counts = None # dict: rejection-reason -> count (pairs)
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

    def _merge_bar_segments(self, rects):
        """Merge vertically stacked segments with similar x into one bar."""
        if not rects:
            return []
        rects = sorted(rects, key=lambda r: (r[0] + r[2] // 2, r[1]))
        merged = []
        for rect in rects:
            x, y, w, h = rect
            cx = x + w // 2
            y2 = y + h
            attached = False
            for i, m in enumerate(merged):
                mx, my, mw, mh = m
                mcx = mx + mw // 2
                my2 = my + mh
                x_close = abs(cx - mcx) <= self.merge_x_tol_px
                # overlap or small gap in y indicates one broken upright.
                y_connected = (y <= my2 + self.merge_y_gap_px) and (y2 >= my - self.merge_y_gap_px)
                if x_close and y_connected:
                    nx = min(x, mx)
                    ny = min(y, my)
                    nx2 = max(x + w, mx + mw)
                    ny2 = max(y2, my2)
                    merged[i] = (nx, ny, nx2 - nx, ny2 - ny)
                    attached = True
                    break
            if not attached:
                merged.append(rect)
        # One pass may create newly adjacent merged bars; iterate to settle.
        if len(merged) < len(rects):
            return self._merge_bar_segments(merged)
        return merged

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

        # 4. Build raw bar segments from contours.
        raw_segments = []
        for c in contours:
            x, y, cw, ch = cv.boundingRect(c)
            raw_segments.append((x, y, cw, ch))

        # Merge split segments first, then apply height threshold.
        merged_segments = self._merge_bar_segments(raw_segments)
        uprights = []
        rejected_bars = []
        for x, y, cw, ch in merged_segments:
            if ch < self.min_height_px:
                rejected_bars.append({
                    "rect": (x, y, cw, ch),
                    "reason": f"h<{self.min_height_px}",
                })
                continue
            uprights.append((x, y, cw, ch))
        self.bars = uprights
        self.rejected_bars = rejected_bars
        bar_counts = {}
        for rb in rejected_bars:
            reason = rb["reason"]
            bar_counts[reason] = bar_counts.get(reason, 0) + 1
        self.reject_bar_counts = bar_counts

        # 5. Need at least two uprights to form a gate candidate
        if len(uprights) < 2:
            self.detected  = False
            self.left_rect = None
            self.right_rect = None
            self.gate_width_px = 0
            self.gates = []
            self.rejected_pairs = []
            self.reject_pair_counts = {}
            return False

        # 6. Build all valid upright pairs as gate candidates.
        uprights.sort(key=lambda r: r[0])   # sort by x position
        max_gate_width_px = self.max_gate_width_px
        gate_candidates = []
        rejected_pairs = []
        n = len(uprights)
        for i in range(n - 1):
            left = uprights[i]
            left_cx = left[0] + left[2] // 2
            left_cy = left[1] + left[3] // 2
            for j in range(i + 1, n):
                right = uprights[j]
                right_cx = right[0] + right[2] // 2
                right_cy = right[1] + right[3] // 2
                width_px = right_cx - left_cx
                if width_px < self.min_gate_width_px:
                    rejected_pairs.append({
                        "left": left,
                        "right": right,
                        "reason": f"w<{self.min_gate_width_px}",
                    })
                    continue
                if width_px > max_gate_width_px:
                    rejected_pairs.append({
                        "left": left,
                        "right": right,
                        "reason": f"w>{max_gate_width_px}",
                    })
                    continue

                # Height-vs-width geometric sanity:
                # Each bar should be at least as tall as the inter-bar spacing.
                # Exception: if a bar is clipped by image boundary (touches top/bottom),
                # we skip this check for that bar since true height may be outside frame.
                left_h = left[3]
                right_h = right[3]
                left_clipped = (left[1] <= 0) or (left[1] + left[3] >= h - 1)
                right_clipped = (right[1] <= 0) or (right[1] + right[3] >= h - 1)
                if (not left_clipped) and (left_h < width_px):
                    rejected_pairs.append({
                        "left": left,
                        "right": right,
                        "reason": "left_h<w",
                    })
                    continue
                if (not right_clipped) and (right_h < width_px):
                    rejected_pairs.append({
                        "left": left,
                        "right": right,
                        "reason": "right_h<w",
                    })
                    continue

                gate_candidates.append({
                    "left": left,
                    "right": right,
                    "cx": (left_cx + right_cx) // 2,
                    "cy": (left_cy + right_cy) // 2,
                    "width_px": width_px,
                })
        self.rejected_pairs = rejected_pairs
        pair_counts = {}
        for rp in rejected_pairs:
            reason = rp["reason"]
            pair_counts[reason] = pair_counts.get(reason, 0) + 1
        self.reject_pair_counts = pair_counts

        if not gate_candidates:
            self.detected  = False
            self.left_rect = None
            self.right_rect = None
            self.gate_width_px = 0
            self.gates = []
            return False

        # 7. Keep all candidates; choose one primary gate for controller use.
        # Primary = candidate whose centre is closest to image centre.
        img_cx = w // 2
        gate_candidates.sort(key=lambda g: (abs(g["cx"] - img_cx), -g["width_px"]))
        best = gate_candidates[0]

        self.gates = gate_candidates
        self.left_rect     = best["left"]
        self.right_rect    = best["right"]
        self.gate_cx       = best["cx"]
        self.gate_cy       = best["cy"]
        self.gate_width_px = best["width_px"]
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
        orange = (0, 165, 255)
        green = (0, 220, 0)
        red = (0, 0, 220)
        blue = (255, 120, 0)
        if not self.detected:
            cv.putText(img, "Gate: not found", (10, 52),
                       cv.FONT_HERSHEY_PLAIN, 1.4, red, thickness=2)
        # Draw rejected bars with reason (red).
        for rb in (self.rejected_bars or []):
            x, y, w, h = rb["rect"]
            cv.rectangle(img, (x, y), (x + w, y + h), red, thickness=1)
            cv.putText(img, rb["reason"], (x, max(12, y - 4)),
                       cv.FONT_HERSHEY_PLAIN, 0.9, red, thickness=1)
        # Draw accepted bars (thin blue) so it's clear what survived bar filtering.
        for rect in (self.bars or []):
            x, y, w, h = rect
            cv.rectangle(img, (x, y), (x + w, y + h), blue, thickness=1)
        # Draw rejected gate pairs with reason (red line + short reason).
        for rp in (self.rejected_pairs or []):
            l = rp["left"]
            r = rp["right"]
            lx = l[0] + l[2] // 2
            ly = l[1] + l[3] // 2
            rx = r[0] + r[2] // 2
            ry = r[1] + r[3] // 2
            mx = (lx + rx) // 2
            my = (ly + ry) // 2
            cv.line(img, (lx, ly), (rx, ry), red, thickness=1)
            cv.putText(img, rp["reason"], (mx + 2, my - 2),
                       cv.FONT_HERSHEY_PLAIN, 0.85, red, thickness=1)
        # Draw all valid gate candidates (thin orange).
        for g in (self.gates or []):
            for rect in (g["left"], g["right"]):
                x, y, w, h = rect
                cv.rectangle(img, (x, y), (x + w, y + h), orange, thickness=1)
            lx = g["left"][0] + g["left"][2] // 2
            ly = g["left"][1] + g["left"][3] // 2
            rx = g["right"][0] + g["right"][2] // 2
            ry = g["right"][1] + g["right"][3] // 2
            cv.line(img, (lx, ly), (rx, ry), orange, thickness=1)
        # Highlight primary gate (thicker green) used by controller.
        if self.detected and self.left_rect is not None and self.right_rect is not None:
            for rect in (self.left_rect, self.right_rect):
                x, y, w, h = rect
                cv.rectangle(img, (x, y), (x + w, y + h), green, thickness=2)
            left_cx  = self.left_rect[0]  + self.left_rect[2]  // 2
            left_cy  = self.left_rect[1]  + self.left_rect[3]  // 2
            right_cx = self.right_rect[0] + self.right_rect[2] // 2
            right_cy = self.right_rect[1] + self.right_rect[3] // 2
            cv.line(img, (left_cx, left_cy), (right_cx, right_cy), green, thickness=2)
            cv.drawMarker(img, (self.gate_cx, self.gate_cy), green,
                          markerType=cv.MARKER_CROSS, markerSize=20, thickness=2)
        # image centre reference line
        img_cx = img.shape[1] // 2
        cv.line(img, (img_cx, 0), (img_cx, img.shape[0]), (200, 200, 200), thickness=1)
        # status text
        err = self.steeringError() if self.detected else 0.0
        label = (f"Gate: width={self.gate_width_px}px err={err:+.2f} "
                 f"cand={len(self.gates or [])} rejBars={len(self.rejected_bars or [])} "
                 f"rejPairs={len(self.rejected_pairs or [])}")
        tx, ty = 10, 72
        if self.detected and self.left_rect is not None and self.right_rect is not None:
            tx = min(self.left_rect[0], self.right_rect[0])
            ty = max(min(self.left_rect[1], self.right_rect[1]) - 8, 12)
        cv.putText(img, label, (tx, ty),
                   cv.FONT_HERSHEY_PLAIN, 1.2, green if self.detected else red, thickness=2)
        # Show active detector constraints for quick field debugging.
        max_gate_w = self.max_gate_width_px
        cond = (f"bar: h>={self.min_height_px}px | "
                f"gate: {self.min_gate_width_px}px<=w<={max_gate_w}px | "
                f"merge dx<={self.merge_x_tol_px}, dy<={self.merge_y_gap_px}")
        cv.putText(img, cond, (10, 28),
                   cv.FONT_HERSHEY_PLAIN, 1.1, (255, 255, 0), thickness=1)
        # Compact rejection counters for quick tuning insight.
        if self.reject_bar_counts:
            s = "bars reject: " + " ".join([f"{k}:{v}" for k, v in sorted(self.reject_bar_counts.items())])
            cv.putText(img, s, (10, 92), cv.FONT_HERSHEY_PLAIN, 1.0, red, thickness=1)
        if self.reject_pair_counts:
            s = "pairs reject: " + " ".join([f"{k}:{v}" for k, v in sorted(self.reject_pair_counts.items())])
            cv.putText(img, s, (10, 110), cv.FONT_HERSHEY_PLAIN, 1.0, red, thickness=1)

    ##########################################################

    def terminate(self):
        print("% SGate:: terminated")


# singleton instance — mirrors pattern used by sedge, scam, sir, etc.
gate = SGate()
