#!/usr/bin/env python3
"""
sline.py
=============================================================================
White line extraction and measurement module.

Strategy:
  1. Threshold -> white mask
  2. Global noise filter: remove ALL blobs smaller than MIN_BLOB_AREA
  3. TWO-PASS strip search:
     Pass 1 (near->far): Track B0. In far strips search opposite side for B1.
     Pass 2 (far->near): If B1 found in far strips, track it back toward robot.
  4. fork_evidence = total valid B1 points across ALL strips (both passes)
  5. Tape width verification on every centroid candidate
  6. Each branch projected to ground independently
  7. active_branch selects which branch drives the robot (L=B0, R=B1)

Fork state machine (two-level):
    fork_candidate  — weak signal, evidence >= FORK_CANDIDATE_MIN (default 2)
    fork_confirmed  — strong signal, evidence >= FORK_CONFIRMED_MIN (default 5)
    fork_detected   — alias for fork_confirmed (backward compatible)

Output (per frame):
    line_valid        : bool
    line_points_robot : list of (X, Y) in robot frame
    line_offset       : float (m)
    line_heading      : float (rad)
    line_curvature    : float (1/m)
    fork_evidence     : int
    fork_candidate    : bool
    fork_confirmed    : bool
    fork_detected     : bool (alias)
    active_branch     : int (0 or 1)
    branches          : list of 2 dicts

Usage:
    python3 sline.py --host 10.197.218.17
"""

import cv2
import numpy as np
import sys
import os

from perception.camera.sground import ground

# --- Thresholds: load from calibration/sline_thresholds.py if present -------------------
# That file is the single source of truth for tape-detection parameters and is
# re-written by test_sline_tuner.py when the user presses 'S'.
try:
    from calibration.sline_thresholds import (
        WHITE_V_MIN, WHITE_S_MAX,
        LINE_ROI_TOP_FRAC, N_STRIPS, MIN_BLOB_AREA,
        INIT_WINDOW_PX, TRACK_WINDOW_PX, TAPE_VERIFY_MARGIN_PX,
        FORK_SEARCH_STRIPS, FORK_MIN_SEP_PX, FORK_MIN_GROUND_SEP_M,
        FORK_CANDIDATE_MIN, FORK_CONFIRMED_MIN,
        MIN_VALID_STRIPS,
    )
    print("% SLine: thresholds loaded from calibration/sline_thresholds.py")
except ImportError:
    print("% SLine: WARNING - calibration/sline_thresholds.py not found, using hardcoded defaults")
    WHITE_V_MIN           = 160
    WHITE_S_MAX           = 60
    LINE_ROI_TOP_FRAC     = 0.45
    N_STRIPS              = 8
    MIN_BLOB_AREA         = 150
    INIT_WINDOW_PX        = 220
    TRACK_WINDOW_PX       = 160
    TAPE_VERIFY_MARGIN_PX = 5
    FORK_SEARCH_STRIPS    = 3
    FORK_MIN_SEP_PX       = 30
    FORK_MIN_GROUND_SEP_M = 0.05
    FORK_CANDIDATE_MIN    = 2
    FORK_CONFIRMED_MIN    = 5
    MIN_VALID_STRIPS      = 3


class SLine:

    ready         = False
    debug         = False
    active_branch = 0

    line_valid        = False
    line_points_robot = []
    line_offset       = 0.0
    line_heading      = 0.0
    line_curvature    = 0.0
    fork_evidence     = 0
    fork_candidate    = False
    fork_confirmed    = False
    fork_detected     = False
    branches          = []
    debug_frame       = None

    _fork_lock_active = False
    _b0_is_left       = True

    white_v_min = WHITE_V_MIN
    white_s_max = WHITE_S_MAX

    def setup(self, params_path: str = '../calibration/camera_params.npz',
              debug: bool = False):
        self.debug = debug
        if not ground.ready:
            ground.setup(params_path)
        if not ground.ready:
            print("% SLine: sground setup failed")
            return
        self.ready = True
        print(f"% SLine: ready  strips={N_STRIPS}  min_blob={MIN_BLOB_AREA}px  "
              f"fork_candidate>={FORK_CANDIDATE_MIN}  fork_confirmed>={FORK_CONFIRMED_MIN}  "
              f"debug={'on' if debug else 'off'}")

    def set_active_branch(self, idx: int):
        self.active_branch = max(0, min(1, idx))
        print(f"% SLine: active_branch -> {self.active_branch}")

    def _white_mask(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           (0,   0,             self.white_v_min),
                           (180, self.white_s_max, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    def _remove_small_blobs(self, mask):
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        clean = np.zeros_like(mask)
        for label in range(1, n_labels):
            if int(stats[label, cv2.CC_STAT_AREA]) >= MIN_BLOB_AREA:
                clean[labels == label] = 255
        return clean

    def _tape_width_ok(self, strip_mask, u_found, img_w):
        u_left  = max(0,       int(u_found - TAPE_VERIFY_MARGIN_PX))
        u_right = min(img_w-1, int(u_found + TAPE_VERIFY_MARGIN_PX))
        left_white  = int(np.sum(strip_mask[:, u_left  : u_left  + 1])) > 0
        right_white = int(np.sum(strip_mask[:, u_right : u_right + 1])) > 0
        return left_white and right_white

    def _centroid_from_col_sums(self, col_sums, lo):
        total = int(col_sums.sum())
        if total == 0:
            return None, 0
        cols = np.arange(lo, lo + len(col_sums), dtype=float)
        u    = float(np.average(cols, weights=col_sums))
        return u, total

    def _primary_centroid(self, strip_mask, u_centre, half_win, img_w):
        lo = max(0,     int(u_centre - half_win))
        hi = min(img_w, int(u_centre + half_win))
        col_sums = np.sum(strip_mask[:, lo:hi], axis=0).astype(float)
        u, _ = self._centroid_from_col_sums(col_sums, lo)
        if u is None:
            return u_centre, False
        return u, self._tape_width_ok(strip_mask, u, img_w)

    def _secondary_centroid_far(self, strip_mask, u_primary, img_w):
        mid = img_w / 2.0
        if u_primary < mid:
            lo, hi  = int(mid), img_w
            b1_side = 'right'
        else:
            lo, hi  = 0, int(mid)
            b1_side = 'left'

        col_sums = np.sum(strip_mask[:, lo:hi], axis=0).astype(float)
        u_sec, total = self._centroid_from_col_sums(col_sums, lo)

        if u_sec is None or total == 0:
            return mid, False, b1_side
        if abs(u_sec - u_primary) < FORK_MIN_SEP_PX:
            return mid, False, b1_side
        return u_sec, self._tape_width_ok(strip_mask, u_sec, img_w), b1_side

    def _track_b1_with_separation(self, strip_mask, u_b1_prev, u_b0, img_w):
        u1, v1_ok = self._primary_centroid(
            strip_mask, u_b1_prev, TRACK_WINDOW_PX, img_w)
        if v1_ok and abs(u1 - u_b0) < FORK_MIN_SEP_PX:
            v1_ok = False
        return u1, v1_ok

    def _strip_search(self, clean_mask, img_h, img_w):
        roi_top = int(LINE_ROI_TOP_FRAC * img_h)
        roi_h   = img_h - roi_top
        strip_h = roi_h / N_STRIPS

        strips = []
        for i in range(N_STRIPS):
            v_bot = img_h - int(i * strip_h)
            v_top = max(img_h - int((i + 1) * strip_h), roi_top)
            v_mid = (v_top + v_bot) // 2
            strips.append((v_top, v_bot, v_mid))

        branch0_px = [(img_w / 2, float(strips[i][2]), False) for i in range(N_STRIPS)]
        branch1_px = [(img_w / 2, float(strips[i][2]), False) for i in range(N_STRIPS)]

        prev_u0         = img_w / 2
        first_strip     = True
        last_b1_u       = img_w / 2
        last_b1_strip   = -1
        b1_side         = None
        far_strip_start = N_STRIPS - FORK_SEARCH_STRIPS

        for i in range(N_STRIPS):
            v_top, v_bot, v_mid = strips[i]
            strip    = clean_mask[v_top:v_bot, :]
            half_win = INIT_WINDOW_PX if first_strip else TRACK_WINDOW_PX
            search_u = img_w / 2     if first_strip else prev_u0

            u0, v0_ok = self._primary_centroid(strip, search_u, half_win, img_w)
            if v0_ok:
                prev_u0     = u0
                first_strip = False
            branch0_px[i] = (u0, float(v_mid), v0_ok)

            if i >= far_strip_start:
                u1, v1_ok, detected_side = self._secondary_centroid_far(strip, u0, img_w)
                if v1_ok:
                    if b1_side is None:
                        b1_side = detected_side
                    u1_is_right = u1 > img_w / 2.0
                    if b1_side == 'right' and not u1_is_right:
                        v1_ok = False
                    elif b1_side == 'left' and u1_is_right:
                        v1_ok = False
                if v1_ok:
                    last_b1_u     = u1
                    last_b1_strip = i
                branch1_px[i] = (u1, float(v_mid), v1_ok)

        if last_b1_strip > 0:
            prev_u1 = last_b1_u
            for i in range(last_b1_strip - 1, -1, -1):
                v_top, v_bot, v_mid = strips[i]
                strip   = clean_mask[v_top:v_bot, :]
                u0_here = branch0_px[i][0]
                u1, v1_ok = self._track_b1_with_separation(strip, prev_u1, u0_here, img_w)
                if v1_ok and b1_side is not None:
                    u1_is_right = u1 > img_w / 2.0
                    if b1_side == 'right' and not u1_is_right:
                        v1_ok = False
                    elif b1_side == 'left' and u1_is_right:
                        v1_ok = False
                if v1_ok:
                    prev_u1 = u1
                branch1_px[i] = (u1, float(v_mid), v1_ok)

        fork_evidence = sum(1 for (u, v, ok) in branch1_px if ok)
        return branch0_px, branch1_px, fork_evidence

    def _project_branch(self, points_px):
        ground_pts = []
        for u, v, ok in points_px:
            if ok:
                proj_ok, X, Y = ground.pixel_to_ground(u, v)
                if proj_ok:
                    ground_pts.append((X, Y))
        return ground_pts

    def _compute_measurements(self, ground_pts):
        pts      = np.array(ground_pts)
        offset   = float(pts[0, 1])
        heading  = 0.0
        if len(pts) >= 2:
            coeffs  = np.polyfit(pts[:, 0], pts[:, 1], 1)
            heading = float(np.arctan(coeffs[0]))
        curvature = 0.0
        if len(pts) >= 3:
            coeffs2   = np.polyfit(pts[:, 0], pts[:, 1], 2)
            curvature = float(2.0 * coeffs2[0])
        return offset, heading, curvature

    def _make_branch_dict(self, ground_pts):
        if len(ground_pts) < MIN_VALID_STRIPS:
            return {"valid": False, "points": ground_pts,
                    "offset": 0.0, "heading": 0.0, "curvature": 0.0}
        offset, heading, curvature = self._compute_measurements(ground_pts)
        return {"valid": True, "points": ground_pts,
                "offset": offset, "heading": heading, "curvature": curvature}

    def process(self, frame):
        if not self.ready:
            return self._empty_result()

        img_h, img_w = frame.shape[:2]
        raw_mask   = self._white_mask(frame)
        clean_mask = self._remove_small_blobs(raw_mask)
        b0_px, b1_px, fork_evidence = self._strip_search(clean_mask, img_h, img_w)
        gpts0 = self._project_branch(b0_px)
        gpts1 = self._project_branch(b1_px)

        if gpts0 and gpts1:
            mean_y0 = float(np.mean([p[1] for p in gpts0]))
            mean_y1 = float(np.mean([p[1] for p in gpts1]))
            if abs(mean_y1 - mean_y0) < FORK_MIN_GROUND_SEP_M:
                gpts1         = []
                fork_evidence = 0

        new_candidate = fork_evidence >= FORK_CANDIDATE_MIN
        if gpts0 and gpts1:
            mean_y0 = float(np.mean([p[1] for p in gpts0]))
            mean_y1 = float(np.mean([p[1] for p in gpts1]))
            if not self._fork_lock_active and new_candidate:
                self._fork_lock_active = True
                self._b0_is_left       = (mean_y0 >= mean_y1)
            if self._fork_lock_active:
                b0_currently_left = (mean_y0 >= mean_y1)
                if self._b0_is_left != b0_currently_left:
                    gpts0, gpts1 = gpts1, gpts0
                    b0_px, b1_px = b1_px, b0_px
                    fork_evidence = sum(1 for (u, v, ok) in b1_px if ok)

        if fork_evidence == 0:
            self._fork_lock_active = False

        branch0 = self._make_branch_dict(gpts0)
        branch1 = self._make_branch_dict(gpts1)

        self.branches       = [branch0, branch1]
        self.fork_evidence  = fork_evidence
        self.fork_candidate = fork_evidence >= FORK_CANDIDATE_MIN
        self.fork_confirmed = fork_evidence >= FORK_CONFIRMED_MIN
        self.fork_detected  = self.fork_confirmed

        ab     = self.active_branch if self.fork_candidate else 0
        active = self.branches[ab]

        self.line_valid        = active["valid"]
        self.line_points_robot = active["points"]
        self.line_offset       = active["offset"]
        self.line_heading      = active["heading"]
        self.line_curvature    = active["curvature"]

        if self.debug:
            self.debug_frame = self._draw_debug(
                frame, raw_mask, clean_mask, b0_px, b1_px, gpts0, gpts1, fork_evidence)

        return self._pack_result()

    def _pack_result(self):
        return {
            "line_valid":        self.line_valid,
            "line_points_robot": self.line_points_robot,
            "line_offset":       self.line_offset,
            "line_heading":      self.line_heading,
            "line_curvature":    self.line_curvature,
            "fork_evidence":     self.fork_evidence,
            "fork_candidate":    self.fork_candidate,
            "fork_confirmed":    self.fork_confirmed,
            "fork_detected":     self.fork_detected,
            "active_branch":     self.active_branch,
            "branches":          self.branches,
        }

    def _empty_result(self):
        empty_branch = {"valid": False, "points": [],
                        "offset": 0.0, "heading": 0.0, "curvature": 0.0}
        return {
            "line_valid":        False,
            "line_points_robot": [],
            "line_offset":       0.0,
            "line_heading":      0.0,
            "line_curvature":    0.0,
            "fork_evidence":     0,
            "fork_candidate":    False,
            "fork_confirmed":    False,
            "fork_detected":     False,
            "active_branch":     self.active_branch,
            "branches":          [empty_branch, empty_branch],
        }

    def _draw_debug(self, frame, raw_mask, clean_mask,
                    b0_px, b1_px, gpts0, gpts1, fork_evidence):
        img_h, img_w = frame.shape[:2]
        vis = frame.copy()

        clean_bgr = np.zeros_like(frame)
        clean_bgr[:, :, 1] = clean_mask
        vis = cv2.addWeighted(vis, 0.75, clean_bgr, 0.4, 0)

        removed = cv2.bitwise_and(raw_mask, cv2.bitwise_not(clean_mask))
        removed_bgr = np.zeros_like(frame)
        removed_bgr[:, :, 2] = removed
        vis = cv2.addWeighted(vis, 1.0, removed_bgr, 0.35, 0)

        roi_y = int(LINE_ROI_TOP_FRAC * img_h)
        cv2.line(vis, (0, roi_y), (img_w, roi_y), (120, 120, 120), 1)

        roi_h          = img_h - roi_y
        strip_h        = roi_h / N_STRIPS
        far_boundary_y = img_h - int((N_STRIPS - FORK_SEARCH_STRIPS) * strip_h)
        cv2.line(vis, (0, far_boundary_y), (img_w, far_boundary_y), (180, 80, 255), 1)

        def draw_branch(points_px, gpts, color, label_prefix):
            gp_idx = 0
            for u, v, ok in points_px:
                if not ok:
                    continue
                cv2.drawMarker(vis, (int(u), int(v)), color,
                               markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
                u_l = max(0,       int(u - TAPE_VERIFY_MARGIN_PX))
                u_r = min(img_w-1, int(u + TAPE_VERIFY_MARGIN_PX))
                cv2.line(vis, (u_l, int(v)-4), (u_l, int(v)+4), (180, 180, 0), 1)
                cv2.line(vis, (u_r, int(v)-4), (u_r, int(v)+4), (180, 180, 0), 1)
                if gp_idx < len(gpts):
                    X, Y = gpts[gp_idx]
                    lbl  = f"{label_prefix} X={X:.2f} Y={Y:+.2f}"
                    gp_idx += 1
                    tx = int(u) + 8 if int(u) + 170 < img_w else int(u) - 170
                    cv2.putText(vis, lbl, (tx, int(v) - 4),
                                cv2.FONT_HERSHEY_PLAIN, 0.85, color, 1)
            valid_px = [(int(u), int(v)) for (u, v, ok) in points_px if ok]
            for a, b in zip(valid_px[:-1], valid_px[1:]):
                cv2.line(vis, a, b, color, 2)

        draw_branch(b0_px, gpts0, (0, 220, 0),   "B0")
        draw_branch(b1_px, gpts1, (0, 220, 220), "B1")

        ab_col = (0, 220, 0) if self.active_branch == 0 else (0, 220, 220)
        cv2.putText(vis, f"ACTIVE: B{self.active_branch}", (img_w - 160, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, ab_col, 2)

        if self.fork_confirmed:
            banner     = f"FORK CONFIRMED  evidence={fork_evidence}/{N_STRIPS}"
            banner_col = (0, 60, 255)
        elif self.fork_candidate:
            banner     = f"fork candidate  evidence={fork_evidence}/{N_STRIPS}"
            banner_col = (0, 165, 255)
        else:
            banner     = f"no fork  evidence={fork_evidence}/{FORK_CANDIDATE_MIN} needed"
            banner_col = (120, 120, 120)
        cv2.putText(vis, banner, (8, roi_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, banner_col, 1)

        b = self.branches[self.active_branch]
        if b["valid"]:
            summary = [
                f"points   : {len(b['points'])}/{N_STRIPS} valid",
                f"curvature: {b['curvature']:+.3f} 1/m",
                f"heading  : {np.degrees(b['heading']):+.1f} deg",
                f"offset   : {b['offset']:+.3f} m",
            ]
            col = (0, 220, 0)
        else:
            b_other = self.branches[1 - self.active_branch]
            summary = [f"B{self.active_branch} NOT VALID  "
                       f"B{1-self.active_branch}: {'valid' if b_other['valid'] else 'invalid'}"]
            col = (0, 80, 220)

        for i, ln in enumerate(summary):
            cv2.putText(vis, ln, (8, img_h - 75 - i * 18),
                        cv2.FONT_HERSHEY_PLAIN, 1.0, col, 1)
        return vis

    def update_threshold(self, white_v_min=None, white_s_max=None):
        if white_v_min is not None:
            self.white_v_min = white_v_min
        if white_s_max is not None:
            self.white_s_max = white_s_max
        print(f"% SLine: threshold V_min={self.white_v_min}  S_max={self.white_s_max}")

    def apply_thresholds(self, **kwargs):
        """Live-update any tape-detection threshold by name.

        Accepted keys (any subset, lowercase):
            white_v_min, white_s_max,
            line_roi_top_frac, n_strips, min_blob_area,
            init_window_px, track_window_px, tape_verify_margin_px,
            fork_search_strips, fork_min_sep_px, fork_min_ground_sep_m,
            fork_candidate_min, fork_confirmed_min,
            min_valid_strips

        White-V/S also have instance attributes (used inside _white_mask via
        self.*); the rest are module-level globals consumed by free helpers,
        so we update both places.
        """
        g = globals()
        for k, v in kwargs.items():
            up = k.upper()
            if up in g:
                g[up] = v
            if hasattr(self, k):
                setattr(self, k, v)

    def terminate(self):
        print("% SLine: terminated")


vision = SLine()


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description="Live white line extraction")
    parser.add_argument("--host",   default="localhost")
    parser.add_argument("--port",   type=int, default=7123)
    parser.add_argument("--params", default="../calibration/camera_params.npz")
    args = parser.parse_args()

    vision.setup(args.params, debug=True)
    if not vision.ready:
        sys.exit(1)

    stream_url = f"http://{args.host}:{args.port}/stream.mjpg"
    print(f"% Connecting to {stream_url} ...")
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("% ERROR: Could not open stream")
        sys.exit(1)

    print("% V/B = brightness +/-5   N/M = saturation +/-5")
    print("% L = branch 0   R = branch 1   Q = quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        vision.process(frame)
        if vision.debug_frame is not None:
            cv2.imshow("Line Extraction", vision.debug_frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('l'):
            vision.set_active_branch(0)
        elif key == ord('r'):
            vision.set_active_branch(1)
        elif key == ord('v'):
            vision.update_threshold(white_v_min=min(255, vision.white_v_min + 5))
        elif key == ord('b'):
            vision.update_threshold(white_v_min=max(0,   vision.white_v_min - 5))
        elif key == ord('n'):
            vision.update_threshold(white_s_max=min(255, vision.white_s_max + 5))
        elif key == ord('m'):
            vision.update_threshold(white_s_max=max(0,   vision.white_s_max - 5))

    cap.release()
    cv2.destroyAllWindows()