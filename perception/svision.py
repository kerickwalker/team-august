"""
svision.py
=============================================================================
White line extraction and measurement module.

Strategy: (can be changed later if needed)
  1. Threshold -> white mask
  2. Global noise filter: remove ALL blobs smaller than MIN_BLOB_AREA from the
     entire mask BEFORE strip search. This kills chalk dust in one shot.
  3. Bottom strip: search within INIT_WINDOW_PX of image centre for the tape
  4. Each subsequent strip: search within TRACK_WINDOW_PX of previous centre
  5. Project valid centres to ground via sground.py
  6. Compute lateral offset, heading, curvature

Key parameter to tune on-site:
    MIN_BLOB_AREA   - raise if chalk still gets through (try 150, 300, 500)
    TRACK_WINDOW_PX - raise if line is lost on sharp turns (try 160, 200)

Output (per frame):
    line_points_robot : list of (X, Y) in robot frame — near to far
    line_offset       : lateral offset at nearest point (m), + = line is left
    line_heading      : direction of line in robot frame (rad)
    line_curvature    : signed curvature (1/m), + = curves left
    line_valid        : bool

Usage (standalone test):
    python3 vision/svision.py --host 10.197.218.17
"""

import cv2
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from perception.sground import ground

# --- Threshold ─--------------------------------------------------------------------------
WHITE_V_MIN = 160   # HSV V minimum  - raise if floor looks white (Update: I think I need to tweak it a bit more)
WHITE_S_MAX = 60    # HSV S maximum  - lower if yellow/grey is picked up

# --- ROI --------------------------------------------------------------------------
LINE_ROI_TOP_FRAC = 0.45   # ignore everything above this row fraction

# --- Strip sampling ------------------------------------------------------------------------
N_STRIPS     = 5    # number of horizontal sample strips (near -> far)

# --- Blob filtering ------------------------------------------------------------------------
# Applied GLOBALLY to the whole mask before strip search.
# Any connected component smaller than this is erased (chalk, dust, reflections).
# The tape should be 200-2000px; chalk marks are usually < 100px each.
# Raise this value if chalk still gets through. Try: 100 -> 200 -> 400
MIN_BLOB_AREA = 150

# --- Tracking window ----------------------------------------------------------------------
# Bottom strip: accept line only within this half-width from image centre.
INIT_WINDOW_PX  = 220

# Subsequent strips: only look within this half-width of previous found centre.
# Make wider for sharp turns, narrower to reject far-away second lines.
TRACK_WINDOW_PX = 160

# --- Validity --------------------------------------------------------------------------
MIN_VALID_STRIPS = 3
# -----------------------------------------------------------------------------


class SVision:
    """
    White-line extractor

    After process(frame):
        line_points_robot : [(X,Y), ...]  robot-frame ground points, near -> far
        line_offset       : float  lateral offset at nearest point (m)
        line_heading      : float  line direction (rad)
        line_curvature    : float  signed curvature (1/m)
        line_valid        : bool
        debug_frame       : BGR overlay image (only when debug=True)
    """

    ready       = False
    debug       = False

    line_points_robot = []
    line_offset       = 0.0
    line_heading      = 0.0
    line_curvature    = 0.0
    line_valid        = False
    debug_frame       = None

    white_v_min = WHITE_V_MIN
    white_s_max = WHITE_S_MAX


    def setup(self, params_path: str = 'calibration/camera_params.npz',
              debug: bool = False):
        self.debug = debug
        if not ground.ready:
            ground.setup(params_path)
        if not ground.ready:
            print("% SVision: ERROR — sground setup failed")
            return
        self.ready = True
        print(f"% SVision: ready  "
              f"(ROI={LINE_ROI_TOP_FRAC*100:.0f}%  strips={N_STRIPS}  "
              f"min_blob={MIN_BLOB_AREA}px  "
              f"init_win=±{INIT_WINDOW_PX}px  track_win=±{TRACK_WINDOW_PX}px  "
              f"debug={'on' if debug else 'off'})")


    def _white_mask(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           (0,   0,             self.white_v_min),
                           (180, self.white_s_max, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


    def _remove_small_blobs(self, mask):
        """
        Erase all connected components smaller than MIN_BLOB_AREA from mask.
        Returns a cleaned mask where only large blobs (tape) survive.
        """
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)

        clean = np.zeros_like(mask)
        for label in range(1, n_labels):
            if int(stats[label, cv2.CC_STAT_AREA]) >= MIN_BLOB_AREA:
                clean[labels == label] = 255

        return clean


    def _centroid_in_window(self, strip_mask, u_centre, half_win, img_w):
        """
        Within [u_centre ± half_win], compute the weighted column centroid
        of white pixels in strip_mask (already blob-filtered).

        Returns (u_found, valid)
        """
        lo = max(0,     int(u_centre - half_win))
        hi = min(img_w, int(u_centre + half_win))

        col_sums = np.sum(strip_mask[:, lo:hi], axis=0)
        total    = int(col_sums.sum())

        if total == 0:
            return u_centre, False

        cols    = np.arange(lo, hi, dtype=float)
        u_found = float(np.average(cols, weights=col_sums))
        return u_found, True


    def _strip_centres_tracked(self, clean_mask, img_h, img_w):
        """
        Near-to-far strip search on the blob-filtered mask.
        Tracking window follows previous found centroid.

        Returns list of (u, v, valid) - one per strip, near (0) to far
        """
        roi_top = int(LINE_ROI_TOP_FRAC * img_h)
        roi_h   = img_h - roi_top
        strip_h = roi_h / N_STRIPS

        points_px   = []
        prev_u      = img_w / 2
        first_strip = True

        for i in range(N_STRIPS):
            v_bot = img_h - int(i * strip_h)
            v_top = img_h - int((i + 1) * strip_h)
            v_top = max(v_top, roi_top)
            v_mid = (v_top + v_bot) // 2

            strip = clean_mask[v_top:v_bot, :]

            half_win = INIT_WINDOW_PX if first_strip else TRACK_WINDOW_PX
            search_u = img_w / 2     if first_strip else prev_u

            u_found, valid = self._centroid_in_window(strip, search_u, half_win, img_w)

            if valid:
                prev_u      = u_found
                first_strip = False

            points_px.append((u_found, float(v_mid), valid))

        return points_px


    def _compute_measurements(self, ground_pts):
        pts = np.array(ground_pts)

        offset = pts[0, 1]

        heading = 0.0
        if len(pts) >= 2:
            coeffs  = np.polyfit(pts[:, 0], pts[:, 1], 1)
            heading = float(np.arctan(coeffs[0]))

        curvature = 0.0
        if len(pts) >= 3:
            coeffs2   = np.polyfit(pts[:, 0], pts[:, 1], 2)
            curvature = float(2.0 * coeffs2[0])

        return offset, heading, curvature


    def process(self, frame):
        """
        Returns dict:
            line_valid        : bool
            line_points_robot : list of (X, Y)
            line_offset       : float (m)
            line_heading      : float (rad)
            line_curvature    : float (1/m)
        """
        if not self.ready:
            print("% SVision: process() called before setup()")
            return self._empty_result()

        img_h, img_w = frame.shape[:2]

        # 1. Threshold
        raw_mask   = self._white_mask(frame)

        # 2. Global blob filter kills chalk in one pass
        clean_mask = self._remove_small_blobs(raw_mask)

        # 3. Strip search with tracking
        points_px  = self._strip_centres_tracked(clean_mask, img_h, img_w)

        # 4. Project to ground
        ground_pts  = []
        valid_flags = []
        for u, v, strip_ok in points_px:
            if strip_ok:
                ok, X, Y = ground.pixel_to_ground(u, v)
                if ok:
                    ground_pts.append((X, Y))
                    valid_flags.append(True)
                    continue
            valid_flags.append(False)

        # 5. Measurements
        if len(ground_pts) < MIN_VALID_STRIPS:
            self.line_valid        = False
            self.line_points_robot = ground_pts
            self.line_offset       = 0.0
            self.line_heading      = 0.0
            self.line_curvature    = 0.0
        else:
            offset, heading, curvature = self._compute_measurements(ground_pts)
            self.line_valid        = True
            self.line_points_robot = ground_pts
            self.line_offset       = offset
            self.line_heading      = heading
            self.line_curvature    = curvature

        if self.debug:
            self.debug_frame = self._draw_debug(
                frame, raw_mask, clean_mask, points_px, valid_flags, ground_pts)

        return self._pack_result()

    def _pack_result(self):
        return {
            "line_valid":        self.line_valid,
            "line_points_robot": self.line_points_robot,
            "line_offset":       self.line_offset,
            "line_heading":      self.line_heading,
            "line_curvature":    self.line_curvature,
        }

    def _empty_result(self):
        return {
            "line_valid":        False,
            "line_points_robot": [],
            "line_offset":       0.0,
            "line_heading":      0.0,
            "line_curvature":    0.0,
        }

    # --------------------------------------------------------------------------

    def _draw_debug(self, frame, raw_mask, clean_mask, points_px, valid_flags, ground_pts):
        img_h, img_w = frame.shape[:2]
        vis = frame.copy()

        # Show clean mask as green tint (chalk already removed)
        clean_bgr = np.zeros_like(frame)
        clean_bgr[:, :, 1] = clean_mask   # green channel only
        vis = cv2.addWeighted(vis, 0.75, clean_bgr, 0.4, 0)

        # Show raw but removed pixels as red tint (chalk)
        removed = cv2.bitwise_and(raw_mask, cv2.bitwise_not(clean_mask))
        removed_bgr = np.zeros_like(frame)
        removed_bgr[:, :, 2] = removed    # red channel only
        vis = cv2.addWeighted(vis, 1.0, removed_bgr, 0.35, 0)

        # ROI top line (? - maybe remove -> satellites???)
        roi_y = int(LINE_ROI_TOP_FRAC * img_h)
        cv2.line(vis, (0, roi_y), (img_w, roi_y), (120, 120, 120), 1)
        cv2.putText(vis, "ROI top", (4, roi_y - 4),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (120, 120, 120), 1)

        # Tracking window boxes + centroid markers
        gp_idx      = 0
        prev_draw_u = img_w / 2
        for i, (u, v, strip_ok) in enumerate(points_px):
            valid = valid_flags[i]
            color = (0, 220, 0) if valid else (0, 60, 220)

            half = INIT_WINDOW_PX if i == 0 else TRACK_WINDOW_PX
            lo   = max(0,     int(prev_draw_u - half))
            hi   = min(img_w, int(prev_draw_u + half))
            cv2.rectangle(vis, (lo, int(v) - 6), (hi, int(v) + 6),
                          (80, 80, 80), 1)

            cv2.drawMarker(vis, (int(u), int(v)), color,
                           markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)

            if valid:
                prev_draw_u = u
                if gp_idx < len(ground_pts):
                    X, Y  = ground_pts[gp_idx]
                    label = f"X={X:.2f} Y={Y:+.2f}"
                    gp_idx += 1
                else:
                    label = "?"
            else:
                label = "no tape"

            tx = int(u) + 8 if int(u) + 160 < img_w else int(u) - 160
            cv2.putText(vis, label, (tx, int(v) - 4),
                        cv2.FONT_HERSHEY_PLAIN, 0.9, color, 1)

        # Connect valid points
        valid_px = [(int(u), int(v)) for (u, v, ok) in points_px if ok]
        for a, b in zip(valid_px[:-1], valid_px[1:]):
            cv2.line(vis, a, b, (0, 200, 0), 2)

        cv2.putText(vis, "green = tape  red = chalk(removed)", (4, img_h - 60),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (180, 180, 180), 1)

        # Measurement summary
        if self.line_valid:
            summary = [
                f"offset   : {self.line_offset:+.3f} m",
                f"heading  : {np.degrees(self.line_heading):+.1f} deg",
                f"curvature: {self.line_curvature:+.3f} 1/m",
                f"points   : {len(ground_pts)}/{N_STRIPS} valid",
            ]
            col = (0, 220, 0)
        else:
            summary = [f"LINE NOT FOUND  ({len(ground_pts)}/{N_STRIPS} strips)"]
            col = (0, 60, 220)

        for i, line in enumerate(summary):
            cv2.putText(vis, line, (8, img_h - 75 - i * 18),
                        cv2.FONT_HERSHEY_PLAIN, 1.0, col, 1)

        return vis

  # --------------------------------------------------------------------------

    def update_threshold(self, white_v_min=None, white_s_max=None):
        if white_v_min is not None:
            self.white_v_min = white_v_min
        if white_s_max is not None:
            self.white_s_max = white_s_max
        print(f"% SVision: threshold — V_min={self.white_v_min}  "
              f"S_max={self.white_s_max}")

    def terminate(self):
        print("% SVision: terminated")


vision = SVision()

if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description="Live line extraction test")
    parser.add_argument("--host",   default="localhost")
    parser.add_argument("--port",   type=int, default=7123)
    parser.add_argument("--params", default="calibration/camera_params.npz")
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

    print("% Stream opened.")
    print("V/B = brightness +/-5   N/M = saturation +/-5   Q = quit")
    print("If chalk still shows green, increase MIN_BLOB_AREA in svision.py")

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
    print(f"% Final threshold: V_min={vision.white_v_min}  S_max={vision.white_s_max}")