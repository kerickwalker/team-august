#!/usr/bin/env python3
"""
sfeatures.py
=============================================================================
Visual ground-plane feature extractor for EKF SLAM.

Detects repeatable image features in the camera's ground ROI and projects
them to robot-frame ground coordinates using sground.pixel_to_ground().

Two feature types (applied in priority order):
    tape_corner   Harris corners on the white-tape binary mask.
                  Corresponds to tape T-junctions, L-junctions, and
                  crossings — the most stable features in the environment.
    fast          FAST corners in the grayscale ground ROI.
                  Extra coverage in areas without visible tape.

Returned per feature (dict):
    X        float   robot-frame forward distance  (m, X = forward)
    Y        float   robot-frame lateral distance  (m, Y = left)
    range    float   Euclidean distance from robot (m)
    bearing  float   angle from robot forward axis (rad, + = left)
    type     str     'tape_corner' | 'fast'
    score    float   detector response (higher → more reliable)
    px       tuple   (u, v) source pixel coordinates

Public interface:
    from sfeatures import feature_extractor
    feature_extractor.setup()
    feats = feature_extractor.extract(frame)   # BGR frame
=============================================================================
"""

from __future__ import annotations

import cv2
import math
import os

import numpy as np

from sground import ground

# ── Tuning ────────────────────────────────────────────────────────────────────

# Ignore top fraction of the image (sky / background / far field)
ROI_TOP_FRAC = 0.40

# Ground-point acceptance window (robot frame, metres)
MIN_X_M     = 0.10   # ignore very close pixels (lens distortion noise floor)
MAX_X_M     = 1.80   # camera only sees ~1.5 m ahead reliably
MAX_ABS_Y_M = 1.20   # ± lateral

# Harris corner parameters
HARRIS_BLOCK   = 5
HARRIS_APERTURE = 3
HARRIS_K       = 0.04
HARRIS_QUALITY = 0.02    # keep corners ≥ quality × max_response
NMS_HALF_WIN   = 9       # non-maximum suppression half-window (px)

# FAST parameters
FAST_THRESH = 25

# Hard cap – limits per-frame state augmentation cost
MAX_FEATURES = 25

# White-tape thresholds (mirrors sline.py)
WHITE_V_MIN = 160
WHITE_S_MAX = 60


# ── Helpers ───────────────────────────────────────────────────────────────────

def _harris_nms(response: np.ndarray, quality: float) -> list:
    """
    Non-maximum suppression on a Harris response map.

    Returns list of (x_px, y_px, score) sorted strongest-first.
    """
    max_r = float(response.max())
    if max_r < 1e-9:
        return []

    thresh = max_r * quality
    ksize  = NMS_HALF_WIN * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
    dilated = cv2.dilate(response, kernel)

    local_max = (response == dilated) & (response >= thresh)
    ys, xs = np.where(local_max)

    pts = [(int(xs[i]), int(ys[i]), float(response[ys[i], xs[i]]))
           for i in range(len(xs))]
    pts.sort(key=lambda p: -p[2])
    return pts


# ── Feature extractor class ───────────────────────────────────────────────────

class SFeatureExtractor:
    """
    Detects and projects visual ground features for EKF SLAM.

    Typical usage
    -------------
    from sfeatures import feature_extractor
    feature_extractor.setup()                  # once at startup
    feats = feature_extractor.extract(frame)   # every camera frame
    """

    def __init__(self):
        self.ready = False
        self._fast: cv2.FastFeatureDetector = None

    # ── Setup ─────────────────────────────────────────────────────────────────

    def setup(self, params_path: str = None):
        """
        Initialise the extractor.  Ensures sground is ready.

        Parameters
        ----------
        params_path : path to camera_params.npz (None = auto-locate)
        """
        if params_path is None:
            params_path = os.path.join(
                os.path.dirname(__file__), 'calibration', 'camera_params.npz')

        if not ground.ready:
            ground.setup(params_path)

        if not ground.ready:
            print("% SFeatureExtractor: sground setup failed — extractor disabled")
            return

        self._fast = cv2.FastFeatureDetector_create(
            threshold=FAST_THRESH, nonmaxSuppression=True)

        self.ready = True
        print(f"% SFeatureExtractor: ready  "
              f"roi_top={ROI_TOP_FRAC:.0%}  "
              f"x=[{MIN_X_M:.2f},{MAX_X_M:.2f}]m  "
              f"max={MAX_FEATURES} feats/frame")

    # ── Extract ───────────────────────────────────────────────────────────────

    def extract(self, frame: np.ndarray,
                white_mask: np.ndarray = None) -> list:
        """
        Detect and project ground features from a BGR camera frame.

        Parameters
        ----------
        frame      : BGR camera frame (full resolution)
        white_mask : pre-computed white binary mask (optional —
                     will be computed from frame if not provided)

        Returns
        -------
        list of dicts, sorted by score (best first):
            {X, Y, range, bearing, type, score, px}
        """
        if not self.ready:
            return []

        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        features: list = []

        # ── 1. Tape corners (Harris on white-tape mask) ───────────────────────

        if white_mask is None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(
                hsv, (0, 0, WHITE_V_MIN), (180, WHITE_S_MAX, 255))

        tape_roi = white_mask[roi_top:].astype(np.float32)

        if tape_roi.max() > 0:
            harris = cv2.cornerHarris(tape_roi, HARRIS_BLOCK,
                                      HARRIS_APERTURE, HARRIS_K)
            for x_px, y_roi, score in _harris_nms(harris, HARRIS_QUALITY):
                y_px = y_roi + roi_top
                ok, X, Y = ground.pixel_to_ground(float(x_px), float(y_px))
                if not ok:
                    continue
                if not (MIN_X_M <= X <= MAX_X_M and abs(Y) <= MAX_ABS_Y_M):
                    continue
                r = math.sqrt(X * X + Y * Y)
                b = math.atan2(Y, X)
                features.append({
                    'X': X, 'Y': Y,
                    'range': r, 'bearing': b,
                    'type': 'tape_corner',
                    'score': score,
                    'px': (float(x_px), float(y_px)),
                })
                if len(features) >= MAX_FEATURES:
                    return features

        # ── 2. FAST corners in ground ROI (supplement / fallback) ────────────

        gray_roi = cv2.cvtColor(frame[roi_top:], cv2.COLOR_BGR2GRAY)
        kps = self._fast.detect(gray_roi, None)
        kps = sorted(kps, key=lambda k: -k.response)

        for kp in kps:
            u = float(kp.pt[0])
            v = float(kp.pt[1]) + roi_top
            ok, X, Y = ground.pixel_to_ground(u, v)
            if not ok:
                continue
            if not (MIN_X_M <= X <= MAX_X_M and abs(Y) <= MAX_ABS_Y_M):
                continue
            r = math.sqrt(X * X + Y * Y)
            b = math.atan2(Y, X)
            features.append({
                'X': X, 'Y': Y,
                'range': r, 'bearing': b,
                'type': 'fast',
                'score': float(kp.response),
                'px': (u, v),
            })
            if len(features) >= MAX_FEATURES:
                break

        features.sort(key=lambda f: -f['score'])
        return features[:MAX_FEATURES]


# Module-level singleton
feature_extractor = SFeatureExtractor()
