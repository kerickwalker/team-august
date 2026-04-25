#!/usr/bin/env python3
"""
samcl.py
=============================================================================
Adaptive Monte Carlo Localization (AMCL) for the Robobot.

Coordinate frame: Kalman world frame
    x   = forward  (0–6 m)
    y   = lateral  (0–7 m)
    yaw = heading  (rad, CCW positive, 0 = facing +x)

Field map uses a swapped frame:
    field_x   = kalman_y  (lateral)
    field_y   = kalman_x  (forward)
    field_yaw = kalman_yaw + π/2

The AMCL maintains N particles each representing a pose hypothesis (x, y, yaw)
with an associated weight.  One iteration:
    1.  motion_update(v, omega, dt)        — velocity motion model with noise
    2.  measurement_update(line, aruco)    — weight particles by sensor likelihood
    3.  resample()                         — systematic resampling

Measurement sources:
    • White tape  (sline output)  — lateral offset + heading relative to robot
    • ArUco markers (saruco)      — range + bearing to known field markers

Public API:
    from samcl import SAMCL
    amcl = SAMCL()
    amcl.setup(field=FIELD, n_particles=500)
    amcl.init_pose(x=1.0, y=5.0, yaw=0.0)
    ...
    amcl.motion_update(v, omega, dt)
    amcl.measurement_update(line_result, aruco_detections)
    amcl.resample()
    px, py, pyaw = amcl.get_estimate()
=============================================================================
"""

from __future__ import annotations

import math
import time
import numpy as np
from typing import List, Dict, Optional, Tuple


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap(a: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) to [-π, π)."""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def _circular_mean(angles: np.ndarray, weights: np.ndarray) -> float:
    """Weighted circular mean of angles (rad)."""
    s = float(np.sum(weights * np.sin(angles)))
    c = float(np.sum(weights * np.cos(angles)))
    return math.atan2(s, c)


# ---------------------------------------------------------------------------
# SAMCL
# ---------------------------------------------------------------------------

class SAMCL:
    """
    Adaptive Monte Carlo Localization particle filter.

    All poses are stored in the Kalman world frame:
        x = forward, y = lateral, yaw = heading (rad).
    """

    # ── Defaults ─────────────────────────────────────────────────────────────
    FIELD_X_MIN = 0.0
    FIELD_X_MAX = 6.0   # Kalman x (forward)
    FIELD_Y_MIN = 0.0
    FIELD_Y_MAX = 7.0   # Kalman y (lateral)

    # Motion model noise coefficients (Probabilistic Robotics, Table 5.3)
    # alpha1..4 control how much rotation/translation noise bleeds into each other
    ALPHA1 = 0.10   # rot noise from rot
    ALPHA2 = 0.05   # rot noise from trans
    ALPHA3 = 0.05   # trans noise from trans
    ALPHA4 = 0.02   # trans noise from rot
    ALPHA5 = 0.02   # final-heading noise from trans
    ALPHA6 = 0.05   # final-heading noise from rot

    # Measurement noise – tape
    SIGMA_LINE_OFFSET  = 0.06   # m
    SIGMA_LINE_HEADING = 0.20   # rad

    # Measurement noise – ArUco
    SIGMA_ARUCO_RANGE   = 0.12  # m
    SIGMA_ARUCO_BEARING = 0.08  # rad

    # Likelihood floor (avoid total particle death)
    LIKELIHOOD_FLOOR = 1e-6

    # Max distance to tape before tape measurement is ignored
    MAX_TAPE_DIST = 1.2  # m

    # Max ArUco range to trust
    MAX_ARUCO_RANGE = 3.0  # m

    # KLD-sampling bounds
    MIN_PARTICLES = 100
    MAX_PARTICLES = 2000

    # ── Init ─────────────────────────────────────────────────────────────────

    def __init__(self):
        self._n = 500
        self._particles: np.ndarray = np.zeros((500, 3))   # (x, y, yaw)
        self._weights:   np.ndarray = np.full(500, 1.0 / 500)

        # Pre-computed field geometry in Kalman frame
        self._seg_p0:  np.ndarray = np.empty((0, 2))   # (N_segs, 2)  [kx, ky]
        self._seg_p1:  np.ndarray = np.empty((0, 2))
        self._seg_dk:  np.ndarray = np.empty((0, 2))   # normalized direction
        self._seg_len: np.ndarray = np.empty((0,))
        self._seg_heading: np.ndarray = np.empty((0,)) # Kalman-frame heading (rad)

        # ArUco: dict {id: (kx, ky)}
        self._aruco_kalman: Dict[int, Tuple[float, float]] = {}

        self._field = None
        self.initialized = False

        # Diagnostics
        self.n_eff: float = 0.0
        self.last_update_time: float = 0.0

    # ── Setup ────────────────────────────────────────────────────────────────

    def setup(self, field=None, n_particles: int = 500):
        """
        Prepare the filter.

        Parameters
        ----------
        field       : CompetitionField2026 object (from field_map_2026.py)
        n_particles : initial number of particles
        """
        self._n = n_particles
        self._particles = np.zeros((n_particles, 3))
        self._weights   = np.full(n_particles, 1.0 / n_particles)

        if field is not None:
            self._field = field
            self._build_tape_segments(field)
            self._build_aruco_map(field)

        print(f"% SAMCL: setup  N={n_particles}  "
              f"tape_segs={len(self._seg_len)}  aruco={len(self._aruco_kalman)}")

    # ── Geometry pre-computation ─────────────────────────────────────────────

    def _build_tape_segments(self, field):
        """
        Extract all tape segments from the field map and convert to Kalman frame.

        Field map:  x = lateral (0-7 m), y = forward (0-6 m)
        Kalman:     x = forward (0-6 m), y = lateral (0-7 m)
        Conversion: kx = field_y,  ky = field_x
        """
        p0_list, p1_list = [], []

        # Tape lines
        for tl in field.tape_lines:
            for i in range(len(tl.waypoints) - 1):
                p0 = tl.waypoints[i]
                p1 = tl.waypoints[i + 1]
                # Convert field → Kalman
                p0_list.append([p0.y, p0.x])
                p1_list.append([p1.y, p1.x])

        # Nav paths (BezierSegment / ArcSegment waypoints)
        for np_ in field.nav_paths:
            for seg in np_.segments:
                if hasattr(seg, 'waypoints') and len(seg.waypoints) >= 2:
                    wps = seg.waypoints
                    for i in range(len(wps) - 1):
                        p0 = wps[i]
                        p1 = wps[i + 1]
                        p0_list.append([p0.y, p0.x])
                        p1_list.append([p1.y, p1.x])

        if not p0_list:
            print("% SAMCL: WARNING – no tape segments found in field map")
            return

        self._seg_p0 = np.array(p0_list, dtype=np.float64)   # (N_segs, 2)
        self._seg_p1 = np.array(p1_list, dtype=np.float64)

        raw_dk = self._seg_p1 - self._seg_p0                  # (N_segs, 2)
        lengths = np.linalg.norm(raw_dk, axis=1)              # (N_segs,)

        # Remove degenerate zero-length segments
        valid = lengths > 1e-6
        self._seg_p0  = self._seg_p0[valid]
        self._seg_p1  = self._seg_p1[valid]
        raw_dk        = raw_dk[valid]
        lengths       = lengths[valid]

        self._seg_len     = lengths
        self._seg_dk      = raw_dk / lengths[:, None]          # normalized
        self._seg_heading = np.arctan2(self._seg_dk[:, 1],
                                       self._seg_dk[:, 0])     # Kalman heading

        print(f"% SAMCL: {len(self._seg_len)} tape segments loaded (Kalman frame)")

    def _build_aruco_map(self, field):
        """
        Load ArUco positions from field map into Kalman frame dict.
        Conversion: kx = field.position.y,  ky = field.position.x
        """
        self._aruco_kalman = {}
        for m in field.all_aruco():
            kx = float(m.position.y)   # field forward → Kalman x
            ky = float(m.position.x)   # field lateral  → Kalman y
            self._aruco_kalman[int(m.id)] = (kx, ky)
        print(f"% SAMCL: {len(self._aruco_kalman)} ArUco markers loaded  "
              f"ids={sorted(self._aruco_kalman.keys())}")

    # ── Initialisation ───────────────────────────────────────────────────────

    def init_pose(self, x: float, y: float, yaw: float,
                  spread_xy: float = 0.30,
                  spread_yaw: float = 0.25):
        """
        Initialise particles in a Gaussian cloud around a known pose.

        Parameters
        ----------
        x, y, yaw   : initial pose in Kalman world frame
        spread_xy   : standard deviation of position cloud (m)
        spread_yaw  : standard deviation of heading cloud (rad)
        """
        n = self._n
        self._particles[:, 0] = x   + np.random.randn(n) * spread_xy
        self._particles[:, 1] = y   + np.random.randn(n) * spread_xy
        self._particles[:, 2] = _wrap(yaw + np.random.randn(n) * spread_yaw)
        self._weights[:] = 1.0 / n
        self.initialized = True
        print(f"% SAMCL: init_pose  x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°  "
              f"N={n}")

    def init_uniform(self, yaw: float = None, spread_yaw: float = None):
        """
        Spread particles uniformly over the field (for global localisation).

        Parameters
        ----------
        yaw        : if provided, constrain heading ± spread_yaw
        spread_yaw : heading spread (rad); defaults to π (fully unknown)
        """
        n = self._n
        self._particles[:, 0] = np.random.uniform(self.FIELD_X_MIN, self.FIELD_X_MAX, n)
        self._particles[:, 1] = np.random.uniform(self.FIELD_Y_MIN, self.FIELD_Y_MAX, n)
        if yaw is not None and spread_yaw is not None:
            self._particles[:, 2] = _wrap(yaw + np.random.randn(n) * spread_yaw)
        else:
            self._particles[:, 2] = np.random.uniform(-math.pi, math.pi, n)
        self._weights[:] = 1.0 / n
        self.initialized = True
        print(f"% SAMCL: init_uniform  N={n}")

    # ── Motion update ────────────────────────────────────────────────────────

    def motion_update(self, v: float, omega: float, dt: float):
        """
        Propagate all particles using a noisy velocity motion model.

        Follows the velocity motion model from Probabilistic Robotics
        (Thrun, Burgard, Fox, 2005) with ALPHA1..6 noise coefficients.

        Parameters
        ----------
        v     : linear velocity (m/s, forward = positive)
        omega : angular velocity (rad/s, CCW = positive)
        dt    : time step (s)
        """
        if dt <= 0 or not self.initialized:
            return

        n = self._n
        a1, a2, a3, a4, a5, a6 = (self.ALPHA1, self.ALPHA2, self.ALPHA3,
                                   self.ALPHA4, self.ALPHA5, self.ALPHA6)

        v2    = v * v
        om2   = omega * omega

        # Sample noisy control for each particle
        v_hat   = v     + np.random.randn(n) * np.sqrt(a1 * v2 + a2 * om2)
        om_hat  = omega + np.random.randn(n) * np.sqrt(a3 * v2 + a4 * om2)
        gam_hat =         np.random.randn(n) * np.sqrt(a5 * v2 + a6 * om2)

        px  = self._particles[:, 0]
        py  = self._particles[:, 1]
        yaw = self._particles[:, 2]

        # Integrate motion (circular arc model; straight-line when ω≈0)
        straight = np.abs(om_hat) < 1e-4

        # Arc case
        r       = np.where(straight, np.inf, v_hat / om_hat)
        d_yaw   = om_hat * dt
        new_yaw = _wrap(yaw + d_yaw + gam_hat * dt)

        new_px_arc = px - r * np.sin(yaw) + r * np.sin(yaw + d_yaw)
        new_py_arc = py + r * np.cos(yaw) - r * np.cos(yaw + d_yaw)

        # Straight case
        new_px_str = px + v_hat * np.cos(yaw) * dt
        new_py_str = py + v_hat * np.sin(yaw) * dt

        self._particles[:, 0] = np.where(straight, new_px_str, new_px_arc)
        self._particles[:, 1] = np.where(straight, new_py_str, new_py_arc)
        self._particles[:, 2] = new_yaw

        # Clamp to field bounds (hard boundary)
        self._particles[:, 0] = np.clip(self._particles[:, 0],
                                        self.FIELD_X_MIN, self.FIELD_X_MAX)
        self._particles[:, 1] = np.clip(self._particles[:, 1],
                                        self.FIELD_Y_MIN, self.FIELD_Y_MAX)

    # ── Measurement update ───────────────────────────────────────────────────

    def measurement_update(self, line_result: dict,
                           aruco_detections: list) -> bool:
        """
        Update particle weights from camera measurements.

        Parameters
        ----------
        line_result       : dict from sline.process()
        aruco_detections  : list of dicts from saruco.process()

        Returns
        -------
        bool : True if at least one measurement was used
        """
        if not self.initialized:
            return False

        updated = False
        w = self._weights.copy()   # work on a copy

        # ── Tape / line measurement ──────────────────────────────────────────
        line_valid = (line_result.get('valid') or
                      line_result.get('line_valid', False))
        if line_valid and len(self._seg_len) > 0:
            line_offset  = float(line_result.get('line_offset', 0.0))
            line_heading = float(line_result.get('line_heading', 0.0))

            w_tape = self._tape_likelihood(line_offset, line_heading)
            w = w * w_tape
            updated = True

        # ── ArUco measurement ────────────────────────────────────────────────
        for det in aruco_detections:
            mid = int(det.get('id', -1))
            r_obs = float(det.get('range', 0.0))
            b_obs = float(det.get('bearing', 0.0))

            if mid not in self._aruco_kalman:
                continue
            if r_obs < 0.05 or r_obs > self.MAX_ARUCO_RANGE:
                continue

            w_aruco = self._aruco_likelihood(mid, r_obs, b_obs)
            w = w * w_aruco
            updated = True

        if updated:
            # Normalise
            total = float(np.sum(w))
            if total > 1e-300:
                self._weights = w / total
            else:
                # All particles died – reinitialise weights uniformly
                print("% SAMCL: WARNING – particle depletion, resetting weights")
                self._weights[:] = 1.0 / self._n
            # Effective particle count
            self.n_eff = float(1.0 / np.sum(self._weights ** 2))
        else:
            self.n_eff = float(self._n)

        self.last_update_time = time.time()
        return updated

    # ── Tape likelihood (vectorised) ─────────────────────────────────────────

    def _tape_likelihood(self, line_offset: float,
                         line_heading: float) -> np.ndarray:
        """
        Compute Gaussian likelihood for the tape measurement for all particles.

        line_offset  : measured lateral offset of tape in robot frame (m, +=left)
        line_heading : measured tape heading relative to robot (rad)

        Returns (N_particles,) array of likelihoods.
        """
        n   = self._n
        px  = self._particles[:, 0]   # (N,)
        py  = self._particles[:, 1]
        yaw = self._particles[:, 2]

        ns = len(self._seg_len)
        if ns == 0:
            return np.ones(n)

        # Vector from each particle to each segment's start point
        # dpx, dpy : (N, ns)
        dpx = px[:, None] - self._seg_p0[None, :, 0]
        dpy = py[:, None] - self._seg_p0[None, :, 1]

        # Clamped projection parameter t ∈ [0, 1]  →  closest point on segment
        t = (dpx * self._seg_dk[None, :, 0] +
             dpy * self._seg_dk[None, :, 1])            # (N, ns)  (unnormalized len)
        t = np.clip(t / self._seg_len[None, :], 0.0, 1.0)  # normalise then clamp

        # Closest point on segment
        cx = self._seg_p0[None, :, 0] + t * (self._seg_p1[None, :, 0] - self._seg_p0[None, :, 0])
        cy = self._seg_p0[None, :, 1] + t * (self._seg_p1[None, :, 1] - self._seg_p0[None, :, 1])

        # 2-D distance from particle to segment
        dist = np.sqrt((px[:, None] - cx) ** 2 + (py[:, None] - cy) ** 2)  # (N, ns)

        # Best (nearest) segment index for each particle
        best = np.argmin(dist, axis=1)                    # (N,)
        best_dist = dist[np.arange(n), best]              # (N,)

        # Skip measurement for particles far from any tape
        near_mask = best_dist < self.MAX_TAPE_DIST        # (N,)

        # ── Lateral residual ─────────────────────────────────────────────────
        # lateral_kalman = (dpx * dky - dpy * dkx) / seg_len
        # But we need to recompute dpx, dpy relative to best segment start
        best_p0x = self._seg_p0[best, 0]   # (N,)
        best_p0y = self._seg_p0[best, 1]
        best_dkx = self._seg_dk[best, 0]
        best_dky = self._seg_dk[best, 1]

        dpx_b = px - best_p0x
        dpy_b = py - best_p0y

        # lateral_kalman > 0 means particle is to the LEFT of segment direction
        # (positive when particle is "above" the directed segment in Kalman frame)
        # lateral_kalman = cross(segment_dir, robot_offset)
        #                = dpx_b * best_dky - dpy_b * best_dkx
        # From slocalize analysis:  residual = line_offset - lateral_kalman ≈ 0
        lateral_kalman = dpx_b * best_dky - dpy_b * best_dkx   # (N,)
        res_lat = line_offset - lateral_kalman                   # (N,)

        # ── Heading residual ─────────────────────────────────────────────────
        seg_heading_k = self._seg_heading[best]                  # (N,)

        # Handle 180° ambiguity: choose direction of segment closest to robot yaw
        diff_fwd = np.abs(_wrap(yaw - seg_heading_k))
        diff_rev = np.abs(_wrap(yaw - (seg_heading_k + math.pi)))
        eff_heading = np.where(diff_rev < diff_fwd,
                               _wrap(seg_heading_k + math.pi),
                               seg_heading_k)                     # (N,)

        # Expected tape heading in robot frame
        expected_rel_heading = _wrap(eff_heading - yaw)          # (N,)
        res_head = _wrap(line_heading - expected_rel_heading)     # (N,)

        # ── Gaussian likelihoods ─────────────────────────────────────────────
        sl = self.SIGMA_LINE_OFFSET
        sh = self.SIGMA_LINE_HEADING
        w = (np.exp(-0.5 * (res_lat / sl) ** 2) *
             np.exp(-0.5 * (res_head / sh) ** 2))                # (N,)

        # Particles far from tape get floor likelihood
        w = np.where(near_mask, w, self.LIKELIHOOD_FLOOR)

        return np.clip(w, self.LIKELIHOOD_FLOOR, None)

    # ── ArUco likelihood (vectorised over particles) ──────────────────────────

    def _aruco_likelihood(self, marker_id: int,
                          r_obs: float,
                          b_obs: float) -> np.ndarray:
        """
        Compute Gaussian likelihood for an ArUco measurement for all particles.

        Returns (N_particles,) array of likelihoods.
        """
        mkx, mky = self._aruco_kalman[marker_id]

        px  = self._particles[:, 0]
        py  = self._particles[:, 1]
        yaw = self._particles[:, 2]

        dx = mkx - px      # (N,)  forward component
        dy = mky - py      # (N,)  lateral component

        r_pred = np.sqrt(dx ** 2 + dy ** 2)                     # (N,)

        # bearing = angle of marker in robot frame (atan2(y,x) - yaw)
        # y=left convention: +y = left of robot → positive bearing
        b_pred = _wrap(np.arctan2(dy, dx) - yaw)                # (N,)

        r_err = r_obs - r_pred
        b_err = _wrap(b_obs - b_pred)

        sr = self.SIGMA_ARUCO_RANGE
        sb = self.SIGMA_ARUCO_BEARING

        w = (np.exp(-0.5 * (r_err / sr) ** 2) *
             np.exp(-0.5 * (b_err / sb) ** 2))

        return np.clip(w, self.LIKELIHOOD_FLOOR, None)

    # ── Resampling ───────────────────────────────────────────────────────────

    def resample(self, n_new: int = None):
        """
        Systematic resampling.

        Parameters
        ----------
        n_new : target number of particles after resampling.
                Defaults to current particle count.
        """
        if not self.initialized:
            return

        if n_new is None:
            n_new = self._n

        n_new = int(np.clip(n_new, self.MIN_PARTICLES, self.MAX_PARTICLES))

        w = self._weights
        cumsum = np.cumsum(w)
        cumsum[-1] = 1.0  # numerical safety

        # Systematic resampling: one random draw, then evenly spaced
        step = 1.0 / n_new
        r    = np.random.uniform(0.0, step)
        positions = r + step * np.arange(n_new)

        indices = np.searchsorted(cumsum, positions)
        indices = np.clip(indices, 0, self._n - 1)

        self._particles = self._particles[indices].copy()
        self._n         = n_new
        self._weights   = np.full(n_new, 1.0 / n_new)

    # ── Estimate ─────────────────────────────────────────────────────────────

    def get_estimate(self) -> Tuple[float, float, float]:
        """
        Return the weighted mean pose (x, y, yaw).

        Uses circular mean for yaw to handle angle wrapping correctly.

        Returns
        -------
        (x, y, yaw) in Kalman world frame
        """
        if not self.initialized:
            return (0.0, 0.0, 0.0)

        w   = self._weights
        px  = float(np.sum(w * self._particles[:, 0]))
        py  = float(np.sum(w * self._particles[:, 1]))
        yaw = _circular_mean(self._particles[:, 2], w)
        return (px, py, float(yaw))

    def get_best_particle(self) -> Tuple[float, float, float]:
        """Return the highest-weight particle pose."""
        idx = int(np.argmax(self._weights))
        p = self._particles[idx]
        return (float(p[0]), float(p[1]), float(p[2]))

    # ── Properties ───────────────────────────────────────────────────────────

    @property
    def particles(self) -> np.ndarray:
        """Return (N, 3) array of particle poses [x, y, yaw]."""
        return self._particles.copy()

    @property
    def weights(self) -> np.ndarray:
        """Return (N,) array of particle weights (sum = 1)."""
        return self._weights.copy()

    @property
    def n_particles(self) -> int:
        return self._n

    def covariance(self) -> np.ndarray:
        """Weighted 2D position covariance matrix (2×2)."""
        w  = self._weights
        px = self._particles[:, 0]
        py = self._particles[:, 1]
        mx = float(np.sum(w * px))
        my = float(np.sum(w * py))
        dx = px - mx
        dy = py - my
        cxx = float(np.sum(w * dx * dx))
        cxy = float(np.sum(w * dx * dy))
        cyy = float(np.sum(w * dy * dy))
        return np.array([[cxx, cxy], [cxy, cyy]])


# Module-level singleton (mirrors skalman / sline conventions)
amcl = SAMCL()
