#!/usr/bin/env python3
"""
sekf_slam.py
=============================================================================
Vision-based EKF SLAM for the Robobot.

State vector  (3 + 2·N elements):
    [ x,  y,  yaw  |  lx₀, ly₀,  lx₁, ly₁,  …,  lxₙ₋₁, lyₙ₋₁ ]

    x, y   : robot position, Kalman world frame (m)
    yaw    : robot heading (rad, CCW positive, 0 = facing +x)
    lxᵢ, lyᵢ : world position of the i-th discovered landmark (m)

Coordinate frame:
    Kalman world frame — x = forward (0–6 m), y = lateral (0–7 m)

Motion model  (unicycle, same as AMCL):
    x_new   = x   + v · cos(yaw) · dt
    y_new   = y   + v · sin(yaw) · dt
    yaw_new = yaw + ω · dt
    landmarks are static (unchanged by predict)

Measurement model  (range + bearing to landmark i):
    r_i   = √((lxᵢ − x)² + (lyᵢ − y)²)
    φ_i   = atan2(lyᵢ − y, lxᵢ − x) − yaw        (wrapped to [-π, π))

Data association:
    Nearest-neighbour with Mahalanobis-distance gate.
    Observations that fail the gate are initialised as new landmarks
    (state augmentation).

Known landmarks (e.g. ArUco markers with surveyed field positions):
    Supplied as {id: (wx, wy)}.  These are never augmented into the state
    because their world position is fixed; they constrain only the robot pose.

Public API:
    from sekf_slam import SEkfSlam
    slam = SEkfSlam()
    slam.setup(known_landmarks={10: (kx, ky), ...})
    slam.init_pose(x, y, yaw)
    ...
    slam.predict(v, omega, dt)
    slam.update_features(feats)          # from sfeatures.extract()
    slam.update_known(id, range, bearing)  # ArUco with known position
    x, y, yaw, P3 = slam.get_pose()
    landmarks      = slam.get_landmarks()  # list of dicts
=============================================================================
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np


# ── Helpers ───────────────────────────────────────────────────────────────────

def _wrap(a: float) -> float:
    """Wrap scalar angle to [-π, π)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _joseph_update(x: np.ndarray, P: np.ndarray,
                   innov: np.ndarray, H: np.ndarray,
                   R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Joseph-form EKF measurement update.

    Numerically more stable than the standard form P_new = (I-KH)P because
    it preserves positive-semidefiniteness even with floating-point errors.

    Returns (x_new, P_new).
    """
    S = H @ P @ H.T + R
    K = np.linalg.solve(S.T, (P @ H.T).T).T   # K = P·Hᵀ·S⁻¹ via solve
    x_new  = x + K @ innov
    I_KH   = np.eye(len(x)) - K @ H
    P_new  = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x_new, P_new


# ── EKF SLAM ──────────────────────────────────────────────────────────────────

class SEkfSlam:
    """
    Vision-based EKF SLAM: jointly estimates robot pose and visual landmark
    positions from range + bearing measurements.

    All world-frame coordinates use the Kalman convention:
        x = forward (0–6 m)
        y = lateral (0–7 m)
        yaw = CCW positive, 0 = facing +x
    """

    # ── Class-level defaults (overridable via setup()) ────────────────────────

    MAX_LANDMARKS = 150          # hard cap on auto-discovered landmarks

    SIGMA_RANGE   = 0.12         # range measurement noise  (m)
    SIGMA_BEARING = 0.08         # bearing measurement noise (rad)

    # Process noise scales — multiplied by |v|·dt or |ω|·dt so they grow
    # proportionally with motion; tiny floor prevents zero noise at rest.
    Q_X_SCALE   = 2e-4
    Q_Y_SCALE   = 2e-4
    Q_YAW_SCALE = 5e-4
    Q_FLOOR     = 1e-6

    # Mahalanobis chi² gate (2 DOF: 95 % = 5.99, 99 % = 9.21)
    GATE_CHI2 = 9.21

    # Additional Euclidean guard: predicted world distance must be < this (m)
    GATE_EUCL = 0.50

    # Minimum observations before a landmark is shown as "reliable"
    MIN_OBS_RELIABLE = 3

    # Feature range limits (skip implausibly close/far measurements)
    MIN_RANGE = 0.05
    MAX_RANGE = 3.00

    # Field bounds for sanity-checking new landmark candidates (m)
    FX_MIN, FX_MAX = -0.5, 6.5
    FY_MIN, FY_MAX = -0.5, 7.5

    # ── Init ─────────────────────────────────────────────────────────────────

    def __init__(self):
        self._x = np.zeros(3)                              # [x, y, yaw]
        self._P = np.diag([1.0, 1.0, math.pi ** 2])       # 3×3 pose covariance

        self._n_lm:   int       = 0    # number of discovered landmarks
        self._lm_obs: List[int] = []   # observation count per landmark

        # {id: (wx, wy)} — known-position landmarks (ArUco from field map)
        self._known: Dict[int, Tuple[float, float]] = {}

        self.R = np.diag([self.SIGMA_RANGE   ** 2,
                          self.SIGMA_BEARING ** 2])

        self.initialized = False

        # Diagnostics (updated after each update_features / update_known call)
        self.last_matched_features: int = 0
        self.last_new_landmarks:    int = 0

    # ── Setup ─────────────────────────────────────────────────────────────────

    def setup(self,
              max_landmarks:   int   = MAX_LANDMARKS,
              sigma_range:     float = SIGMA_RANGE,
              sigma_bearing:   float = SIGMA_BEARING,
              known_landmarks: Dict[int, Tuple[float, float]] = None):
        """
        Configure the SLAM filter before first use.

        Parameters
        ----------
        max_landmarks    : cap on auto-discovered landmarks (limits state size)
        sigma_range      : 1-σ range noise (m)
        sigma_bearing    : 1-σ bearing noise (rad)
        known_landmarks  : dict {id: (world_x, world_y)} for fixed landmarks
                           (e.g. ArUco markers whose field position is surveyed)
        """
        self.MAX_LANDMARKS = max_landmarks
        self.R = np.diag([sigma_range ** 2, sigma_bearing ** 2])

        if known_landmarks:
            self._known = dict(known_landmarks)

        print(f"% SEkfSlam: setup  max_lm={max_landmarks}  "
              f"s_r={sigma_range:.3f} m  "
              f"s_b={math.degrees(sigma_bearing):.1f} deg  "
              f"known_lm={len(self._known)}")

    # ── Pose init ─────────────────────────────────────────────────────────────

    def init_pose(self, x: float, y: float, yaw: float,
                  spread_xy:  float = 0.30,
                  spread_yaw: float = 0.25):
        """
        Set the initial robot pose and uncertainty.

        Discovered landmarks are kept if any exist; only the pose block
        of the covariance is reset.

        Parameters
        ----------
        x, y, yaw   : initial pose in Kalman world frame
        spread_xy   : 1-σ position uncertainty (m)
        spread_yaw  : 1-σ heading uncertainty (rad)
        """
        n = 3 + 2 * self._n_lm
        self._x[0] = x
        self._x[1] = y
        self._x[2] = yaw

        # Reset pose block of covariance; keep landmark uncertainties intact.
        if self._P.shape[0] != n:
            self._P = np.zeros((n, n))

        self._P[0, 0] = spread_xy  ** 2
        self._P[1, 1] = spread_xy  ** 2
        self._P[2, 2] = spread_yaw ** 2

        self.initialized = True
        print(f"% SEkfSlam: init_pose  x={x:.2f}  y={y:.2f}  "
              f"yaw={math.degrees(yaw):.1f}°  "
              f"spread_xy={spread_xy:.2f}  spread_yaw={math.degrees(spread_yaw):.1f}°")

    # ── Predict ───────────────────────────────────────────────────────────────

    def predict(self, v: float, omega: float, dt: float):
        """
        Unicycle motion model prediction step.

        Parameters
        ----------
        v     : linear speed (m/s, forward = positive)
        omega : yaw rate (rad/s, CCW = positive)
        dt    : time step (s)
        """
        if not self.initialized or dt <= 0.0:
            return

        x, y, yaw = self._x[0], self._x[1], self._x[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        n = len(self._x)

        # State transition (landmarks unchanged)
        self._x[0] += v * cy * dt
        self._x[1] += v * sy * dt
        self._x[2]  = _wrap(yaw + omega * dt)

        # Jacobian F = ∂f/∂x  (n × n)
        F = np.eye(n)
        F[0, 2] = -v * sy * dt   # ∂x_new / ∂yaw
        F[1, 2] =  v * cy * dt   # ∂y_new / ∂yaw

        # Process noise — scaled by motion magnitude
        motion_xy  = max(abs(v)     * dt, self.Q_FLOOR)
        motion_yaw = max(abs(omega) * dt, self.Q_FLOOR)
        Q = np.zeros((n, n))
        Q[0, 0] = self.Q_X_SCALE   * motion_xy
        Q[1, 1] = self.Q_Y_SCALE   * motion_xy
        Q[2, 2] = self.Q_YAW_SCALE * motion_yaw

        self._P = F @ self._P @ F.T + Q

    # ── Update: known landmarks (ArUco with surveyed world position) ──────────

    def update_known(self, landmark_id: int,
                     range_meas:   float,
                     bearing_meas: float):
        """
        EKF update from a landmark with a known world position (e.g. ArUco).

        This constrains the robot pose without augmenting the state vector.

        Parameters
        ----------
        landmark_id   : key into the known-landmarks dict supplied at setup
        range_meas    : observed range (m)
        bearing_meas  : observed bearing (rad, + = left of robot)
        """
        if not self.initialized:
            return
        if landmark_id not in self._known:
            return
        if not (self.MIN_RANGE <= range_meas <= self.MAX_RANGE):
            return

        lx, ly = self._known[landmark_id]
        self._ekf_update(lx, ly, lm_idx=None,
                         range_meas=range_meas, bearing_meas=bearing_meas)

    # ── Update: visual feature landmarks ─────────────────────────────────────

    def update_features(self, features: list) -> Tuple[int, int]:
        """
        Process a batch of range+bearing feature measurements.

        Performs nearest-neighbour Mahalanobis data association and
        augments the state with new landmarks when no match is found.

        Parameters
        ----------
        features : list of dicts from sfeatures.SFeatureExtractor.extract().
                   Required keys: 'range', 'bearing'.

        Returns
        -------
        (n_matched, n_new)  — number of matched and newly added landmarks
        """
        if not self.initialized:
            return 0, 0

        n_matched = 0
        n_new     = 0

        x, y, yaw = self._x[0], self._x[1], self._x[2]

        # Only match against landmarks that already existed before this call.
        # Freshly-created landmarks in this batch are NOT eligible for matching
        # within the same frame — prevents a single physical corner from being
        # detected 10 times and funnelled into 1 landmark with 10× excess confidence.
        n_lm_before = self._n_lm

        for feat in features:
            r_obs = float(feat['range'])
            b_obs = float(feat['bearing'])

            if not (self.MIN_RANGE <= r_obs <= self.MAX_RANGE):
                continue

            # Predicted world position from current pose estimate
            cos_yb = math.cos(b_obs + yaw)
            sin_yb = math.sin(b_obs + yaw)
            wx_pred = x + r_obs * cos_yb
            wy_pred = y + r_obs * sin_yb

            # Field-bounds sanity check
            if not (self.FX_MIN <= wx_pred <= self.FX_MAX and
                    self.FY_MIN <= wy_pred <= self.FY_MAX):
                continue

            # Data association (only search landmarks that existed before this batch)
            best_idx, best_d2 = self._nearest_landmark(r_obs, b_obs,
                                                        max_lm=n_lm_before)

            if best_idx is not None and best_d2 < self.GATE_CHI2:
                # ── Matched an existing landmark — EKF update ─────────────
                base = 3 + 2 * best_idx
                lx   = self._x[base]
                ly   = self._x[base + 1]
                self._ekf_update(lx, ly, lm_idx=best_idx,
                                 range_meas=r_obs, bearing_meas=b_obs)
                self._lm_obs[best_idx] += 1
                n_matched += 1

            elif self._n_lm < self.MAX_LANDMARKS:
                # ── New landmark — augment state ──────────────────────────
                self._add_landmark(wx_pred, wy_pred, r_obs, b_obs)
                n_new += 1

        self.last_matched_features = n_matched
        self.last_new_landmarks    = n_new
        return n_matched, n_new

    # ── Accessors ─────────────────────────────────────────────────────────────

    def get_pose(self) -> Tuple[float, float, float, np.ndarray]:
        """
        Return (x, y, yaw, P_3x3) — robot pose and its 3×3 covariance.
        """
        return (float(self._x[0]),
                float(self._x[1]),
                float(self._x[2]),
                self._P[:3, :3].copy())

    def get_landmarks(self) -> List[dict]:
        """
        Return a list of landmark dicts (all discovered landmarks):
            x, y      : world position (m)
            P         : 2×2 position covariance
            n_obs     : total observation count
            reliable  : True if n_obs ≥ MIN_OBS_RELIABLE
        """
        result = []
        for i in range(self._n_lm):
            base = 3 + 2 * i
            result.append({
                'x':       float(self._x[base]),
                'y':       float(self._x[base + 1]),
                'P':       self._P[base:base + 2, base:base + 2].copy(),
                'n_obs':   self._lm_obs[i],
                'reliable': self._lm_obs[i] >= self.MIN_OBS_RELIABLE,
            })
        return result

    def get_n_landmarks(self) -> int:
        return self._n_lm

    def get_n_reliable_landmarks(self) -> int:
        return sum(1 for n in self._lm_obs if n >= self.MIN_OBS_RELIABLE)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _nearest_landmark(self, r_obs: float, b_obs: float,
                          max_lm: int = None) -> Tuple[Optional[int], float]:
        """
        Nearest-neighbour data association using Mahalanobis distance.

        Returns (best_index, chi2_distance).  If no landmarks exist,
        returns (None, inf).

        Parameters
        ----------
        max_lm : if set, only search among the first max_lm landmarks
                 (used to exclude landmarks added in the current batch).
        """
        n_search = self._n_lm if max_lm is None else min(max_lm, self._n_lm)
        if n_search == 0:
            return None, float('inf')

        x, y, yaw = self._x[0], self._x[1], self._x[2]
        best_idx  = None
        best_d2   = float('inf')

        for i in range(n_search):
            base = 3 + 2 * i
            lx   = self._x[base]
            ly   = self._x[base + 1]

            dx = lx - x
            dy = ly - y
            r_pred = math.sqrt(dx * dx + dy * dy)
            if r_pred < 1e-4:
                continue

            b_pred = _wrap(math.atan2(dy, dx) - yaw)

            # Euclidean guard (cheap first filter)
            wx_obs = x + r_obs * math.cos(b_obs + yaw)
            wy_obs = y + r_obs * math.sin(b_obs + yaw)
            if math.sqrt((wx_obs - lx) ** 2 + (wy_obs - ly) ** 2) > self.GATE_EUCL:
                continue

            innov = np.array([r_obs - r_pred,
                              _wrap(b_obs - b_pred)])

            # Innovation covariance using pose + this landmark block only
            H_pose, H_lm = _meas_jacobians(x, y, yaw, lx, ly, r_pred, dx, dy)
            idx = [0, 1, 2, base, base + 1]
            P_sub = self._P[np.ix_(idx, idx)]
            H_sub = np.hstack([H_pose, H_lm])   # (2, 5)
            S = H_sub @ P_sub @ H_sub.T + self.R

            try:
                d2 = float(innov @ np.linalg.solve(S, innov))
            except np.linalg.LinAlgError:
                continue

            if d2 < best_d2:
                best_d2  = d2
                best_idx = i

        return best_idx, best_d2

    def _ekf_update(self, lx: float, ly: float,
                    lm_idx:      Optional[int],
                    range_meas:  float,
                    bearing_meas: float):
        """
        Iterative full-state Joseph-form EKF update for a single range+bearing
        observation. Performs multiple iterations for better linearization.

        lm_idx=None  → the landmark world position is fixed (known landmark).
                        Only the 3 robot-pose rows/cols of H are non-zero.
        lm_idx=i     → auto-discovered landmark at state index 3+2i.
        """
        max_iter = 3  # Number of iterations
        for iteration in range(max_iter):
            x, y, yaw = self._x[0], self._x[1], self._x[2]
            dx = lx - x
            dy = ly - y
            r_pred = math.sqrt(dx * dx + dy * dy)
            if r_pred < 1e-4:
                return

            b_pred = _wrap(math.atan2(dy, dx) - yaw)
            innov  = np.array([range_meas - r_pred,
                               _wrap(bearing_meas - b_pred)])

            n = len(self._x)
            H = np.zeros((2, n))
            H_pose, H_lm = _meas_jacobians(x, y, yaw, lx, ly, r_pred, dx, dy)
            H[:, :3] = H_pose

            if lm_idx is not None:
                base = 3 + 2 * lm_idx
                H[:, base:base + 2] = H_lm

            self._x, self._P = _joseph_update(self._x, self._P, innov, H, self.R)
            self._x[2] = _wrap(self._x[2])

    def _add_landmark(self, wx: float, wy: float,
                      r_obs: float, b_obs: float):
        """
        Augment the state vector with a new landmark at (wx, wy).

        The landmark's initial covariance is propagated from the current
        pose uncertainty using the inverse observation function Jacobian
        (Dissanayake et al., 2001).
        """
        n = len(self._x)

        # Augment state
        x_new = np.empty(n + 2)
        x_new[:n] = self._x
        x_new[n]  = wx
        x_new[n + 1] = wy

        # Jacobian of the inverse observation function
        #   [wx, wy] = [x + r·cos(b+yaw),  y + r·sin(b+yaw)]
        # G_r = ∂[wx, wy] / ∂[x, y, yaw]   (2×3)
        # G_z = ∂[wx, wy] / ∂[r,  b  ]     (2×2)
        cyb = math.cos(b_obs + self._x[2])
        syb = math.sin(b_obs + self._x[2])
        G_r = np.array([[1.0, 0.0, -r_obs * syb],
                         [0.0, 1.0,  r_obs * cyb]])
        G_z = np.array([[cyb, -r_obs * syb],
                         [syb,  r_obs * cyb]])

        # Augmented covariance
        # P_new[0:n, 0:n]     = P  (unchanged)
        # P_new[n:,  0:n]     = G_r @ P[0:3, :]            (cross-cov lm↔old)
        # P_new[0:n, n:]      = (G_r @ P[0:3, :]).T
        # P_new[n:,  n:]      = G_r @ P[0:3,0:3] @ G_r.T + G_z @ R @ G_z.T
        P_new = np.zeros((n + 2, n + 2))
        P_new[:n, :n]   = self._P
        cross           = G_r @ self._P[:3, :]       # (2, n)
        P_new[n:, :n]   = cross
        P_new[:n, n:]   = cross.T
        P_rr            = self._P[:3, :3]
        P_new[n:, n:]   = G_r @ P_rr @ G_r.T + G_z @ self.R @ G_z.T

        self._x   = x_new
        self._P   = P_new
        self._n_lm += 1
        self._lm_obs.append(1)


# ── Measurement Jacobians (module-level for reuse) ────────────────────────────

def _meas_jacobians(x: float, y: float, yaw: float,
                    lx: float, ly: float,
                    r: float, dx: float,
                    dy: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute H_pose (2×3) and H_lm (2×2) for a range+bearing measurement.

    Range row:    ∂r/∂(x,y,yaw),  ∂r/∂(lx,ly)
    Bearing row:  ∂φ/∂(x,y,yaw),  ∂φ/∂(lx,ly)
    """
    r2 = r * r
    H_pose = np.array([
        [-dx / r,  -dy / r,   0.0],
        [ dy / r2, -dx / r2, -1.0],
    ])
    H_lm = np.array([
        [ dx / r,   dy / r],
        [-dy / r2,  dx / r2],
    ])
    return H_pose, H_lm
