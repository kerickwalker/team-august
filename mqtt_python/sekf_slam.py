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
    SIGMA_Z       = 0.05         # height noise from tape-width estimate (m)

    # Process noise scales — multiplied by |v|·dt or |ω|·dt so they grow
    # proportionally with motion; tiny floor prevents zero noise at rest.
    Q_X_SCALE   = 2e-4
    Q_Y_SCALE   = 2e-4
    Q_YAW_SCALE = 5e-4
    Q_FLOOR     = 1e-6

    # Mahalanobis chi² gate (2 DOF: 95 % = 5.99, 99 % = 9.21)
    GATE_CHI2 = 9.21
    # 3 DOF gate for range+bearing+Z matching (99 % = 11.35)
    GATE_CHI2_3D = 11.35

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
    FZ_MIN, FZ_MAX = -0.40, 0.40  # world Z sanity bounds for new landmarks (m)

    # ── Init ─────────────────────────────────────────────────────────────────

    def __init__(self):
        self._x = np.zeros(3)                              # [x, y, yaw]
        self._P = np.diag([1.0, 1.0, math.pi ** 2])       # 3×3 pose covariance

        self._n_lm:   int       = 0    # number of discovered landmarks
        self._lm_obs: List[int] = []   # observation count per landmark

        # {id: (wx, wy)} — fixed-position landmarks (ArUco from field map)
        self._known: Dict[int, Tuple[float, float]] = {}

        # {aruco_id: lm_state_index} — ArUco as SLAM landmarks (unknown pos)
        self._aruco_lm: Dict[int, int] = {}

        # When True: use loaded map as-is, never augment, only update pose
        self.localize_only: bool = False

        self.R    = np.diag([self.SIGMA_RANGE   ** 2,
                             self.SIGMA_BEARING ** 2])
        self.R_3d = np.diag([self.SIGMA_RANGE   ** 2,
                             self.SIGMA_BEARING ** 2,
                             self.SIGMA_Z       ** 2])

        self.initialized = False

        # Diagnostics (updated after each update_features / update_known call)
        self.last_matched_features: int = 0
        self.last_new_landmarks:    int = 0

    # ── Setup ─────────────────────────────────────────────────────────────────

    def setup(self,
              max_landmarks:   int   = MAX_LANDMARKS,
              sigma_range:     float = SIGMA_RANGE,
              sigma_bearing:   float = SIGMA_BEARING,
              sigma_z:         float = SIGMA_Z,
              known_landmarks: Dict[int, Tuple[float, float]] = None):
        """
        Configure the SLAM filter before first use.

        Parameters
        ----------
        max_landmarks    : cap on auto-discovered landmarks (limits state size)
        sigma_range      : 1-sigma range noise (m)
        sigma_bearing    : 1-sigma bearing noise (rad)
        sigma_z          : 1-sigma height noise (m) for tape-width Z estimate
        known_landmarks  : dict {id: (world_x, world_y)} for fixed landmarks
                           (e.g. ArUco markers whose field position is surveyed)
        """
        self.MAX_LANDMARKS = max_landmarks
        self.R    = np.diag([sigma_range ** 2, sigma_bearing ** 2])
        self.R_3d = np.diag([sigma_range ** 2, sigma_bearing ** 2, sigma_z ** 2])

        if known_landmarks:
            self._known = dict(known_landmarks)

        print(f"% SEkfSlam: setup  max_lm={max_landmarks}  "
              f"s_r={sigma_range:.3f} m  "
              f"s_b={math.degrees(sigma_bearing):.1f} deg  "
              f"s_z={sigma_z:.3f} m  "
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
        n = 3 + 3 * self._n_lm
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

    # ── Update: external pose estimate (e.g. Kalman EKF from encoders+IMU) ───

    def update_kalman_pose(self,
                           kx:         float,
                           ky:         float,
                           kyaw:       float,
                           sigma_xy:   float = 0.10,
                           sigma_yaw:  float = 0.08):
        """
        Fuse an external (encoder + IMU) pose estimate into the SLAM state.

        Treats the supplied pose as a direct noisy observation of the robot's
        (x, y, yaw) states.  Measurement model is linear:

            z = H · state + noise,   H = [I₃ | 0_{3×2N}]
            R = diag(sigma_xy², sigma_xy², sigma_yaw²)

        Use this every frame when the on-board Kalman EKF estimate is
        available (from kalman_log.jsonl replay or live MQTT).  The visual
        landmark updates then act as a *correction* layer: if the wheels slip,
        the fixed landmarks pull the pose back from the drifted Kalman estimate.

        Parameters
        ----------
        kx, ky, kyaw : Kalman EKF pose in the Kalman world frame (m, m, rad)
        sigma_xy     : position uncertainty of the external estimate (m)
        sigma_yaw    : heading uncertainty (rad)
        """
        if not self.initialized:
            return

        n = len(self._x)
        H = np.zeros((3, n))
        H[0, 0] = 1.0   # x
        H[1, 1] = 1.0   # y
        H[2, 2] = 1.0   # yaw

        innov = np.array([
            kx   - float(self._x[0]),
            ky   - float(self._x[1]),
            _wrap(kyaw - float(self._x[2])),
        ])

        R_pose = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2])
        self._x, self._P = _joseph_update(self._x, self._P, innov, H, R_pose)
        self._x[2] = _wrap(float(self._x[2]))

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
        self._ekf_update(lx, ly, 0.0, lm_idx=None,
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

            z_obs = float(feat.get('Z', 0.0))

            # Data association (only search landmarks that existed before this batch)
            best_idx, best_d2 = self._nearest_landmark(r_obs, b_obs,
                                                        z_obs=z_obs,
                                                        max_lm=n_lm_before)

            if best_idx is not None and best_d2 < self.GATE_CHI2_3D:
                # ── Matched landmark — EKF update ─────────────────────────
                base = 3 + 3 * best_idx
                lx   = self._x[base]
                ly   = self._x[base + 1]
                lz   = self._x[base + 2]
                # In localize_only: treat landmark pos as fixed (pose update only)
                eff_idx = None if self.localize_only else best_idx
                self._ekf_update(lx, ly, lz, lm_idx=eff_idx,
                                 range_meas=r_obs, bearing_meas=b_obs,
                                 z_obs=z_obs)
                self._lm_obs[best_idx] += 1
                n_matched += 1

            elif not self.localize_only and self._n_lm < self.MAX_LANDMARKS:
                # ── New landmark — augment state (mapping only) ───────────
                wz = z_obs if (self.FZ_MIN <= z_obs <= self.FZ_MAX) else 0.0
                self._add_landmark(wx_pred, wy_pred, wz, r_obs, b_obs, z_obs)
                n_new += 1

        self.last_matched_features = n_matched
        self.last_new_landmarks    = n_new
        return n_matched, n_new

    # ── Update: ArUco as SLAM landmarks (unknown position) ───────────────────

    def update_aruco_slam(self, aruco_id: int,
                          range_meas:   float,
                          bearing_meas: float):
        """
        Process an ArUco detection as a SLAM landmark with unknown position.

        ArUco IDs provide exact data association — no Mahalanobis gate needed.
        On first observation the landmark is augmented into the state.
        On subsequent observations the EKF update corrects both the robot
        pose and (in mapping mode) the landmark's estimated world position.

        In localize_only mode the landmark positions are frozen; only the
        robot pose is updated.

        Parameters
        ----------
        aruco_id      : ArUco marker ID (unique integer)
        range_meas    : measured range (m)
        bearing_meas  : measured bearing (rad, + = left of robot)
        """
        if not self.initialized:
            return
        if not (self.MIN_RANGE <= range_meas <= self.MAX_RANGE):
            return

        if aruco_id in self._aruco_lm:
            # Already in map — EKF update (2D range+bearing; ArUco has no Z meas)
            idx  = self._aruco_lm[aruco_id]
            base = 3 + 3 * idx
            lx   = self._x[base]
            ly   = self._x[base + 1]
            lz   = self._x[base + 2]
            eff_idx = None if self.localize_only else idx
            self._ekf_update(lx, ly, lz, lm_idx=eff_idx,
                             range_meas=range_meas, bearing_meas=bearing_meas)
            self._lm_obs[idx] += 1

        elif not self.localize_only and self._n_lm < self.MAX_LANDMARKS:
            # New ArUco landmark — augment state at Z=0 (floor level)
            x, y, yaw = self._x[0], self._x[1], self._x[2]
            cyb = math.cos(bearing_meas + yaw)
            syb = math.sin(bearing_meas + yaw)
            wx  = x + range_meas * cyb
            wy  = y + range_meas * syb
            if (self.FX_MIN <= wx <= self.FX_MAX and
                    self.FY_MIN <= wy <= self.FY_MAX):
                self._aruco_lm[aruco_id] = self._n_lm
                self._add_landmark(wx, wy, 0.0, range_meas, bearing_meas, z_obs=0.0)

    # ── Map persistence ───────────────────────────────────────────────────────

    def save_map(self, path: str):
        """
        Save the current landmark map to a JSON file.

        Includes robot pose at save time, all landmark world positions,
        their observation counts, and which are ArUco vs visual features.
        """
        import json as _json
        idx_to_aruco = {v: k for k, v in self._aruco_lm.items()}
        lm_list = []
        for i in range(self._n_lm):
            base = 3 + 3 * i
            lm_list.append({
                'x':        round(float(self._x[base]),     5),
                'y':        round(float(self._x[base + 1]), 5),
                'z':        round(float(self._x[base + 2]), 5),
                'n_obs':    self._lm_obs[i],
                'aruco_id': idx_to_aruco.get(i),          # None for visual
                'P': [[round(float(v), 6) for v in row]
                      for row in self._P[base:base+3, base:base+3].tolist()],
            })
        data = {
            'pose':      [round(float(self._x[0]), 4),
                          round(float(self._x[1]), 4),
                          round(float(self._x[2]), 4)],
            'landmarks': lm_list,
        }
        with open(path, 'w') as fh:
            _json.dump(data, fh, indent=2)
        n_aruco  = len(self._aruco_lm)
        n_visual = self._n_lm - n_aruco
        print(f"% SEkfSlam: map saved  {self._n_lm} landmarks "
              f"(ArUco:{n_aruco} Visual:{n_visual}) -> {path}")

    def load_map(self, path: str):
        """
        Load a previously saved landmark map.

        All landmark positions are treated as known (no further augmentation).
        The robot pose is reset to high uncertainty so the first measurements
        will re-localise the robot within the loaded map.

        Sets localize_only = True automatically.
        """
        import json as _json
        with open(path) as fh:
            data = _json.load(fh)
        lm_list = data.get('landmarks', [])
        n = len(lm_list)

        # Rebuild state vector (3D landmarks: x, y, z per landmark)
        x_new = np.zeros(3 + 3 * n)
        saved_pose = data.get('pose', [0.0, 0.0, 0.0])
        x_new[0], x_new[1], x_new[2] = saved_pose

        # Large pose uncertainty — let measurements re-localise
        P_new = np.zeros((3 + 3 * n, 3 + 3 * n))
        P_new[0, 0] = 1.5 ** 2
        P_new[1, 1] = 1.5 ** 2
        P_new[2, 2] = math.pi ** 2

        self._aruco_lm = {}
        self._lm_obs   = []

        for i, lm in enumerate(lm_list):
            base = 3 + 3 * i
            x_new[base]     = float(lm['x'])
            x_new[base + 1] = float(lm['y'])
            x_new[base + 2] = float(lm.get('z', 0.0))   # backward-compatible
            P_lm = np.array(lm['P'], dtype=float)
            if P_lm.shape == (2, 2):
                # Upgrade old 2D map: extend to 3x3 with sigma_z on diagonal
                P_lm_3 = np.zeros((3, 3))
                P_lm_3[:2, :2] = P_lm
                P_lm_3[2, 2]   = self.SIGMA_Z ** 2
                P_lm = P_lm_3
            P_new[base:base+3, base:base+3] = P_lm
            self._lm_obs.append(int(lm.get('n_obs', 0)))
            aid = lm.get('aruco_id')
            if aid is not None:
                self._aruco_lm[int(aid)] = i

        self._x            = x_new
        self._P            = P_new
        self._n_lm         = n
        self.initialized   = True
        self.localize_only = True

        n_aruco  = len(self._aruco_lm)
        n_visual = n - n_aruco
        print(f"% SEkfSlam: map loaded  {n} landmarks "
              f"(ArUco:{n_aruco} Visual:{n_visual}) <- {path}")

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
            reliable  : True if n_obs >= MIN_OBS_RELIABLE
            aruco_id  : int if ArUco landmark, None if visual feature
        """
        idx_to_aruco = {v: k for k, v in self._aruco_lm.items()}
        result = []
        for i in range(self._n_lm):
            base = 3 + 3 * i
            result.append({
                'x':        float(self._x[base]),
                'y':        float(self._x[base + 1]),
                'z':        float(self._x[base + 2]),
                'P':        self._P[base:base + 3, base:base + 3].copy(),
                'n_obs':    self._lm_obs[i],
                'reliable': self._lm_obs[i] >= self.MIN_OBS_RELIABLE,
                'aruco_id': idx_to_aruco.get(i),
            })
        return result

    def get_n_landmarks(self) -> int:
        return self._n_lm

    def get_n_reliable_landmarks(self) -> int:
        return sum(1 for n in self._lm_obs if n >= self.MIN_OBS_RELIABLE)

    def get_n_aruco_landmarks(self) -> int:
        return len(self._aruco_lm)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _nearest_landmark(self, r_obs: float, b_obs: float,
                          z_obs: float = None,
                          max_lm: int = None) -> Tuple[Optional[int], float]:
        """
        Nearest-neighbour data association using Mahalanobis distance.

        When z_obs is provided (tape-corner features), uses 3D matching
        (range, bearing, Z) so that features at different heights are
        correctly distinguished, even when horizontally co-located.

        Returns (best_index, chi2_distance).  If no landmarks exist,
        returns (None, inf).

        Parameters
        ----------
        z_obs  : measured world-frame Z of the feature (m), or None for 2D
        max_lm : if set, only search among the first max_lm landmarks
                 (used to exclude landmarks added in the current batch).
        """
        n_search = self._n_lm if max_lm is None else min(max_lm, self._n_lm)
        if n_search == 0:
            return None, float('inf')

        use_3d = z_obs is not None
        x, y, yaw = self._x[0], self._x[1], self._x[2]
        best_idx  = None
        best_d2   = float('inf')

        for i in range(n_search):
            base = 3 + 3 * i
            lx   = self._x[base]
            ly   = self._x[base + 1]
            lz   = self._x[base + 2]

            dx = lx - x
            dy = ly - y
            r_pred = math.sqrt(dx * dx + dy * dy)
            if r_pred < 1e-4:
                continue

            b_pred = _wrap(math.atan2(dy, dx) - yaw)

            # Euclidean guard in 3D (cheap first filter)
            wx_obs = x + r_obs * math.cos(b_obs + yaw)
            wy_obs = y + r_obs * math.sin(b_obs + yaw)
            wz_obs = z_obs if use_3d else lz
            eucl_d = math.sqrt((wx_obs - lx) ** 2 + (wy_obs - ly) ** 2
                               + (wz_obs - lz) ** 2)
            if eucl_d > self.GATE_EUCL:
                continue

            H_pose, H_lm_2d = _meas_jacobians(x, y, yaw, lx, ly, r_pred, dx, dy)

            if use_3d:
                # 3D innovation: [range, bearing, z]
                innov = np.array([r_obs - r_pred,
                                  _wrap(b_obs - b_pred),
                                  z_obs - lz])
                # H_lm_3d: H_lm_2d is already (2×3); add z row [0, 0, 1]
                H_lm_3d = np.zeros((3, 3))
                H_lm_3d[:2, :] = H_lm_2d
                H_lm_3d[2, 2]  = 1.0
                H_pose_3d = np.zeros((3, 3))
                H_pose_3d[:2, :] = H_pose
                idx = [0, 1, 2, base, base + 1, base + 2]
                P_sub = self._P[np.ix_(idx, idx)]
                H_sub = np.hstack([H_pose_3d, H_lm_3d])   # (3, 6)
                S = H_sub @ P_sub @ H_sub.T + self.R_3d
            else:
                innov = np.array([r_obs - r_pred,
                                  _wrap(b_obs - b_pred)])
                idx = [0, 1, 2, base, base + 1, base + 2]
                P_sub = self._P[np.ix_(idx, idx)]
                H_sub = np.hstack([H_pose, H_lm_2d])   # (2, 6)
                S = H_sub @ P_sub @ H_sub.T + self.R

            try:
                d2 = float(innov @ np.linalg.solve(S, innov))
            except np.linalg.LinAlgError:
                continue

            if d2 < best_d2:
                best_d2  = d2
                best_idx = i

        return best_idx, best_d2

    def _ekf_update(self, lx: float, ly: float, lz: float,
                    lm_idx:       Optional[int],
                    range_meas:   float,
                    bearing_meas: float,
                    z_obs:        Optional[float] = None):
        """
        Full-state Joseph-form EKF update for a single range+bearing
        observation.

        lm_idx=None  -> the landmark world position is fixed (known landmark).
                        Only the 3 robot-pose rows/cols of H are non-zero.
        lm_idx=i     -> auto-discovered landmark at state index 3+3i.
        z_obs        -> if provided (and lm_idx is not None), performs a 3D
                        update that also refines the landmark's Z state.
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

    def _add_landmark(self, wx: float, wy: float, wz: float,
                      r_obs: float, b_obs: float, z_obs: float = 0.0):
        """
        Augment the state vector with a new 3D landmark at (wx, wy, wz).

        The landmark's initial covariance is propagated from the current
        pose uncertainty using the inverse observation function Jacobian
        (Dissanayake et al., 2001), extended to 3 dimensions.

        World-frame position:
            wx = x + r*cos(b+yaw)   (horizontal, from range+bearing)
            wy = y + r*sin(b+yaw)
            wz = z_obs              (direct height observation from tape width)
        """
        n = len(self._x)

        # Augment state
        x_new = np.empty(n + 3)
        x_new[:n]    = self._x
        x_new[n]     = wx
        x_new[n + 1] = wy
        x_new[n + 2] = wz

        cyb = math.cos(b_obs + self._x[2])
        syb = math.sin(b_obs + self._x[2])

        # G_r = d(wx,wy,wz)/d(robot x,y,yaw)  (3x3)
        # wz is directly observed so its row is all zeros
        G_r = np.array([[1.0, 0.0, -r_obs * syb],
                        [0.0, 1.0,  r_obs * cyb],
                        [0.0, 0.0,  0.0        ]])

        # G_z = d(wx,wy,wz)/d(measurement r,b,z)  (3x3)
        G_z = np.array([[cyb, -r_obs * syb, 0.0],
                        [syb,  r_obs * cyb, 0.0],
                        [0.0,  0.0,         1.0]])

        # Augmented covariance (3x3 landmark block)
        P_new = np.zeros((n + 3, n + 3))
        P_new[:n, :n]  = self._P
        cross          = G_r @ self._P[:3, :]       # (3, n)
        P_new[n:, :n]  = cross
        P_new[:n, n:]  = cross.T
        P_rr           = self._P[:3, :3]
        P_new[n:, n:]  = G_r @ P_rr @ G_r.T + G_z @ self.R_3d @ G_z.T

        self._x    = x_new
        self._P    = P_new
        self._n_lm += 1
        self._lm_obs.append(1)


# ── Measurement Jacobians (module-level for reuse) ────────────────────────────

def _meas_jacobians(x: float, y: float, yaw: float,
                    lx: float, ly: float,
                    r: float, dx: float,
                    dy: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute H_pose (2x3) and H_lm (2x3) for a horizontal range+bearing
    measurement against a 3D landmark (lx, ly, lz).

    Range row:    d(r)/d(x,y,yaw),   d(r)/d(lx,ly,lz)
    Bearing row:  d(phi)/d(x,y,yaw), d(phi)/d(lx,ly,lz)

    Note: horizontal range and bearing do not depend on lz, so the
    third column of H_lm is always zero.
    """
    r2 = r * r
    H_pose = np.array([
        [-dx / r,  -dy / r,   0.0],
        [ dy / r2, -dx / r2, -1.0],
    ])
    H_lm = np.array([
        [ dx / r,   dy / r,  0.0],
        [-dy / r2,  dx / r2, 0.0],
    ])
    return H_pose, H_lm
