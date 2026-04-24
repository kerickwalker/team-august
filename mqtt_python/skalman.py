#!/usr/bin/env python3
"""
Two-stage IMU EKF for robobot pose estimation.

Stage 1 – AHRS EKF  (6 states)
   State: [roll, pitch, yaw, bias_gx, bias_gy, bias_gz]
   Predict : Euler-angle rate kinematics driven by corrected gyro at ~100 Hz.
   Measure : accelerometer  → roll + pitch (gravity tilt)
             tilt-compensated magnetometer → yaw

Stage 2 – Pose EKF  (7 states)
   State: [x, y, v, omega, yaw, z, pitch]
   Predict : 3-D unicycle model; omega from AHRS yaw-rate; z integrated via pitch.
   Measure : encoder odometry → x, y, v, omega, yaw
             AHRS yaw         → yaw   (tight coupling at gyro rate)
             AHRS pitch       → pitch (tight coupling at gyro rate)
             vision pose      → x, y, yaw, z, pitch

Public API (SKalman) is backward-compatible with the old linear KF and
returns a 7-element list:  [x, y, gyro_bias_z, v, omega, yaw, pitch]
"""

from datetime import datetime
import json
import math

import numpy as np

GRAVITY = 9.82   # m/s²


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _unwrap_near(meas: float, ref: float) -> float:
    """Shift meas by ±2π so it is closest to ref."""
    return ref + _wrap(meas - ref)


def _joseph_update(x, P, z, h, H, R):
    """
    Joseph-form EKF scalar / vector measurement update.

    Parameters
    ----------
    x, P  : current state and covariance
    z, h  : measurement and predicted measurement
    H     : measurement Jacobian  (m × n)
    R     : measurement noise covariance  (m × m)

    Returns
    -------
    x_new, P_new
    """
    y = z - h                                   # innovation
    S = H @ P @ H.T + R
    K = np.linalg.solve(S.T, (P @ H.T).T).T    # K = P Hᵀ S⁻¹  (via solve)
    x_new = x + K @ y
    I_KH  = np.eye(len(x)) - K @ H
    P_new = I_KH @ P @ I_KH.T + K @ R @ K.T   # Joseph form
    return x_new, P_new


# ===========================================================================
# Stage 1 – AHRS EKF
# ===========================================================================
# State  idx  description
#  roll   0   [rad]   roll  (rotation around forward axis)
#  pitch  1   [rad]   pitch (rotation around lateral axis)
#  yaw    2   [rad]   heading (world frame, CCW positive)
#  bx     3   [rad/s] gyro x-axis bias
#  by     4   [rad/s] gyro y-axis bias
#  bz     5   [rad/s] gyro z-axis bias

class _AHRS_EKF:
    """
    Euler-angle AHRS EKF driven by 9-axis IMU.

    Provides accurate roll, pitch, yaw, and full gyro bias vector.
    Attitude singularity at pitch = ±90° is guarded with a 0.1 rad margin.
    """

    ROLL, PITCH, YAW, BX, BY, BZ = 0, 1, 2, 3, 4, 5
    N = 6

    def __init__(self):
        self.x = np.zeros(self.N)
        self.P = np.diag([0.1, 0.1, 1.0, 0.01, 0.01, 0.01])

        # Process noise
        self.Q = np.diag([
            1e-5,   # roll  — slow covariance growth, trust gyro integration
            1e-5,   # pitch
            3e-5,   # yaw
            1e-6,   # bx — raised 10× so accel can correct X-gyro bias (prevents roll drift)
            1e-6,   # by — raised 10× so accel can correct Y-gyro bias (prevents pitch drift)
            1e-7,   # bz
        ])

        # Measurement noise
        self.R_acc = np.diag([0.08, 0.06])   # restored: EMA (alpha=0.05) handles output jitter
        self.R_mag = np.array([[0.60]])       # was 0.20          — trust mag less
        self._mag_offset = None               # calibration offset

    # ---------------------------------------------------------------- predict

    def predict(self, gx: float, gy: float, gz: float, dt: float):
        """
        Predict attitude from raw (biased) gyro readings.

        Uses Euler-angle rate kinematics:
          roll_dot  = wx + (wy·sin(roll) + wz·cos(roll))·tan(pitch)
          pitch_dot = wy·cos(roll) - wz·sin(roll)
          yaw_dot   = (wy·sin(roll) + wz·cos(roll)) / cos(pitch)
        where wx,wy,wz = gx-bx, gy-by, gz-bz  (bias-corrected).
        """
        roll, pitch, yaw, bx, by, bz = self.x

        wx = gx - bx
        wy = gy - by
        wz = gz - bz

        # Guard against gimbal-lock singularity (pitch ≈ ±90°)
        cp = math.cos(pitch)
        if abs(cp) < 0.05:
            cp = math.copysign(0.05, cp)
        tp = math.sin(pitch) / cp   # tan(pitch)

        sr, cr = math.sin(roll), math.cos(roll)

        roll_dot  = wx + (wy * sr + wz * cr) * tp
        pitch_dot = wy * cr - wz * sr
        yaw_dot   = (wy * sr + wz * cr) / cp

        xn = np.array([
            _wrap(roll  + roll_dot  * dt),
            _wrap(pitch + pitch_dot * dt),
            _wrap(yaw   + yaw_dot   * dt),
            bx,
            by,
            bz,
        ])

        # Analytical Jacobian  F = ∂f/∂x  (bias rows are identity)
        #  ∂roll_dot / ∂roll   = (wy·cr - wz·sr)·tp
        #  ∂roll_dot / ∂pitch  = (wy·sr + wz·cr) / cp²
        #  ∂pitch_dot/ ∂roll   = -wy·sr - wz·cr
        #  ∂yaw_dot  / ∂roll   = (wy·cr - wz·sr) / cp
        #  ∂yaw_dot  / ∂pitch  = (wy·sr + wz·cr)·sin(pitch) / cp²

        sp = math.sin(pitch)
        F = np.eye(self.N)
        # roll row
        F[0, 0] = 1.0 + (wy * cr - wz * sr) * tp * dt
        F[0, 1] = (wy * sr + wz * cr) / (cp * cp) * dt
        F[0, 3] = -dt
        F[0, 4] = -sr * tp * dt
        F[0, 5] = -cr * tp * dt
        # pitch row
        F[1, 0] = (-wy * sr - wz * cr) * dt
        F[1, 4] = -cr * dt
        F[1, 5] =  sr * dt
        # yaw row
        F[2, 0] = (wy * cr - wz * sr) / cp * dt
        F[2, 1] = (wy * sr + wz * cr) * sp / (cp * cp) * dt
        F[2, 4] = -sr / cp * dt
        F[2, 5] = -cr / cp * dt

        self.x = xn
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------ measurement: acc → attitude

    def update_acc(self, ax: float, ay: float, az: float):
        """
        Update roll and pitch from accelerometer.
        Valid only when the robot is not accelerating strongly.
        """
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 0.5 * GRAVITY or norm > 2.0 * GRAVITY:
            return   # excessive linear acceleration — skip
        roll_m  = math.atan2(ay, math.sqrt(ax * ax + az * az))
        pitch_m = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        H = np.zeros((2, self.N))
        H[0, self.ROLL]  = 1.0
        H[1, self.PITCH] = 1.0
        h = np.array([self.x[self.ROLL], self.x[self.PITCH]])
        z = np.array([
            _unwrap_near(roll_m,  self.x[self.ROLL]),
            _unwrap_near(pitch_m, self.x[self.PITCH]),
        ])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_acc)
        self.x[self.ROLL]  = _wrap(self.x[self.ROLL])
        self.x[self.PITCH] = _wrap(self.x[self.PITCH])

    # ------------------------------------------------ measurement: mag → yaw

    def update_mag(self, mx: float, my: float, mz: float):
        """Tilt-compensated magnetometer yaw update."""
        roll  = self.x[self.ROLL]
        pitch = self.x[self.PITCH]
        # Rotate mag into horizontal plane
        mx_h = (mx * math.cos(pitch)
                + my * math.sin(pitch) * math.sin(roll)
                + mz * math.sin(pitch) * math.cos(roll))
        my_h = my * math.cos(roll) - mz * math.sin(roll)
        if abs(mx_h) < 1e-9 and abs(my_h) < 1e-9:
            return
        yaw_raw = math.atan2(-my_h, mx_h)
        if self._mag_offset is None:
            self._mag_offset = yaw_raw - self.x[self.YAW]
        yaw_m = _wrap(yaw_raw - self._mag_offset)

        H = np.zeros((1, self.N))
        H[0, self.YAW] = 1.0
        h = np.array([self.x[self.YAW]])
        z = np.array([_unwrap_near(yaw_m, self.x[self.YAW])])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_mag)
        self.x[self.YAW] = _wrap(self.x[self.YAW])

    # ------------------------------------------------------------- properties

    @property
    def roll(self):  return float(self.x[self.ROLL])
    @property
    def pitch(self): return float(self.x[self.PITCH])
    @property
    def yaw(self):   return float(self.x[self.YAW])
    @property
    def bias(self):  return self.x[self.BX:self.BZ + 1].tolist()   # [bx,by,bz]


# ===========================================================================
# Stage 2 – Pose EKF
# ===========================================================================
# State  idx  description
#  x      0   [m]     world-frame x position
#  y      1   [m]     world-frame y position
#  v      2   [m/s]   forward speed
#  omega  3   [rad/s] yaw rate (debiased, from AHRS)
#  yaw    4   [rad]   heading (world frame)
#  z      5   [m]     vertical position (integrated from v·sin(pitch))
#  pitch  6   [rad]   pitch angle (tightly coupled from AHRS)

class _PoseEKF:
    """
    Full 3-D unicycle motion-model EKF for position + velocity.

    Heading and pitch are tightly coupled to the AHRS EKF output at
    ~100 Hz.  Vertical position z is integrated from forward speed
    projected through pitch.  Encoder odometry and vision provide
    absolute position corrections.
    """

    X, Y, V, OM, YAW, Z, PITCH = 0, 1, 2, 3, 4, 5, 6
    N = 7

    def __init__(self):
        self.x = np.zeros(self.N)
        self.P = np.diag([1.0, 1.0, 0.5, 0.5, 1.0, 0.5, 0.3])

        # Process noise
        self.Q = np.diag([
            2e-4,   # x
            2e-4,   # y
            0.04,   # v      (model uncertainty)
            0.04,   # omega  (directly set from AHRS; uncertainty resets each step)
            5e-4,   # yaw
            5e-3,   # z      (larger – allows slope integration to accumulate)
            1e-4,   # pitch  (slow variation; tight-coupling update follows)
        ])

        # Measurement noise
        self.R_enc_pose   = np.diag([0.02, 0.02, 0.04])           # x, y, yaw
        self.R_enc_vel    = np.diag([0.05, 0.05])                  # v, omega
        self.R_ahrs_yaw   = np.array([[0.10]])   # was 0.02 — less yaw jitter
        self.R_ahrs_pitch = np.array([[0.30]])   # was 0.10 — extra damping for noisy pitch
        self.R_vision     = np.diag([0.05, 0.05, 0.08, 0.10, 0.06])  # x,y,yaw,z,pitch

    # ---------------------------------------------------------------- predict

    def predict(self, omega_ahrs: float, pitch_ahrs: float, dt: float):
        """
        3-D unicycle prediction driven by bias-corrected AHRS rates.

        Parameters
        ----------
        omega_ahrs  : debiased yaw rate from AHRS  [rad/s]
        pitch_ahrs  : current AHRS pitch estimate   [rad]
        dt          : time step                     [s]
        """
        x, y, v, omega, yaw, z, pitch = self.x
        cy, sy = math.cos(yaw),   math.sin(yaw)
        # Use the current AHRS pitch for prediction so z integration uses
        # the freshest available pitch instead of the stale prior state.
        cp, sp = math.cos(pitch_ahrs), math.sin(pitch_ahrs)

        xn = np.array([
            x + v * cy * cp * dt,
            y + v * sy * cp * dt,
            v,                              # constant velocity model
            omega_ahrs,                     # yaw-rate injected from AHRS
            _wrap(yaw + omega_ahrs * dt),
            z - v * sp * dt,               # vertical integration via pitch (down = negative)
            pitch_ahrs,                     # pre-load AHRS pitch; tight-coupling update follows
        ])

        # Jacobian F = ∂f/∂x  (7×7)  evaluated at pitch_ahrs
        F = np.eye(self.N)
        # x row
        F[self.X,  self.V]     =  cy * cp * dt
        F[self.X,  self.YAW]   = -v * sy * cp * dt
        F[self.X,  self.PITCH] = -v * cy * sp * dt
        # y row
        F[self.Y,  self.V]     =  sy * cp * dt
        F[self.Y,  self.YAW]   =  v * cy * cp * dt
        F[self.Y,  self.PITCH] = -v * sy * sp * dt
        # omega row — zero: directly set from AHRS, no propagation from prior state
        F[self.OM, :] = 0.0
        # yaw row
        F[self.YAW, self.OM]  = dt
        F[self.YAW, self.YAW] = 1.0
        # z row
        F[self.Z, self.V]     = -sp * dt
        F[self.Z, self.PITCH] = -v * cp * dt
        F[self.Z, self.Z]     = 1.0
        # pitch row — identity (pre-loaded from AHRS; tight coupling update below)

        self.x = xn
        self.P = F @ self.P @ F.T + self.Q

    # ----------------------------------------------------- measurement helpers

    def update_ahrs_yaw(self, yaw_ahrs: float):
        """Tight AHRS yaw coupling at ~100 Hz."""
        H = np.zeros((1, self.N));  H[0, self.YAW] = 1.0
        h = np.array([self.x[self.YAW]])
        z = np.array([_unwrap_near(yaw_ahrs, self.x[self.YAW])])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_ahrs_yaw)
        self.x[self.YAW] = _wrap(self.x[self.YAW])

    def update_ahrs_pitch(self, pitch_ahrs: float):
        """Tight AHRS pitch coupling at ~100 Hz."""
        H = np.zeros((1, self.N));  H[0, self.PITCH] = 1.0
        h = np.array([self.x[self.PITCH]])
        z = np.array([_unwrap_near(pitch_ahrs, self.x[self.PITCH])])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_ahrs_pitch)
        self.x[self.PITCH] = _wrap(self.x[self.PITCH])

    def _decouple_z(self):
        """Zero z and pitch cross-covariances with flat-ground states.
        Must be called BEFORE any encoder update so K[Z] and K[PITCH]
        are zero during the Kalman gain computation."""
        flat = (self.X, self.Y, self.V, self.OM, self.YAW)
        for i in flat:
            self.P[self.Z,     i] = 0.0
            self.P[i, self.Z]     = 0.0
            self.P[self.PITCH,  i] = 0.0
            self.P[i, self.PITCH] = 0.0

    def update_encoder_pose(self, x_enc: float, y_enc: float, yaw_enc: float):
        # Decouple z/pitch BEFORE update so encoder cannot pull them toward 0
        self._decouple_z()
        H = np.zeros((3, self.N))
        H[0, self.X]   = 1.0
        H[1, self.Y]   = 1.0
        H[2, self.YAW] = 1.0
        h = np.array([self.x[self.X], self.x[self.Y], self.x[self.YAW]])
        z = np.array([x_enc, y_enc, yaw_enc])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_enc_pose)

    def update_encoder_vel(self, v_enc: float, omega_enc: float):
        # Decouple z/pitch BEFORE update so velocity correction cannot leak into z
        self._decouple_z()
        H = np.zeros((2, self.N))
        H[0, self.V]  = 1.0
        H[1, self.OM] = 1.0
        h = np.array([self.x[self.V], self.x[self.OM]])
        z = np.array([v_enc, omega_enc])
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, self.R_enc_vel)

    def update_vision_pose(self, x_v: float, y_v: float, yaw_v: float,
                           z_v: float = None, pitch_v: float = None):
        """
        Update from vision.  x, y, yaw always used; z and pitch when provided.
        """
        if z_v is None and pitch_v is None:
            # 3-measurement update
            H = np.zeros((3, self.N))
            H[0, self.X]   = 1.0
            H[1, self.Y]   = 1.0
            H[2, self.YAW] = 1.0
            h = np.array([self.x[self.X], self.x[self.Y], self.x[self.YAW]])
            z = np.array([x_v, y_v, _unwrap_near(yaw_v, self.x[self.YAW])])
            R = self.R_vision[:3, :3]
        else:
            # 5-measurement update (full 3-D vision)
            z_v     = float(z_v)     if z_v     is not None else self.x[self.Z]
            pitch_v = float(pitch_v) if pitch_v is not None else self.x[self.PITCH]
            H = np.zeros((5, self.N))
            H[0, self.X]     = 1.0
            H[1, self.Y]     = 1.0
            H[2, self.YAW]   = 1.0
            H[3, self.Z]     = 1.0
            H[4, self.PITCH] = 1.0
            h = np.array([self.x[self.X], self.x[self.Y], self.x[self.YAW],
                          self.x[self.Z], self.x[self.PITCH]])
            z = np.array([x_v, y_v, _unwrap_near(yaw_v, self.x[self.YAW]),
                          z_v, _unwrap_near(pitch_v, self.x[self.PITCH])])
            R = self.R_vision
        self.x, self.P = _joseph_update(self.x, self.P, z, h, H, R)
        self.x[self.YAW]   = _wrap(self.x[self.YAW])
        self.x[self.PITCH] = _wrap(self.x[self.PITCH])


# ===========================================================================
# Public service class
# ===========================================================================

class SKalman:
    """
    Two-stage EKF pose estimator.

    Stage 1 (_AHRS_EKF) fuses all 9 raw IMU axes into roll, pitch,
    yaw, and 3-axis gyro biases at ~100 Hz.

    Stage 2 (_PoseEKF) tracks x, y, v, omega, yaw and accepts updates
    from encoder odometry and vision, with AHRS yaw tightly coupled.

    External estimate() returns a 7-element list preserving the index
    layout of the old linear KF so that uservice.py / ulog.py are
    unaffected:
      [x, y, gyro_bias_z, v, omega, yaw, pitch]
    """

    UPDATE_TOPICS = {
        "T0/gyro", "T0/acc", "T0/pose", "T0/vel",
        "T0/mag",  "T0/vision_pose",
    }

    # Default starting pose  (X = forward, Y = lateral)
    # X = 0.235 m from start wall (forward), Y = 4.775 m from left wall (lateral)
    _DEFAULT_X    = 0.235
    _DEFAULT_Y    = 4.775
    _DEFAULT_YAW  = 0.0

    def __init__(self):
        self.ahrs = _AHRS_EKF()
        self.pose = _PoseEKF()

        self.enabled      = False
        self.update_count = 0

        # Timing for EKF prediction
        self._last_gyro_time = None
        self._last_gyro_cnt  = -1

        # Deduplication counters
        self._last_acc_cnt  = -1
        self._last_pose_cnt = -1
        self._last_vel_cnt  = -1
        self._last_mag_cnt  = -1

        # Velocity derived from consecutive encoder-pose deltas (fallback for T0/vel)
        self._last_enc_xy:   tuple | None = None   # (x, y, yaw) of previous pose
        self._last_enc_time: object | None = None  # datetime of previous pose

        # Vision
        self._vision      = None
        self._vision_time = None
        self._vision_cnt  = 0

        # Teleoperation
        self.teleop_enabled   = False
        self.teleop_cmd       = None
        self._teleop_cmd_time = None
        self.teleop_timeout_s = float('inf')
        self._manual_u        = None

        # Logging helpers
        self._last_meas: dict = {}
        self._last_u          = np.zeros(2)

        # Publish rate limiter
        self._last_pub_time = datetime.now()
        self._pub_interval  = 0.1   # 10 Hz

        # EMA smoother on AHRS outputs (decouples noisy EKF ticks from published values)
        self._ema_alpha       = 0.15   # 0 = no update, 1 = no smoothing  (roll + yaw)
        self._ema_alpha_pitch = 0.05   # slower smoothing for noisier pitch axis
        self._ema_roll:  float | None = None
        self._ema_pitch: float | None = None
        self._ema_yaw:   float | None = None

    # --------------------------------------------------------------- setup / reset

    def setup(self):
        self.ahrs = _AHRS_EKF()
        self.pose = _PoseEKF()
        self.pose.x[self.pose.X]   = self._DEFAULT_X
        self.pose.x[self.pose.Y]   = self._DEFAULT_Y
        self.pose.x[self.pose.YAW] = self._DEFAULT_YAW
        self.ahrs.x[self.ahrs.YAW] = self._DEFAULT_YAW

        self._last_gyro_time = None
        self._last_gyro_cnt  = -1
        self._last_acc_cnt   = -1
        self._last_pose_cnt  = -1
        self._enc_yaw_offset = None
        self._enc_x_offset   = None
        self._enc_y_offset   = None
        self._last_vel_cnt   = -1
        self._last_mag_cnt   = -1
        self._last_enc_xy    = None
        self._last_enc_time  = None
        self._vision         = None
        self._vision_cnt     = 0
        self.update_count    = 0
        self.enabled         = True
        self._last_meas      = {}
        self._ema_roll       = None
        self._ema_pitch      = None
        self._ema_yaw        = None

    def reset(self, state=None):
        """Reset to default or given 7-element [x,y,bias_z,v,omega,yaw,pitch] list."""
        self.ahrs = _AHRS_EKF()
        self.pose = _PoseEKF()
        if state is not None:
            arr = np.array(state, dtype=float).flatten()
            if arr.size != 7:
                raise ValueError("reset expects 7 state values")
            self.pose.x[self.pose.X]     = arr[0]
            self.pose.x[self.pose.Y]     = arr[1]
            self.pose.x[self.pose.V]     = arr[3]
            self.pose.x[self.pose.OM]    = arr[4]
            self.pose.x[self.pose.YAW]   = _wrap(arr[5])
            self.pose.x[self.pose.PITCH] = _wrap(arr[6])
            self.ahrs.x[self.ahrs.YAW]   = _wrap(arr[5])
            self.ahrs.x[self.ahrs.PITCH] = _wrap(arr[6])
            self.ahrs.x[self.ahrs.BZ]    = arr[2]
        else:
            self.pose.x[self.pose.X]   = self._DEFAULT_X
            self.pose.x[self.pose.Y]   = self._DEFAULT_Y
            self.pose.x[self.pose.YAW] = self._DEFAULT_YAW
            self.ahrs.x[self.ahrs.YAW] = self._DEFAULT_YAW
        self._last_gyro_time = None
        self.update_count    = 0
        self._ema_roll     = None
        self._ema_pitch    = None
        self._ema_yaw      = None
        self._last_enc_xy   = None
        self._last_enc_time = None

    def reset_from_measurement(self):
        """Bootstrap state from latest sensor readings."""
        self.ahrs = _AHRS_EKF()
        self.pose = _PoseEKF()
        self.pose.x[self.pose.X]   = self._DEFAULT_X
        self.pose.x[self.pose.Y]   = self._DEFAULT_Y
        self.pose.x[self.pose.YAW] = self._DEFAULT_YAW
        self.ahrs.x[self.ahrs.YAW] = self._DEFAULT_YAW
        try:
            from simu  import imu
            from spose import pose as enc_pose
            if enc_pose.poseCnt > 0:
                self.pose.x[self.pose.X]   = float(enc_pose.pose[0])
                self.pose.x[self.pose.Y]   = float(enc_pose.pose[1])
                yaw_init = _wrap(float(enc_pose.pose[2]) + math.pi / 2.0)
                self.pose.x[self.pose.YAW] = yaw_init
                self.ahrs.x[self.ahrs.YAW] = yaw_init
            if imu.accUpdCnt > 0:
                ax, ay, az = float(imu.acc[0]), float(imu.acc[1]), float(imu.acc[2])
                roll_init  = math.atan2(ay, math.sqrt(ax*ax + az*az))
                pitch_init = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                self.ahrs.x[self.ahrs.ROLL]  = roll_init
                self.ahrs.x[self.ahrs.PITCH] = pitch_init
                self.pose.x[self.pose.PITCH] = pitch_init
        except Exception:
            pass
        if self._vision is not None:
            self.pose.x[self.pose.X]   = float(self._vision[0])
            self.pose.x[self.pose.Y]   = float(self._vision[1])
            self.pose.x[self.pose.YAW] = float(self._vision[3])
            self.ahrs.x[self.ahrs.YAW] = float(self._vision[3])
        self._last_gyro_time = None
        self.update_count    = 1
        self._ema_roll  = None
        self._ema_pitch = None
        self._ema_yaw   = None

    def _update_ema(self):
        """Update exponential moving average of AHRS outputs."""
        alpha       = self._ema_alpha
        alpha_pitch = self._ema_alpha_pitch
        r, p, y = self.ahrs.roll, self.ahrs.pitch, self.ahrs.yaw
        if self._ema_roll is None:
            self._ema_roll, self._ema_pitch, self._ema_yaw = r, p, y
        else:
            self._ema_roll  = alpha * r + (1.0 - alpha) * self._ema_roll
            # Pitch uses a slower alpha — it is noisier than yaw
            self._ema_pitch = alpha_pitch * p + (1.0 - alpha_pitch) * self._ema_pitch
            # Wrap-safe yaw EMA: interpolate along shortest arc
            d = _wrap(y - self._ema_yaw)
            self._ema_yaw = _wrap(self._ema_yaw + alpha * d)

    # --------------------------------------------------------------- queries

    def has_estimate(self) -> bool:
        return self.enabled and self.update_count > 0

    def estimate(self):
        """
        Return current 7-element state list, or None if no estimate yet.
        Layout: [x, y, z, v, omega, yaw, pitch]
        pitch is now taken from the Pose EKF (tightly coupled to AHRS).
        """
        if not self.has_estimate():
            return None
        return [
            float(self.pose.x[self.pose.X]),
            float(self.pose.x[self.pose.Y]),
            float(self.pose.x[self.pose.Z]),      # z (height)
            float(self.pose.x[self.pose.V]),
            float(self.pose.x[self.pose.OM]),
            float(self.pose.x[self.pose.YAW]),
            float(self.pose.x[self.pose.PITCH]),  # from Pose EKF (AHRS-coupled)
        ]

    def predict(self):
        """Return last pre-update predicted state (7-element list) — for logging."""
        return self.estimate() or [0.0] * 7

    def last_input(self):
        return self._last_u.tolist()

    def get_measurements(self) -> dict:
        return dict(self._last_meas)

    # --------------------------------------------------------------- teleop / manual

    def set_manual_input(self, u_left: float, u_right: float):
        self._manual_u = np.array([float(u_left), float(u_right)])

    def clear_manual_input(self):
        self._manual_u = None

    def decode_teleoperation(self, msg: str) -> bool:
        try:
            data   = json.loads(msg)
            lin_v  = float(data.get('linear_velocity',  0.0))
            ang_v  = float(data.get('angular_velocity', 0.0))
            half_b = 0.23 / 2.0
            self.teleop_cmd = {
                'linear_velocity':  lin_v,
                'angular_velocity': ang_v,
                'v_left':  lin_v - half_b * ang_v,
                'v_right': lin_v + half_b * ang_v,
            }
            self._teleop_cmd_time = datetime.now()
            self.teleop_enabled   = True
            return True
        except Exception as e:
            print(f"# Kalman: failed to decode teleoperation: {e}")
            return False

    def _current_u(self) -> np.ndarray:
        if self.teleop_enabled and self.teleop_cmd and self._teleop_cmd_time:
            if (datetime.now() - self._teleop_cmd_time).total_seconds() <= self.teleop_timeout_s:
                return np.array([self.teleop_cmd['v_left'], self.teleop_cmd['v_right']])
            self.teleop_enabled = False
        if self._manual_u is not None:
            return self._manual_u.copy()
        try:
            from spose import pose
            if pose.wheelVelocityCnt > 0:
                return np.array([float(pose.wheelVelocity[0]),
                                  float(pose.wheelVelocity[1])])
        except Exception:
            pass
        return np.zeros(2)

    # --------------------------------------------------------------- vision pose

    def set_vision_pose(self, x, y, yaw, pitch=0.0, z=0.0, timestamp=None):
        if timestamp is None:
            timestamp = datetime.now()
        elif isinstance(timestamp, (int, float)):
            timestamp = datetime.fromtimestamp(float(timestamp))
        self._vision      = np.array([float(x), float(y), float(z),
                                       _wrap(float(yaw)), _wrap(float(pitch))])
        self._vision_time = timestamp
        self._vision_cnt += 1

    def _decode_vision_pose(self, msg: str) -> bool:
        gg = msg.split()
        if len(gg) < 5:
            return False
        ts = float(gg[0])
        if len(gg) >= 6:
            self.set_vision_pose(gg[1], gg[2], gg[4], pitch=gg[5], z=gg[3], timestamp=ts)
        else:
            self.set_vision_pose(gg[1], gg[2], gg[3], pitch=gg[4], timestamp=ts)
        return True

    # --------------------------------------------------------------- state publish

    def publish_state(self) -> bool:
        try:
            now = datetime.now()
            if (now - self._last_pub_time).total_seconds() < self._pub_interval:
                return False
            if not self.has_estimate():
                return False
            from uservice import service
            px = self.pose.x
            ax = self.ahrs.x
            P_pose = np.diag(self.pose.P)
            P_ahrs = np.diag(self.ahrs.P)
            msg = {
                'timestamp':    now.isoformat(),
                'update_count': self.update_count,
                'position':     {'x': float(px[self.pose.X]),
                                  'y': float(px[self.pose.Y]),
                                  'z': float(px[self.pose.Z])},

                'velocity':     {'linear':  float(px[self.pose.V]),
                                  'angular': float(px[self.pose.OM])},
                'orientation':  {'yaw':   float(px[self.pose.YAW]),
                                  'pitch': float(px[self.pose.PITCH]),
                                  'roll':  float(self._ema_roll  if self._ema_roll  is not None else ax[self.ahrs.ROLL])},
                'gyro_bias':    {'x': float(ax[self.ahrs.BX]),
                                  'y': float(ax[self.ahrs.BY]),
                                  'z': float(ax[self.ahrs.BZ])},
                'std_dev': {
                    'x':     float(math.sqrt(max(0.0, P_pose[self.pose.X]))),
                    'y':     float(math.sqrt(max(0.0, P_pose[self.pose.Y]))),
                    'z':     float(math.sqrt(max(0.0, P_pose[self.pose.Z]))),
                    'yaw':   float(math.sqrt(max(0.0, P_pose[self.pose.YAW]))),
                    'pitch': float(math.sqrt(max(0.0, P_pose[self.pose.PITCH]))),
                    'roll':  float(math.sqrt(max(0.0, P_ahrs[self.ahrs.ROLL]))),
                },
            }
            service.send("robobot/kalman/state", json.dumps(msg))
            self._last_pub_time = now
            return True
        except Exception:
            return False

    # --------------------------------------------------------------- MQTT decode

    def decode(self, topic: str, msg: str) -> bool:
        """
        Dispatch incoming MQTT message to the appropriate EKF step.

        T0/gyro        → AHRS predict + Pose predict + acc attitude update
        T0/acc         → AHRS acc update (when arriving independently)
        T0/pose        → Pose encoder-pose update
        T0/vel         → Pose encoder-velocity update
        T0/mag         → AHRS mag-yaw update
        T0/vision_pose → Pose vision update
        """
        if not self.enabled:
            return False

        # Vision can arrive on any path
        if topic == "T0/vision_pose":
            if self._decode_vision_pose(msg) and self.has_estimate():
                self.pose.update_vision_pose(
                    float(self._vision[0]),
                    float(self._vision[1]),
                    float(self._vision[3]),
                    z_v=float(self._vision[2]),
                    pitch_v=float(self._vision[4]),
                )
            return True

        if topic not in self.UPDATE_TOPICS:
            return False

        try:
            from simu  import imu
            from spose import pose as enc_pose
        except Exception:
            return False

        # ── T0/gyro ── prediction tick at ~100 Hz ─────────────────────────────
        if topic == "T0/gyro":
            if imu.gyroUpdCnt == self._last_gyro_cnt:
                return True
            self._last_gyro_cnt = imu.gyroUpdCnt

            now = imu.gyroTime
            if self._last_gyro_time is None or self.update_count == 0:
                self._last_gyro_time = now
                self.update_count    = 1
                return True

            dt = (now - self._last_gyro_time).total_seconds()
            if dt <= 1e-6:
                return True
            dt = max(0.002, min(0.2, dt))
            self._last_gyro_time = now

            # T0/gyro arrives in deg/s — convert to rad/s for the EKF
            gx = float(imu.gyro[0]) * math.pi / 180.0
            gy = float(imu.gyro[1]) * math.pi / 180.0
            gz = float(imu.gyro[2]) * math.pi / 180.0
            ax = float(imu.acc[0]) * GRAVITY if imu.accUpdCnt > 0 else 0.0
            ay = float(imu.acc[1]) * GRAVITY if imu.accUpdCnt > 0 else 0.0
            az = float(imu.acc[2]) * GRAVITY if imu.accUpdCnt > 0 else -GRAVITY

            # ── Stage 1: AHRS predict + acc update ────────────────────────────
            self.ahrs.predict(gx, gy, gz, dt)
            # Only update when acc data is actually new — repeated identical
            # measurements collapse P[PITCH] / P[BY,PITCH] to near-zero and
            # kill bias estimation, causing monotonic pitch drift.
            if imu.accUpdCnt > 0 and imu.accUpdCnt != self._last_acc_cnt:
                self.ahrs.update_acc(ax, ay, az)
                self._last_acc_cnt = imu.accUpdCnt

            # Smooth AHRS output with EMA before feeding Pose EKF
            self._update_ema()

            # ── Stage 2: Pose predict (driven by AHRS yaw rate + pitch) ──────
            bz         = float(self.ahrs.x[self.ahrs.BZ])
            omega_ahrs = gz - bz                 # bias-corrected yaw rate
            self.pose.predict(omega_ahrs, self._ema_pitch, dt)

            # Tight coupling: smoothed AHRS yaw + pitch → Pose EKF
            self.pose.update_ahrs_yaw(self._ema_yaw)
            self.pose.update_ahrs_pitch(self._ema_pitch)

            # Logging
            self._last_u = self._current_u()
            self._last_meas.update({
                'gyro': [gx, gy, gz],
                'acc':  [ax, ay, az],
                'ahrs_yaw':   self._ema_yaw,
                'ahrs_pitch': self._ema_pitch,
                'ahrs_roll':  self._ema_roll,
                'dt':         dt,
            })
            self.update_count += 1
            self.publish_state()
            return True

        # ── T0/acc ── extra attitude update when acc arrives separately ────────
        if topic == "T0/acc":
            if imu.accUpdCnt == self._last_acc_cnt:
                return True
            self._last_acc_cnt = imu.accUpdCnt
            if not self.has_estimate():
                return True
            self.ahrs.update_acc(
                float(imu.acc[0]) * GRAVITY, float(imu.acc[1]) * GRAVITY, float(imu.acc[2]) * GRAVITY
            )
            return True

        # ── T0/pose ── encoder odometry pose ──────────────────────────────────
        if topic == "T0/pose":
            if enc_pose.poseCnt == self._last_pose_cnt or enc_pose.poseCnt == 0:
                return True
            self._last_pose_cnt = enc_pose.poseCnt
            if not self.has_estimate():
                return True
            x_enc   = float(enc_pose.pose[0])
            y_enc   = float(enc_pose.pose[1])
            raw_yaw = _wrap(float(enc_pose.pose[2]) + math.pi / 2.0)
            if self._enc_yaw_offset is None:
                self._enc_yaw_offset = raw_yaw
                self._enc_x_offset   = x_enc
                self._enc_y_offset   = y_enc
            yaw_enc = _wrap(raw_yaw - self._enc_yaw_offset)
            # Offset encoder x/y so dead-reckoning starts at the real world origin
            x_enc = self._DEFAULT_X + (x_enc - self._enc_x_offset)
            y_enc = self._DEFAULT_Y + (y_enc - self._enc_y_offset)
            self.pose.update_encoder_pose(x_enc, y_enc, yaw_enc)

            # Derive forward velocity and yaw-rate from consecutive pose deltas.
            # This acts as a fallback when T0/vel is not arriving (e.g. hand-push)
            # and as a corroborating measurement when it is.
            now = enc_pose.poseTime
            if self._last_enc_xy is not None and self._last_enc_time is not None:
                enc_dt = (now - self._last_enc_time).total_seconds()
                if 0.005 < enc_dt < 1.0:
                    lx, ly, lyaw = self._last_enc_xy
                    dx, dy = x_enc - lx, y_enc - ly
                    dist = math.sqrt(dx * dx + dy * dy)
                    # sign: positive if moving in heading direction
                    mid_yaw = _wrap(lyaw + 0.5 * _wrap(yaw_enc - lyaw))
                    if dx * math.cos(mid_yaw) + dy * math.sin(mid_yaw) < 0:
                        dist = -dist
                    v_derived     = dist / enc_dt
                    omega_derived = _wrap(yaw_enc - lyaw) / enc_dt
                    self.pose.update_encoder_vel(v_derived, omega_derived)
                    self._last_meas.update({'enc_v_derived': v_derived,
                                           'enc_om_derived': omega_derived})
            self._last_enc_xy   = (x_enc, y_enc, yaw_enc)
            self._last_enc_time = now

            self._last_meas.update({
                'enc_x':   x_enc,
                'enc_y':   y_enc,
                'enc_yaw': yaw_enc,
            })
            return True

        # ── T0/vel ── encoder velocity ─────────────────────────────────────────
        if topic == "T0/vel":
            if enc_pose.wheelVelocityCnt == self._last_vel_cnt or enc_pose.wheelVelocityCnt == 0:
                return True
            self._last_vel_cnt = enc_pose.wheelVelocityCnt
            if not self.has_estimate():
                return True
            v_enc     = enc_pose.velocity()
            omega_enc = enc_pose.turnrate()
            self.pose.update_encoder_vel(v_enc, omega_enc)
            self._last_meas.update({'enc_v': v_enc, 'enc_om': omega_enc})
            return True

        # ── T0/mag ── magnetometer yaw ─────────────────────────────────────────
        if topic == "T0/mag":
            if imu.magUpdCnt == self._last_mag_cnt or imu.magUpdCnt == 0:
                return True
            self._last_mag_cnt = imu.magUpdCnt
            if not self.has_estimate():
                return True
            self.ahrs.update_mag(
                float(imu.mag[0]), float(imu.mag[1]), float(imu.mag[2])
            )
            # Re-smooth after mag corrects yaw, then propagate to pose EKF
            self._update_ema()
            self.pose.update_ahrs_yaw(self._ema_yaw)
            self.pose.update_ahrs_pitch(self._ema_pitch)
            self._last_meas['mag_yaw'] = self._ema_yaw
            return True

        return False

    # --------------------------------------------------------------- misc

    def terminate(self):
        print(f"% EKF terminated  updates={self.update_count}"
              f"  gyro_bias=[{self.ahrs.x[self.ahrs.BX]:.4f},"
              f" {self.ahrs.x[self.ahrs.BY]:.4f},"
              f" {self.ahrs.x[self.ahrs.BZ]:.4f}] rad/s")


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------
kalman = SKalman()
