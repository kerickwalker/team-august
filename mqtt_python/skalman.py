#!/usr/bin/env python3

import math

import numpy as np

from Kalman_filter_seperate_files.Kalman_class import KalmanFilter


def _wrap_angle_rad(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _unwrap_near(meas: float, ref: float) -> float:
    """Shift measurement by +/-2pi so it stays closest to reference."""
    return ref + _wrap_angle_rad(meas - ref)


def _build_state_transition_matrix(yaw: float, pitch: float, dt_s: float) -> np.ndarray:
    """Linearized transition used in the previous Kalman offline tests."""
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    return np.array(
        [
            [1.0, 0.0, 0.0, cp * sy * dt_s, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, cp * cy * dt_s, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, sp * dt_s, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, dt_s, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


class SKalman:
    """Realtime 7-state Kalman estimator for robot pose and orientation."""

    STATE_NAMES = ["x", "y", "z", "velocity", "angular_velocity", "yaw", "pitch"]

    def __init__(self):
        self.track_width = 0.23
        self.kf = None
        self.last_update_time = None
        self.update_count = 0
        self.enabled = True
        self.manual_u = None
        self.last_u = np.zeros((2, 1), dtype=float)
        self.default_state = np.zeros((7, 1), dtype=float)

    def _create_filter(self):
        from spose import pose

        dt = max(0.01, min(0.2, pose.poseInterval if pose.poseInterval > 0.0 else 0.05))
        A = _build_state_transition_matrix(float(pose.pose[2]), float(pose.pose[3]), dt)
        B = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                # Linear speed state gets the mean wheel linear speed.
                [1.0, 1.0],
                # Angular speed state gets differential-drive yaw rate.
                [-1.0 / self.track_width, 1.0 / self.track_width],
                [0.0, 0.0],
                [0.0, 0.0],
            ],
            dtype=float,
        )
        H = np.eye(7, dtype=float)

        Q = np.diag([0.01, 0.01, 0.02, 0.04, 0.05, 0.02, 0.02]).astype(float)
        # z is weakly observed in this runtime setup, so keep a larger measurement variance there.
        R = np.diag([0.05, 0.05, 4.0, 0.08, 0.08, 0.05, 0.08]).astype(float)

        self.kf = KalmanFilter(A, B, H, Q, R)

    def setup(self):
        self._create_filter()
        # Start from the configured default origin state.
        self.kf.x = self.default_state.copy()
        self.kf.P = np.eye(7, dtype=float)
        self.last_update_time = None
        self.update_count = 0
        self.last_u = np.zeros((2, 1), dtype=float)

    def _meas_vector(self) -> np.ndarray:
        from spose import pose
        from simu import imu

        x_m = float(pose.pose[0])
        y_m = float(pose.pose[1])
        z_m = 0.0
        v_m = float(pose.velocity())

        if imu.gyroUpdCnt > 0:
            omega_m = float(imu.gyro[2])
        else:
            omega_m = float(pose.turnrate())

        yaw_m = float(pose.pose[2])
        pitch_m = float(pose.pose[3])
        return np.array([[x_m], [y_m], [z_m], [v_m], [omega_m], [yaw_m], [pitch_m]], dtype=float)

    def _control_vector(self) -> np.ndarray:
        from spose import pose

        u_left = float(pose.wheelVelocity[0])
        u_right = float(pose.wheelVelocity[1])
        return np.array([[u_left], [u_right]], dtype=float)

    def set_manual_input(self, u_left: float, u_right: float):
        self.manual_u = np.array([[float(u_left)], [float(u_right)]], dtype=float)

    def clear_manual_input(self):
        self.manual_u = None

    def _current_u(self) -> np.ndarray:
        if self.manual_u is not None:
            return self.manual_u.copy()
        return self._control_vector()

    def _bootstrap_from_measurement(self, z: np.ndarray):
        self.kf.x = z.copy()
        self.kf.P = np.eye(7, dtype=float)

    def reset(self, state=None):
        """Reset filter to default or provided 7-state vector."""
        if self.kf is None:
            self._create_filter()
        if state is None:
            x0 = self.default_state.copy()
        else:
            arr = np.array(state, dtype=float).reshape(-1)
            if arr.size != 7:
                raise ValueError("reset expects 7 state values")
            x0 = arr.reshape((7, 1))
        x0[5, 0] = _wrap_angle_rad(float(x0[5, 0]))
        x0[6, 0] = _wrap_angle_rad(float(x0[6, 0]))
        self.kf.x = x0
        self.kf.P = np.eye(7, dtype=float)
        self.last_update_time = None
        self.update_count = 1
        self.last_u = np.zeros((2, 1), dtype=float)

    def reset_from_measurement(self):
        """Reset state directly from latest measurement vector."""
        z = self._meas_vector()
        self.reset(z.flatten().tolist())

    def decode(self, topic, _msg):
        # Only update on pose timestamps to keep dt stable and aligned to odometry updates.
        if not self.enabled or topic != "T0/pose":
            return False

        from spose import pose

        now_t = pose.poseTime
        if self.last_update_time is None:
            z = self._meas_vector()
            self._bootstrap_from_measurement(z)
            self.last_update_time = now_t
            self.update_count = 1
            return True

        dt_s = (now_t - self.last_update_time).total_seconds()
        if dt_s <= 1e-6:
            return True
        dt_s = max(0.005, min(0.2, dt_s))

        u = self._current_u()
        self.last_u = u.copy()

        yaw_hat = float(self.kf.x[5, 0])
        pitch_hat = float(self.kf.x[6, 0])
        self.kf.A = _build_state_transition_matrix(yaw_hat, pitch_hat, dt_s)
        self.kf.predict(u)

        z = self._meas_vector()
        z[5, 0] = _unwrap_near(float(z[5, 0]), float(self.kf.x[5, 0]))
        z[6, 0] = _unwrap_near(float(z[6, 0]), float(self.kf.x[6, 0]))
        self.kf.update(z)

        self.kf.x[5, 0] = _wrap_angle_rad(float(self.kf.x[5, 0]))
        self.kf.x[6, 0] = _wrap_angle_rad(float(self.kf.x[6, 0]))

        self.last_update_time = now_t
        self.update_count += 1
        return True

    def predict_only(self, dt_s: float, u_left=None, u_right=None) -> bool:
        """Run model prediction without measurement correction."""
        if not self.enabled:
            return False

        dt_s = float(dt_s)
        if dt_s <= 0.0:
            return False
        dt_s = max(0.001, min(1.0, dt_s))

        if self.kf is None:
            self._create_filter()

        if not self.has_estimate():
            z = self._meas_vector()
            self._bootstrap_from_measurement(z)
            self.update_count = 1

        if u_left is not None and u_right is not None:
            u = np.array([[float(u_left)], [float(u_right)]], dtype=float)
        else:
            u = self._current_u()
        self.last_u = u.copy()

        yaw_hat = float(self.kf.x[5, 0])
        pitch_hat = float(self.kf.x[6, 0])
        self.kf.A = _build_state_transition_matrix(yaw_hat, pitch_hat, dt_s)
        self.kf.predict(u)
        self.kf.x[5, 0] = _wrap_angle_rad(float(self.kf.x[5, 0]))
        self.kf.x[6, 0] = _wrap_angle_rad(float(self.kf.x[6, 0]))
        self.update_count += 1
        return True

    def has_estimate(self) -> bool:
        return self.kf is not None and self.update_count > 0

    def estimate(self):
        if not self.has_estimate():
            return None
        return self.kf.x.flatten().tolist()

    def last_input(self):
        return self.last_u.flatten().tolist()

    def terminate(self):
        print(f"% Kalman estimator terminated (updates={self.update_count})")


kalman = SKalman()
