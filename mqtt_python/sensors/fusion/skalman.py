#!/usr/bin/env python3

from datetime import datetime
import math
import json
import time as t

import numpy as np

try:
    from Kalman_filter_seperate_files.Kalman_class import KalmanFilter
except ModuleNotFoundError:
    class KalmanFilter:
        """Minimal linear Kalman filter fallback.

        This fallback is used when the external Kalman module is not present
        on the runtime machine. It supports the subset of API used by SKalman.
        """

        def __init__(self, A, B, H, Q, R):
            self.A = np.array(A, dtype=float)
            self.B = np.array(B, dtype=float)
            self.H = np.array(H, dtype=float)
            self.Q = np.array(Q, dtype=float)
            self.R = np.array(R, dtype=float)

            n = self.A.shape[0]
            self.x = np.zeros((n, 1), dtype=float)
            self.P = np.eye(n, dtype=float)

        def predict(self, u=None):
            if u is None:
                u = np.zeros((self.B.shape[1], 1), dtype=float)
            u = np.array(u, dtype=float).reshape((self.B.shape[1], 1))
            self.x = self.A @ self.x + self.B @ u
            self.P = self.A @ self.P @ self.A.T + self.Q

        def update(self, z):
            z = np.array(z, dtype=float).reshape((self.H.shape[0], 1))
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            # Use pseudo-inverse for robustness when S is near-singular.
            K = self.P @ self.H.T @ np.linalg.pinv(S)
            self.x = self.x + K @ y
            I = np.eye(self.P.shape[0], dtype=float)
            self.P = (I - K @ self.H) @ self.P

from sensors.fusion.imu_derived_measurements import imu_derived


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
    UPDATE_TOPICS = {"T0/pose", "T0/vel", "T0/gyro", "T0/acc", "T0/yawg", "T0/mag", "T0/vision_pose"}
    # Fixed stacked measurement channels (expanded to 20 for all IMU-derived).
    # ========== ENCODER/ODOMETRY (0-6) ==========
    # 0: encoder x
    # 1: encoder y
    # 2: encoder z (fixed to 0)
    # 3: encoder velocity
    # 4: gyro angular velocity
    # 5: encoder yaw
    # 6: encoder angular velocity
    #
    # ========== VISION (7-11) - DISABLED when not available ==========
    # 7: vision x
    # 8: vision y
    # 9: vision z
    # 10: vision yaw
    # 11: (was vision pitch, now vel_acc_integrated)
    #
    # ========== IMU DERIVED (12-16) - MULTI-SOURCE SENSOR FUSION ==========
    # 12: yaw_imu_integrated (IMU integrated yaw from gyro)
    # 13: yaw_mag_derived (magnetometer heading)
    # 14: pitch_acc_derived (accelerometer-derived pitch - NEW)
    # 15: pitch_mag_derived (magnetometer-derived pitch - RESTORED)
    # 16: pitch_vision (vision-derived pitch)
    #
    MEAS_NAMES = [
        # Encoder/Odometry (0-6)
        "pose_x_enc",
        "pose_y_enc",
        "pose_z_enc",
        "vel_enc",
        "omega_gyro",
        "yaw_enc",
        "omega_enc",
        # Vision (7-11)
        "pose_x_vision",
        "pose_y_vision",
        "pose_z_vision",
        "yaw_vision",
        "vel_acc_integrated",
        # IMU Derived - Accelerometer (12-13)
        "yaw_imu_integrated",
        "yaw_mag_derived",
        # Multi-source pitch measurements (14-16)
        "pitch_acc_derived",
        "pitch_mag_derived",
        "pitch_vision",
    ]

    def __init__(self):
        self.track_width = 0.23
        self.kf = None
        self.last_update_time = None
        self.update_count = 0
        self.enabled = False
        self.manual_u = None
        self.last_u = np.zeros((2, 1), dtype=float)
        self.default_state = np.array([[4.775], [0.235], [0.0], [0.0], [0.0], [1.5708], [0.0]], dtype=float)
        self._manual_reset = False
        self.vision_pose = np.zeros((5, 1), dtype=float)
        self.vision_pose_time = None         # wire timestamp from sender (used for dt math)
        self.vision_pose_recv_time = None    # local receive time (used for freshness)
        self.vision_pose_count = 0
        self.vision_pose_source = 'unknown'  # 'tape' | 'aruco' | 'unknown'
        # Throttle "vision stale" diagnostic so it doesn't spam at the
        # _measurement_vector_and_cov call rate.
        self._vision_stale_log_time = None
        # Optional measurement noise tags from svision_pose. None = sender did
        # not provide them; fall back to legacy hardcoded variances in that
        # case (see _measurement_z for the gating logic).
        self.vision_pose_sigma_xy  = None    # metres
        self.vision_pose_sigma_yaw = None    # radians
        self.vision_pose_n_used    = None
        self.vision_pose_confidence = None

        # Teleoperation input
        self.teleop_cmd = None  # {"linear_velocity": float, "angular_velocity": float}
        self.teleop_cmd_time = None
        self.teleop_enabled = False
        self.teleop_timeout_s = float('inf')  # Commands persist until new input; set to finite value (e.g., 5.0) to auto-stop
        
        # State publishing
        self.last_state_publish_time = datetime.now()
        self.state_publish_interval = 0.1  # Publish state every 100ms
        
        # Measurement storage for monitoring
        self.last_measurement = None
        self.measurement_names = self.MEAS_NAMES
        
        # Predicted state storage (for monitoring/display)
        self.x_pred_last = np.zeros((7, 1), dtype=float)  # Last predicted state from motion model

    def _create_filter(self):
        from sensors.spose import pose

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
        H = self._fixed_measurement_matrix()

        Q = np.diag([0.8, 0.8, 0.8, 0.5, 0.5, 0.02, 1e6]).astype(float)
        R = np.diag(self._base_measurement_variances()).astype(float)

        self.kf = KalmanFilter(A, B, H, Q, R)

    def setup(self):
        self._create_filter()
        self.kf.x = self.default_state.copy()
        self.kf.P = np.eye(7, dtype=float)
        self.last_update_time = None
        self._manual_reset = True
        self.update_count = 0
        self.last_u = np.zeros((2, 1), dtype=float)
        self.enabled = True

    def _fixed_measurement_matrix(self) -> np.ndarray:
        # H matrix: 17x7 (measurements x states)
        # STATE: [x, y, z, velocity, angular_velocity, yaw, pitch]
        H = np.zeros((len(self.MEAS_NAMES), len(self.STATE_NAMES)), dtype=float)
        
        # ========== ENCODER/ODOMETRY (0-6) ==========
        H[0, 0] = 0.0  # encoder x DISABLED - absolute pos from encoder is relative
        H[1, 1] = 0.0  # encoder y DISABLED - absolute pos from encoder is relative
        H[2, 2] = 1.0  # encoder z
        H[3, 3] = 1.0  # encoder velocity
        H[4, 4] = 1.0  # gyro angular velocity
        H[5, 5] = 1.0  # encoder yaw
        H[6, 4] = 1.0  # encoder angular velocity (duplicate omega)
        
        # ========== VISION (7-11) - active when vision_pose is fresh ==========
        H[7, 0] = 1.0   # vision x
        H[8, 1] = 1.0   # vision y
        H[9, 2] = 1.0   # vision z
        H[10, 5] = 1.0  # vision yaw
        H[11, 3] = 1.0  # vel_acc_integrated measures linear velocity
        
        # ========== IMU DERIVED - GYRO & MAGNETOMETER (12-13) ==========
        H[12, 5] = 0.0  # yaw_imu_integrated DISABLED - relative not absolute
        H[13, 5] = 1.0  # yaw_mag_derived measures yaw (from magnetometer)
        
        # ========== MULTI-SOURCE PITCH (14-16) ==========
        H[14, 6] = 1.0  # pitch_acc_derived measures pitch (from accelerometer - NEW)
        H[15, 6] = 1.0  # pitch_mag_derived measures pitch (from magnetometer - RESTORED)
        H[16, 6] = 1.0  # pitch_vision measures pitch (from vision)
        
        return H

    def _base_measurement_variances(self):
        return [
            # ========== ENCODER/ODOMETRY (0-6) ==========
            0.05,   # 0: pose_x_enc
            0.05,   # 1: pose_y_enc
            4.0,    # 2: pose_z_enc
            0.08,   # 3: vel_enc
            0.08,   # 4: omega_gyro
            0.05,   # 5: yaw_enc
            0.10,   # 6: omega_enc
            # ========== VISION (7-11) ==========
            1e6,    # 7: pose_x_vision (DISABLED - no vision system)
            1e6,    # 8: pose_y_vision (DISABLED - no vision system)
            1e6,    # 9: pose_z_vision (DISABLED - no vision system)
            1e6,    # 10: yaw_vision (DISABLED - no vision system)
            0.20,   # 11: vel_acc_integrated (from accelerometer integration)
            # ========== IMU DERIVED - YAW SOURCES (12-13) ==========
            0.10,   # 12: yaw_imu_integrated (from gyro integration)
            0.15,   # 13: yaw_mag_derived (from magnetometer heading - usually 0 if inactive)
            # ========== MULTI-SOURCE PITCH (14-16) ==========
            0.12,   # 14: pitch_acc_derived (accelerometer-based, NEW)
            0.20,   # 15: pitch_mag_derived (magnetometer-based, RESTORED)
            1e6,   # 16: pitch_vision (vision-based)
        ]

    def _measurement_vector_and_cov(self, predicted_state: np.ndarray):
        from sensors.spose import pose
        from sensors.simu import imu

        imu_derived.update_from_streams()

        x_pred = predicted_state
        yaw_ref = float(x_pred[5, 0])
        pitch_ref = float(x_pred[6, 0])

        z = np.zeros((len(self.MEAS_NAMES), 1), dtype=float)
        variances = self._base_measurement_variances()
        missing_var = 1e6

        # ========== ENCODER/ODOMETRY (0-6) ==========
        z[0, 0] = float(pose.pose[0])                      # encoder x
        z[1, 0] = float(pose.pose[1])                      # encoder y
        z[2, 0] = 0.0                                       # encoder z (fixed)
        z[3, 0] = float(pose.velocity())                   # encoder velocity
        
        if imu.gyroUpdCnt > 0:
            z[4, 0] = float(imu.gyro[2])                   # gyro omega Z
        else:
            z[4, 0] = 0.0
            variances[4] = missing_var
        
        z[5, 0] = _unwrap_near(float(pose.pose[2]) + 1.5708, yaw_ref)  # encoder yaw + start offset
        z[6, 0] = float(pose.turnrate())                   # encoder omega (turnrate)

        # ========== VISION (7-11) ==========
        # Use vision pose when fresh (within 0.5 s of current local time).
        # Freshness compares against the local receive timestamp, not the
        # sender's wire timestamp — avoids clock-skew bugs that could close
        # the window even when vision is publishing on schedule.
        _now = datetime.now()
        if (self.vision_pose_count > 0
                and self.vision_pose_recv_time is not None):
            _vision_age = (_now - self.vision_pose_recv_time).total_seconds()
            _vision_fresh = _vision_age < 0.5
        else:
            _vision_age = None
            _vision_fresh = False

        if (not _vision_fresh
                and self.vision_pose_count > 0
                and (self._vision_stale_log_time is None
                     or (_now - self._vision_stale_log_time).total_seconds() > 2.0)):
            print(f"% SKalman: vision stale  age={_vision_age:.2f}s  "
                  f"src={self.vision_pose_source}  count={self.vision_pose_count}")
            self._vision_stale_log_time = _now
        if _vision_fresh:
            z[7, 0] = float(self.vision_pose[0, 0])                            # vision x
            z[8, 0] = float(self.vision_pose[1, 0])                            # vision y
            z[9, 0] = float(self.vision_pose[2, 0])                            # vision z
            z[10, 0] = _unwrap_near(float(self.vision_pose[3, 0]), yaw_ref)    # vision yaw
            z[16, 0] = _unwrap_near(float(self.vision_pose[4, 0]), pitch_ref)  # vision pitch

            # XY ve YAW için ölçüm varyansı: sender (svision_pose) bir
            # sigma sağladıysa ondan türet (variance = σ²); yoksa eski
            # kaynağa-özel hardcoded değerlere düş. Sigma minimum bir
            # tabana clamp'lenir ki yanlış config Kalman'ı patlatmasın.
            VAR_XY_FLOOR_M2  = 0.0001          # σ_xy en az 1 cm
            VAR_YAW_FLOOR_RAD2 = 1.0e-4        # σ_yaw en az ~0.6°

            if self.vision_pose_sigma_xy is not None:
                vxy = max(self.vision_pose_sigma_xy ** 2, VAR_XY_FLOOR_M2)
                variances[7]  = vxy
                variances[8]  = vxy
            elif self.vision_pose_source == 'aruco':
                # ArUco: x, y tamamen kamera + field map → güvenilir
                variances[7]  = 0.10
                variances[8]  = 0.10
            elif self.vision_pose_source == 'landmark':
                # Landmark fix: kameranın gördüğü Y-fork / T / fixture
                # endpoint'in haritadaki konumunu doğrudan kullanır. Mutlak
                # bir fix (drift'siz), ama bant ucuna ne kadar yakınsak
                # o kadar belirsizlik bırakır — confidence_radius=0.20 m
                # disc'i içinde uniform varsayımıyla σ ≈ R/√3 ≈ 11 cm,
                # yani var ≈ 0.013 m². 0.05 = σ ≈ 22 cm seçimi muhafazakar
                # tarafta; sahadan veri gelince keskinleştirilir.
                variances[7]  = 0.05
                variances[8]  = 0.05
            else:
                # Bant: along-track encoder/IMU'dan geliyor → x, y'ye az güven
                # Yalnızca yaw ve lateral düzeltmeye izin ver
                variances[7]  = 0.80
                variances[8]  = 0.80

            variances[9]  = 0.50   # z her zaman field map'ten

            if self.vision_pose_sigma_yaw is not None:
                variances[10] = max(self.vision_pose_sigma_yaw ** 2, VAR_YAW_FLOOR_RAD2)
            else:
                variances[10] = 0.05   # yaw her zaman kamera + field map → güvenilir

            variances[16] = 0.20   # pitch
        else:
            z[7, 0] = 0.0
            z[8, 0] = 0.0
            z[9, 0] = 0.0
            z[10, 0] = 0.0
            z[16, 0] = float(x_pred[6, 0])
            variances[7]  = missing_var
            variances[8]  = missing_var
            variances[9]  = missing_var
            variances[10] = missing_var

        # ========== IMU DERIVED - ACCELEROMETER VELOCITY (11) ==========
        # Velocity from accelerometer integration (imu_derived.acc_velocity)
        if imu_derived.acc_velocity_upd_cnt > 0:
            z[11, 0] = float(imu_derived.acc_velocity)     # accel velocity
        else:
            z[11, 0] = 0.0
            variances[11] = missing_var

        # ========== IMU DERIVED - YAW SOURCES (12-13) ==========
        # Yaw from gyro integration (imu.yawg)
        if imu.yawgUpdCnt > 0:
            z[12, 0] = _unwrap_near(float(imu.yawg), yaw_ref)  # gyro integrated yaw
        else:
            z[12, 0] = 0.0
            variances[12] = missing_var
        
        # Yaw from magnetometer (imu_derived.mag_yaw)
        if imu_derived.mag_yaw_upd_cnt > 0:
            z[13, 0] = _unwrap_near(float(imu_derived.mag_yaw), yaw_ref)  # magnetometer yaw
        else:
            z[13, 0] = 0.0
            variances[13] = missing_var

        # ========== MULTI-SOURCE PITCH (14-16) ==========
        # Pitch from accelerometer: atan2(acc_x, -acc_z)
        if imu.accUpdCnt > 0:
            acc_x = float(imu.acc[0])
            acc_z = float(imu.acc[2])
            pitch_acc = math.atan2(acc_x, -acc_z)
            z[14, 0] = _unwrap_near(pitch_acc, pitch_ref)  # accelerometer pitch (NEW)
        else:
            z[14, 0] = float(x_pred[6, 0])
            variances[14] = missing_var
        
        # Pitch from magnetometer (imu_derived.mag_pitch)
        if imu_derived.mag_pitch_upd_cnt > 0:
            z[15, 0] = _unwrap_near(float(imu_derived.mag_pitch), pitch_ref)  # magnetometer pitch (RESTORED)
        else:
            z[15, 0] = float(x_pred[6, 0])
            variances[15] = missing_var
        
        R = np.diag(variances).astype(float)
        return z, R

    def _control_vector(self) -> np.ndarray:
        from sensors.spose import pose

        u_left = float(pose.wheelVelocity[0])
        u_right = float(pose.wheelVelocity[1])
        return np.array([[u_left], [u_right]], dtype=float)

    def set_manual_input(self, u_left: float, u_right: float):
        self.manual_u = np.array([[float(u_left)], [float(u_right)]], dtype=float)

    def clear_manual_input(self):
        self.manual_u = None
    
    def decode_teleoperation(self, msg: str) -> bool:
        """Decode teleoperation command from JSON payload.
        
        Expected format:
        {"linear_velocity": 0.5, "angular_velocity": 0.2, "timestamp": "..."}
        
        Converts linear velocity and angular velocity to left/right wheel velocities.
        """
        try:
            data = json.loads(msg)
            linear_v = float(data.get('linear_velocity', 0.0))
            angular_v = float(data.get('angular_velocity', 0.0))
            
            # Convert (v, omega) to wheel velocities using differential drive kinematics
            # v_left = v - (wheelbase/2) * omega
            # v_right = v + (wheelbase/2) * omega
            half_wheelbase = self.track_width / 2.0
            v_left = linear_v - half_wheelbase * angular_v
            v_right = linear_v + half_wheelbase * angular_v
            
            # Store as control input
            self.teleop_cmd = {
                'linear_velocity': linear_v,
                'angular_velocity': angular_v,
                'v_left': v_left,
                'v_right': v_right
            }
            self.teleop_cmd_time = datetime.now()
            self.teleop_enabled = True
            return True
        except Exception as e:
            print(f"# Kalman: Failed to decode teleoperation: {e}")
            return False
    
    def _get_teleoperation_u(self) -> object:
        """Get teleoperation control input if available and fresh.
        
        Returns None if teleoperation is not active or has timed out.
        """
        if not self.teleop_enabled or self.teleop_cmd is None or self.teleop_cmd_time is None:
            return None
        
        elapsed = (datetime.now() - self.teleop_cmd_time).total_seconds()
        if elapsed > self.teleop_timeout_s:
            self.teleop_enabled = False
            return None
        
        return np.array([
            [float(self.teleop_cmd['v_left'])],
            [float(self.teleop_cmd['v_right'])]
        ], dtype=float)

    def _current_u(self) -> np.ndarray:
        # Check teleoperation first (higher priority)
        teleop_u = self._get_teleoperation_u()
        if teleop_u is not None:
            return teleop_u
        
        if self.manual_u is not None:
            return self.manual_u.copy()
        return self._control_vector()
    
    def get_measurements(self):
        """Return last measurement vector as dict with named channels."""
        if self.last_measurement is None:
            return {}
        return {name: val for name, val in zip(self.MEAS_NAMES, self.last_measurement)}
    
    def publish_state(self) -> bool:
        """Publish current state estimate to MQTT.
        
        Publishes to: robobot/kalman/state
        Format: JSON with position, velocity, orientation, and covariance.
        """
        try:
            from util.uservice import service
            
            # Rate limit publishing
            now = datetime.now()
            elapsed = (now - self.last_state_publish_time).total_seconds()
            if elapsed < self.state_publish_interval:
                return False
            
            if not self.has_estimate():
                return False
            
            # Extract state
            x = self.kf.x.flatten()
            P_diag = np.diag(self.kf.P)
            
            # Build message
            state_msg = {
                'timestamp': now.isoformat(),
                'update_count': self.update_count,
                'position': {
                    'x': float(x[0]),
                    'y': float(x[1]),
                    'z': float(x[2])
                },
                'velocity': {
                    'linear': float(x[3]),
                    'angular': float(x[4])
                },
                'orientation': {
                    'yaw': float(x[5]),
                    'pitch': float(x[6])
                },
                'covariance_diag': {
                    'x_std': float(np.sqrt(max(0, P_diag[0]))),
                    'y_std': float(np.sqrt(max(0, P_diag[1]))),
                    'z_std': float(np.sqrt(max(0, P_diag[2]))),
                    'velocity_std': float(np.sqrt(max(0, P_diag[3]))),
                    'angular_std': float(np.sqrt(max(0, P_diag[4]))),
                    'yaw_std': float(np.sqrt(max(0, P_diag[5]))),
                    'pitch_std': float(np.sqrt(max(0, P_diag[6])))
                },
                'raw_state': x.tolist(),
                'teleop_active': self.teleop_enabled
            }
            
            # Predicted state (motion model output, before measurement update)
            if self.x_pred_last is not None:
                xp = self.x_pred_last.flatten()
                state_msg['x_pred'] = {
                    'x':                float(xp[0]),
                    'y':                float(xp[1]),
                    'z':                float(xp[2]),
                    'velocity':         float(xp[3]),
                    'angular_velocity': float(xp[4]),
                    'yaw':              float(xp[5]),
                    'pitch':            float(xp[6]),
                }

            # Raw measurement vector (all 17 channels, named)
            if self.last_measurement is not None:
                state_msg['measurements'] = {
                    name: val
                    for name, val in zip(self.MEAS_NAMES, self.last_measurement)
                }

            payload = json.dumps(state_msg)
            service.send("robobot/kalman/state", payload)
            self.last_state_publish_time = now
            return True
            
        except Exception as e:
            # Don't spam errors
            pass
        
        return False

    def _bootstrap_from_measurement(self):
        from sensors.spose import pose
        from sensors.simu import imu

        # MQTT callbacks can arrive before setup() in startup races.
        if self.kf is None:
            self._create_filter()

        imu_derived.update_from_streams()

        x0 = self.default_state.copy()
        if self.vision_pose_count > 0:
            x0[0, 0] = float(self.vision_pose[0, 0])
            x0[1, 0] = float(self.vision_pose[1, 0])
            x0[2, 0] = float(self.vision_pose[2, 0])
            x0[5, 0] = float(self.vision_pose[3, 0])
            x0[6, 0] = float(self.vision_pose[4, 0])
        else:
            pass  # x,y,z come from default_state, no encoder override
        x0[3, 0] = float(imu_derived.acc_velocity) if imu_derived.acc_velocity_upd_cnt > 0 else float(pose.velocity())
        x0[4, 0] = float(imu.gyro[2]) if imu.gyroUpdCnt > 0 else float(pose.turnrate())
        x0[5, 0] = _wrap_angle_rad(float(x0[5, 0]))
        x0[6, 0] = _wrap_angle_rad(float(x0[6, 0]))
        self.kf.x = x0
        self.kf.P = np.eye(7, dtype=float)

    def set_vision_pose(self, x: float, y: float, yaw: float, pitch: float = 0.0, z: float = 0.0, timestamp=None, source: str = 'unknown',
                          sigma_xy: float = None, sigma_yaw: float = None,
                          n_used: int = None, confidence: float = None):
        if timestamp is None:
            timestamp = datetime.now()
        elif isinstance(timestamp, (int, float)):
            timestamp = datetime.fromtimestamp(float(timestamp))
        self.vision_pose = np.array(
            [[float(x)], [float(y)], [float(z)], [_wrap_angle_rad(float(yaw))], [_wrap_angle_rad(float(pitch))]],
            dtype=float,
        )
        self.vision_pose_time = timestamp
        # Freshness uses local receive time, not the wire timestamp. The wire
        # timestamp is sender-side time.time(); if sender and receiver clocks
        # disagree (different machines, NTP skew, etc.) the staleness window
        # is closed before any vision update can land. Local receive time
        # avoids that whole class of bug.
        self.vision_pose_recv_time = datetime.now()
        self.vision_pose_count += 1
        self.vision_pose_source = source
        self.vision_pose_sigma_xy   = float(sigma_xy)   if sigma_xy   is not None else None
        self.vision_pose_sigma_yaw  = float(sigma_yaw)  if sigma_yaw  is not None else None
        self.vision_pose_n_used     = int(n_used)       if n_used     is not None else None
        self.vision_pose_confidence = float(confidence) if confidence is not None else None

    def clear_vision_pose(self):
        self.vision_pose = np.zeros((5, 1), dtype=float)
        self.vision_pose_time = None
        self.vision_pose_recv_time = None
        self.vision_pose_count = 0
        self.vision_pose_source = 'unknown'
        self.vision_pose_sigma_xy   = None
        self.vision_pose_sigma_yaw  = None
        self.vision_pose_n_used     = None
        self.vision_pose_confidence = None

    def _decode_vision_pose(self, msg: str) -> bool:
        gg = msg.split()
        if len(gg) < 5:
            return False
        timestamp = float(gg[0])
        # Positional layout (frozen for backward compatibility):
        #   gg[0] timestamp
        #   gg[1..5] x y z yaw pitch    (6-token short form omits z)
        #   gg[6]    source             (optional in oldest senders)
        # Trailing tokens may be KEY=VALUE extras (sxy=, syaw=, n=, conf=) —
        # these are produced by SVisionPose-aware senders and are safe to
        # ignore on consumers that don't expect them.
        # gg[6] is `source` only when it isn't a KEY=VALUE extra. Otherwise
        # extras start at gg[6]; this happens with senders that omit the
        # source token but still attach sigma metadata.
        if len(gg) >= 7 and '=' not in gg[6]:
            source = gg[6]
            extras_start = 7
        else:
            source = 'unknown'
            extras_start = 6

        extras = {}
        for tok in gg[extras_start:]:
            if '=' in tok:
                k, _, v = tok.partition('=')
                extras[k] = v

        sigma_xy   = self._safe_float(extras.get('sxy'))
        sigma_yaw  = self._safe_float(extras.get('syaw'))
        n_used     = self._safe_int(extras.get('n'))
        confidence = self._safe_float(extras.get('conf'))

        if len(gg) >= 6:
            self.set_vision_pose(gg[1], gg[2], gg[4], pitch=gg[5], z=gg[3],
                                  timestamp=timestamp, source=source,
                                  sigma_xy=sigma_xy, sigma_yaw=sigma_yaw,
                                  n_used=n_used, confidence=confidence)
        else:
            self.set_vision_pose(gg[1], gg[2], gg[3], pitch=gg[4], z=0.0,
                                  timestamp=timestamp, source=source,
                                  sigma_xy=sigma_xy, sigma_yaw=sigma_yaw,
                                  n_used=n_used, confidence=confidence)
        return True

    @staticmethod
    def _safe_float(s):
        try:
            return float(s) if s is not None else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _safe_int(s):
        try:
            return int(s) if s is not None else None
        except (TypeError, ValueError):
            return None

    def _measurement_time(self, topic: str):
        from sensors.spose import pose
        from sensors.simu import imu

        if topic == "T0/pose":
            return pose.poseTime
        if topic == "T0/vel":
            return pose.wheelVelocityTime
        if topic == "T0/gyro":
            return imu.gyroTime
        if topic == "T0/acc":
            return imu.accTime
        if topic == "T0/yawg":
            return imu.yawgTime
        if topic == "T0/mag":
            return imu.magTime
        if topic == "T0/vision_pose":
            return self.vision_pose_time
        return None

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
        self._manual_reset = True
        self.update_count = 1
        self.last_u = np.zeros((2, 1), dtype=float)

    def reset_from_measurement(self):
        """Reset state directly from latest measurement vector."""
        if self.kf is None:
            self._create_filter()
        self._bootstrap_from_measurement()
        self.last_update_time = None
        self.update_count = 1
        self.last_u = np.zeros((2, 1), dtype=float)

    def decode(self, topic, _msg):
        if not self.enabled:
            return False

        if topic == "T0/vision_pose":
            if not self._decode_vision_pose(_msg):
                return False
        elif topic == "teleop/cmd":
            # Teleoperation command from keyboard/controller
            self.decode_teleoperation(_msg)
            return False  # Don't update filter on teleop only
        elif topic not in self.UPDATE_TOPICS:
            return False

        now_t = self._measurement_time(topic)
        if now_t is None:
            return False
        if self.last_update_time is None:
            # Bootstrap disabled — always use the state set via reset()
            self._manual_reset = False
            self.last_update_time = now_t
            self.update_count = 1
            self.publish_state()
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
        
        # Store predicted state for monitoring/display
        self.x_pred_last = self.kf.x.copy()

        z, R = self._measurement_vector_and_cov(self.kf.x)
        self.kf.H = self._fixed_measurement_matrix()
        self.kf.R = R
        self.last_measurement = z.flatten().tolist()  # Store for publishing
        self.kf.update(z)

        self.kf.x[5, 0] = _wrap_angle_rad(float(self.kf.x[5, 0]))
        self.kf.x[6, 0] = _wrap_angle_rad(float(self.kf.x[6, 0]))

        self.last_update_time = now_t
        self.update_count += 1
        self.publish_state()
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
            self._bootstrap_from_measurement()
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

    def predict(self):
        """Get the last predicted state (before measurement update)."""
        return self.x_pred_last.flatten().tolist() if self.x_pred_last is not None else None

    def last_input(self):
        return self.last_u.flatten().tolist()

    def terminate(self):
        print(f"% Kalman estimator terminated (updates={self.update_count})")


kalman = SKalman()