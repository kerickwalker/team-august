#!/usr/bin/env python3

import math
from datetime import datetime


def _wrap_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ImuDerivedMeasurements:
    """Derived IMU channels used by Kalman."""

    def __init__(self):
        self.gravity_m_s2 = 9.82
        self.is_initialized = False
        self.init_time = None

        self.acc_velocity = 0.0
        self.acc_velocity_time = datetime.now()
        self.acc_velocity_upd_cnt = 0
        self.acc_velocity_interval = 1.0
        self.acc_bias = 0.0
        self.acc_forward_offset = 0.0
        self.acc_pitch = 0.0
        self.acc_pitch_offset = 0.0
        self.acc_pitch_time = datetime.now()
        self.acc_pitch_upd_cnt = 0
        self.acc_pitch_interval = 1.0
        self._last_acc_upd_seen = 0

        self.mag_yaw = 0.0
        self.mag_yaw_time = datetime.now()
        self.mag_yaw_upd_cnt = 0
        self.mag_yaw_interval = 1.0
        self.mag_yaw_offset = 0.0
        self.mag_pitch = 0.0
        self.mag_pitch_time = datetime.now()
        self.mag_pitch_upd_cnt = 0
        self.mag_pitch_interval = 1.0
        self.mag_pitch_offset = 0.0
        self._last_mag_upd_seen = 0

    def _compute_acc_pitch(self, imu) -> float:
        ax = float(imu.acc[0])
        az = float(imu.acc[2])
        return math.atan2(ax, -az)

    def _compute_mag_orientation(self, imu, pose):
        mx = float(imu.mag[0])
        my = float(imu.mag[1])
        mz = float(imu.mag[2])
        pitch_for_yaw = float(pose.pose[3]) if pose.poseCnt > 0 else self.acc_pitch

        mx_level = mx * math.cos(pitch_for_yaw) + mz * math.sin(pitch_for_yaw)
        my_level = my
        if abs(mx_level) < 1e-9 and abs(my_level) < 1e-9:
            return None, None

        yaw = math.atan2(my_level, mx_level)
        horiz = math.sqrt(mx * mx + my * my)
        pitch = math.atan2(-mz, horiz)
        return yaw, pitch

    def initialize(self):
        """Capture startup references so derived IMU channels become relative to robot start pose."""
        from simu import imu
        from spose import pose

        if imu.accUpdCnt > 0:
            self.acc_pitch_offset = self._compute_acc_pitch(imu)
            self.acc_pitch = 0.0
            self.acc_pitch_time = imu.accTime
            self.acc_pitch_upd_cnt = 1
            self.acc_forward_offset = float(imu.acc[0]) - self.gravity_m_s2 * math.sin(self.acc_pitch_offset)
            self.acc_bias = 0.0
            self.acc_velocity = 0.0
            self.acc_velocity_time = imu.accTime
            self.acc_velocity_upd_cnt = 0
            self._last_acc_upd_seen = imu.accUpdCnt

        if imu.magUpdCnt > 0:
            yaw0, pitch0 = self._compute_mag_orientation(imu, pose)
            if yaw0 is not None:
                self.mag_yaw_offset = yaw0
                self.mag_pitch_offset = pitch0
                self.mag_yaw = 0.0
                self.mag_pitch = 0.0
                self.mag_yaw_time = imu.magTime
                self.mag_pitch_time = imu.magTime
                self.mag_yaw_upd_cnt = 0
                self.mag_pitch_upd_cnt = 0
                self._last_mag_upd_seen = imu.magUpdCnt

        self.init_time = datetime.now()
        self.is_initialized = True

    def _update_acc_velocity(self, imu, pose):
        if imu.accUpdCnt == 0:
            return
        if imu.accUpdCnt == self._last_acc_upd_seen:
            return
        self._last_acc_upd_seen = imu.accUpdCnt

        if self.acc_velocity_upd_cnt == 0:
            self.acc_velocity_time = imu.accTime
            self.acc_velocity = 0.0
            self.acc_velocity_upd_cnt = 1
            return

        dt = (imu.accTime - self.acc_velocity_time).total_seconds()
        self.acc_velocity_time = imu.accTime
        if dt <= 1e-6:
            return
        dt = max(0.001, min(0.2, dt))

        acc_pitch_raw = self._compute_acc_pitch(imu)
        dt_pitch = (imu.accTime - self.acc_pitch_time).total_seconds()
        self.acc_pitch = _wrap_angle_rad(acc_pitch_raw - self.acc_pitch_offset)
        self.acc_pitch_time = imu.accTime
        if dt_pitch > 1e-6:
            if self.acc_pitch_upd_cnt <= 1:
                self.acc_pitch_interval = dt_pitch
            else:
                self.acc_pitch_interval = (self.acc_pitch_interval * 99 + dt_pitch) / 100
        if self.acc_pitch_upd_cnt == 0:
            self.acc_pitch_upd_cnt = 1
        else:
            self.acc_pitch_upd_cnt += 1

        pitch = float(pose.pose[3]) if pose.poseCnt > 0 else self.acc_pitch
        forward_acc = float(imu.acc[0]) - self.gravity_m_s2 * math.sin(pitch) - self.acc_forward_offset
        encoder_velocity = float(pose.velocity()) if pose.wheelVelocityCnt > 0 else 0.0

        if pose.wheelVelocityCnt > 0 and abs(encoder_velocity) < 0.03:
            self.acc_bias = (self.acc_bias * 99.0 + forward_acc) / 100.0

        corrected_acc = forward_acc - self.acc_bias
        if abs(corrected_acc) < 0.03:
            corrected_acc = 0.0

        self.acc_velocity += corrected_acc * dt
        if pose.wheelVelocityCnt > 0:
            blend = 0.02 if abs(encoder_velocity) > 0.02 else 0.10
            self.acc_velocity = (1.0 - blend) * self.acc_velocity + blend * encoder_velocity
        if pose.wheelVelocityCnt > 0 and abs(encoder_velocity) < 0.02 and abs(self.acc_velocity) < 0.01:
            self.acc_velocity = 0.0

        if self.acc_velocity_upd_cnt == 1:
            self.acc_velocity_interval = dt
        else:
            self.acc_velocity_interval = (self.acc_velocity_interval * 99 + dt) / 100
        self.acc_velocity_upd_cnt += 1

    def _update_mag_yaw(self, imu, pose):
        if imu.magUpdCnt == 0:
            return
        if imu.magUpdCnt == self._last_mag_upd_seen:
            return
        self._last_mag_upd_seen = imu.magUpdCnt

        yaw_raw, pitch_raw = self._compute_mag_orientation(imu, pose)
        if yaw_raw is None:
            return

        self.mag_yaw = _wrap_angle_rad(yaw_raw - self.mag_yaw_offset)
        self.mag_pitch = _wrap_angle_rad(pitch_raw - self.mag_pitch_offset)
        if self.mag_yaw_upd_cnt == 0:
            self.mag_yaw_time = imu.magTime
            self.mag_yaw_upd_cnt = 1
            self.mag_pitch_time = imu.magTime
            self.mag_pitch_upd_cnt = 1
            return

        dt = (imu.magTime - self.mag_yaw_time).total_seconds()
        self.mag_yaw_time = imu.magTime
        self.mag_pitch_time = imu.magTime
        if dt > 1e-6:
            if self.mag_yaw_upd_cnt == 1:
                self.mag_yaw_interval = dt
                self.mag_pitch_interval = dt
            else:
                self.mag_yaw_interval = (self.mag_yaw_interval * 99 + dt) / 100
                self.mag_pitch_interval = (self.mag_pitch_interval * 99 + dt) / 100
        self.mag_yaw_upd_cnt += 1
        self.mag_pitch_upd_cnt += 1

    def update_from_streams(self):
        from simu import imu
        from spose import pose

        self._update_acc_velocity(imu, pose)
        self._update_mag_yaw(imu, pose)


imu_derived = ImuDerivedMeasurements()
