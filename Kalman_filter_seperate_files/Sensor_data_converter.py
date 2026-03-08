from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence

import numpy as np


def _wrap_angle_rad(angle: float) -> float:
	return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _safe_cos_pitch(pitch_rad: float, eps: float = 1e-6) -> float:
	c = math.cos(pitch_rad)
	if abs(c) < eps:
		return eps if c >= 0.0 else -eps
	return c


def _rk4_step_scalar(value: float, dt_s: float, deriv_value: float) -> float:
	"""RK4 for dy/dt = deriv_value (held constant over dt)."""
	k1 = deriv_value
	k2 = deriv_value
	k3 = deriv_value
	k4 = deriv_value
	return value + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def _trapezoid_step_scalar(value: float, dt_s: float, prev_deriv: float, curr_deriv: float) -> float:
	"""Integrate sampled derivative using trapezoidal rule (Tustin)."""
	return value + 0.5 * dt_s * (prev_deriv + curr_deriv)


@dataclass
class _InternalState:
	pos_x_enc_m: float = 0.0
	pos_y_enc_m: float = 0.0
	pos_z_enc_m: float = 0.0
	vel_acc_mps: float = 0.0
	vel_acc_initialized: bool = False
	prev_acc_forward_mps2: Optional[float] = None
	acc_bias_mps2: float = 0.0
	yaw_gyro_rad: float = 0.0
	pitch_gyro_rad: float = 0.0
	prev_yaw_mag_rad: Optional[float] = None


class SensorDataConverter:
	"""Build 14 measurements for a 7-state Kalman filter (2 sensors per state).

	Output order and key names:
	1.  pos_x_s1, pos_y_s1, pos_z_s1   (vision)
	2.  velocity_s1                     (wheel-encoder linear speed)
	3.  angular_velocity_s1             (yaw rate from magnetometer derivative)
	4.  yaw_s1                          (magnetometer yaw)
	5.  pitch_s1                        (accelerometer pitch)
	6.  pos_x_s2, pos_y_s2, pos_z_s2   (encoder + yaw + pitch dead reckoning)
	7.  velocity_s2                     (accel integrated speed via trapezoidal rule)
	8.  angular_velocity_s2             (yaw rate from gyro + pitch)
	9.  yaw_s2                          (gyro integrated yaw with pitch correction)
	10. pitch_s2                        (gyro integrated pitch)
	"""

	VECTOR_ORDER_14 = [
		"pos_x_s1",
		"pos_y_s1",
		"pos_z_s1",
		"velocity_s1",
		"angular_velocity_s1",
		"yaw_s1",
		"pitch_s1",
		"pos_x_s2",
		"pos_y_s2",
		"pos_z_s2",
		"velocity_s2",
		"angular_velocity_s2",
		"yaw_s2",
		"pitch_s2",
	]

	def __init__(
		self,
		wheel_radius_m: float,
		gravity_mps2: float = 9.80665,
		velocity_correction_gain: float = 0.30,
		accel_bias_adapt_gain: float = 0.01,
	) -> None:
		if wheel_radius_m <= 0.0:
			raise ValueError("wheel_radius_m must be > 0")
		if velocity_correction_gain < 0.0:
			raise ValueError("velocity_correction_gain must be >= 0")
		if accel_bias_adapt_gain < 0.0:
			raise ValueError("accel_bias_adapt_gain must be >= 0")

		self.wheel_radius_m = float(wheel_radius_m)
		self.gravity_mps2 = float(gravity_mps2)
		self.velocity_correction_gain = float(velocity_correction_gain)
		self.accel_bias_adapt_gain = float(accel_bias_adapt_gain)
		self.state = _InternalState()

	def reset(self) -> None:
		self.state = _InternalState()

	@staticmethod
	def accel_pitch_rad(ax_mps2: float, ay_mps2: float, az_mps2: float) -> float:
		return math.atan2(-ax_mps2, math.sqrt(ay_mps2 * ay_mps2 + az_mps2 * az_mps2))

	@staticmethod
	def tilt_compensated_yaw_mag_rad(
		mx: float,
		my: float,
		mz: float,
		pitch_rad: float,
		roll_rad: float = 0.0,
	) -> float:
		cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
		cr, sr = math.cos(roll_rad), math.sin(roll_rad)

		mx_h = mx * cp + mz * sp
		my_h = mx * sr * sp + my * cr - mz * sr * cp
		return _wrap_angle_rad(math.atan2(-my_h, mx_h))

	def encoder_speed_mps(self, left_mps: float, right_mps: float) -> float:
		return 0.5 * (float(left_mps) + float(right_mps))

	def encoder_angular_to_linear_mps(self, omega_radps: float) -> float:
		return float(omega_radps) * self.wheel_radius_m

	def encoder_step_distance_m(self, left_mps: float, right_mps: float, dt_s: float) -> float:
		return self.encoder_speed_mps(left_mps, right_mps) * dt_s

	def gravity_compensated_forward_accel_mps2(self, ax_mps2: float, pitch_rad: float) -> float:
		"""Estimate forward linear acceleration from body-x accel using pitch."""
		return float(ax_mps2) + self.gravity_mps2 * math.sin(float(pitch_rad))

	def update(
		self,
		dt_s: float,
		vision_pos_xyz_m: Sequence[float],
		encoder_left: float,
		encoder_right: float,
		accel_xyz_mps2: Sequence[float],
		gyro_xyz_radps: Sequence[float],
		mag_xyz: Sequence[float],
		encoder_input_mode: str = "linear_mps",
		pitch_for_yaw_rad: Optional[float] = None,
		pitch_for_velocity_rad: Optional[float] = None,
		yaw_for_position_rad: Optional[float] = None,
		pitch_for_position_rad: Optional[float] = None,
	) -> Dict[str, float]:
		"""Create one 14-measurement sample from raw sensors + optional Kalman pitch/yaw."""
		if dt_s <= 0.0:
			raise ValueError("dt_s must be > 0")

		vx_vis, vy_vis, vz_vis = map(float, vision_pos_xyz_m)
		ax, ay, az = map(float, accel_xyz_mps2)
		_gx, gy, gz = map(float, gyro_xyz_radps)
		mx, my, mz = map(float, mag_xyz)

		if encoder_input_mode == "linear_mps":
			left_mps = float(encoder_left)
			right_mps = float(encoder_right)
		elif encoder_input_mode == "angular_radps":
			left_mps = self.encoder_angular_to_linear_mps(float(encoder_left))
			right_mps = self.encoder_angular_to_linear_mps(float(encoder_right))
		else:
			raise ValueError("encoder_input_mode must be 'linear_mps' or 'angular_radps'")

		pitch_s1 = self.accel_pitch_rad(ax, ay, az)
		pitch_used_for_yaw = pitch_s1 if pitch_for_yaw_rad is None else float(pitch_for_yaw_rad)

		yaw_s1 = self.tilt_compensated_yaw_mag_rad(mx, my, mz, pitch_used_for_yaw)

		if self.state.prev_yaw_mag_rad is None:
			angular_velocity_s1 = 0.0
		else:
			dyaw = _wrap_angle_rad(yaw_s1 - self.state.prev_yaw_mag_rad)
			angular_velocity_s1 = dyaw / dt_s
		self.state.prev_yaw_mag_rad = yaw_s1

		velocity_s1 = self.encoder_speed_mps(left_mps, right_mps)

		pitch_s2 = _wrap_angle_rad(_rk4_step_scalar(self.state.pitch_gyro_rad, dt_s, gy))
		self.state.pitch_gyro_rad = pitch_s2

		cos_pitch = _safe_cos_pitch(pitch_used_for_yaw)
		yaw_rate_from_gyro = gz / cos_pitch
		angular_velocity_s2 = yaw_rate_from_gyro

		yaw_s2 = _wrap_angle_rad(_rk4_step_scalar(self.state.yaw_gyro_rad, dt_s, yaw_rate_from_gyro))
		self.state.yaw_gyro_rad = yaw_s2

		distance_m = self.encoder_step_distance_m(left_mps, right_mps, dt_s)
		yaw_used_for_position = yaw_s2 if yaw_for_position_rad is None else float(yaw_for_position_rad)
		pitch_used_for_position = pitch_s2 if pitch_for_position_rad is None else float(pitch_for_position_rad)

		horizontal = distance_m * math.cos(pitch_used_for_position)
		self.state.pos_x_enc_m += horizontal * math.cos(yaw_used_for_position)
		self.state.pos_y_enc_m += horizontal * math.sin(yaw_used_for_position)
		self.state.pos_z_enc_m += distance_m * math.sin(pitch_used_for_position)

		# Remove gravity projection from body-x accel before speed integration.
		if not self.state.vel_acc_initialized:
			self.state.vel_acc_mps = velocity_s1
			self.state.vel_acc_initialized = True
		pitch_used_for_velocity = pitch_s2 if pitch_for_velocity_rad is None else float(pitch_for_velocity_rad)
		acc_forward_mps2 = self.gravity_compensated_forward_accel_mps2(ax, pitch_used_for_velocity)

		# Adapt accel bias from encoder-vs-integrated speed error to limit long-term drift.
		vel_err = velocity_s1 - self.state.vel_acc_mps
		if dt_s > 1e-6:
			self.state.acc_bias_mps2 -= self.accel_bias_adapt_gain * (vel_err / dt_s)
		acc_forward_corrected_mps2 = acc_forward_mps2 - self.state.acc_bias_mps2

		if self.state.prev_acc_forward_mps2 is None:
			# First sample has no previous derivative; use Euler bootstrap once.
			vel2 = self.state.vel_acc_mps + dt_s * acc_forward_corrected_mps2
		else:
			vel2 = _trapezoid_step_scalar(
				self.state.vel_acc_mps,
				dt_s,
				self.state.prev_acc_forward_mps2,
				acc_forward_corrected_mps2,
			)

		# Light complementary correction anchors drift without replacing accel integration.
		vel2 += self.velocity_correction_gain * dt_s * (velocity_s1 - vel2)

		self.state.prev_acc_forward_mps2 = acc_forward_corrected_mps2
		self.state.vel_acc_mps = vel2

		sample = {
			"pos_x_s1": vx_vis,
			"pos_y_s1": vy_vis,
			"pos_z_s1": vz_vis,
			"velocity_s1": velocity_s1,
			"angular_velocity_s1": angular_velocity_s1,
			"yaw_s1": yaw_s1,
			"pitch_s1": pitch_s1,
			"pos_x_s2": self.state.pos_x_enc_m,
			"pos_y_s2": self.state.pos_y_enc_m,
			"pos_z_s2": self.state.pos_z_enc_m,
			"velocity_s2": vel2,
			"angular_velocity_s2": angular_velocity_s2,
			"yaw_s2": yaw_s2,
			"pitch_s2": pitch_s2,
		}
		return sample

	def as_measurement_vector_14(self, sample: Dict[str, float]) -> np.ndarray:
		return np.array([[float(sample[key])] for key in self.VECTOR_ORDER_14], dtype=float)


