#!/usr/bin/env python3
"""Roundabout test: drive onto platform, turn, arc around roundabout.
Run: python mqtt-roundabout-test.py -t
teensy_interface is managed automatically.

Tuning workflow:
  1. Set SKIP_PLATFORM=True and test the arc alone first.
  2. Once arc looks good, set SKIP_PLATFORM=False and test full sequence.
"""
import math
import time as t
import numpy as np
from setproctitle import setproctitle
from spose import pose
from simu import imu
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface

# --- pitch test mode: drive forward for 2 s while printing pitch ---
PITCH_TEST          = True   # True = run pitch test print loop until you stop (Ctrl+C)

# --- platform approach ---
SKIP_PLATFORM       = False   # True = skip drive-onto-platform (arc only)
PLATFORM_VELOCITY   = 0.2     # m/s while driving onto platform
# Pitch thresholds are tuned in DEGREES (more intuitive than radians).
PLATFORM_PITCH_DETECT_DEG = 5.0  # deg: pitch below -3° signals we hit the edge (climbing = negative pitch)
PLATFORM_PITCH_LEVEL_DEG  = 2.0  # deg: considered level when |pitch| < 2° again
PLATFORM_DEBOUNCE           = 3    # consecutive samples required to confirm edge or level (avoids single-spike false triggers)
PLATFORM_MAX_DIST_M   = 1.5    # m: safety stop if platform never detected

# Optional pitch bias calibration: estimate "level" pitch at startup and subtract it.
CALIBRATE_PITCH_BIAS  = True   # True = remove static mounting/IMU pitch offset
PITCH_BIAS_SAMPLES    = 20     # samples used for bias estimation (~0.05s apart during startup)

# Filter pitch to reduce accelerometer noise.
# Moving-average on tilt degrees adds a little lag but makes the debounce thresholds work better.
PITCH_MOVAVG_N        = 3      # larger = smoother but more lag

# Teensy already publishes a tilt estimate on robobot/drive/T0/pose.
# In spose.py this is decoded into pose.pose[3] (radians), so we use that directly.

# --- turn after platform ---
PLATFORM_EXTRA_M = -0.05  # metres after platform (negative = back up, positive = forward)
TURN_DEGREES     = 90     # degrees to turn once on platform
TURN_DIRECTION   = "right"  # "left" or "right"

# --- roundabout arc ---
ARC_ANGLE_DEG = 270     # degrees of heading change around the roundabout
ARC_RADIUS_M  = 0.60    # metres (path radius; shrink if too wide)
ARC_DIRECTION = "left"  # "left" or "right"
ARC_VELOCITY  = 0.2     # m/s
# ----------------------------------------

def pitch_deg():
  """Tilt angle (degrees) from Teensy pose stream. Positive = nose up.

  Teensy angle may wrap into [0, 360). Normalize so that 360 becomes 0,
  and values near the wrap boundary behave like signed angles.
  """
  return normalize_deg180(float(np.degrees(pose.pose[3])))

def normalize_deg180(deg):
  """Normalize an angle in degrees to [-180, 180)."""
  deg = (deg + 180.0) % 360.0 - 180.0
  if abs(deg) < 1e-9:
    deg = 0.0
  return deg

def moving_average(values):
  """Mean of a list (safe for empty)."""
  return sum(values) / max(1, len(values))

def wait_for_pose_update(prev_pose_cnt, timeout_s=2.0):
  """Wait until spose.py has decoded at least one new pose update."""
  t0 = t.time()
  while not service.stop and (pose.poseCnt == prev_pose_cnt) and (t.time() - t0) < timeout_s:
    t.sleep(0.01)
  return pose.poseCnt != prev_pose_cnt

def circular_mean_deg180(degs):
  """Circular mean of angles in degrees for wrap-around safety.

  Input degs can be in any range; output is normalized to [-180, 180).
  """
  if not degs:
    return 0.0
  thetas = np.radians(np.array(degs, dtype=float))
  s = float(np.sin(thetas).sum())
  c = float(np.cos(thetas).sum())
  if abs(s) < 1e-12 and abs(c) < 1e-12:
    return 0.0
  mean = float(np.degrees(np.arctan2(s, c)))
  return normalize_deg180(mean)

def driveOntoRoundabout():
  """Drive forward slowly until level on the platform, then stop."""
  state = 0  # 0=approach, 1=tilting (on edge), 99=done
  detect_count = 0  # consecutive samples below PLATFORM_PITCH_DETECT
  level_count  = 0  # consecutive samples within PLATFORM_PITCH_LEVEL
  pitch_bias_deg = 0.0
  pose.tripBreset()

  # Estimate static pitch bias before motion starts.
  if CALIBRATE_PITCH_BIAS:
    bias_samples = []
    prev = pose.poseCnt
    for _ in range(PITCH_BIAS_SAMPLES):
      wait_for_pose_update(prev)
      prev = pose.poseCnt
      bias_samples.append(pitch_deg())
    pitch_bias_deg = circular_mean_deg180(bias_samples)
    if not service.is_quiet():
      print(f"% pitch bias calibrated: {pitch_bias_deg:+.2f}° from {len(bias_samples)} pose samples")

  print(f"% driveOntoRoundabout: forward at {PLATFORM_VELOCITY} m/s")
  service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue: moving
  service.send("robobot/cmd/ti", f"rc {PLATFORM_VELOCITY:.2f} 0.0")
  pitch_hist = []  # tilt history degrees for moving-average smoothing
  while not service.stop:
    p_raw_deg = normalize_deg180(pitch_deg() - pitch_bias_deg)
    pitch_hist.append(p_raw_deg)
    if len(pitch_hist) > PITCH_MOVAVG_N:
      pitch_hist = pitch_hist[-PITCH_MOVAVG_N:]
    p = normalize_deg180(moving_average(pitch_hist))
    dist = pose.tripB
    print(
      f"# platform state={state}  tilt_filt={p:+.1f}° (tilt_raw {p_raw_deg:+.1f}°)  "
      f"dist={dist:.3f} m  det={detect_count} lvl={level_count}"
    )
    if dist > PLATFORM_MAX_DIST_M:
      print("% driveOntoRoundabout: max distance reached without platform detection — stopping")
      break
    if state == 0:
      if p < -PLATFORM_PITCH_DETECT_DEG:   # climbing: pitch goes negative
        detect_count += 1
        if detect_count >= PLATFORM_DEBOUNCE:
          print(f"% tilt confirmed ({p:+.1f}° × {PLATFORM_DEBOUNCE}) — waiting for level")
          state = 1
          detect_count = 0
      else:
        detect_count = 0  # spike: reset
    elif state == 1:
      if abs(p) < PLATFORM_PITCH_LEVEL_DEG:
        level_count += 1
        if level_count >= PLATFORM_DEBOUNCE:
          print(f"% level confirmed ({p:+.1f}° × {PLATFORM_DEBOUNCE}) — stopping")
          break
      else:
        level_count = 0  # spike: reset
    t.sleep(0.05)
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green: done
  t.sleep(0.3)  # let robot settle before turning
  print("% driveOntoRoundabout — end")

def driveDistance(meters, velocity=0.2):
  """Drive for meters (m). Negative meters = reverse. velocity is always positive.
  Forward: uses odometry (pose.tripB). Reverse: uses time (tripB doesn't track backward)."""
  sign = -1 if meters < 0 else 1
  target = abs(meters)
  vel = sign * abs(velocity)
  direction = "back" if sign < 0 else "fwd"
  print(f"% driveDistance: {meters:.2f} m ({direction}) at {vel:.2f} m/s")
  service.send("robobot/cmd/T0", "leds 16 0 0 30")
  service.send("robobot/cmd/ti", f"rc {vel:.3f} 0.0")
  if sign < 0:
    # reverse: odometry doesn't track backward, use time instead
    t.sleep(target / abs(vel))
  else:
    pose.tripBreset()
    while not service.stop:
      if pose.tripB >= target or pose.tripBtimePassed() > 20:
        break
      t.sleep(0.05)
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  t.sleep(0.1)  # brief settle
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  print(f"% driveDistance end")

def driveTurn(angle_deg, direction):
  """Turn in place by angle_deg. direction: 'left' or 'right'."""
  angle_rad = np.radians(angle_deg)
  turn_rate = 0.5 if direction == "left" else -0.5
  state = 0
  pose.tripBreset()
  print(f"% driveTurn: {angle_deg:.0f}° {direction}")
  service.send("robobot/cmd/T0", "leds 16 0 0 30")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc 0.0 {turn_rate:.2f}")
      state = 1
    elif state == 1:
      if abs(pose.tripBh) >= angle_rad:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        state = 2
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        break
    else:
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  print(f"% driveTurn end — turned {np.degrees(pose.tripBh):.1f}°")

def driveArc(angle_deg, radius_m, direction, velocity=ARC_VELOCITY):
  """Drive a circular arc using odometry heading as stop condition."""
  angle_rad = np.radians(angle_deg)
  sign = 1.0 if direction == "left" else -1.0
  turn_rate = sign * velocity / radius_m
  state = 0
  pose.tripBreset()
  print(f"% driveArc: {angle_deg:.0f}° {direction}, R={radius_m:.2f} m, "
        f"v={velocity:.2f} m/s → w={turn_rate:.3f} rad/s")
  service.send("robobot/cmd/T0", "leds 16 0 0 30")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc {velocity:.3f} {turn_rate:.3f}")
      state = 1
    elif state == 1:
      if abs(pose.tripBh) >= angle_rad:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        state = 2
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        break
    else:
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  print(f"% driveArc end — turned {np.degrees(pose.tripBh):.1f}° in {pose.tripBtimePassed():.2f} s")

def pitch_test():
  """Continuously print tilt and the raw IMU terms that drive it (no driving).

  Teensy tilt (pose.pose[3]) is estimated in `UImu2::estimateTilt()` using:
  - accAng = atan2(-acc[0], -acc[2])
  - gyroTiltRate = gyro[1] (deg/s) -> rad/s
  - u = accAng + gyroTiltRate * tau (firmware uses tau = 1.0; see notes/change_log.md)
  - complementary filter state encoder.pose[3] (published as pose tilt)
  """
  print("% PITCH_TEST mode — printing tilt terms continuously (no driving)")
  print("% columns: t_s  tilt_deg  accAng_deg  gyroY_deg_s  u_deg  acc_x  acc_y  acc_z  gyro_x  gyro_y  gyro_z")
  pose.tripBreset()
  # Ensure robot is not moving while we debug pitch.
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  t0 = t.time()
  while not service.stop:
    # Published Teensy tilt (pose pose[3]) in degrees
    tilt_deg = normalize_deg180(float(np.degrees(pose.pose[3])))

    # Replicate Teensy accAng calculation in degrees:
    # accAng = atan2(-acc[0], -acc[2])
    accAng_deg = normalize_deg180(float(np.degrees(math.atan2(-imu.acc[0], -imu.acc[2]))))

    # Gyro pitch-rate term (Teensy uses gyro[1] and converts to rad/s; in Python gyro is already deg/s)
    gyroY_deg_s = float(imu.gyro[1])

    # Mirrors firmware: u = accAng + gyroTiltRate * tau, tau = 1.0 (gyro in deg/s here)
    u_deg = normalize_deg180(accAng_deg + gyroY_deg_s * 1.0)
    print(
      f"  t={t.time()-t0:.2f}s  "
      f"tilt={tilt_deg:+7.2f}°  "
      f"accAng={accAng_deg:+7.2f}°  "
      f"gyroY={gyroY_deg_s:+7.2f}°/s  "
      f"u={u_deg:+7.2f}°  "
      f"acc=[{imu.acc[0]:+6.3f}, {imu.acc[1]:+6.3f}, {imu.acc[2]:+6.3f}]  "
      f"gyro=[{imu.gyro[0]:+6.3f}, {imu.gyro[1]:+6.3f}, {imu.gyro[2]:+6.3f}]"
    )
    t.sleep(0.1)
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  print("% PITCH_TEST done")

if __name__ == "__main__":
  setproctitle("mqtt-client")
  start_teensy_interface()
  service.setup("localhost")
  if service.connected:
    if PITCH_TEST:
      pitch_test()
    else:
      if not SKIP_PLATFORM:
        driveOntoRoundabout()
        driveDistance(PLATFORM_EXTRA_M)  # nudge forward to centre on platform
        driveTurn(TURN_DEGREES, TURN_DIRECTION)
      driveArc(ARC_ANGLE_DEG, ARC_RADIUS_M, ARC_DIRECTION, ARC_VELOCITY)
  service.terminate()
  stop_teensy_interface()
  print("% Done")
