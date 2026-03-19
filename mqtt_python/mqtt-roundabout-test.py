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
PITCH_TEST          = False   # True = drive 2 s forward while printing pitch, then stop

# --- platform approach ---
SKIP_PLATFORM       = False   # True = skip drive-onto-platform (arc only)
PLATFORM_VELOCITY   = 0.2     # m/s while driving onto platform
PLATFORM_PITCH_DETECT = 0.052  # rad (~3°): pitch below -3° signals we hit the edge (climbing = negative pitch)
PLATFORM_PITCH_LEVEL  = 0.035  # rad (~2°): considered level when |pitch| < 2° again
PLATFORM_DEBOUNCE     = 3      # consecutive samples required to confirm edge or level (avoids single-spike false triggers)
PLATFORM_MAX_DIST_M   = 1.5    # m: safety stop if platform never detected

# --- turn after platform ---
PLATFORM_EXTRA_M = -0.05  # metres after platform (negative = back up, positive = forward)
TURN_DEGREES     = 90     # degrees to turn once on platform
TURN_DIRECTION   = "right"  # "left" or "right"

# --- roundabout arc ---
ARC_ANGLE_DEG = 270     # degrees of heading change around the roundabout
ARC_RADIUS_M  = 0.50    # metres (path radius; shrink if too wide)
ARC_DIRECTION = "left"  # "left" or "right"
ARC_VELOCITY  = 0.2     # m/s
# ----------------------------------------

def pitch_rad():
  """Pitch angle (rad) from accelerometer. Positive = nose up."""
  return math.atan2(-imu.acc[0], imu.acc[2])

def driveOntoRoundabout():
  """Drive forward slowly until level on the platform, then stop."""
  state = 0  # 0=approach, 1=tilting (on edge), 99=done
  detect_count = 0  # consecutive samples below PLATFORM_PITCH_DETECT
  level_count  = 0  # consecutive samples within PLATFORM_PITCH_LEVEL
  pose.tripBreset()
  print(f"% driveOntoRoundabout: forward at {PLATFORM_VELOCITY} m/s")
  service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue: moving
  service.send("robobot/cmd/ti", f"rc {PLATFORM_VELOCITY:.2f} 0.0")
  while not service.stop:
    p = pitch_rad()
    dist = pose.tripB
    print(f"# platform state={state}  pitch={np.degrees(p):+.1f}°  dist={dist:.3f} m  "
          f"det={detect_count} lvl={level_count}")
    if dist > PLATFORM_MAX_DIST_M:
      print("% driveOntoRoundabout: max distance reached without platform detection — stopping")
      break
    if state == 0:
      if p < -PLATFORM_PITCH_DETECT:   # climbing: pitch goes negative
        detect_count += 1
        if detect_count >= PLATFORM_DEBOUNCE:
          print(f"% tilt confirmed ({np.degrees(p):+.1f}° × {PLATFORM_DEBOUNCE}) — waiting for level")
          state = 1
          detect_count = 0
      else:
        detect_count = 0  # spike: reset
    elif state == 1:
      if abs(p) < PLATFORM_PITCH_LEVEL:
        level_count += 1
        if level_count >= PLATFORM_DEBOUNCE:
          print(f"% level confirmed ({np.degrees(p):+.1f}° × {PLATFORM_DEBOUNCE}) — stopping")
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
  """Drive forward at PLATFORM_VELOCITY for 2 s while printing pitch. Ctrl+C to stop early."""
  print("% PITCH_TEST mode — driving 2 s forward while printing pitch")
  print("% columns: time_s  pitch_deg  acc_x  acc_y  acc_z")
  pose.tripBreset()
  service.send("robobot/cmd/ti", f"rc {PLATFORM_VELOCITY:.2f} 0.0")
  t0 = t.time()
  det = 0
  while not service.stop and (t.time() - t0) < 2.0:
    p = pitch_rad()
    det = det + 1 if p < -PLATFORM_PITCH_DETECT else 0
    print(f"  t={t.time()-t0:.2f}s  pitch={np.degrees(p):+6.2f}°  det={det}  "
          f"acc=[{imu.acc[0]:+6.3f}, {imu.acc[1]:+6.3f}, {imu.acc[2]:+6.3f}]")
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
