#!/usr/bin/env python3

# Standalone roundabout test script for Robobot (DTU).
#
# Mission: follow a straight line until it ends, then drive a full arc
# (circle/roundabout) for a configurable number of degrees.
#
# Run with:
#   python3 mqtt_roundabout_test.py          (waits for IR start-gate)
#   python3 mqtt_roundabout_test.py --now    (skip IR gate, start immediately)
#   python3 mqtt_roundabout_test.py --now -s (silent mode, no debug prints)
#
# Tune the parameters in the TUNING section below, then run to test.
# Once happy with the behaviour, integrate driveRoundabout() into the main mission.

import time as t
import numpy as np
from datetime import datetime
from setproctitle import setproctitle

from spose import pose
from sir import ir
from sedge import edge
from sgpio import gpio
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface

############################################################
# TUNING — adjust these parameters to match your setup
############################################################

# Geometry of the roundabout circle
CIRCLE_DIAMETER_M = 0.8     # diameter of the circle the robot drives (metres)
                             # measure the physical platform diameter and set this

# How far around the circle to travel
ARC_TOTAL_DEGREES = 450.0   # total heading change in degrees
                             # 360 = one full lap, 450 = 1.25 laps, 720 = two laps
                             # increase if the robot needs to pass through gates multiple times

# Speed on the arc
ARC_VELOCITY = 0.2           # forward speed in m/s (keep low for first tests)
                             # turn rate is derived automatically: w = v / radius

# Direction of the circle
ARC_DIRECTION = "left"       # "left" (CCW) or "right" (CW)

# Line-end detection — how many consecutive samples with no valid line
# before we decide the line has truly ended (not just sensor noise)
LINE_LOST_CONFIRM = 5        # number of 10 ms samples (= 50 ms) all below threshold
LINE_LOST_CNT     = 2        # lineValidCnt value below which a sample counts as "no line"

# Safety timeout for the line-follow phase
LINE_FOLLOW_TIMEOUT = 20.0   # seconds; abort if line is never lost within this time

############################################################
# Phase 1: Follow the line until it ends
############################################################

def driveLineUntilEnd():
  """Follow the centre of a line at default speed until the line disappears.

  Uses the sedge PD controller (edge.lineControl). When the line sensor
  confidence (lineValidCnt) drops below LINE_LOST_CNT for LINE_LOST_CONFIRM
  consecutive samples, the line is considered ended and the function returns.
  """
  pose.tripBreset()
  lost_count = 0   # consecutive samples with no valid line

  if not service.is_quiet():
    print("% driveLineUntilEnd: waiting for line...")

  # Wait for IR start-gate (same as main mission)
  while not service.stop:
    if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
      break
    t.sleep(0.05)

  service.send("robobot/cmd/T0", "leds 16 0 100 0")   # green: line following

  # Wait until the line sensor array sees the line before enabling control
  if not service.is_quiet():
    print("% driveLineUntilEnd: looking for line, then following...")

  # Start the PD line controller (target = centre of sensor array, error=0)
  edge.lineControl(refPosition=0)

  t0 = t.time()
  while not service.stop:

    # Safety timeout
    if t.time() - t0 > LINE_FOLLOW_TIMEOUT:
      if not service.is_quiet():
        print("% driveLineUntilEnd: timeout — line never ended")
      break

    # Count consecutive samples where line confidence is below threshold
    if edge.lineValidCnt < LINE_LOST_CNT:
      lost_count += 1
    else:
      lost_count = 0   # line is still there, reset counter

    if lost_count >= LINE_LOST_CONFIRM:
      # Line has definitively ended (not just a momentary noise dip)
      if not service.is_quiet():
        print(f"% driveLineUntilEnd: line ended after {pose.tripB:.2f} m "
              f"in {pose.tripBtimePassed():.1f} s")
      break

    t.sleep(0.01)   # 100 Hz loop

  # Stop motors and disable the line controller before handing off to arc phase
  edge.lineControl(0)
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  service.send("robobot/cmd/T0", "leds 16 0 0 0")

############################################################
# Phase 2: Drive the roundabout arc
############################################################

def driveRoundabout():
  """Drive a circular arc for ARC_TOTAL_DEGREES at ARC_VELOCITY.

  The circle radius is derived from CIRCLE_DIAMETER_M.
  Turn rate is computed as:  w = v / r  (rad/s)

  Progress is tracked using pose.tripBh (odometry heading change since reset).
  The function returns when the robot has turned the required number of degrees.
  """
  radius_m  = CIRCLE_DIAMETER_M / 2.0
  sign      = 1.0 if ARC_DIRECTION == "left" else -1.0
  turn_rate = sign * ARC_VELOCITY / radius_m    # rad/s
  target_rad = np.radians(ARC_TOTAL_DEGREES)

  pose.tripBreset()   # reset heading and distance counters for this phase

  if not service.is_quiet():
    print(f"% driveRoundabout: diameter={CIRCLE_DIAMETER_M:.2f} m, "
          f"radius={radius_m:.2f} m, v={ARC_VELOCITY:.2f} m/s, "
          f"w={turn_rate:.3f} rad/s, target={ARC_TOTAL_DEGREES:.0f}°")

  service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue: arc running
  service.send("robobot/cmd/ti", f"rc {ARC_VELOCITY:.3f} {turn_rate:.3f}")

  last_print = t.time()
  while not service.stop:
    degrees_done = np.degrees(abs(pose.tripBh))

    # Print progress every 0.5 s
    if not service.is_quiet() and t.time() - last_print >= 0.5:
      print(f"# arc: {degrees_done:.1f}° / {ARC_TOTAL_DEGREES:.0f}°  "
            f"dist={pose.tripB:.2f} m")
      last_print = t.time()

    # Stop when the target heading change is reached
    if abs(pose.tripBh) >= target_rad:
      if not service.is_quiet():
        print(f"% driveRoundabout: target reached "
              f"({degrees_done:.1f}° in {pose.tripBtimePassed():.1f} s, "
              f"{pose.tripB:.2f} m)")
      break

    t.sleep(0.02)   # 50 Hz loop

  # Stop motors and wait for the robot to fully halt
  service.send("robobot/cmd/ti", "rc 0.0 0.0")
  while not service.stop and abs(pose.velocity()) > 0.001:
    t.sleep(0.02)

  service.send("robobot/cmd/T0", "leds 16 0 0 0")
  if not service.is_quiet():
    print("% driveRoundabout: done")

############################################################
# Top-level loop
############################################################

def loop():
  SERVO_1_DOWN_POS = 480
  SERVO_2_DOWN_POS = -480

  SERVO_1UP_POS = -475
  SERVO_2UP_POS = 475

  SERVO_DOWN_VEL = 100     
  SERVO_UP_VEL = 100     

  # Test servo 1
  #service.send("robobot/cmd/T0", f"servo 1 {SERVO_1UP_POS} {SERVO_DOWN_VEL}")
  # t.sleep(3)
  #service.send("robobot/cmd/T0", f"servo 1 {-2000} {0}")

  # Test servo 2
  # service.send("robobot/cmd/T0", f"servo 2 {-2000} {0}")
  # service.send("robobot/cmd/T0", f"servo 1 {-2000} {0}")
  # t.sleep(1)
  service.send("robobot/cmd/T0", f"servo 1 {SERVO_1UP_POS} {SERVO_DOWN_VEL}")
  service.send("robobot/cmd/T0", f"servo 2 {SERVO_2UP_POS} {SERVO_DOWN_VEL}")
  #service.send("robobot/cmd/T0", f"servo 2 {SERVO_2_DOWN_POS} {SERVO_DOWN_VEL}")
  # t.sleep(3)
  # service.send("robobot/cmd/T0", f"servo 2 {-2000} {0}")
  # service.send("robobot/cmd/T0", f"servo 1 {-2000} {0}")
  #service.send("robobot/cmd/T0", f"servo 1 {200} {SERVO_DOWN_VEL}")
  #service.send("robobot/cmd/T0", f"servo 2 {0} {SERVO_DOWN_VEL}")


############################################################
# Entry point
############################################################

if __name__ == "__main__":
  if service.process_running("mqtt-client"):
    print("% mqtt-client is already running - terminating")
    print("%   to kill a stuck instance: pkill mqtt-client  (or pkill -9 mqtt-client)")
  else:
    setproctitle("mqtt-client")
    if not service.is_quiet():
      print("% Starting roundabout test")
    start_teensy_interface()
    service.setup('localhost')
    if service.connected:
      loop()
    service.terminate()
    stop_teensy_interface()
  if not service.is_quiet():
    print("% Main Terminated")
