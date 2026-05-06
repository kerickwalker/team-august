#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

#import sys
#import threading
import time as t
import threading
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service

############################################################

def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      if not gpio.onPi:
        try:
          cv.imshow('frame for analysis', img)
        except:
          print("% mqtt-client::imageAnalysis: failed to show camera image");
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      else:
        print("# imageAnalysis:: image not saved")
      pass
    pass
  pass

############################################################

stateTime = datetime.now()

def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()

############################################################

def driveOneMeter():
  state = 0
  pose.tripBreset()
  print("% Driving 1m -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green
  while not (service.stop):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
      # service.send("robobot/cmd/T0","servo 1 -800 300") # (servo up slow)
      state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        # service.send("robobot/cmd/T0","servo 1 0 0") # (servo front fast)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds; left {edge.posLeft}, right {edge.posRight}")
    t.sleep(0.05)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Driving 1m ------------------------- end")

####################################################################3

def driveMotorsPwmDuration(left_pwm, right_pwm, duration_s):
  """Drive each motor with requested PWM for a fixed duration.

  teensy_interface drives motors from "rc" commands, not direct incoming
  motv commands. So left/right PWM-like values are mapped to equivalent
  (linear velocity, turn rate) and sent as rc.
  """
  max_pwm = 4096.0
  max_linvel = 1  # m/s at full-scale request (conservative)
  wheelbase = 0.22   # m

  # Keep requested values inside expected range for safe conversion.
  left_pwm = max(-max_pwm, min(max_pwm, float(left_pwm)))
  right_pwm = max(-max_pwm, min(max_pwm, float(right_pwm)))
  duration_s = max(0.0, float(duration_s))

  left_n = left_pwm / max_pwm
  right_n = right_pwm / max_pwm
  left_v = left_n * max_linvel
  right_v = right_n * max_linvel

  linvel = 0.5 * (left_v + right_v)
  turnrate = (right_v - left_v) / wheelbase

  print("% Drive motors PWM-duration -----------")
  print(f"# left_pwm={left_pwm:.0f}, right_pwm={right_pwm:.0f}, duration={duration_s:.3f}s")
  service.send("robobot/cmd/T0", "leds 16 0 0 30") # blue while running
  service.send("robobot/cmd/ti", f"rc {linvel:.3f} {turnrate:.3f}")

  t0 = t.time()
  while not service.stop and (t.time() - t0) < duration_s:
    t.sleep(0.02)

  # Always send stop at end.
  service.send("robobot/cmd/ti", "rc 0 0")
  service.send("robobot/cmd/T0", "leds 16 0 100 0") # green: done
  print("% Drive motors PWM-duration ----------- end")

####################################################################3

# Mission / state machine: crossing-count phases (use crossings to differentiate phases)
CROSSING_AT_CNT = 2   # we're "at" a crossing when crossingLineCnt >= this (same debounce as hard turn)
CROSSINGS_GO_STRAIGHT = 2  # first N crossings: go straight (don't hard turn); after that hard turns allowed
CROSSING_STOP_AT = 4  # at this crossing number (1-based): stop; 3rd crossing = hard turn, 4th = stop
CROSSING_LEAVE_DELAY_S = 0.5  # only count "left crossing" after we've been clear of crossing for this long (timer resets while crossing still detected)
START_AT_CROSSING = 0  # 0 = normal (start from first crossing); set 4 to place robot before crossing 4 for testing

def driveToLine():
  state = 0
  pose.tripBreset()
  dist_to_line = 0
  lost_line_since = None  # when line was first lost (for recovery timeout)
  last_hard_turn_time = None  # cooldown after a 90° turn
  hard_turn_cooldown_s = 2.0
  # Crossing-count state machine: only increment after we've been clear of crossing for CROSSING_LEAVE_DELAY_S
  crossing_count = 0   # number of crossings we've left (incremented after leave delay)
  was_at_crossing = False  # True while we were at a crossing and haven't yet completed the leave delay
  last_time_at_crossing = None  # time when we were last at a crossing (reset every sample while at crossing)
  if not service.is_quiet():
    print("% Driving to line ---------------------- right ir start ---")
  service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
  while not (service.stop):
    if state == 0: # forward towards line
      if ir.ir[0] < 0.2:
        service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0/","lognow 3") # (start Teensy log)
        # service.send("robobot/cmd/T0","servo 1 -800 300") # (servo up slow)
        state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      if edge.lineValidCnt > 4:
        # start follow line (center, e=0)
        edge.lineControl(refPosition=0)
        # service.send("robobot/cmd/T0","servo 1 0 0") # (move servo to position 0 - front)
        dist_to_line = pose.tripB
        pose.tripBreset()
        lost_line_since = None
        if START_AT_CROSSING > 0:
          crossing_count = START_AT_CROSSING - 1
          if not service.is_quiet():
            print(f"% start at crossing {START_AT_CROSSING} -> crossing_count={crossing_count}")
        if not service.is_quiet():
          print(" to state 10")
        state = 10
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        if not service.is_quiet():
          print(" to state 99")
        state = 99
    elif state == 10:
      edge.mission_crossing_count = crossing_count  # for follow-line print
      # Crossing-count: only count "left crossing" after we've been clear for CROSSING_LEAVE_DELAY_S (timer resets while still at crossing)
      at_crossing_now = edge.crossingLineCnt >= CROSSING_AT_CNT
      now = datetime.now()
      if at_crossing_now:
        last_time_at_crossing = now
        was_at_crossing = True
      else:
        if was_at_crossing and last_time_at_crossing is not None:
          if (now - last_time_at_crossing).total_seconds() >= CROSSING_LEAVE_DELAY_S:
            crossing_count += 1
            was_at_crossing = False
            last_time_at_crossing = None
            if not service.is_quiet():
              print(f"% left crossing (after {CROSSING_LEAVE_DELAY_S}s clear) -> crossing_count={crossing_count}")
      # Crossing-only logic: hard turns and 4th-stop are considered only when we're actually at a crossing
      if at_crossing_now:
        at_4th_crossing = crossing_count == CROSSING_STOP_AT - 1
        # 4th crossing: always stop (never treat as hard turn)
        if at_4th_crossing:
          if not service.is_quiet():
            print("% 4th crossing -> stop, then turn 45° right and drive 1.3 m")
          edge.lineControl(0)
          service.send("robobot/cmd/ti", "rc 0.0 0.0")
          state = 20
        # Hard 90° turns only at crossing 3 (and 5+); never at crossing 1, 2, or 4
        elif crossing_count >= CROSSINGS_GO_STRAIGHT:
          now = datetime.now()
          cooldown_ok = last_hard_turn_time is None or (now - last_hard_turn_time).total_seconds() >= hard_turn_cooldown_s
          if cooldown_ok:
            hard_left = (edge.leftmostAboveIndex == 0 and edge.rightmostAboveIndex is not None and edge.rightmostAboveIndex <= 4)
            hard_right = (edge.rightmostAboveIndex == 7 and edge.leftmostAboveIndex is not None and edge.leftmostAboveIndex >= 3)
            if hard_left:
              edge.lineControl(0)
              service.send("robobot/cmd/ti", "rc 0.0 0.0")
              t.sleep(0.05)
              driveTurn(90, "left")
              last_hard_turn_time = datetime.now()
              edge.lineControl(refPosition=0)  # resume following center (e=0)
            elif hard_right:
              edge.lineControl(0)
              service.send("robobot/cmd/ti", "rc 0.0 0.0")
              t.sleep(0.05)
              driveTurn(90, "right")
              last_hard_turn_time = datetime.now()
              edge.lineControl(refPosition=0)  # resume following center (e=0)
        # Crossing 1 or 2: do nothing (go straight), just keep following line
      # Line control + recovery run in sedge (turn in place toward lastLineSide when lost)
      if edge.lineValidCnt >= 4:
        lost_line_since = None  # have line again
      elif edge.lineValidCnt < 2:
        if lost_line_since is None:
          lost_line_since = datetime.now()
        if (datetime.now() - lost_line_since).total_seconds() >= edge.recovery_timeout_s:
          edge.lineControl(0)
          service.send("robobot/cmd/ti","rc 0.0 0.0")
          if not service.is_quiet():
            print(" to state 2 (recovery timeout)")
          pose.tripBreset()
          state = 2
      pass
    elif state == 20:
      # After 4th crossing: turn right 45°, then forward 1.3 m
      t.sleep(0.1)
      driveTurn(45, "right")
      driveDistance(1.3)
      state = 2
    else:
      if not service.is_quiet():
        print(f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      # service.send("robobot/cmd/T0","servo 1 500 200") # (move servo down slow)
      break;
    # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
    t.sleep(0.01)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  if not service.is_quiet():
    print("% Driving to line ------------------------- end")

####################################################################3

def driveTurn(angle_deg, direction):
  """Turn in place by angle_deg (degrees). direction: 'left' (positive) or 'right' (negative). Usable from driveToLine."""
  state = 0
  pose.tripBreset()
  turn_rate = 0.5 if direction == "left" else -0.5
  angle_rad = np.radians(angle_deg)
  if not service.is_quiet():
    print(f"% Driving {angle_deg:.0f}° turn {direction} -------------------------")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc 0.0 {turn_rate:.2f}")
      state = 1
    elif state == 1:
      # Stop when we've turned at least angle_rad (use magnitude: odometry sign may differ by robot)
      done = abs(pose.tripBh) >= angle_rad
      if done:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        state = 2
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      if not service.is_quiet():
        print(f"# driveTurn {direction}: turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} s")
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 0 0")
  if not service.is_quiet():
    print("% Driving turn ------------------------- end")

def driveDistance(meters, velocity=0.2):
  """Drive forward for meters (m). Same logic as driveOneMeter; usable from driveToLine."""
  state = 0
  pose.tripBreset()
  if not service.is_quiet():
    print(f"% Driving {meters:.2f} m forward -------------------------")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc {velocity:.2f} 0.0")
      state = 1
    elif state == 1:
      if pose.tripB >= meters or pose.tripBtimePassed() > 20:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        state = 2
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      if not service.is_quiet():
        print(f"# driveDistance: drove {pose.tripB:.3f} m in {pose.tripBtimePassed():.3f} s")
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 0 0")
  if not service.is_quiet():
    print("% Driving forward ------------------------- end")

def driveTurnPi():
  state = 0
  pose.tripBreset()
  print("% Driving a Pi turn -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green
  while not (service.stop):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti","rc 0.0 0.5") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      print(f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds; left {edge.posLeft}, right {edge.posRight}")
    t.sleep(0.05)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Driving a Pi turn ------------------------- end")

####################################################################3

def loop():
  from ulog import flog
  state = 0
  images = 0
  ledon = True
  oldstate = -1
  service.send("robobot/cmd/T0", "leds 16 30 30 0") # LED 16: yellow - waiting
  if service.args.meter:
    state = 101 # run 1m
  elif service.args.pi:
    state = 102 # turn 180 deg
  elif service.args.edge:
    state = 103 # find edge and follow line
  elif service.args.motpwm is not None:
    state = 104 # one-shot per-motor PWM test
  elif service.args.usestate > 0:
    state = service.args.usestate
  else:
    # Default: stationary mode (state 200)
    state = 200
  if not service.is_quiet():
    print(f"% Starting at state {state}")
  # elif not service.args.now:
  #   print("% Ready, press start button")
  # main state machine
  edge.lineControl(0)  # make sure line control is off (velocity 0)
  while not (service.stop):
    if state == 0: # wait for start signal
      start = True # gpio.start() or service.args.now
      if start:
        if not service.is_quiet():
          print("% Starting")
        service.send("robobot/cmd/T0","leds 16 0 0 30") # blue: running
        service.send("robobot/cmd/ti","rc 0.25 0.0") # (forward m/s, turn-rate rad/sec)
        # service.send("robobot/cmd/T0","servo 1 100 300") # (servo down slow)
        state = 12 # until no more line
        pose.tripBreset() # use trip counter/timer B
    elif state == 12: # following line
      if pose.tripB > 0.5 or pose.tripBtimePassed() > 10:
        # start turning
        edge.lineControl(0)  # stop following line
        pose.tripBreset()
        service.send("robobot/cmd/ti","rc 0.1 0.5") # turn left
        # service.send("robobot/cmd/T0","servo 1 -800 1000") # (servo up faster)
        state = 14 # turn left
    elif state == 14: # turning left
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 10:
        state = 20 # finished
        service.send("robobot/cmd/ti","rc 0 0") # stop for images
        # service.send("robobot/cmd/T0","servo 1 0 1000") # (servo forward faster)
      # print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
    elif state == 20: # image analysis
      imageAnalysis(images == 2)
      images += 1
      # blink LED
      if ledon:
        service.send("robobot/cmd/T0","leds 16 0 64 0")
        gpio.set_value(20, 1)
      else:
        service.send("robobot/cmd/T0","leds 16 0 30 30")
        gpio.set_value(20, 0)
      ledon = not ledon
      # finished?
      if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
        images = 0
        state = 99
      pass
    elif state == 101:
      driveOneMeter();
      state = 100
    elif state == 102:
      driveTurnPi();
      state = 100
    elif state == 103:
      driveToLine()
      state = 100
    elif state == 104:
      left_pwm, right_pwm, duration_s = service.args.motpwm
      driveMotorsPwmDuration(left_pwm, right_pwm, duration_s)
      state = 100
    elif state == 200: # stationary mode - just wait for stop signal
      service.send("robobot/cmd/T0","leds 16 0 100 0") # green: stationary
      service.send("robobot/cmd/ti","rc 0.0 0.0") # ensure stopped
      if stateTimePassed() > 300.0:
        print("% Stationary timeout reached (60s), stopping")
        state = 99
      t.sleep(1.0) # just loop and wait
      pass
    else: # abort
      if not service.is_quiet():
        print(f"% Mission finished/aborted; state={state}")
      break
    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    # if cam.useCam:
    #   imageAnalysis(True)
    #   if not gpio.onPi:
    #     # do not wait is no image is shown
    #     key = cv.waitKey(100) # ms
    #     if key > 0: # e.g. Esc (key=27) pressed with focus on image
    #       break
    #
    # note state change and reset state timer
    if state != oldstate:
      # flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      if not service.is_quiet():
        print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.1)
    pass # end of while loop
  # end of mission, turn LEDs off and stop
  service.send("robobot/cmd/T0","leds 16 0 0 0")
  gpio.set_value(20, 0)
  edge.lineControl(0)  # stop following line
  service.send("robobot/cmd/ti","rc 0 0")
  # service.send("robobot/cmd/T0","servo 1 0 0")
  t.sleep(0.05)
  pass

############################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      # set title of process, so that it is not just called Python
      setproctitle("mqtt-client")
      if not service.is_quiet():
        print("% Starting")
      # where is the MQTT data server:
      service.setup('localhost') # localhost
      #service.setup('10.197.217.81') # Juniper
      #service.setup('10.197.217.80') # Newton
      # service.setup('bode.local') # Bode
      if service.connected:
        start_mjpeg_server(port=7124)
        t_perception = threading.Thread(
            target=perception_thread,
            kwargs={"show_window": False},
            daemon=True,
            name="perception",
        )
        t_perception.start()
        loop()
      service.terminate()
    if not service.is_quiet():
      print("% Main Terminated")
