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
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from sensors.spose import pose
from sensors.sir import ir
from sensors.srobot import robot
from sensors.scam import cam
from sensors.sedge import edge
from sgpio import gpio
from sensors.scam import cam
from util.uservice import service

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
      service.send("robobot/cmd/T0","servo 1 -800 300") # (servo up slow)
      state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0","servo 1 0 0") # (servo front fast)
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

def driveToLine():
  state = 0
  pose.tripBreset()
  dist_to_line = 0;
  print("% Driving to line ---------------------- right ir start ---")
  service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
  while not (service.stop):
    if state == 0: # forward towards line
      if ir.ir[0] < 0.2:
        service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0/","lognow 3") # (start Teensy log)
        service.send("robobot/cmd/T0","servo 1 -800 300") # (servo up slow)
        state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      if edge.lineValidCnt > 4:
        # start follow line
        edge.lineControl(0.2, True)
        service.send("robobot/cmd/T0","servo 1 0 0") # (move servo to position 0 - front)
        dist_to_line = pose.tripB
        pose.tripBreset()
        print(" to state 10")
        state = 10
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        print(" to state 99")
        state = 99
    elif state == 10:
      if edge.lineValidCnt < 2:
        edge.lineControl(0, True)
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        print(" to state 2")
        pose.tripBreset()
        state = 2
    else:
      print(f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      service.send("robobot/cmd/T0","servo 1 500 200") # (move servo down slow)
      break;
    # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
    t.sleep(0.01)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Driving to line ------------------------- end")

####################################################################3

def driveTurnPi():
  state = 0
  pose.tripBreset()
  print("% Driving a Pi turn -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green
  while not (service.stop):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti","rc 0.2 0.5") # (forward m/s, turn-rate rad/sec)
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
  from util.ulog import flog
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
  elif service.args.usestate > 0:
    state = service.args.usestate
  print(f"% Starting at state {state}")
  # elif not service.args.now:
  #   print("% Ready, press start button")
  # main state machine
  edge.lineControl(0, True) # make sure line control is off (velocity 0)
  while not (service.stop):
    if state == 0: # wait for start signal
      start = True # gpio.start() or service.args.now
      if start:
        print("% Starting")
        service.send("robobot/cmd/T0","leds 16 0 0 30") # blue: running
        service.send("robobot/cmd/ti","rc 0.25 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0","servo 1 100 300") # (servo down slow)
        state = 12 # until no more line
        pose.tripBreset() # use trip counter/timer B
    elif state == 12: # following line
      if pose.tripB > 0.5 or pose.tripBtimePassed() > 10:
        # start turning
        edge.lineControl(0, True) # stop following line
        pose.tripBreset()
        service.send("robobot/cmd/ti","rc 0.1 0.5") # turn left
        service.send("robobot/cmd/T0","servo 1 -800 1000") # (servo up faster)
        state = 14 # turn left
    elif state == 14: # turning left
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 10:
        state = 20 # finished
        service.send("robobot/cmd/ti","rc 0 0") # stop for images
        service.send("robobot/cmd/T0","servo 1 0 1000") # (servo forward faster)
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
    else: # abort
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
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.1)
    pass # end of while loop
  # end of mission, turn LEDs off and stop
  service.send("robobot/cmd/T0","leds 16 0 0 0")
  gpio.set_value(20, 0)
  edge.lineControl(0, True) # stop following line
  service.send("robobot/cmd/ti","rc 0 0")
  service.send("robobot/cmd/T0","servo 1 0 0")
  t.sleep(0.05)
  pass

##########################################################
# ---------------- CUSTOM FUNCTIONALITY  --------------- #
##########################################################
import sys
import os
import time as t

# Manual teleop control using W/S/A/D keys for forward/backward/left/right

# Platform-specific key handling
if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty
    import select

def read_key():
    """Non-blocking read of latest key (Windows + Linux/macOS)."""
    if os.name == "nt":
        key = None
        # Drain all pending keypresses; keep the latest
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            # Skip special keys (arrows, function keys) which come as b'\x00' or b'\xe0'
            if ch in (b"\x00", b"\xe0"):
                _ = msvcrt.getch()
                continue
            try:
                key = ch.decode("utf-8").lower()
            except UnicodeDecodeError:
                key = None
        return key
    else:
        key = None
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            ch = sys.stdin.read(1)
            if ch:
                key = ch.lower()
        return key

def catch_ball():
  SERVO_DOWN_POS = 100     
  SERVO_UP_POS = -840     
  SERVO_DOWN_VEL = 100     # slow speed
  SERVO_UP_VEL = 100     # slow speed


  # 1) Servo down
  service.send("robobot/cmd/T0", f"servo 1 {SERVO_DOWN_POS} {SERVO_DOWN_VEL}")
  t.sleep(2)

  # 3) Release servo
  service.send("robobot/cmd/T0", f"servo 1 {-2000} {0}")
  t.sleep(5)

  # 3) Servo up
  service.send("robobot/cmd/T0", f"servo 1 {SERVO_UP_POS} {SERVO_UP_VEL}")
  t.sleep(3)


def teleop_control():
    print("WSAD: move | SPACE: stop | C: catch | Q: quit")

    # Put terminal in cbreak mode on Unix; no-op on Windows
    if os.name != "nt":
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
    else:
        old_settings = None

    service.send("robobot/cmd/T0", "leds 16 0 0 50")

    v = 0.0
    w = 0.0
    LOOP_DT = 0.02  # 50 Hz

    try:
        while not service.stop:
            key = read_key()

            if key == 'w':        # forward
                v, w = 0.20, 0.0
            elif key == 's':      # backward
                v, w = -0.20, 0.0
            elif key == 'a':      # left turn
                v, w = 0.0, 0.50
            elif key == 'd':      # right turn
                v, w = 0.0, -0.50
            elif key == ' ':      # stop
                v, w = 0.0, 0.0
            elif key == 'c':      # catch routine
                v, w = 0.0, 0.0
                service.send("robobot/cmd/ti", "rc 0 0")
                catch_ball()
            elif key == 'q':      # quit
                break

            # Always send current command at fixed rate
            service.send("robobot/cmd/ti", f"rc {v} {w}")
            t.sleep(LOOP_DT)

    finally:
        if os.name != "nt" and old_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")

def calibrate_line_sensor_only():
    """
    Calibration-only mode.

    If program is started with -w, this function will:
    - initialize the edge sensor subsystem
    - run white calibration through edge.setup()
    - do nothing else (no driving, no servo motion)
    - return when calibration is done
    """
    print("% calibrate_line_sensor_only(): starting")
    edge.setup()
    print("% calibrate_line_sensor_only(): finished")

def line_follow_test(speed=0.15, follow_left=True, ref_position=0.0):
    """
    Minimal test function for continuous line following.

    - Initializes the line sensor
    - Enables the built-in line follower in sedge.py
    - Keeps running until stopped externally
    """
    print("% line_follow_test(): starting")

    # Make sure line sensor is initialized and data is streaming
    edge.setup()

    # Optional: wait until some normalized sensor data has arrived
    while not service.stop and edge.edge_nUpdCnt == 0:
        t.sleep(0.05)

    print("% line_follow_test(): sensor data received")

    # Enable built-in line follower
    edge.lineControl(speed, follow_left, ref_position)

    try:
        while not service.stop:
            # Optional debug print every 0.1 s
            print(f"# line valid cnt={edge.lineValidCnt}, "
                  f"left={edge.posLeft:.2f}, right={edge.posRight:.2f}, "
                  f"high={edge.high}, avg={edge.average:.1f}")
            t.sleep(0.1)

    finally:
        # Stop safely when function exits
        edge.lineControl(0, follow_left, ref_position)
        service.send("robobot/cmd/ti", "rc 0 0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        print("% line_follow_test(): stopped")
        
##########################################################
# ------------ END OF CUSTOM FUNCTIONALITY  ------------ #
##########################################################

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
      print("% Starting")
      # where is the MQTT data server:
      service.setup('localhost') # localhost
      #service.setup('10.197.218.17') # Public IP
      if service.connected:
        #loop()
        #teleop_control() # manual teleop control
        line_follow_test(speed=0.15, follow_left=True, ref_position=0.0) # test line following
      service.terminate()
    print("% Main Terminated")