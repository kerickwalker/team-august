#!/usr/bin/env python3

# Line-following mission for Robobot (DTU).
#
# Run with:   python3 mqtt-linefollow.py -e        (normal)
#             python3 mqtt-linefollow.py -e -s      (silent, no debug prints)
#             python3 mqtt-linefollow.py -e --now   (skip IR start-gate, go immediately)
#
# The robot drives forward until it finds the line, then follows it.
# It counts crossings and reacts:
#   crossing 1, 2  → go straight through
#   crossing 3     → hard 90° turn (left or right, detected from sensor pattern)
#   crossing 4     → stop, turn 45° right, drive ~1.3 m, done

import time as t
import numpy as np
from datetime import datetime
from setproctitle import setproctitle

# Robot modules
from spose import pose        # encoder odometry (tripB = distance, tripBh = heading change)
from sir import ir            # IR distance sensor (used to detect start-gate)
from sedge import edge        # line/edge sensor array + PD line controller
from sgpio import gpio        # GPIO (LEDs, start button)
from uservice import service  # MQTT connection, argparse, send/stop helpers
from uteensy import start_teensy_interface, stop_teensy_interface

############################################################
# Crossing-count tuning constants
############################################################

CROSSING_AT_CNT       = 2    # minimum sensor lines active to call it a crossing
CROSSINGS_GO_STRAIGHT = 2    # first N crossings: go straight (no hard turn)
CROSSING_STOP_AT      = 4    # stop at this crossing number (1-based)
CROSSING_LEAVE_DELAY_S = 0.5 # debounce: must be clear of crossing for this long before counting it
START_AT_CROSSING     = 0    # debug: set to N to skip ahead (e.g. 4 = place robot before crossing 4)

############################################################
# Primitive drive helpers
############################################################

def driveTurn(angle_deg, direction):
  """Rotate in place by angle_deg. direction: 'left' or 'right'.
  Stops when odometry heading change (pose.tripBh) reaches the target."""
  state = 0
  pose.tripBreset()
  turn_rate = 0.5 if direction == "left" else -0.5   # rad/s; positive = left
  angle_rad = np.radians(angle_deg)
  if not service.is_quiet():
    print(f"% driveTurn: {angle_deg:.0f}° {direction}")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc 0.0 {turn_rate:.2f}")
      state = 1
    elif state == 1:
      # Use magnitude so the check works regardless of odometry sign convention
      if abs(pose.tripBh) >= angle_rad:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        state = 2
    elif state == 2:
      # Wait for the robot to actually stop before returning
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      if not service.is_quiet():
        print(f"# driveTurn: turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} s")
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 0 0")

def driveDistance(meters, velocity=0.2):
  """Drive straight forward for a fixed distance (m) at velocity (m/s).
  Stops when odometry distance (pose.tripB) reaches the target."""
  state = 0
  pose.tripBreset()
  if not service.is_quiet():
    print(f"% driveDistance: {meters:.2f} m at {velocity:.2f} m/s")
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
      # Wait for the robot to fully stop
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      if not service.is_quiet():
        print(f"# driveDistance: drove {pose.tripB:.3f} m in {pose.tripBtimePassed():.3f} s")
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break
    t.sleep(0.05)
  service.send("robobot/cmd/T0", "leds 16 0 0 0")

############################################################
# Main mission
############################################################

def driveToLine():
  """Full line-following mission.

  Phases:
    state 0-1 : drive forward until the IR start-gate clears and line is found
    state 10  : follow the line; handle crossings by count
    state 20  : end sequence after 4th crossing (turn + drive away)
    state 2   : wait for full stop then exit
  """
  state = 0
  pose.tripBreset()
  dist_to_line = 0

  # Crossing-count debounce state
  crossing_count = 0
  was_at_crossing = False
  last_time_at_crossing = None

  # Recovery: remember when line was last valid
  lost_line_since = None

  # Hard-turn cooldown: avoid double-turning at the same crossing
  last_hard_turn_time = None
  hard_turn_cooldown_s = 2.0

  if not service.is_quiet():
    print("% driveToLine: starting")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green: running

  while not service.stop:

    # --- State 0: wait for IR start-gate, then begin moving ---
    if state == 0:
      # --now flag skips the IR check and starts immediately
      if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
        service.send("robobot/cmd/T0", "servo 1 -800 100")          # raise servo
        service.send("robobot/cmd/ti", "rc 0.2 0.0")                # drive forward
        service.send("robobot/cmd/T0/", "lognow 3")                 # start Teensy log
        state = 1

    # --- State 1: driving toward the line ---
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        # Timeout: line never found — stop
        service.send("robobot/cmd/ti/", "rc 0.0 0.0")
        state = 2
      if edge.lineValidCnt > 4:
        # Line found — start PD line controller (target = centre, error = 0)
        edge.lineControl(refPosition=0)
        dist_to_line = pose.tripB
        pose.tripBreset()
        lost_line_since = None
        if START_AT_CROSSING > 0:
          # Debug: pretend we've already passed some crossings
          crossing_count = START_AT_CROSSING - 1
          if not service.is_quiet():
            print(f"% skipping to crossing_count={crossing_count}")
        if not service.is_quiet():
          print(f"% line found after {dist_to_line:.2f} m -> state 10")
        state = 10

    # --- State 2: wait for the robot to stop, then exit ---
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99

    # --- State 10: line-following + crossing logic ---
    elif state == 10:
      edge.mission_crossing_count = crossing_count  # displayed in sedge status line

      # --- Crossing detection with leave-delay debounce ---
      at_crossing_now = edge.crossingLineCnt >= CROSSING_AT_CNT
      now = datetime.now()
      if at_crossing_now:
        # Reset leave timer while we're still at the crossing
        last_time_at_crossing = now
        was_at_crossing = True
      else:
        # Only count the crossing after we've been clear for CROSSING_LEAVE_DELAY_S
        if was_at_crossing and last_time_at_crossing is not None:
          if (now - last_time_at_crossing).total_seconds() >= CROSSING_LEAVE_DELAY_S:
            crossing_count += 1
            was_at_crossing = False
            last_time_at_crossing = None
            if not service.is_quiet():
              print(f"% left crossing -> crossing_count={crossing_count}")

      # --- React to crossings (only while physically at one) ---
      if at_crossing_now:
        at_4th_crossing = crossing_count == CROSSING_STOP_AT - 1

        if at_4th_crossing:
          # Crossing 4: stop and run the end sequence
          if not service.is_quiet():
            print("% 4th crossing -> end sequence")
          edge.lineControl(0)
          service.send("robobot/cmd/ti", "rc 0.0 0.0")
          state = 20

        elif crossing_count >= CROSSINGS_GO_STRAIGHT:
          # Crossing 3 (and 5+): hard 90° turn if cooldown has elapsed
          now = datetime.now()
          cooldown_ok = (last_hard_turn_time is None or
                         (now - last_hard_turn_time).total_seconds() >= hard_turn_cooldown_s)
          if cooldown_ok:
            # Detect turn direction from which end of the sensor array is active
            hard_left  = (edge.leftmostAboveIndex == 0 and
                          edge.rightmostAboveIndex is not None and
                          edge.rightmostAboveIndex <= 4)
            hard_right = (edge.rightmostAboveIndex == 7 and
                          edge.leftmostAboveIndex is not None and
                          edge.leftmostAboveIndex >= 3)
            if hard_left:
              edge.lineControl(0)
              service.send("robobot/cmd/ti", "rc 0.0 0.0")
              t.sleep(0.05)
              driveTurn(90, "left")
              last_hard_turn_time = datetime.now()
              edge.lineControl(refPosition=0)   # resume line following
            elif hard_right:
              edge.lineControl(0)
              service.send("robobot/cmd/ti", "rc 0.0 0.0")
              t.sleep(0.05)
              driveTurn(90, "right")
              last_hard_turn_time = datetime.now()
              edge.lineControl(refPosition=0)   # resume line following
        # Crossings 1 and 2: go straight, do nothing

      # --- Line-loss recovery ---
      if edge.lineValidCnt >= 4:
        lost_line_since = None   # line is healthy
      elif edge.lineValidCnt < 2:
        if lost_line_since is None:
          lost_line_since = datetime.now()
        # If line has been lost too long, give up and stop
        if (datetime.now() - lost_line_since).total_seconds() >= edge.recovery_timeout_s:
          edge.lineControl(0)
          service.send("robobot/cmd/ti", "rc 0.0 0.0")
          if not service.is_quiet():
            print("% line lost too long -> recovery stop")
          pose.tripBreset()
          state = 2

    # --- State 20: end sequence after 4th crossing ---
    elif state == 20:
      service.send("robobot/cmd/T0", "servo 1 -800 100")
      t.sleep(0.1)
      driveDistance(0.1)          # clear the crossing
      driveTurn(45, "right")      # aim toward exit
      driveDistance(1.1)          # drive away
      driveTurn(90, "right")
      driveDistance(0.15)

      # Lower servo briefly (e.g. to drop a flag/marker), then raise again
      service.send("robobot/cmd/T0", "servo 1 100 100")
      t.sleep(3.0)
      service.send("robobot/cmd/T0", "servo 1 -800 100")

      state = 2   # wait for full stop then exit

    else:
      # state 99 or any unexpected value → finished
      if not service.is_quiet():
        print(f"% driveToLine: done (dist to line {dist_to_line:.2f} m, "
              f"along line {pose.tripB:.2f} m in {pose.tripBtimePassed():.1f} s)")
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      break

    t.sleep(0.01)   # 100 Hz control loop

  service.send("robobot/cmd/T0", "leds 16 0 0 0")  # LEDs off
  if not service.is_quiet():
    print("% driveToLine: end")

############################################################
# Top-level loop (called once after MQTT connect)
############################################################

def loop():
  from ulog import flog
  oldstate = -1
  state_time = datetime.now()

  service.send("robobot/cmd/T0", "leds 16 30 30 0")  # yellow: ready
  edge.lineControl(0)   # ensure motors are stopped before mission starts

  # Run the line-follow mission, then exit
  state = 103
  while not service.stop:
    if state == 103:
      driveToLine()
      state = 99   # mission complete
    else:
      if not service.is_quiet():
        print(f"% loop: finished (state={state})")
      break

    # Log state transitions
    if state != oldstate:
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      if not service.is_quiet():
        print(f"% State change from {oldstate} to {state}")
      oldstate = state
      state_time = datetime.now()

    t.sleep(0.1)

  # Cleanup: stop robot and turn off LEDs
  service.send("robobot/cmd/T0", "leds 16 0 0 0")
  gpio.set_value(20, 0)
  edge.lineControl(0)
  service.send("robobot/cmd/ti", "rc 0 0")
  t.sleep(0.05)

############################################################
# Entry point
############################################################

if __name__ == "__main__":
  if service.process_running("mqtt-client"):
    print("% mqtt-client is already running - terminating")
    print("%   to kill a stuck instance: pkill mqtt-client   (or pkill -9 mqtt-client)")
  else:
    setproctitle("mqtt-client")
    if not service.is_quiet():
      print("% Starting")
    start_teensy_interface()
    service.setup('localhost')   # connect to local MQTT broker
    if service.connected:
      loop()
    service.terminate()
    stop_teensy_interface()
  if not service.is_quiet():
    print("% Main Terminated")