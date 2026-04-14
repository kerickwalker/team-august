#!/usr/bin/env python3

# Line-following mission for Robobot (DTU).
#
# Run with:   python3 mqtt-linefollow.py -e        (normal)
#             python3 mqtt-linefollow.py -e -s      (silent, no debug prints)
#             python3 mqtt-linefollow.py -e --now   (skip IR start-gate, go immediately)
#
# ── Mission overview ─────────────────────────────────────────────────────────
#
#  1. Drive forward until the IR gate clears and the line sensor finds the line.
#  2. Follow the line.
#       crossing 1  → slight left nudge (Block A), then resume following
#       crossing 2  → go straight
#       crossing 3  → hard 90° turn (direction auto-detected from sensor pattern)
#       crossing 4  → end sequence (state 20)
#  3. When the line physically ends (Block B: Roundabout):
#       slow crawl → stop → 45° right → drive 30 cm → roundabout arc
#  4. After the arc (Block B continued):
#       90° right → drive until line found → resume following from step 2
#  5. End sequence (state 20, unchanged):
#       clear crossing → turns + drive → servo action → done
#
# ── How to enable / disable blocks ──────────────────────────────────────────
#
#  Each optional block has an ENABLE_ flag in the tuning section below.
#  Set it to False to skip that block entirely with no other code changes.
#
# ─────────────────────────────────────────────────────────────────────────────

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

CROSSING_AT_CNT        = 2    # minimum sensor lines active to call it a crossing
CROSSINGS_GO_STRAIGHT  = 2    # first N crossings: go straight before hard turns kick in
CROSSING_STOP_AT       = 4    # stop the mission at this crossing number (1-based)
CROSSING_LEAVE_DELAY_S = 0.5  # debounce: must be clear of crossing for this long before counting it
START_AT_CROSSING      = 0    # debug: set to N to skip ahead (e.g. 4 = place robot before crossing 4)

############################################################
# Block A — Junction nudge
# Set ENABLE_JUNCTION_NUDGE = False to skip this block entirely.
############################################################

ENABLE_JUNCTION_NUDGE = True   # True = slight left turn at the configured crossing
JUNCTION_NUDGE_AT     = 1      # 1-based crossing number that triggers the nudge
JUNCTION_NUDGE_DEG    = 15.0   # degrees to turn left (increase for a sharper nudge)
JUNCTION_NUDGE_RATE   = 0.5    # turn rate in rad/s (lower = slower, more controlled)

############################################################
# Block B — Roundabout
# Set ENABLE_ROUNDABOUT = False to skip the entire roundabout block.
############################################################

ENABLE_ROUNDABOUT = True

# ── Line-end detection (runs inside state 10) ─────────────────────────────────
LINE_END_LOST_CNT     = 2      # lineValidCnt below this counts as "no line" for a sample
LINE_END_LOST_CONFIRM = 5      # consecutive 10 ms samples needed to confirm true line end

# ── Slow-crawl before stopping at line end ────────────────────────────────────
LINE_END_SLOW_SPEED   = 0.08   # m/s during the brief crawl (prevents jerky stop)
LINE_END_SLOW_DIST    = 0.10   # metres to crawl before halting

# ── Approach manoeuvres before the arc ───────────────────────────────────────
ROUNDABOUT_APPROACH_TURN_DEG = 45.0   # CW (right) turn before driving toward roundabout
ROUNDABOUT_APPROACH_DIST_M   = 0.30   # metres to drive straight before entering arc

# ── Arc geometry ─────────────────────────────────────────────────────────────
CIRCLE_DIAMETER_M  = 0.8     # physical diameter of the circle in metres
ARC_TOTAL_DEGREES  = 450.0   # degrees of heading change (360 = one lap, 450 = 1.25 laps)
ARC_VELOCITY       = 0.2     # forward speed on the arc in m/s; turn rate derived automatically
ARC_DIRECTION      = "left"  # "left" (CCW) or "right" (CW)

# ── Post-arc re-acquisition ───────────────────────────────────────────────────
ROUNDABOUT_EXIT_TURN_DEG = 90.0   # CW (right) turn to face the return line
FIND_LINE_VELOCITY       = 0.15   # m/s while searching for the line after the arc
FIND_LINE_TIMEOUT_S      = 10.0   # seconds; give up if line not found within this time

############################################################
# Primitive drive helpers
############################################################

def driveTurn(angle_deg, direction, turn_rate=0.5):
  """Rotate in place by angle_deg. direction: 'left' or 'right'.
  turn_rate (rad/s) controls how quickly the robot rotates (default 0.5).
  Stops when odometry heading change (pose.tripBh) reaches the target."""
  state = 0
  pose.tripBreset()
  signed_rate = turn_rate if direction == "left" else -turn_rate  # positive = left
  angle_rad = np.radians(angle_deg)
  if not service.is_quiet():
    print(f"% driveTurn: {angle_deg:.0f}° {direction} at {turn_rate:.2f} rad/s")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")
  while not service.stop:
    if state == 0:
      service.send("robobot/cmd/ti", f"rc 0.0 {signed_rate:.2f}")
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

def driveRoundabout():
  """Drive a circular arc for ARC_TOTAL_DEGREES at ARC_VELOCITY.

  The circle radius is derived from CIRCLE_DIAMETER_M.
  Turn rate is computed as:  w = v / r  (rad/s)

  Progress is tracked using pose.tripBh (odometry heading change since reset).
  Returns when the robot has turned the required number of degrees."""
  radius_m   = CIRCLE_DIAMETER_M / 2.0
  sign       = 1.0 if ARC_DIRECTION == "left" else -1.0
  turn_rate  = sign * ARC_VELOCITY / radius_m   # rad/s
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

    # Print arc progress every 0.5 s
    if not service.is_quiet() and t.time() - last_print >= 0.5:
      print(f"# arc: {degrees_done:.1f}° / {ARC_TOTAL_DEGREES:.0f}°  "
            f"dist={pose.tripB:.2f} m")
      last_print = t.time()

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
# Main mission
############################################################

def driveToLine():
  """Full line-following mission with optional junction nudge and roundabout.

  State machine:
    state 0   : wait for IR start-gate (or --now flag), then begin moving
    state 1   : drive forward until line is found
    state 2   : wait for full stop, then exit
    state 10  : line-following + crossing logic + line-end detection
    state 11  : [Block B] line ended → slow stop → approach → roundabout arc
    state 12  : [Block B] post-arc → turn right → search for line → resume state 10
    state 20  : end sequence after CROSSING_STOP_AT (turns, servo, drive away)
    state 99  : done
  """
  state = 0
  pose.tripBreset()
  dist_to_line = 0

  # Crossing-count debounce state
  crossing_count       = 0
  was_at_crossing      = False
  last_time_at_crossing = None

  # Recovery: remember when line was last valid
  lost_line_since = None

  # Hard-turn cooldown: avoid double-turning at the same crossing
  last_hard_turn_time  = None
  hard_turn_cooldown_s = 2.0

  # Block A state
  junction_nudge_done = False   # True once the crossing-1 nudge has fired

  # Block B state
  roundabout_done      = False  # True once the arc has fired (prevents a second trigger)
  line_end_lost_count  = 0      # consecutive samples below LINE_END_LOST_CNT

  if not service.is_quiet():
    print("% driveToLine: starting")
  service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green: running

  while not service.stop:

    # ── State 0: wait for IR start-gate, then begin moving ───────────────────
    if state == 0:
      # --now flag skips the IR check and starts immediately
      if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
        service.send("robobot/cmd/T0", "servo 1 -800 100")          # raise servo
        service.send("robobot/cmd/ti", "rc 0.2 0.0")                # drive forward
        service.send("robobot/cmd/T0/", "lognow 3")                 # start Teensy log
        state = 1

    # ── State 1: driving toward the line ─────────────────────────────────────
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

    # ── State 2: wait for the robot to stop, then exit ───────────────────────
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99

    # ── State 10: line-following + crossing logic + line-end detection ───────
    elif state == 10:
      edge.mission_crossing_count = crossing_count  # displayed in sedge status line

      # ── Crossing detection with leave-delay debounce ──────────────────────
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

      # ── React to crossings (only while physically at one) ─────────────────
      if at_crossing_now:
        # current_crossing_1based: +1 because count increments on *leaving*
        current_crossing_1based = crossing_count + 1

        # ── Block A: junction nudge ───────────────────────────────────────────
        # Slight left turn in place at the configured crossing, then resume.
        # Disable this block by setting ENABLE_JUNCTION_NUDGE = False.
        if (ENABLE_JUNCTION_NUDGE and
                current_crossing_1based == JUNCTION_NUDGE_AT and
                not junction_nudge_done):
          if not service.is_quiet():
            print(f"% crossing {current_crossing_1based}: junction nudge left "
                  f"{JUNCTION_NUDGE_DEG:.0f}°")
          edge.lineControl(0)
          service.send("robobot/cmd/ti", "rc 0.0 0.0")
          t.sleep(0.05)
          driveTurn(JUNCTION_NUDGE_DEG, "left", JUNCTION_NUDGE_RATE)
          junction_nudge_done = True
          edge.lineControl(refPosition=0)   # resume line following

        # ── Crossing CROSSING_STOP_AT: trigger the end sequence ──────────────
        elif crossing_count == CROSSING_STOP_AT - 1:
          if not service.is_quiet():
            print(f"% crossing {current_crossing_1based} -> end sequence (state 20)")
          edge.lineControl(0)
          service.send("robobot/cmd/ti", "rc 0.0 0.0")
          state = 20

        # ── Hard 90° turn at crossing 3 (and beyond CROSSINGS_GO_STRAIGHT) ──
        elif crossing_count >= CROSSINGS_GO_STRAIGHT:
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
              edge.lineControl(refPosition=0)
            elif hard_right:
              edge.lineControl(0)
              service.send("robobot/cmd/ti", "rc 0.0 0.0")
              t.sleep(0.05)
              driveTurn(90, "right")
              last_hard_turn_time = datetime.now()
              edge.lineControl(refPosition=0)
        # Crossings 1 and 2 (before CROSSINGS_GO_STRAIGHT): go straight, do nothing

      # ── Block B: line-end detection ───────────────────────────────────────
      # Count consecutive samples where line confidence is below threshold.
      # When enough consecutive samples confirm the line has ended, transition
      # to state 11 (roundabout approach).
      # Disable this block by setting ENABLE_ROUNDABOUT = False.
      if ENABLE_ROUNDABOUT and not roundabout_done and state == 10:
        if edge.lineValidCnt < LINE_END_LOST_CNT:
          line_end_lost_count += 1
        else:
          line_end_lost_count = 0   # line is present, reset counter

        if line_end_lost_count >= LINE_END_LOST_CONFIRM:
          if not service.is_quiet():
            print(f"% line ended (lost {line_end_lost_count} consecutive samples) "
                  f"-> state 11 (roundabout)")
          edge.lineControl(0)
          state = 11

      # ── Line-loss recovery (fallback when roundabout block is disabled) ───
      # If ENABLE_ROUNDABOUT is True, the block above handles line loss first
      # so this only fires when roundabout is disabled or already done.
      if state == 10:   # only run if no state change happened above
        if edge.lineValidCnt >= 4:
          lost_line_since = None   # line is healthy
        elif edge.lineValidCnt < 2:
          if lost_line_since is None:
            lost_line_since = datetime.now()
          if (datetime.now() - lost_line_since).total_seconds() >= edge.recovery_timeout_s:
            edge.lineControl(0)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            if not service.is_quiet():
              print("% line lost too long -> recovery stop")
            pose.tripBreset()
            state = 2

    # ── State 11: Block B — line ended → slow stop → approach → roundabout ──
    # To remove this block: set ENABLE_ROUNDABOUT = False at the top of the file.
    elif state == 11:
      if not service.is_quiet():
        print("% State 11: line ended — slow stop, approach, roundabout arc")

      # (a) Slow crawl to avoid a jerky halt
      pose.tripBreset()
      service.send("robobot/cmd/ti", f"rc {LINE_END_SLOW_SPEED:.2f} 0.0")
      while not service.stop and pose.tripB < LINE_END_SLOW_DIST:
        t.sleep(0.01)
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      while not service.stop and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)

      # (b) Turn clockwise to face the roundabout entry
      driveTurn(ROUNDABOUT_APPROACH_TURN_DEG, "right")

      # (c) Drive toward the roundabout circle
      driveDistance(ROUNDABOUT_APPROACH_DIST_M)

      # (d) Execute the roundabout arc
      driveRoundabout()

      t.sleep(0.1)
      roundabout_done = True
      state = 12

    # ── State 12: Block B — find line after roundabout ────────────────────────
    # Turn right, drive forward until the line sensor sees the line, then resume.
    elif state == 12:
      if not service.is_quiet():
        print("% State 12: post-roundabout — turn right, search for line")

      # (a) Turn clockwise to face the return line
      driveTurn(ROUNDABOUT_EXIT_TURN_DEG, "right")

      # (b) Drive forward until the line sensor is confident, or timeout
      pose.tripBreset()
      service.send("robobot/cmd/ti", f"rc {FIND_LINE_VELOCITY:.2f} 0.0")
      while not service.stop:
        if edge.lineValidCnt > 4:
          break
        if pose.tripBtimePassed() > FIND_LINE_TIMEOUT_S:
          break
        t.sleep(0.01)
      service.send("robobot/cmd/ti", "rc 0.0 0.0")
      while not service.stop and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)

      if edge.lineValidCnt > 4:
        # Line found — re-enable PD controller and go back to line following
        edge.lineControl(refPosition=0)
        lost_line_since     = None
        line_end_lost_count = 0
        if not service.is_quiet():
          print("% Line found after roundabout — resuming line follow (state 10)")
        state = 10
      else:
        if not service.is_quiet():
          print("% Failed to find line after roundabout — stopping")
        state = 2

    # ── State 20: end sequence after CROSSING_STOP_AT ────────────────────────
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
