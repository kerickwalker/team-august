#!/usr/bin/env python3

# Full mission script for Robobot (DTU).
#
# Run with:   python3 mqtt-full-mission.py -e        (normal)
#             python3 mqtt-full-mission.py -e -s      (silent, no debug prints)
#             python3 mqtt-full-mission.py -e --now   (skip IR start-gate, go immediately)
#
# ── Mission overview ─────────────────────────────────────────────────────────
#
#  [INIT]     Drive forward until IR gate clears and line sensor finds the line.
#
#  [FOLLOW]   Follow the line. React at each crossing:
#               Crossing 1 → turn LEFT 45° (while moving forward), resume following
#               Crossing 2 → go straight
#               Crossing 3 → go straight
#               Crossing 4 → hard 90° turn (direction auto-detected from sensor)
#               Crossing 5 → stop → [CATCH BALL SEQUENCE]
#
#  [ROUNDABOUT]  When the line physically ends (between crossings 1 and 2):
#               drive 20 cm → turn LEFT 45° → arc 450° → turn LEFT 90°
#               → find line → resume [FOLLOW] with crossing_count = 1
#
#  [CATCH BALL]  Turn right 90° → drive 10 cm → lower servo 1 & 2 →
#               drive 3 cm → raise servo 1 & 2 → reverse 10 cm → done.
#
# ── How to tune ──────────────────────────────────────────────────────────────
#
#  All tunable values live in the CONFIGURATION section below, grouped by
#  mission phase (LINE, CROSSINGS, CROSSING1, ROUNDABOUT_*, LINE_END, CATCH_BALL).
#  The sequences (ROUNDABOUT_SEQUENCE, END_SEQUENCE) define each blocking
#  phase as an ordered list of steps — add, remove, or reorder steps freely.
#
# ─────────────────────────────────────────────────────────────────────────────

import time as t
import numpy as np
from datetime import datetime
from types import SimpleNamespace
from setproctitle import setproctitle

# Robot modules
from spose import pose        # encoder odometry: tripB = distance, tripBh = heading change
from sir import ir            # IR distance sensor — used to detect the start gate
from sedge import edge        # line/edge sensor array + PD line controller
from sgpio import gpio        # GPIO (LEDs, start button)
from uservice import service  # MQTT connection, argparse, send/stop helpers
from uteensy import start_teensy_interface, stop_teensy_interface


################################################################
# CONFIGURATION
# All tunable values are here — grouped by mission phase.
# Edit only this section; mission logic below stays unchanged.
################################################################

# ── General line following ────────────────────────────────────────────────────
LINE = SimpleNamespace(
    speed           = 0.25,  # m/s — normal line-following speed
    approach_speed  = 0.15,  # m/s — reduced speed after crossing 1 until roundabout done
    lost_timeout    = 5.0,   # s   — stop if line lost for longer than this (recovery)
)

# ── Crossing detection and counting ──────────────────────────────────────────
CROSSINGS = SimpleNamespace(
    sensor_threshold     = 2,     # crossingLineCnt must reach this to register a crossing
    leave_delay_s        = 0.5,   # s — robot must be clear of crossing this long before count increments
    go_straight_until    = 3,     # crossings 1..N go straight; above this = hard turn
                                  #   (crossing 1 is handled separately; 2, 3 go straight; 4 turns)
    stop_at              = 5,     # stop and run end sequence at this crossing number (1-based)
    hard_turn_cooldown_s = 2.0,   # s — minimum time between hard turns (prevents double-turn)
)

# ── Crossing 1 — turn left while moving forward, then resume line following ──
CROSSING1 = SimpleNamespace(
    turn_deg      = 35.0,   # degrees to turn
    turn_dir      = "left", # "left" or "right"
    turn_rate     = 0.5,    # rad/s
    forward_speed = 0.15,   # m/s forward while turning (0 = full stop before turning)
)

# ── Roundabout entry — triggered when the line physically ends ────────────────
ROUNDABOUT_ENTRY = SimpleNamespace(
    drive_dist_m = 0.29,    # m — drive forward after line ends before turning
    drive_speed  = 0.3,    # m/s
    turn_deg     = 90.0,    # degrees to turn to face the roundabout circle
    turn_dir     = "right",  # "left" or "right"
)

# ── Roundabout arc — geometry and speed ───────────────────────────────────────
# Turn rate is derived automatically: w = speed / (diameter / 2)
ROUNDABOUT_ARC = SimpleNamespace(
    diameter_m    = 0.73,     # m — physical diameter of the circle
    total_degrees = 450.0,   # ° — heading change target (360 = one full lap, 450 = 1.25 laps)
    speed         = 0.20,    # m/s — forward speed on the arc
    direction     = "left",  # "left" (CCW) or "right" (CW)
)

# ── Roundabout exit — find the return line after the arc ─────────────────────
ROUNDABOUT_EXIT = SimpleNamespace(
    turn_deg     = 0.0,    # degrees to turn after the arc
    turn_dir     = "right",  # "left" or "right"
    seek_speed   = 0.20,    # m/s while searching for the line
    seek_timeout = 15.0,    # s — give up if line not found within this time
)

# ── Line-end detection — runs inside line-following state ─────────────────────
# Uses a debounce counter to avoid false triggers from sensor noise.
LINE_END = SimpleNamespace(
    lost_cnt     = 2,    # lineValidCnt below this counts as "no line" for one sample
    lost_confirm = 5,    # consecutive 10 ms samples needed to confirm the line has ended
)

# ── Catch-ball sequence — runs after the final crossing ───────────────────────
CATCH_BALL = SimpleNamespace(
    turn_right_deg = 90.0,   # ° — turn right to face the ball
    fwd1_dist_m    = 0.15,   # m — drive forward after turning
    fwd2_dist_m    = 0.03,   # m — drive forward after servos go down
    back_dist_m    = 0.10,   # m — reverse after raising servos
    drive_speed    = 0.20,   # m/s
    servo1_up      = -475,   # servo 1 PWM: raised
    servo1_down    =  480,   # servo 1 PWM: lowered
    servo2_up      =  475,   # servo 2 PWM: raised
    servo2_down    = -480,   # servo 2 PWM: lowered
    servo_move_s   =  1.0,   # s — wait after each servo command for motion to complete
)


################################################################
# MISSION SEQUENCES
#
# These lists define what happens during each blocking phase.
# Each entry is ("action_name", {params}).
# run_sequence() below executes them in order.
#
# Supported actions:
#   "drive"     — drive straight: dist (m), speed (m/s)
#   "turn"      — rotate in place: deg (°), dir ("left"/"right"), rate (rad/s, optional)
#   "arc"       — drive the roundabout arc (uses ROUNDABOUT_ARC settings)
#   "seek_line" — drive forward until line found: speed (m/s), timeout (s)
#   "servo"     — move servo: num (1 or 2, default 1), pos (PWM value),
#                 wait (s — sleep for servo to reach position), hold (s, optional — extra hold)
################################################################

# Steps executed when the line physically ends → roundabout phase
ROUNDABOUT_SEQUENCE = [
    ("drive",     {"dist":  ROUNDABOUT_ENTRY.drive_dist_m,  "speed":   ROUNDABOUT_ENTRY.drive_speed}),
    ("turn",      {"deg":   ROUNDABOUT_ENTRY.turn_deg,       "dir":     ROUNDABOUT_ENTRY.turn_dir}),
    ("arc",       {}),
    ("turn",      {"deg":   ROUNDABOUT_EXIT.turn_deg,        "dir":     ROUNDABOUT_EXIT.turn_dir}),
    ("seek_line", {"speed": ROUNDABOUT_EXIT.seek_speed,      "timeout": ROUNDABOUT_EXIT.seek_timeout}),
]

# Steps executed after the final crossing → catch-ball phase
CATCH_BALL_SEQUENCE = [
    ("turn",  {"deg":  CATCH_BALL.turn_right_deg, "dir":   "right"}),
    ("drive", {"dist": CATCH_BALL.fwd1_dist_m,    "speed": CATCH_BALL.drive_speed}),
    ("servo", {"num": 1, "pos": CATCH_BALL.servo1_down, "wait": CATCH_BALL.servo_move_s}),
    ("servo", {"num": 2, "pos": CATCH_BALL.servo2_down, "wait": CATCH_BALL.servo_move_s}),
    ("drive", {"dist": CATCH_BALL.fwd2_dist_m,    "speed": CATCH_BALL.drive_speed}),
    ("servo", {"num": 1, "pos": CATCH_BALL.servo1_up,   "wait": CATCH_BALL.servo_move_s}),
    ("servo", {"num": 2, "pos": CATCH_BALL.servo2_up,   "wait": CATCH_BALL.servo_move_s}),
    ("drive", {"dist": -CATCH_BALL.back_dist_m,   "speed": CATCH_BALL.drive_speed}),
]


################################################################
# PRIMITIVE DRIVE HELPERS
# Low-level building blocks used by run_sequence() and the mission.
################################################################

def driveTurn(deg, direction, turn_rate=0.5):
    """Rotate in place by deg degrees.
    direction: 'left' (positive turn rate) or 'right' (negative).
    Waits until odometry heading change (pose.tripBh) reaches the target."""
    pose.tripBreset()
    signed_rate = turn_rate if direction == "left" else -turn_rate
    target_rad  = np.radians(deg)
    state = 0
    if not service.is_quiet():
        print(f"% driveTurn: {deg:.0f}° {direction} at {turn_rate:.2f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", f"rc 0.0 {signed_rate:.2f}")
            state = 1
        elif state == 1:
            if abs(pose.tripBh) >= target_rad:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                break
        t.sleep(0.05)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveTurn: turned {np.degrees(pose.tripBh):.1f}° in {pose.tripBtimePassed():.2f} s")


def driveTurnForward(deg, direction, linvel, turn_rate=0.5, stop_after=True):
    """Move forward at linvel m/s while turning deg degrees.
    Combined rc command — smoother than stopping before turning.
    stop_after=False: skip stop command so caller can hand off to line controller
    while robot is still moving."""
    pose.tripBreset()
    signed_rate = turn_rate if direction == "left" else -turn_rate
    target_rad  = np.radians(deg)
    if not service.is_quiet():
        print(f"% driveTurnForward: {deg:.0f}° {direction}, v={linvel:.2f} m/s, w={turn_rate:.2f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc {linvel:.2f} {signed_rate:.2f}")
    while not service.stop:
        if abs(pose.tripBh) >= target_rad:
            break
        t.sleep(0.02)
    if stop_after:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        while not service.stop and (abs(pose.velocity()) > 0.001 or abs(pose.turnrate()) > 0.001):
            t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveTurnForward: turned {np.degrees(pose.tripBh):.1f}°")


def driveDistance(dist, speed=0.2):
    """Drive straight for dist metres at speed m/s.
    Negative dist drives backward. Waits until odometry distance reaches the target."""
    actual_speed = speed if dist >= 0 else -speed
    target_dist  = abs(dist)
    pose.tripBreset()
    state = 0
    if not service.is_quiet():
        print(f"% driveDistance: {dist:.2f} m at {actual_speed:.2f} m/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", f"rc {actual_speed:.2f} 0.0")
            state = 1
        elif state == 1:
            if pose.tripB >= target_dist or pose.tripBtimePassed() > 20:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                break
        t.sleep(0.05)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveDistance: drove {pose.tripB:.3f} m in {pose.tripBtimePassed():.2f} s")


def driveRoundabout():
    """Drive a circular arc using ROUNDABOUT_ARC settings.
    Turn rate is derived from diameter and speed: w = speed / (diameter / 2).
    Progress is tracked by odometry heading change (pose.tripBh)."""
    radius_m   = ROUNDABOUT_ARC.diameter_m / 2.0
    sign       = 1.0 if ROUNDABOUT_ARC.direction == "left" else -1.0
    turn_rate  = sign * ROUNDABOUT_ARC.speed / radius_m
    target_rad = np.radians(ROUNDABOUT_ARC.total_degrees)
    pose.tripBreset()
    if not service.is_quiet():
        print(f"% driveRoundabout: ⌀{ROUNDABOUT_ARC.diameter_m:.2f} m, "
              f"v={ROUNDABOUT_ARC.speed:.2f} m/s, w={turn_rate:.3f} rad/s, "
              f"target={ROUNDABOUT_ARC.total_degrees:.0f}°")
    service.send("robobot/cmd/T0", "leds 16 0 0 30")  # blue: arc running
    service.send("robobot/cmd/ti", f"rc {ROUNDABOUT_ARC.speed:.3f} {turn_rate:.3f}")
    last_print = t.time()
    while not service.stop:
        if abs(pose.tripBh) >= target_rad:
            break
        if not service.is_quiet() and t.time() - last_print >= 0.5:
            print(f"# arc: {np.degrees(abs(pose.tripBh)):.1f}° / {ROUNDABOUT_ARC.total_degrees:.0f}°")
            last_print = t.time()
        t.sleep(0.02)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not service.stop and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"% driveRoundabout: done ({np.degrees(abs(pose.tripBh)):.1f}° in {pose.tripBtimePassed():.1f} s)")


def seekLine(speed, timeout_s):
    """Drive forward at speed until the line sensor is confident (lineValidCnt > 4) or timeout."""
    pose.tripBreset()
    service.send("robobot/cmd/ti", f"rc {speed:.2f} 0.0")
    while not service.stop:
        if edge.lineValidCnt > 4:
            break
        if pose.tripBtimePassed() > timeout_s:
            break
        t.sleep(0.01)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not service.stop and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    if not service.is_quiet():
        result = "found" if edge.lineValidCnt > 4 else "timeout"
        print(f"% seekLine: {result} after {pose.tripBtimePassed():.1f} s")


def run_sequence(steps):
    """Execute a list of (action, params) steps in order.
    Stops early if service.stop is set (emergency stop).

    Supported actions and their params:
      "drive"     — dist (m), speed (m/s)
      "turn"      — deg (°), dir ("left"/"right"), rate (rad/s, optional)
      "arc"       — no params; uses ROUNDABOUT_ARC settings
      "seek_line" — speed (m/s), timeout (s)
      "servo"     — pos (PWM), hold (s, optional)
    """
    for action, params in steps:
        if service.stop:
            break
        if action == "drive":
            driveDistance(params["dist"], params.get("speed", 0.2))
        elif action == "turn":
            driveTurn(params["deg"], params["dir"], params.get("rate", 0.5))
        elif action == "arc":
            driveRoundabout()
        elif action == "seek_line":
            seekLine(params["speed"], params["timeout"])
        elif action == "servo":
            num = params.get("num", 1)
            service.send("robobot/cmd/T0", f"servo {num} {params['pos']} 100")
            if "wait" in params:   # wait for servo to reach position
                t.sleep(params["wait"])
            if "hold" in params:   # optional extra hold at position
                t.sleep(params["hold"])
        else:
            if not service.is_quiet():
                print(f"% run_sequence: unknown action '{action}' — skipped")


################################################################
# MAIN MISSION
################################################################

def driveMission():
    """Full mission state machine.

    States:
      0  — wait for IR start gate (or --now flag), then drive forward
      1  — drive until line found, then start PD controller
      2  — wait for full stop, then exit
      10 — line following + crossing reactions + line-end detection
      11 — roundabout sequence (blocking), then resume line following
      20 — end sequence (blocking), then wait for stop
      99 — mission complete (exits loop)
    """
    state = 0
    pose.tripBreset()
    dist_to_line = 0.0

    # ── Crossing debounce state ───────────────────────────────────────────────
    crossing_count        = 0       # how many crossings have been fully cleared (0-based)
    was_at_crossing       = False   # True while robot is (or was) on a crossing
    last_time_at_crossing = None    # timestamp when robot was last detected at a crossing

    # ── Per-crossing flags ────────────────────────────────────────────────────
    first_cross_done  = False   # True once crossing-1 turn has been executed
    last_hard_turn_time = None  # timestamp of last hard 90° turn (for cooldown)

    # ── Roundabout state ──────────────────────────────────────────────────────
    roundabout_done     = False   # True after the roundabout sequence has run (prevents re-entry)
    line_end_lost_count = 0       # consecutive samples below LINE_END.lost_cnt

    # ── Line-loss recovery (active only after roundabout) ────────────────────
    lost_line_since = None

    if not service.is_quiet():
        print("% driveMission: starting")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green: running

    while not service.stop:

        # ── State 0: wait for IR gate, then start moving ──────────────────────
        if state == 0:
            # --now flag skips the IR check and starts immediately
            if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
                service.send("robobot/cmd/T0", "servo 1 -475 100")   # raise — match CATCH_BALL.servo1_up
                service.send("robobot/cmd/T0", "servo 2  475 100")   # raise — match CATCH_BALL.servo2_up
                service.send("robobot/cmd/ti", "rc 0.2 0.0")         # drive forward
                service.send("robobot/cmd/T0/", "lognow 3")          # start Teensy log
                state = 1

        # ── State 1: drive until line is found ───────────────────────────────
        elif state == 1:
            # Safety: abort if line never appears
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
            # Line found: activate PD controller and start following
            if edge.lineValidCnt > 4:
                edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")
                dist_to_line = pose.tripB
                pose.tripBreset()
                if not service.is_quiet():
                    print(f"% line found after {dist_to_line:.2f} m → state 10")
                state = 10

        # ── State 2: wait for full stop, then exit ────────────────────────────
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99

        # ── State 10: line following + crossing reactions + line-end detection ─
        elif state == 10:
            edge.mission_crossing_count = crossing_count  # shown in sedge status line

            # ── Crossing detection (leave-delay debounce) ─────────────────────
            # The crossing is counted only after the robot has been *clear* of it
            # for CROSSINGS.leave_delay_s seconds — prevents double-counting jitter.
            at_crossing = edge.crossingLineCnt >= CROSSINGS.sensor_threshold
            now = datetime.now()
            if at_crossing:
                last_time_at_crossing = now
                was_at_crossing = True
            elif was_at_crossing and last_time_at_crossing is not None:
                clear_s = (now - last_time_at_crossing).total_seconds()
                if clear_s >= CROSSINGS.leave_delay_s:
                    crossing_count += 1
                    was_at_crossing = False
                    last_time_at_crossing = None
                    print(f"% crossing {crossing_count}")

            # ── Crossing reactions (only while physically on a crossing) ───────
            if at_crossing:
                # crossing_number is 1-based; crossing_count increments on *leaving*,
                # so while we're still at crossing N, crossing_count = N-1.
                crossing_number = crossing_count + 1

                if crossing_number == 1 and not first_cross_done:
                    # ── Crossing 1: turn left while moving forward, then resume ─
                    if not service.is_quiet():
                        print(f"% crossing 1 → turn {CROSSING1.turn_dir} {CROSSING1.turn_deg:.0f}°"
                              f" (fwd={CROSSING1.forward_speed:.2f} m/s)")
                    edge.lineControl(0)
                    if CROSSING1.forward_speed > 0:
                        driveTurnForward(CROSSING1.turn_deg, CROSSING1.turn_dir,
                                         CROSSING1.forward_speed, CROSSING1.turn_rate,
                                         stop_after=False)
                    else:
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        t.sleep(0.05)
                        driveTurn(CROSSING1.turn_deg, CROSSING1.turn_dir, CROSSING1.turn_rate)
                    first_cross_done = True
                    edge.lineControl(velocity=LINE.approach_speed, refPosition=0, params="slow")

                elif crossing_count == CROSSINGS.stop_at - 1:
                    # ── Final crossing: stop and run end sequence ─────────────
                    print(f"% crossing {crossing_number}")
                    if not service.is_quiet():
                        print(f"% crossing {crossing_number} → end sequence")
                    edge.lineControl(0)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 20

                elif crossing_count >= CROSSINGS.go_straight_until:
                    # ── Hard 90° turn (crossing 4 and above) ─────────────────
                    # Direction is auto-detected from which end of the sensor array is lit.
                    # A cooldown prevents the same crossing from triggering a double-turn.
                    cooldown_ok = (
                        last_hard_turn_time is None or
                        (now - last_hard_turn_time).total_seconds() >= CROSSINGS.hard_turn_cooldown_s
                    )
                    if cooldown_ok:
                        hard_left  = (edge.leftmostAboveIndex == 0 and
                                      edge.rightmostAboveIndex is not None and
                                      edge.rightmostAboveIndex <= 4)
                        hard_right = (edge.rightmostAboveIndex == 7 and
                                      edge.leftmostAboveIndex is not None and
                                      edge.leftmostAboveIndex >= 3)
                        if hard_left or hard_right:
                            direction = "left" if hard_left else "right"
                            if not service.is_quiet():
                                print(f"% crossing {crossing_number} → hard turn {direction} 90°")
                            edge.lineControl(0)
                            service.send("robobot/cmd/ti", "rc 0.0 0.0")
                            t.sleep(0.05)
                            driveTurn(90.0, direction)
                            last_hard_turn_time = datetime.now()
                            edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")
                # Crossings 2 and 3 (crossing_count 1, 2 < go_straight_until=3): go straight

            # ── Line-end detection (only before roundabout has run) ───────────
            # Counts consecutive "no line" samples; confirms line end on 5 in a row.
            if not roundabout_done and state == 10:
                if edge.lineValidCnt < LINE_END.lost_cnt:
                    line_end_lost_count += 1
                else:
                    line_end_lost_count = 0
                if line_end_lost_count >= LINE_END.lost_confirm:
                    if not service.is_quiet():
                        print("% line ended → starting roundabout sequence")
                    edge.lineControl(0)
                    state = 11

            # ── Line-loss recovery (only after roundabout has run) ────────────
            # If the line disappears unexpectedly on the return leg, stop safely.
            elif roundabout_done and state == 10:
                if edge.lineValidCnt >= 4:
                    lost_line_since = None
                elif edge.lineValidCnt < 2:
                    if lost_line_since is None:
                        lost_line_since = datetime.now()
                    elapsed = (datetime.now() - lost_line_since).total_seconds()
                    if elapsed >= LINE.lost_timeout:
                        edge.lineControl(0)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        if not service.is_quiet():
                            print("% line lost too long → recovery stop")
                        state = 2

        # ── State 11: roundabout sequence ────────────────────────────────────
        # Runs ROUNDABOUT_SEQUENCE step-by-step, then resumes line following
        # with crossing_count = 1 so the remaining crossings (2-5) map correctly.
        elif state == 11:
            run_sequence(ROUNDABOUT_SEQUENCE)
            crossing_count  = 1      # crossing 1 was already handled before the roundabout
            roundabout_done = True   # prevent line-end detection from triggering again
            lost_line_since = None
            edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")
            if not service.is_quiet():
                print("% roundabout done → resuming line follow (crossing_count=1)")
            state = 10

        # ── State 20: catch-ball sequence ─────────────────────────────────────
        # Runs CATCH_BALL_SEQUENCE step-by-step, then waits for full stop.
        elif state == 20:
            run_sequence(CATCH_BALL_SEQUENCE)
            state = 2

        else:
            # state 99 (or unexpected): mission complete
            if not service.is_quiet():
                print(f"% driveMission: done — dist to line {dist_to_line:.2f} m, "
                      f"along line {pose.tripB:.2f} m in {pose.tripBtimePassed():.1f} s")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break

        t.sleep(0.01)   # 100 Hz control loop

    service.send("robobot/cmd/T0", "leds 16 0 0 0")  # LEDs off
    if not service.is_quiet():
        print("% driveMission: end")


################################################################
# TOP-LEVEL LOOP
################################################################

def loop():
    from ulog import flog
    oldstate   = -1
    state_time = datetime.now()

    service.send("robobot/cmd/T0", "leds 16 30 30 0")  # yellow: ready
    edge.lineControl(0)   # ensure motors stopped before mission starts

    state = 103
    while not service.stop:
        if state == 103:
            driveMission()
            state = 99
        else:
            if not service.is_quiet():
                print(f"% loop: finished (state={state})")
            break

        if state != oldstate:
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            if not service.is_quiet():
                print(f"% State change from {oldstate} to {state}")
            oldstate   = state
            state_time = datetime.now()

        t.sleep(0.1)

    # Cleanup: stop robot and turn off LEDs
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    gpio.set_value(20, 0)
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)


################################################################
# ENTRY POINT
################################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running - terminating")
        print("%   to kill a stuck instance: pkill mqtt-client   (or pkill -9 mqtt-client)")
    else:
        setproctitle("mqtt-client")
        if not service.is_quiet():
            print("% Starting full mission")
        start_teensy_interface()
        service.setup('localhost')
        if service.connected:
            loop()
        service.terminate()
        stop_teensy_interface()
    if not service.is_quiet():
        print("% Main Terminated")
