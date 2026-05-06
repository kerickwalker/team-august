#!/usr/bin/env python3

# Half-mission script for Robobot (DTU).
#
# Run with:   python3 mqtt-full-mission-simple-half.py -e        (normal)
#             python3 mqtt-full-mission-simple-half.py -e -s      (silent, no debug prints)
#             python3 mqtt-full-mission-simple-half.py -e --now   (skip IR start-gate, go immediately)
#
# ── Mission overview ─────────────────────────────────────────────────────────
#
#  [INIT]       Drive forward until IR gate clears and line sensor finds the line.
#
#  [FOLLOW]     Follow the line. React at each crossing:
#                 Crossing 1 → turn LEFT (while moving forward), resume following
#               When line physically ends → [ROUNDABOUT]
#
#  [ROUNDABOUT] drive forward → turn right 90° → arc → seek line → resume follow
#
#  [TIMED 10s]  Follow line at normal speed for 10 seconds.
#
#  [TURN 180°]  Rotate 180° in place.
#
#  [TIMED 6s]   Follow line at normal speed for 6 seconds.
#
#  [SLOW FOLLOW] Switch to slow speed; follow until line is lost.
#
#  [ROUNDABOUT2] New tunable roundabout entry:
#                 drive forward → turn left 30° → drive straight until line seen
#                 → stop → turn left 30° → follow line slowly (continuous).
#
# ── How to tune ──────────────────────────────────────────────────────────────
#
#  All tunable values live in the CONFIGURATION section below.
#  HALF_MISSION and ROUNDABOUT2_ENTRY are new — tune these for the return leg.
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
    approach_speed  = 0.15,  # m/s — reduced/slow speed
    lost_timeout    = 5.0,   # s   — stop if line lost for longer than this (recovery)
)

# ── Crossing detection and counting ──────────────────────────────────────────
CROSSINGS = SimpleNamespace(
    sensor_threshold     = 2,     # crossingLineCnt must reach this to register a crossing
    leave_delay_s        = 0.5,   # s — robot must be clear of crossing this long before count increments
    go_straight_until    = 3,     # crossings 1..N go straight; above this = hard turn
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
    drive_dist_m = 0.30,    # m — drive forward after line ends before turning
    drive_speed  = 0.25,    # m/s
    turn_deg     = 90.0,    # degrees to turn to face the roundabout circle
    turn_dir     = "right",  # "left" or "right"
)

# ── Roundabout arc — geometry and speed ───────────────────────────────────────
ROUNDABOUT_ARC = SimpleNamespace(
    diameter_m    = 0.73,    # m — physical diameter of the circle
    total_degrees = 435.0,  # ° — heading change target (360 = one full lap)
    speed         = 0.20,   # m/s — forward speed on the arc
    direction     = "left", # "left" (CCW) or "right" (CW)
)

# ── Roundabout exit — find the return line after the arc ─────────────────────
ROUNDABOUT_EXIT = SimpleNamespace(
    turn_deg     = 0.0,     # degrees to turn after the arc
    turn_dir     = "right", # "left" or "right"
    seek_speed   = 0.20,    # m/s while searching for the line
    seek_timeout = 15.0,    # s — give up if line not found within this time
)

# ── Line-end detection — runs inside line-following states ────────────────────
LINE_END = SimpleNamespace(
    lost_cnt     = 2,    # lineValidCnt below this counts as "no line" for one sample
    lost_confirm = 5,    # consecutive 10 ms samples needed to confirm the line has ended
)

# ── Half-mission timed phases ─────────────────────────────────────────────────
HALF_MISSION = SimpleNamespace(
    follow1_s    = 15.0,   # s — follow line at normal speed after roundabout
    turn180_deg  = 180.0,  # ° — turn in place after first timed follow
    follow2_s    = 12.0,    # s — follow line at normal speed after 180° turn
)

# ── Second roundabout entry — new tunable block ───────────────────────────────
ROUNDABOUT2_ENTRY = SimpleNamespace(
    drive_dist_m = 0.30,   # m — drive forward before turning (same as ROUNDABOUT_ENTRY)
    drive_speed  = 0.25,   # m/s
    turn_deg     = 30.0,   # ° — left turn to face the roundabout circle
    align_deg    = 60.0,   # ° — left turn after line is found to align with it
    seek_speed   = 0.20,   # m/s while seeking the line after turning
    seek_timeout = 15.0,   # s — give up if line not found within this time
)


################################################################
# MISSION SEQUENCES
################################################################

# Steps executed when the line physically ends → first roundabout phase
ROUNDABOUT_SEQUENCE = [
    ("drive",     {"dist":  ROUNDABOUT_ENTRY.drive_dist_m,  "speed":   ROUNDABOUT_ENTRY.drive_speed}),
    ("turn",      {"deg":   ROUNDABOUT_ENTRY.turn_deg,       "dir":     ROUNDABOUT_ENTRY.turn_dir}),
    ("arc",       {}),
    ("turn",      {"deg":   ROUNDABOUT_EXIT.turn_deg,        "dir":     ROUNDABOUT_EXIT.turn_dir}),
    ("seek_line", {"speed": ROUNDABOUT_EXIT.seek_speed,      "timeout": ROUNDABOUT_EXIT.seek_timeout}),
]


################################################################
# PRIMITIVE DRIVE HELPERS
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
    """Move forward at linvel m/s while turning deg degrees."""
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
    """Drive straight for dist metres at speed m/s."""
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
    """Drive a circular arc using ROUNDABOUT_ARC settings."""
    radius_m   = ROUNDABOUT_ARC.diameter_m / 2.0
    sign       = 1.0 if ROUNDABOUT_ARC.direction == "left" else -1.0
    turn_rate  = sign * ROUNDABOUT_ARC.speed / radius_m
    target_rad = np.radians(ROUNDABOUT_ARC.total_degrees)
    pose.tripBreset()
    if not service.is_quiet():
        print(f"% driveRoundabout: ⌀{ROUNDABOUT_ARC.diameter_m:.2f} m, "
              f"v={ROUNDABOUT_ARC.speed:.2f} m/s, w={turn_rate:.3f} rad/s, "
              f"target={ROUNDABOUT_ARC.total_degrees:.0f}°")
    service.send("robobot/cmd/T0", "leds 16 0 0 30")
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


def driveLineFollow(dist, speed):
    """Follow the line for dist metres using the PD controller, then stop."""
    pose.tripBreset()
    edge.lineControl(velocity=speed, refPosition=0)
    if not service.is_quiet():
        print(f"% driveLineFollow: {dist:.2f} m at {speed:.2f} m/s")
    while not service.stop:
        if pose.tripB >= dist:
            break
        t.sleep(0.01)
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not service.stop and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    if not service.is_quiet():
        print(f"# driveLineFollow: drove {pose.tripB:.3f} m")


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
    """Execute a list of (action, params) steps in order."""
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
        elif action == "follow_line":
            driveLineFollow(params["dist"], params.get("speed", LINE.speed))
        elif action == "servo":
            num = params.get("num", 1)
            service.send("robobot/cmd/T0", f"servo {num} {params['pos']} 100")
            if "wait" in params:
                t.sleep(params["wait"])
            if "hold" in params:
                t.sleep(params["hold"])
        else:
            if not service.is_quiet():
                print(f"% run_sequence: unknown action '{action}' — skipped")


################################################################
# MAIN MISSION
################################################################

def driveMission():
    """Half-mission state machine.

    States:
      0  — wait for IR start gate (or --now flag), then drive forward
      1  — drive until line found, then start PD controller
      2  — wait for full stop, then exit
      10 — line following + crossing reactions + line-end detection
      11 — first roundabout sequence (blocking), then go to state 12
      12 — follow line at normal speed for HALF_MISSION.follow1_s seconds
      13 — turn 180° in place
      14 — follow line at normal speed for HALF_MISSION.follow2_s seconds
      15 — follow line at slow speed until line is lost
      16 — new roundabout entry: drive fwd → turn left → seek line → align → state 17
      17 — follow line continuously at slow speed (until stop button)
      99 — mission complete (exits loop)
    """
    state = 0
    pose.tripBreset()
    dist_to_line = 0.0

    # ── Crossing debounce state ───────────────────────────────────────────────
    crossing_count        = 0
    was_at_crossing       = False
    last_time_at_crossing = None

    # ── Per-crossing flags ────────────────────────────────────────────────────
    first_cross_done    = False
    last_hard_turn_time = None

    # ── Roundabout state ──────────────────────────────────────────────────────
    roundabout_done     = False
    line_end_lost_count = 0

    # ── Timed-follow state (states 12, 14) ───────────────────────────────────
    state12_start = None
    state14_start = None

    # ── Slow-follow line-loss detection (state 15) ────────────────────────────
    line_end_lost_count_15 = 0

    # ── State 17 init guard ───────────────────────────────────────────────────
    state17_started = False

    if not service.is_quiet():
        print("% driveMission: starting")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")

    while not service.stop:

        # ── State 0: wait for IR gate, then start moving ──────────────────────
        if state == 0:
            if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
                service.send("robobot/cmd/T0", "servo 1 -475 100")
                service.send("robobot/cmd/T0", "servo 2  475 100")
                service.send("robobot/cmd/ti", "rc 0.2 0.0")
                service.send("robobot/cmd/T0/", "lognow 3")
                state = 1

        # ── State 1: drive until line is found ───────────────────────────────
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
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
            edge.mission_crossing_count = crossing_count

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
                    if not service.is_quiet():
                        print(f"% crossing {crossing_count}")

            if at_crossing:
                crossing_number = crossing_count + 1

                if crossing_number == 1 and not first_cross_done:
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

                elif crossing_count >= CROSSINGS.go_straight_until:
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

            # ── Line-end detection (only before roundabout has run) ───────────
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

        # ── State 11: first roundabout sequence ───────────────────────────────
        elif state == 11:
            run_sequence(ROUNDABOUT_SEQUENCE)   # arc → seek line (drives until line found)
            roundabout_done = True
            edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")  # start following as soon as line is found
            state12_start = datetime.now()      # timer begins the moment we start following
            if not service.is_quiet():
                print(f"% roundabout done, line found → timed follow {HALF_MISSION.follow1_s:.0f}s (state 12)")
            state = 12

        # ── State 12: follow line at normal speed for follow1_s seconds ───────
        elif state == 12:
            elapsed = (datetime.now() - state12_start).total_seconds()
            if elapsed >= HALF_MISSION.follow1_s:
                edge.lineControl(0)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                if not service.is_quiet():
                    print(f"% state 12: done ({elapsed:.1f}s) → turning 180°")
                state = 13

        # ── State 13: turn 180° in place ──────────────────────────────────────
        elif state == 13:
            driveTurn(HALF_MISSION.turn180_deg, "left")
            state14_start = None
            if not service.is_quiet():
                print("% state 13: 180° done → timed fast follow (state 14)")
            state = 14

        # ── State 14: follow line at normal speed for follow2_s seconds ───────
        elif state == 14:
            if state14_start is None:
                state14_start = datetime.now()
                edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")
                if not service.is_quiet():
                    print(f"% state 14: follow line for {HALF_MISSION.follow2_s:.0f}s (normal speed)")
            elapsed = (datetime.now() - state14_start).total_seconds()
            if elapsed >= HALF_MISSION.follow2_s:
                edge.lineControl(velocity=LINE.approach_speed, refPosition=0, params="slow")
                line_end_lost_count_15 = 0
                if not service.is_quiet():
                    print(f"% state 14: done ({elapsed:.1f}s) → slow follow until line lost (state 15)")
                state = 15

        # ── State 15: slow follow until line is lost ──────────────────────────
        elif state == 15:
            if edge.lineValidCnt < LINE_END.lost_cnt:
                line_end_lost_count_15 += 1
            else:
                line_end_lost_count_15 = 0
            if line_end_lost_count_15 >= LINE_END.lost_confirm:
                edge.lineControl(0)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                if not service.is_quiet():
                    print("% state 15: line ended → new roundabout entry (state 16)")
                state = 16

        # ── State 16: new roundabout entry ────────────────────────────────────
        elif state == 16:
            if not service.is_quiet():
                print("% state 16: new roundabout entry")
            driveDistance(ROUNDABOUT2_ENTRY.drive_dist_m, ROUNDABOUT2_ENTRY.drive_speed)
            if not service.stop:
                driveTurn(ROUNDABOUT2_ENTRY.turn_deg, "left")
            if not service.stop:
                seekLine(ROUNDABOUT2_ENTRY.seek_speed, ROUNDABOUT2_ENTRY.seek_timeout)
            if not service.stop:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                while not service.stop and abs(pose.velocity()) > 0.001:
                    t.sleep(0.02)
                driveTurn(ROUNDABOUT2_ENTRY.align_deg, "left")
            state17_started = False
            if not service.is_quiet():
                print("% state 16: done → continuous slow follow (state 17)")
            state = 17

        # ── State 17: continuous slow line follow (runs until stop button) ────
        elif state == 17:
            if not state17_started:
                state17_started = True
                edge.lineControl(velocity=LINE.approach_speed, refPosition=0, params="slow")
                if not service.is_quiet():
                    print("% state 17: continuous slow line follow — press red button to stop")
            # no exit condition

        else:
            # state 99 (or unexpected): mission complete
            if not service.is_quiet():
                print(f"% driveMission: done — dist to line {dist_to_line:.2f} m, "
                      f"along line {pose.tripB:.2f} m in {pose.tripBtimePassed():.1f} s")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break

        t.sleep(0.01)

    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print("% driveMission: end")


################################################################
# TOP-LEVEL LOOP
################################################################

def loop():
    oldstate   = -1
    state_time = datetime.now()

    edge.print_follow_line_block = False  # suppress sedge per-sensor prints
    service.send("robobot/cmd/T0", "leds 16 30 30 0")  # yellow: ready
    edge.lineControl(0)

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
            if not service.is_quiet():
                print(f"% State change from {oldstate} to {state}")
            oldstate   = state
            state_time = datetime.now()

        t.sleep(0.1)

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
            print("% Starting half mission")
        start_teensy_interface()
        service.setup('localhost')
        if service.connected:
            loop()
        service.terminate()
        stop_teensy_interface()
    if not service.is_quiet():
        print("% Main Terminated")
