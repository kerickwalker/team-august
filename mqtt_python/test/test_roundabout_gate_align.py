#!/usr/bin/env python3

# Roundabout gate-alignment test for Robobot.
#
# Phase 1: line-follow with full crossing detection (mirrors mqtt-full-mission-simple.py):
#            crossing 1 → turn LEFT 35°, slow to approach_speed with "slow" PID params
#            line physically ends → stop, enter phase 2
# Phase 2: rotate in place using gate1 vision to reach a target pose relative
#          to the orange gate post.  When the gate is centred AND at the correct
#          apparent distance for ALIGN_CONFIRM consecutive frames, motors stop
#          and the script waits for Enter before releasing.
#
# Usage:
#   python3 test/test_roundabout_gate_align.py <robot-ip>
#   python3 test/test_roundabout_gate_align.py <robot-ip> --now
#   python3 test/test_roundabout_gate_align.py <robot-ip> --now -s
#
# Live camera stream (SSH-friendly — no display required):
#   Open http://<robot-ip>:5000/ in any browser on the same network.
#
# Env var overrides (all optional):
#   GATE1_TARGET_X      target bar x normalised [-1..1]   (default 0.302)
#   GATE1_TARGET_W      target bar width px                (default 128.0)
#   GATE1_STOP_W        stop_width_px gate1_ctrl param     (default 80.0)
#   GATE_SEARCH_W       rotation rate when gate not seen   (default -0.3 rad/s)
#   GATE_ALIGN_TOL_X    lateral error tolerance            (default 0.08)
#   GATE_ALIGN_TOL_W    depth error tolerance              (default 0.15)
#   GATE_ALIGN_CONFIRM  consecutive frames to confirm done (default 10)
#   GATE_ALIGN_MAX_W    max turn rate during alignment     (default 0.5 rad/s)

import os
import sys
import time as t
import threading
import numpy as np
import cv2 as cv
from datetime import datetime
from types import SimpleNamespace
from setproctitle import setproctitle
from flask import Flask, Response

# Robot IP is extracted here (before uservice touches sys.argv).
# Any non-flag positional argument is taken as the robot IP and removed so
# uservice argparse does not see an unexpected positional argument.
_robot_ip = 'http://10.197.219.117'

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from spose import pose
from sir import ir
from sedge import edge
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface
from sgate_1 import gate1, gate1_ctrl

############################################################
# CONFIGURATION — mirrors mqtt-full-mission-simple.py
############################################################

# ── General line following ────────────────────────────────
LINE = SimpleNamespace(
    speed          = 0.25,  # m/s — normal line-following speed
    approach_speed = 0.15,  # m/s — reduced speed after crossing 1 until line ends
    lost_timeout   = 20.0,  # s   — abort if line never ends
)

# ── Crossing detection and counting ──────────────────────
CROSSINGS = SimpleNamespace(
    sensor_threshold     = 2,    # crossingLineCnt threshold to register a crossing
    leave_delay_s        = 0.5,  # s — robot must be clear before count increments
    go_straight_until    = 3,    # crossings above this would be hard turns (unused here)
    hard_turn_cooldown_s = 2.0,  # s — minimum time between hard turns
)

# ── Crossing 1 — turn left while moving forward ──────────
CROSSING1 = SimpleNamespace(
    turn_deg      = 35.0,   # degrees to turn
    turn_dir      = "left", # "left" or "right"
    turn_rate     = 0.5,    # rad/s
    forward_speed = 0.15,   # m/s forward while turning (0 = stop before turning)
)

# ── Line-end detection ────────────────────────────────────
LINE_END = SimpleNamespace(
    lost_cnt     = 2,  # lineValidCnt below this counts as "no line"
    lost_confirm = 5,  # consecutive 10 ms samples to confirm line ended
)

# ── Gate alignment ────────────────────────────────────────
SEARCH_W      = -0.3   # rad/s  negative = rotate right (toward roundabout)
ALIGN_TOL_X   = 0.08   # normalised lateral error tolerance  (±1 scale)
ALIGN_TOL_W   = 0.15   # normalised depth error tolerance    (±1 scale)
ALIGN_CONFIRM = 10     # frames both errors must stay in tolerance
ALIGN_MAX_W   = 0.5    # rad/s  max turn rate during alignment

############################################################
# MJPEG browser stream (replaces cv.imshow — works over SSH)
############################################################

_stream_frame = None
_stream_lock  = threading.Lock()
_flask_app    = Flask(__name__)


def _push_frame(frame):
    global _stream_frame
    with _stream_lock:
        _stream_frame = frame.copy()


def _mjpeg_generate():
    while True:
        with _stream_lock:
            f = _stream_frame
        if f is not None:
            ok, buf = cv.imencode('.jpg', f, [cv.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + buf.tobytes() + b'\r\n')
        t.sleep(0.033)  # ~30 fps cap


@_flask_app.route('/')
def _index():
    return ('<html><body style="background:#000;margin:0">'
            '<img src="/stream" style="max-width:100%"></body></html>')


@_flask_app.route('/stream')
def _stream_view():
    return Response(_mjpeg_generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def start_stream_server(port=5000):
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    threading.Thread(
        target=lambda: _flask_app.run(host='0.0.0.0', port=port, threaded=True),
        daemon=True,
    ).start()
    print(f"% Stream server: open http://<robot-ip>:{port}/ in your browser")


############################################################
# Drive helpers (mirrors mqtt-full-mission-simple.py)
############################################################

def driveTurn(deg, direction, turn_rate=0.5):
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
        print(f"# driveTurn: turned {np.degrees(pose.tripBh):.1f}° "
              f"in {pose.tripBtimePassed():.2f} s")


def driveTurnForward(deg, direction, linvel, turn_rate=0.5, stop_after=True):
    pose.tripBreset()
    signed_rate = turn_rate if direction == "left" else -turn_rate
    target_rad  = np.radians(deg)
    if not service.is_quiet():
        print(f"% driveTurnForward: {deg:.0f}° {direction}, "
              f"v={linvel:.2f} m/s, w={turn_rate:.2f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc {linvel:.2f} {signed_rate:.2f}")
    while not service.stop:
        if abs(pose.tripBh) >= target_rad:
            break
        t.sleep(0.02)
    if stop_after:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        while not service.stop and (abs(pose.velocity()) > 0.001
                                    or abs(pose.turnrate()) > 0.001):
            t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveTurnForward: turned {np.degrees(pose.tripBh):.1f}°")


############################################################
# Phase 1: Follow line with full crossing detection
############################################################

def driveLineWithCrossings():
    """Follow the line until it physically ends.

    Mirrors mqtt-full-mission-simple.py states 0 → 1 → 10:
      - Wait for IR gate (or --now), drive forward to find the line.
      - Follow the line at LINE.speed with params="normal".
      - At crossing 1: turn LEFT, resume at LINE.approach_speed with params="slow".
      - When LINE_END.lost_confirm consecutive no-line samples are seen: stop and return.
    """
    if not service.is_quiet():
        print("% driveLineWithCrossings: waiting for start signal...")

    # ── Phase A: wait for IR gate / --now, then drive forward ────────────
    while not service.stop:
        if getattr(service.args, "now", False) or ir.ir[0] < 0.2:
            break
        t.sleep(0.05)

    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", "rc 0.2 0.0")
    pose.tripBreset()

    if not service.is_quiet():
        print("% driveLineWithCrossings: driving forward to find line...")

    # ── Phase B: find the line ────────────────────────────────────────────
    while not service.stop:
        if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            if not service.is_quiet():
                print("% driveLineWithCrossings: line not found — aborting")
            return
        if edge.lineValidCnt > 4:
            edge.lineControl(velocity=LINE.speed, refPosition=0, params="normal")
            pose.tripBreset()
            if not service.is_quiet():
                print(f"% driveLineWithCrossings: line found after {pose.tripB:.2f} m"
                      " → following")
            break
        t.sleep(0.01)

    # ── Phase C: line-following loop with crossing detection ─────────────
    crossing_count        = 0
    first_cross_done      = False
    was_at_crossing       = False
    last_time_at_crossing = None
    line_end_lost_count   = 0

    t0 = t.time()
    while not service.stop:
        if t.time() - t0 > LINE.lost_timeout:
            if not service.is_quiet():
                print("% driveLineWithCrossings: timeout — line never ended")
            break

        # Crossing detection (leave-delay debounce)
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

        # Crossing reactions
        if at_crossing:
            crossing_number = crossing_count + 1

            if crossing_number == 1 and not first_cross_done:
                if not service.is_quiet():
                    print(f"% crossing 1 → turn {CROSSING1.turn_dir} "
                          f"{CROSSING1.turn_deg:.0f}° "
                          f"(fwd={CROSSING1.forward_speed:.2f} m/s)")
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
                edge.lineControl(velocity=LINE.approach_speed,
                                 refPosition=0, params="slow")
            # Crossings 2, 3: go straight (no action needed)

        # Line-end detection
        if edge.lineValidCnt < LINE_END.lost_cnt:
            line_end_lost_count += 1
        else:
            line_end_lost_count = 0

        if line_end_lost_count >= LINE_END.lost_confirm:
            if not service.is_quiet():
                print(f"% driveLineWithCrossings: line ended after "
                      f"{pose.tripB:.2f} m in {pose.tripBtimePassed():.1f} s")
            break

        t.sleep(0.01)   # 100 Hz loop

    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")


############################################################
# Display helpers
############################################################

_STATE_COLOR = {
    "SEARCH": (0,  80, 255),    # red-orange
    "ALIGN":  (0, 220, 220),    # yellow
    "DONE":   (0, 255,  80),    # green
}


def _annotate(frame, state, pose_dict, v_cmd, w_cmd, confirm_count, confirm_max):
    """Draw debug overlay on frame in-place."""
    gate1.paint(frame)

    h, w = frame.shape[:2]
    tx_px = int((1.0 + gate1_ctrl.target_x_norm) * w / 2.0)
    cv.line(frame, (tx_px, 0), (tx_px, h), (0, 200, 0), 1)

    color = _STATE_COLOR.get(state, (200, 200, 200))
    y, dy = 30, 22

    cv.putText(frame, f"STATE: {state}", (10, y),
               cv.FONT_HERSHEY_PLAIN, 1.4, color, thickness=2)
    y += dy

    if pose_dict["valid"]:
        cv.putText(frame,
                   f"e_x={pose_dict['lateral_error']:+.3f}  "
                   f"e_w={pose_dict['depth_error']:+.3f}  "
                   f"conf={pose_dict['confidence']:.2f}",
                   (10, y), cv.FONT_HERSHEY_PLAIN, 1.3, (200, 200, 200), thickness=2)
    else:
        cv.putText(frame, "gate not found",
                   (10, y), cv.FONT_HERSHEY_PLAIN, 1.3, (100, 100, 100), thickness=2)
    y += dy

    cv.putText(frame, f"v={v_cmd:+.3f}  w={w_cmd:+.3f}",
               (10, y), cv.FONT_HERSHEY_PLAIN, 1.3, (0, 200, 200), thickness=2)
    y += dy

    cv.putText(frame, f"confirm: {confirm_count}/{confirm_max}",
               (10, y), cv.FONT_HERSHEY_PLAIN, 1.3, color, thickness=2)


############################################################
# Phase 2: Gate alignment (replaces fixed 90° turn)
############################################################

def driveGateAlign(robot_ip):
    # Env var overrides of gate controller params
    def _ef(name):
        v = os.getenv(name)
        return float(v) if v is not None else None

    if (v := _ef("GATE1_TARGET_X")) is not None: gate1_ctrl.target_x_norm   = v
    if (v := _ef("GATE1_TARGET_W")) is not None: gate1_ctrl.target_width_px  = v
    if (v := _ef("GATE1_STOP_W"))   is not None: gate1_ctrl.stop_width_px    = v

    search_w    = float(os.getenv("GATE_SEARCH_W",     str(SEARCH_W)))
    tol_x       = float(os.getenv("GATE_ALIGN_TOL_X",  str(ALIGN_TOL_X)))
    tol_w       = float(os.getenv("GATE_ALIGN_TOL_W",  str(ALIGN_TOL_W)))
    confirm_max = int(  os.getenv("GATE_ALIGN_CONFIRM", str(ALIGN_CONFIRM)))
    align_max_w = float(os.getenv("GATE_ALIGN_MAX_W",   str(ALIGN_MAX_W)))

    stream_url = f"http://{robot_ip}:7123/stream.mjpg"
    if not service.is_quiet():
        print(f"% driveGateAlign: stream={stream_url}")
        print(f"% driveGateAlign: target_x={gate1_ctrl.target_x_norm:+.3f}  "
              f"target_w={gate1_ctrl.target_width_px:.1f}  stop_w={gate1_ctrl.stop_width_px:.1f}")
        print(f"% driveGateAlign: tol_x={tol_x:.3f}  tol_w={tol_w:.3f}  "
              f"confirm={confirm_max}  search_w={search_w:+.3f}  align_max_w={align_max_w:.3f}")
        print("% Ctrl+C to abort")

    cap = cv.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"% ERROR: could not open stream at {stream_url}")
        return

    gate1.setup()
    gate1_ctrl.reset()

    service.send("robobot/cmd/T0", "leds 16 30 30 0")   # yellow: aligning

    state         = "SEARCH"
    confirm_count = 0
    frame_count   = 0
    last_frame    = None
    last_time     = t.time()
    t_print       = last_time
    pose_dict     = {"valid": False, "lateral_error": 0.0,
                     "depth_error": 0.0, "confidence": 0.0}

    try:
        while not service.stop:
            ret, frame = cap.read()
            if not ret:
                print("% driveGateAlign: lost stream")
                break

            now    = t.time()
            dt     = max(now - last_time, 1e-3)
            last_time = now
            frame_count += 1

            # Run detector + get filtered pose (updates EMA inside gate1_ctrl)
            gate1.detect(frame)
            _, _, pose_dict = gate1_ctrl.command(gate1, dt=dt)

            # --- Three-state FSM ---
            if state == "DONE":
                v_cmd, w_cmd = 0.0, 0.0

            elif not gate1.detected or not pose_dict["valid"]:
                state         = "SEARCH"
                confirm_count = 0
                v_cmd         = 0.0
                w_cmd         = search_w

            else:
                e_x   = pose_dict["lateral_error"]
                e_w   = pose_dict["depth_error"]
                state = "ALIGN"

                # Pure in-place rotation: P controller on lateral error.
                w_cmd = float(np.clip(gate1_ctrl.kx * e_x, -align_max_w, align_max_w))
                v_cmd = 0.0

                if abs(e_x) < tol_x and abs(e_w) < tol_w:
                    confirm_count += 1
                else:
                    confirm_count = 0

                if confirm_count >= confirm_max:
                    state = "DONE"
                    v_cmd, w_cmd = 0.0, 0.0

            service.send("robobot/cmd/ti", f"rc {v_cmd:.3f} {w_cmd:.3f}")

            # --- Push annotated frame to browser stream ---
            _annotate(frame, state, pose_dict, v_cmd, w_cmd, confirm_count, confirm_max)
            _push_frame(frame)
            last_frame = frame

            # --- Terminal status at ~2 Hz ---
            if not service.is_quiet() and now - t_print >= 0.5:
                t_print = now
                if pose_dict["valid"]:
                    print(f"% [{state:<6}]  cx={gate1.bar_cx:4d}  "
                          f"w={gate1.bar_width_px:5.1f}px  "
                          f"e_x={pose_dict['lateral_error']:+.3f}  "
                          f"e_w={pose_dict['depth_error']:+.3f}  "
                          f"confirm={confirm_count}/{confirm_max}  "
                          f"rc={v_cmd:+.3f},{w_cmd:+.3f}")
                else:
                    print(f"% [{state:<6}]  gate not found  "
                          f"rc={v_cmd:+.3f},{w_cmd:+.3f}")

            # --- DONE: report final pose, wait for Enter ---
            if state == "DONE":
                service.send("robobot/cmd/ti", "rc 0 0")
                service.send("robobot/cmd/T0", "leds 16 0 100 0")   # green: done
                print("\n% ======== ALIGNMENT DONE ========")
                print(f"%  lateral_error  = {pose_dict['lateral_error']:+.6f}  "
                      f"(tol {tol_x:.3f})")
                print(f"%  depth_error    = {pose_dict['depth_error']:+.6f}  "
                      f"(tol {tol_w:.3f})")
                print(f"%  confidence     = {pose_dict['confidence']:.4f}")
                print(f"%  bar cx/w/h     = {gate1.bar_cx} / "
                      f"{gate1.bar_width_px} / {gate1.bar_height_px} px")
                print(f"%  target_x_norm  = {gate1_ctrl.target_x_norm:+.3f}")
                print(f"%  target_width   = {gate1_ctrl.target_width_px:.1f} px")
                print("% =================================")
                print("% Press Enter to stop test and release robot...")
                try:
                    input()
                except EOFError:
                    pass
                break

    except KeyboardInterrupt:
        pass
    finally:
        service.send("robobot/cmd/ti", "rc 0 0")
        cap.release()
        gate1.terminate()

    if last_frame is not None:
        ts = datetime.now().strftime("%Y_%b_%d_%H%M%S")
        fn = os.path.join(os.path.dirname(__file__),
                          f"roundabout_align_{ts}_{frame_count:04d}.jpg")
        cv.imwrite(fn, last_frame)
        if not service.is_quiet():
            print(f"% Saved final frame to {fn}")

    if not service.is_quiet():
        print(f"% driveGateAlign: complete — {frame_count} frames, "
              f"{gate1.detCnt} detections")


############################################################
# Top-level loop
############################################################

def loop():
    if _robot_ip is None:
        print("% Usage: python3 test/test_roundabout_gate_align.py "
              "<robot-ip> [--now] [-s]")
        return

    service.send("robobot/cmd/T0", "leds 16 30 30 0")   # yellow: ready
    edge.lineControl(0)

    if not service.is_quiet():
        print(f"% Roundabout gate-align test  robot_ip={_robot_ip}")
        print("% Phase 1: line follow with crossing detection until line ends")

    driveLineWithCrossings()
    if service.stop:
        return

    t.sleep(0.1)

    if not service.is_quiet():
        print("% Phase 2: gate alignment (replaces fixed 90° turn)")

    driveGateAlign(_robot_ip)

    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.send("robobot/cmd/ti", "rc 0 0")
    if not service.is_quiet():
        print("% Test complete")


############################################################
# Entry point
############################################################

if __name__ == "__main__":
    # Extract the robot IP (first non-flag arg) before uservice sees sys.argv.
    for _i, _a in enumerate(sys.argv[1:], 1):
        if not _a.startswith("-"):
            _robot_ip = _a
            sys.argv.pop(_i)
            break

    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running")
        print("%   to kill: pkill mqtt-client  (or pkill -9 mqtt-client)")
    else:
        setproctitle("mqtt-client")
        start_teensy_interface()
        service.setup('localhost')
        if service.connected:
            start_stream_server(port=5000)
            loop()
        service.terminate()
        stop_teensy_interface()
    if not service.is_quiet():
        print("% Main Terminated")
