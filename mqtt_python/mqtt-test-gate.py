#!/usr/bin/env python3

# Gate detection and approach script using the full uservice/uteensy framework.
#
# Detects a single orange upright (gate post) via camera and drives toward it:
#   ALIGN   — turns in place when lateral error is too large
#   APPROACH — drives forward + steers when aligned
#   SEARCH  — rotates slowly to reacquire if gate lost
#
# Run with:
#   python3 mqtt-test-gate.py -e          (start on IR gate trigger)
#   python3 mqtt-test-gate.py -e --now    (start immediately)
#   python3 mqtt-test-gate.py -e --now -s (start immediately, silent)
#
# Tuning: edit the GATE config block below. All values are in SI units.

import time as t
import cv2 as cv
import numpy as np
from datetime import datetime
from types import SimpleNamespace
from setproctitle import setproctitle

from spose import pose
from sgpio import gpio
from scam  import cam
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface
from sgate_1 import gate1, gate1_ctrl


################################################################
# CONFIGURATION
################################################################

GATE = SimpleNamespace(
    # ── Detection / alignment ─────────────────────────────────
    target_x_norm   = 0.302,   # desired bar x position (calibrated from field)
    target_width_px = 128.0,   # bar width px at good entry pose
    stop_width_px   = 80.0,    # stop approaching when bar is this wide (robot close)

    # ── Speed limits ──────────────────────────────────────────
    min_v           = 0.03,    # m/s minimum forward speed
    max_v           = 0.08,    # m/s maximum forward speed
    max_w           = 0.45,    # rad/s maximum turn rate

    # ── Align phase (turn in place) ───────────────────────────
    align_threshold = 0.25,    # lateral error above which robot turns in place
    align_w         = 0.45,    # rad/s during in-place alignment

    # ── Search phase (gate lost) ──────────────────────────────
    search_w        = 0.25,    # rad/s slow rotation to reacquire gate

    # ── Safety ────────────────────────────────────────────────
    loss_stop_frames = 5,      # consecutive frames without gate before deadman stop
    timeout_s        = 30.0,   # maximum gate approach time before giving up
)


################################################################
# HELPERS
################################################################

def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


################################################################
# GATE APPROACH MISSION
################################################################

def driveGateApproach():
    """Camera-based gate approach loop.

    Runs until:
      - Gate bar reaches GATE.stop_width_px  (success — robot is at the gate)
      - Gate is lost for GATE.loss_stop_frames consecutive frames after deadman
      - GATE.timeout_s elapsed
      - service.stop is set (emergency stop)
    """
    # ── Apply config to controller ────────────────────────────
    gate1_ctrl.reset()
    gate1_ctrl.target_x_norm   = GATE.target_x_norm
    gate1_ctrl.target_width_px = GATE.target_width_px
    gate1_ctrl.stop_width_px   = GATE.stop_width_px
    gate1_ctrl.min_v           = GATE.min_v
    gate1_ctrl.max_v           = GATE.max_v
    gate1_ctrl.max_w           = GATE.max_w

    t_start     = t.time()
    last_t      = t_start
    t_last_print = t_start
    loss_frames = 0
    frame_count = 0
    mode        = "SEARCH"
    reached     = False

    service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue: gate approach running
    if service.is_quiet():
        print(f"% Gate approach  align_thr={GATE.align_threshold:.2f}  align_w={GATE.align_w:.2f}  stop_w={GATE.stop_width_px:.0f}px  timeout={GATE.timeout_s:.0f}s")

    while not service.stop:
        now = t.time()

        # ── Timeout guard ─────────────────────────────────────
        if now - t_start > GATE.timeout_s:
            if service.is_quiet():
                print("% Gate approach: timeout")
            break

        # ── Get camera frame ──────────────────────────────────
        ok, frame, _ = cam.getImage()
        if not ok:
            t.sleep(0.01)
            continue
        frame_count += 1

        dt     = max(now - last_t, 1e-3)
        last_t = now

        # ── Detect and compute base command ───────────────────
        gate1.detect(frame)
        v_cmd, w_cmd, pose_info = gate1_ctrl.command(gate1, dt=dt)

        # ── Mode selection ────────────────────────────────────
        if pose_info["valid"]:
            loss_frames = 0
            e_x = pose_info["lateral_error"]

            if abs(e_x) > GATE.align_threshold:
                # Turn in place: rotate toward target x, no forward movement.
                sign  = 1.0 if e_x > 0 else -1.0
                v_cmd = 0.0
                w_cmd = sign * GATE.align_w
                mode  = "ALIGN"
            else:
                mode = "APPROACH"
                # Success: bar is wide enough — we are at the gate.
                if gate1.bar_width_px >= GATE.stop_width_px:
                    reached = True
                    break

        else:
            loss_frames += 1
            mode = "SEARCH"

        # ── Deadman: stop if gate missing too long ────────────
        if loss_frames >= GATE.loss_stop_frames:
            v_cmd = 0.0
            w_cmd = 0.0

        # ── Final clamps ──────────────────────────────────────
        v_cmd = _clamp(v_cmd, 0.0, GATE.max_v)
        w_cmd = _clamp(w_cmd, -GATE.max_w, GATE.max_w)

        service.send("robobot/cmd/ti", f"rc {v_cmd:.3f} {w_cmd:.3f}")

        # ── Terminal status at ~2 Hz (only with -s) ──────────
        if service.is_quiet() and now - t_last_print >= 0.5:
            t_last_print = now
            if pose_info["valid"]:
                e_x  = pose_info["lateral_error"]
                e_w  = pose_info["depth_error"]
                side = "right" if e_x > 0 else "left"
                if mode == "ALIGN":
                    behavior = f"turning in place {side} to centre gate"
                elif mode == "APPROACH":
                    steer = f", steering {side}" if abs(w_cmd) > 0.02 else ", straight"
                    behavior = f"moving forward{steer}"
                else:
                    behavior = "stopped (deadman)"
                print(f"% [{mode:<7}] "
                      f"cx={gate1.bar_cx:4d}px  w={gate1.bar_width_px:3d}px  "
                      f"e_x={e_x:+.3f}  e_w={e_w:+.3f}  "
                      f"rc v={v_cmd:+.3f} w={w_cmd:+.3f}  "
                      f"=> {behavior}")
            else:
                side = gate1.bar_side_hint
                behavior = f"rotating {side} to reacquire" if side != "unknown" else "rotating to search"
                print(f"% [{'SEARCH':<7}] gate not found ({loss_frames} lost frames)  "
                      f"rc v={v_cmd:+.3f} w={w_cmd:+.3f}  "
                      f"=> {behavior}")

    # ── Stop robot ────────────────────────────────────────────
    service.send("robobot/cmd/ti", "rc 0 0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")

    elapsed = t.time() - t_start
    if service.is_quiet():
        status = "REACHED GATE" if reached else "stopped (timeout or gate lost)"
        print(f"% Gate approach done: {status}  frames={frame_count}  detections={gate1.detCnt}  time={elapsed:.1f}s")


################################################################
# TOP-LEVEL LOOP
################################################################

def loop():
    service.send("robobot/cmd/T0", "leds 16 30 30 0")   # yellow: ready

    # ── Optional start wait ───────────────────────────────────
    if not getattr(service.args, "now", False):
        print("% Waiting for start signal (--now to skip)...")
        while not service.stop:
            if gpio.test_stop_button():
                break
            t.sleep(0.05)

    if service.stop:
        return

    # ── Run gate approach ─────────────────────────────────────
    driveGateApproach()

    # ── Cleanup ───────────────────────────────────────────────
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)

    if service.is_quiet():
        print("% Done")


################################################################
# ENTRY POINT
################################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running - terminating")
        print("%   to kill a stuck instance: pkill mqtt-client   (or pkill -9 mqtt-client)")
    else:
        setproctitle("mqtt-client")
        gate1.setup()
        print("% Starting gate approach test")
        start_teensy_interface()
        service.setup('localhost')
        if service.connected:
            loop()
        service.terminate()
        stop_teensy_interface()
    print("% Main terminated")
