#!/usr/bin/env python3
"""
Keyboard teleop control for Robobot over MQTT.

Default control mode is step/pulse: each keypress sends a fixed movement pulse.
This is usually more predictable than hold-to-run under heavy CPU load.

  w / UP    : forward
  s / DOWN  : backward
  a / LEFT  : turn left (CCW)
  d / RIGHT : turn right (CW)
  SPACE     : stop immediately
  + / -     : increase / decrease linear speed
  ] / [     : increase / decrease turn-rate
  m         : toggle step <-> hold mode
  q / ESC   : quit

Run with -s to suppress verbose MQTT send prints:
  python3 mqtt_teleop_control.py -s
"""

import sys
import threading
import tty
import termios
import select
import time as t
import cv2 as cv
import argparse
from setproctitle import setproctitle
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface
from sgate import gate
from sgate_1 import gate1
try:
    from flask import Flask, Response
    _flask_available = True
except ImportError:
    _flask_available = False

LINEAR_VEL  = 0.4
TURN_RATE   = 0.5
LINEAR_STEP = 0.2
TURN_STEP   = 0.35
KEY_TIMEOUT     = 0.02
HOLD_WINDOW     = 0.30
RESEND_INTERVAL = 0.10
STEP_LINEAR_SEC = 0.35
STEP_TURN_SEC   = 0.25


_stream_frame  = None
_stream_lock   = threading.Lock()
_teleop_status = {"linvel": 0.0, "turnrate": 0.0}

if _flask_available:
    _flask_app = Flask(__name__)

    def _push_frame(frame):
        global _stream_frame
        with _stream_lock:
            _stream_frame = frame.copy()

    def _mjpeg_generate():
        while True:
            with _stream_lock:
                f = _stream_frame
            if f is not None:
                ok, buf = cv.imencode(".jpg", f, [cv.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + buf.tobytes() + b"\r\n")
            t.sleep(0.033)

    @_flask_app.route("/")
    def _index():
        return ('<html><body style="background:#000;margin:0">'
                '<img src="/stream" style="max-width:100%"></body></html>')

    @_flask_app.route("/stream")
    def _stream_view():
        return Response(
            _mjpeg_generate(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def start_stream_server(port=5000):
        import logging

        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        threading.Thread(
            target=lambda: _flask_app.run(host="0.0.0.0", port=port, threaded=True),
            daemon=True,
        ).start()
        print(f"% Camera stream: http://{service.host}:{port}/")

else:
    def start_stream_server(port=5000):
        print("% Flask not installed — camera stream disabled (pip install flask)")


def _camera_loop():
    """Stream gate detections over MJPEG using a dedicated capture connection."""
    url = f"http://{service.host}:7123/stream.mjpg"
    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% Camera loop: could not open {url} — web view will be empty")
        return
    print(f"% Camera loop: streaming from {url}")
    while not service.stop:
        ret, img = cap.read()
        if not ret:
            t.sleep(0.05)
            continue
        gate.detect(img)
        gate.paint(img)
        gate1.detect(img)
        gate1.paint(img)
        s = _teleop_status
        cv.putText(
            img,
            f"vel={s['linvel']:+.2f} m/s  turn={s['turnrate']:+.2f} rad/s",
            (8, 20),
            cv.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            1,
            cv.LINE_AA,
        )
        if _flask_available:
            _push_frame(img)
    cap.release()


def read_key(fd):
    """Return the next keypress or None if nothing arrived before timeout."""
    r, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r2:
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                r3, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r3:
                    ch3 = sys.stdin.read(1)
                    return "arrow_" + ch3
        return "\x1b"
    return ch


def print_status(linvel, turnrate, lin_speed, turn_speed):
    print(
        f"\r  step: lin={lin_speed:.2f}  turn={turn_speed:.2f}"
        f"  |  command={'MOVE' if (linvel or turnrate) else 'STOP'}   ",
        end="",
        flush=True,
    )


def loop(step_mode=True, step_linear_sec=STEP_LINEAR_SEC, step_turn_sec=STEP_TURN_SEC):
    lin_speed = LINEAR_VEL
    turn_speed = TURN_RATE
    linvel = 0.0
    turnrate = 0.0
    held_linvel = 0.0
    held_turnrate = 0.0
    last_key_time = 0.0
    last_send_time = 0.0
    pulse_until = 0.0
    force_send = False

    print("=== Robobot Keyboard Teleop ===")
    print("  w/s or UP/DOWN   : forward / backward")
    print("  a/d or LEFT/RIGHT: turn left / right")
    print("  SPACE            : stop")
    print("  +/-              : linear speed step")
    print("  ]/[              : turn rate step")
    print("  m                : toggle step/hold mode")
    print("  q or ESC         : quit")
    print(
        f"  mode             : {'step' if step_mode else 'hold'}"
        f" (step lin={step_linear_sec:.2f}s, turn={step_turn_sec:.2f}s)"
    )
    print()

    service.send("robobot/cmd/T0", "leds 16 0 0 30")
    service.send("robobot/cmd/ti", "rc 0.0 0.0")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not service.stop:
            key = read_key(fd)
            now = t.monotonic()
            update_speed = False

            if key in ("w", "arrow_A"):
                held_linvel, held_turnrate = lin_speed, 0.0
                last_key_time = now
                force_send = True
                if step_mode:
                    pulse_until = now + step_linear_sec
            elif key in ("s", "arrow_B"):
                held_linvel, held_turnrate = -lin_speed, 0.0
                last_key_time = now
                force_send = True
                if step_mode:
                    pulse_until = now + step_linear_sec
            elif key in ("a", "arrow_D"):
                held_linvel, held_turnrate = 0.0, turn_speed
                last_key_time = now
                force_send = True
                if step_mode:
                    pulse_until = now + step_turn_sec
            elif key in ("d", "arrow_C"):
                held_linvel, held_turnrate = 0.0, -turn_speed
                last_key_time = now
                force_send = True
                if step_mode:
                    pulse_until = now + step_turn_sec
            elif key == " ":
                held_linvel, held_turnrate = 0.0, 0.0
                last_key_time = 0.0
                pulse_until = 0.0
                force_send = True
            elif key == "+":
                lin_speed = min(1.0, lin_speed + LINEAR_STEP)
                update_speed = True
            elif key == "-":
                lin_speed = max(0.05, lin_speed - LINEAR_STEP)
                update_speed = True
            elif key == "]":
                turn_speed = min(3.0, turn_speed + TURN_STEP)
                update_speed = True
            elif key == "[":
                turn_speed = max(0.1, turn_speed - TURN_STEP)
                update_speed = True
            elif key == "m":
                step_mode = not step_mode
                pulse_until = 0.0
                held_linvel, held_turnrate = 0.0, 0.0
                last_key_time = 0.0
                print(
                    f"\n% Control mode: {'step' if step_mode else 'hold'} "
                    f"(lin={step_linear_sec:.2f}s turn={step_turn_sec:.2f}s)"
                )
                update_speed = True
                force_send = True
            elif key in ("q", "\x1b"):
                break

            if step_mode and now < pulse_until:
                target_linvel, target_turnrate = held_linvel, held_turnrate
            elif (not step_mode) and (now - last_key_time) < HOLD_WINDOW:
                target_linvel, target_turnrate = held_linvel, held_turnrate
            else:
                target_linvel, target_turnrate = 0.0, 0.0

            changed = (target_linvel != linvel or target_turnrate != turnrate)
            due = (now - last_send_time) >= RESEND_INTERVAL
            if changed or due or update_speed or force_send:
                linvel, turnrate = target_linvel, target_turnrate
                _teleop_status["linvel"] = linvel
                _teleop_status["turnrate"] = turnrate
                service.send("robobot/cmd/ti", f"rc {linvel:.2f} {turnrate:.2f}")
                last_send_time = now
                print_status(linvel, turnrate, lin_speed, turn_speed)
                force_send = False

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        print("% Teleop stopped")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Keyboard teleop over MQTT")
    parser.add_argument(
        "--vision",
        choices=("on", "off"),
        default="off",
        help="Enable camera + CV processing thread (default: off).",
    )
    parser.add_argument(
        "--web-stream",
        choices=("on", "off"),
        default="off",
        help="Enable local MJPEG web stream server (default: off).",
    )
    parser.add_argument(
        "--mode",
        choices=("step", "hold"),
        default="step",
        help="Control mode: step pulse per keypress or hold mode (default: step).",
    )
    parser.add_argument(
        "--step-linear-sec",
        type=float,
        default=STEP_LINEAR_SEC,
        help="Pulse duration for forward/back in step mode.",
    )
    parser.add_argument(
        "--step-turn-sec",
        type=float,
        default=STEP_TURN_SEC,
        help="Pulse duration for turning in step mode.",
    )
    args, unknown = parser.parse_known_args()
    # Keep downstream parsers (e.g. uservice) from seeing teleop-only flags.
    sys.argv = [sys.argv[0]] + unknown

    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running — terminating")
    else:
        setproctitle("mqtt-teleop-control")
        if "-s" not in sys.argv and "--silent" not in sys.argv:
            sys.argv.append("--silent")
        start_teensy_interface()
        service.setup("localhost")
        if service.connected:
            if args.web_stream == "on":
                start_stream_server(port=5000)
            if args.vision == "on":
                threading.Thread(target=_camera_loop, daemon=True).start()
            else:
                print("% Vision disabled (--vision off).")
            loop(
                step_mode=(args.mode == "step"),
                step_linear_sec=max(0.05, args.step_linear_sec),
                step_turn_sec=max(0.05, args.step_turn_sec),
            )
        service.terminate()
        stop_teensy_interface()
    print("% Teleop terminated")
