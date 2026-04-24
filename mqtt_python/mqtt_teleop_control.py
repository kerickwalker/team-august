#!/usr/bin/env python3
"""
Keyboard teleop control for Robobot over MQTT.

Hold a key to move; release to stop.

  w / UP    : forward
  s / DOWN  : backward
  a / LEFT  : turn left (CCW)
  d / RIGHT : turn right (CW)
  SPACE     : stop immediately
  + / -     : increase / decrease linear speed step
  ] / [     : increase / decrease turn-rate step
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

LINEAR_VEL  = 0.4   # m/s
TURN_RATE   = 0.5   # rad/s
LINEAR_STEP = 0.2  # m/s per +/- press
TURN_STEP   = 0.35   # rad/s per ]/[ press
KEY_TIMEOUT     = 0.02   # seconds to wait for a keypress before treating as "no key"
HOLD_WINDOW     = 0.30   # seconds to keep sending last movement after key release (bridges OS key-repeat gap)
RESEND_INTERVAL = 0.10   # seconds between periodic resends to keep Teensy velocity controller refreshed


################################################################
# MJPEG web stream with gate-detection overlay
################################################################

_stream_frame  = None
_stream_lock   = threading.Lock()
_teleop_status = {'linvel': 0.0, 'turnrate': 0.0}  # updated by loop()

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
        print(f"% Camera stream: http://{service.host}:{port}/")

else:
    def start_stream_server(port=5000):
        print("% Flask not installed — camera stream disabled (pip install flask)")


def _camera_loop():
    """Background thread: open a direct connection to the MJPEG stream, run gate
    detections on each frame, and push the annotated frame to the web stream.
    Opens its own VideoCapture so scam.py's single-consumer flag doesn't interfere."""
    url = f'http://{service.host}:7123/stream.mjpg'
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
        cv.putText(img, f"vel={s['linvel']:+.2f} m/s  turn={s['turnrate']:+.2f} rad/s",
                   (8, 20), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv.LINE_AA)
        if _flask_available:
            _push_frame(img)
    cap.release()


################################################################


def read_key(fd):
    """Return the next keypress as a string, or None if nothing arrived within KEY_TIMEOUT."""
    r, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r2:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                r3, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r3:
                    ch3 = sys.stdin.read(1)
                    return 'arrow_' + ch3  # A=up B=down C=right D=left
        return '\x1b'
    return ch


def print_status(linvel, turnrate, lin_speed, turn_speed):
    print(
        f"\r  vel={linvel:+.2f} m/s  turn={turnrate:+.2f} rad/s"
        f"  |  step: lin={lin_speed:.2f}  turn={turn_speed:.2f}   ",
        end='', flush=True
    )


def loop():
    lin_speed  = LINEAR_VEL
    turn_speed = TURN_RATE
    linvel     = 0.0
    turnrate   = 0.0

    # Held-key state: remember the last movement command and when it was set.
    # The robot keeps moving for HOLD_WINDOW seconds after the last key event,
    # which bridges the OS key-repeat initial delay (typically 250-500 ms).
    held_linvel    = 0.0
    held_turnrate  = 0.0
    last_key_time  = 0.0   # t.monotonic() of last movement key; 0 = no key held
    last_send_time = 0.0

    print("=== Robobot Keyboard Teleop ===")
    print("  w/s or UP/DOWN   : forward / backward")
    print("  a/d or LEFT/RIGHT: turn left / right")
    print("  SPACE            : stop")
    print("  +/-              : linear speed step")
    print("  ]/[              : turn rate step")
    print("  q or ESC         : quit")
    print()

    service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue = teleop active
    service.send("robobot/cmd/ti", "rc 0.0 0.0")

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not service.stop:
            key = read_key(fd)
            now = t.monotonic()
            update_speed = False

            if key in ('w', 'arrow_A'):      # forward
                held_linvel, held_turnrate = lin_speed, 0.0
                last_key_time = now
            elif key in ('s', 'arrow_B'):    # backward
                held_linvel, held_turnrate = -lin_speed, 0.0
                last_key_time = now
            elif key in ('a', 'arrow_D'):    # turn left (CCW)
                held_linvel, held_turnrate = 0.0, turn_speed
                last_key_time = now
            elif key in ('d', 'arrow_C'):    # turn right (CW)
                held_linvel, held_turnrate = 0.0, -turn_speed
                last_key_time = now
            elif key == ' ':                 # explicit stop — clear hold immediately
                held_linvel, held_turnrate = 0.0, 0.0
                last_key_time = 0.0
            elif key == '+':
                lin_speed  = min(1.0, lin_speed + LINEAR_STEP)
                update_speed = True
            elif key == '-':
                lin_speed  = max(0.05, lin_speed - LINEAR_STEP)
                update_speed = True
            elif key == ']':
                turn_speed = min(3.0, turn_speed + TURN_STEP)
                update_speed = True
            elif key == '[':
                turn_speed = max(0.1, turn_speed - TURN_STEP)
                update_speed = True
            elif key in ('q', '\x1b'):
                break
            # key is None (timeout) → no new key; hold window below decides whether to keep moving

            # Keep moving if within HOLD_WINDOW of last key event, else stop.
            if (now - last_key_time) < HOLD_WINDOW:
                target_linvel, target_turnrate = held_linvel, held_turnrate
            else:
                target_linvel, target_turnrate = 0.0, 0.0

            changed = (target_linvel != linvel or target_turnrate != turnrate)
            due     = (now - last_send_time) >= RESEND_INTERVAL
            if changed or due or update_speed:
                linvel, turnrate = target_linvel, target_turnrate
                _teleop_status['linvel']   = linvel
                _teleop_status['turnrate'] = turnrate
                service.send("robobot/cmd/ti", f"rc {linvel:.2f} {turnrate:.2f}")
                last_send_time = now
                print_status(linvel, turnrate, lin_speed, turn_speed)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()  # newline after status line
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        print("% Teleop stopped")


if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running — terminating")
    else:
        setproctitle("mqtt-teleop-control")
        # Default to silent so MQTT send prints don't clutter the status line.
        if '-s' not in sys.argv and '--silent' not in sys.argv:
            sys.argv.append('--silent')
        start_teensy_interface()
        service.setup('localhost')
        if service.connected:
            start_stream_server(port=5000)
            threading.Thread(target=_camera_loop, daemon=True).start()
            loop()
        service.terminate()
        stop_teensy_interface()
    print("% Teleop terminated")
