#!/usr/bin/env python3
"""Keyboard teleoperation — arrow keys / WASD, no extra dependencies.

  W / ↑   forward          S / ↓   backward
  A / ←   turn left        D / →   turn right
  Q / E   strafe left/right (angular only)
  Space   emergency stop
  +/-     increase/decrease speed step
  X       quit

Publishes to robobot/teleop/cmd (same as teleop_input.py).
"""

import sys, os, tty, termios, select, json, time, threading
from datetime import datetime

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("% pip install paho-mqtt"); sys.exit(1)

# ── tunables ────────────────────────────────────────────────────────────────
MQTT_HOST       = 'localhost'
MQTT_PORT       = 1883
WHEELBASE       = 0.23          # m
MAX_LINEAR      = 0.5           # m/s
MAX_ANGULAR     = 1.5           # rad/s
SPEED_STEP      = 0.05          # m/s per +/- press
RESEND_HZ       = 10            # republish rate while key held
# ────────────────────────────────────────────────────────────────────────────

def _parse_args():
    import argparse
    p = argparse.ArgumentParser(description='Keyboard teleop')
    p.add_argument('-i', '--host', default=MQTT_HOST)
    p.add_argument('-p', '--port', type=int, default=MQTT_PORT)
    return p.parse_args()


class KeyboardTeleop:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.connected = False
        self.client = None

        self.linear  = 0.0
        self.angular = 0.0
        self.speed   = 0.3       # current speed magnitude

        self._stop = threading.Event()
        self._setup_mqtt()
        self._resend_thread = threading.Thread(target=self._resend_loop, daemon=True)
        self._resend_thread.start()

    # ── MQTT ────────────────────────────────────────────────────────────────
    def _setup_mqtt(self):
        if hasattr(mqtt_client, 'CallbackAPIVersion'):
            self.client = mqtt_client.Client(
                client_id='teleop-keyboard',
                callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1)
        else:
            self.client = mqtt_client.Client(client_id='teleop-keyboard')
        self.client.on_connect    = lambda c,u,f,rc: setattr(self, 'connected', rc == 0)
        self.client.on_disconnect = lambda c,u,rc:   setattr(self, 'connected', False)
        self.client.connect(self.host, self.port, keepalive=60)
        self.client.loop_start()
        for _ in range(30):
            if self.connected: break
            time.sleep(0.1)
        if not self.connected:
            print(f"% ERROR: cannot connect to {self.host}:{self.port}")
            sys.exit(1)

    def _publish(self):
        if not self.connected:
            return
        lv = max(-MAX_LINEAR,  min(MAX_LINEAR,  self.linear))
        av = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.angular))
        left  = lv - (WHEELBASE / 2.0) * av
        right = lv + (WHEELBASE / 2.0) * av
        cmd = {
            "linear_velocity":  lv,
            "angular_velocity": av,
            "v_left":  left,
            "v_right": right,
            "timestamp": datetime.now().isoformat(),
        }
        self.client.publish("robobot/teleop/cmd", json.dumps(cmd), qos=0)

    def _resend_loop(self):
        interval = 1.0 / RESEND_HZ
        while not self._stop.is_set():
            self._publish()
            time.sleep(interval)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self._publish()
        self._stop.set()
        self.client.loop_stop()
        self.client.disconnect()

    # ── key handling ────────────────────────────────────────────────────────
    def _apply_key(self, key):
        """Return False to quit."""
        k = key.lower()
        if k in ('x',):
            return False
        elif k in (' ',):                   # space → stop
            self.linear  = 0.0
            self.angular = 0.0
        elif k in ('w', '\x1b[A'):          # W / ↑
            self.linear  =  self.speed
            self.angular =  0.0
        elif k in ('s', '\x1b[B'):          # S / ↓
            self.linear  = -self.speed
            self.angular =  0.0
        elif k in ('a', '\x1b[D'):          # A / ←
            self.linear  =  0.0
            self.angular =  self.speed
        elif k in ('d', '\x1b[C'):          # D / →
            self.linear  =  0.0
            self.angular = -self.speed
        elif k == 'q':                      # Q  turn-left while moving
            self.angular =  self.speed
        elif k == 'e':                      # E  turn-right while moving
            self.angular = -self.speed
        elif key in ('+', '='):
            self.speed = min(MAX_LINEAR, round(self.speed + SPEED_STEP, 3))
            print(f"\r  speed step: {self.speed:.2f} m/s          ", end='', flush=True)
        elif key == '-':
            self.speed = max(SPEED_STEP, round(self.speed - SPEED_STEP, 3))
            print(f"\r  speed step: {self.speed:.2f} m/s          ", end='', flush=True)
        return True

    # ── main loop ───────────────────────────────────────────────────────────
    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        print("\n  Keyboard teleop ready")
        print("  W/↑ fwd  S/↓ back  A/← left  D/→ right")
        print("  Q/E angular only   Space stop   +/- speed   X quit\n")
        print(f"  speed: {self.speed:.2f} m/s", flush=True)
        try:
            tty.setraw(fd)
            buf = ''
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                # accumulate escape sequences (arrow keys = ESC [ A/B/C/D)
                if ch == '\x1b':
                    buf = ch
                    continue
                if buf:
                    buf += ch
                    if len(buf) == 2 and ch == '[':
                        continue          # still reading
                    key = buf
                    buf = ''
                else:
                    key = ch
                if not self._apply_key(key):
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.stop()
            print("\n% Keyboard teleop stopped")


def main():
    args = _parse_args()
    t = KeyboardTeleop(args.host, args.port)
    t.run()


if __name__ == '__main__':
    main()
