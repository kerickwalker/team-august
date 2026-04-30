#!/usr/bin/env python3
"""
Servo diagnostics for Robobot.

Purpose:
- Start teensy_interface
- Connect to local MQTT broker
- Send servo commands directly on robobot/cmd/T0
- Listen for robobot/drive/T0/svo feedback
- Print a clear diagnosis about command path vs likely hardware/power issue
"""

import argparse
import queue
import sys
import threading
import time
from datetime import datetime

from paho.mqtt import client as mqtt

from uteensy import start_teensy_interface, stop_teensy_interface

MQTT_HOST_DEFAULT = "localhost"
MQTT_PORT_DEFAULT = 1883

TOPIC_CMD_T0 = "robobot/cmd/T0"
TOPIC_CMD_TI = "robobot/cmd/ti"
TOPIC_SVO = "robobot/drive/T0/svo"
TOPIC_INFO = "robobot/drive/T0/info"
TOPIC_MASTER = "robobot/drive/master"


def parse_args():
    p = argparse.ArgumentParser(description="Run automatic servo pipeline diagnostics.")
    p.add_argument("--host", default=MQTT_HOST_DEFAULT, help="MQTT host (default: localhost)")
    p.add_argument("--port", type=int, default=MQTT_PORT_DEFAULT, help="MQTT port (default: 1883)")
    p.add_argument("--servo", type=int, default=3, help="Servo ID to test (default: 3)")
    p.add_argument("--speed", type=int, default=100, help="Servo speed (default: 100)")
    p.add_argument("--pos-a", type=int, default=-300, dest="pos_a", help="First position (default: -300)")
    p.add_argument("--pos-b", type=int, default=300, dest="pos_b", help="Second position (default: 300)")
    p.add_argument("--wait-svo", type=float, default=1.5, dest="wait_svo",
                   help="Seconds to wait for svo after each command (default: 1.5)")
    p.add_argument("--hold-s", type=float, default=2.0, dest="hold_s",
                   help="Hold time after each command before evaluation (default: 2.0)")
    p.add_argument("--reach-timeout", type=float, default=2.5, dest="reach_timeout",
                   help="Seconds to wait for servo to approach target (default: 2.5)")
    p.add_argument("--tolerance", type=int, default=35,
                   help="Allowed position error to count as reached (default: 35)")
    p.add_argument("--skip-start-ti", action="store_true",
                   help="Skip launching teensy_interface (use if already running)")
    return p.parse_args()


class DiagState:
    def __init__(self):
        self.connected = False
        self.svo_queue = queue.Queue()
        self.info_queue = queue.Queue()
        self.master_queue = queue.Queue()
        self.last_svo = None
        self.last_info = None
        self.last_master = None
        self.stop_alive = False


def wait_for_queue(q, timeout_s):
    try:
        return q.get(timeout=timeout_s)
    except queue.Empty:
        return None


def drain_queue(q):
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return


def parse_svo_position(payload, servo_id):
    """
    Parse robobot/drive/T0/svo payload:
      "<time> en1 pos1 vel1  en2 pos2 vel2  ..."
    Returns servo position for servo_id (1-based), or None on parse failure.
    """
    try:
        vals = payload.strip().split()
        if len(vals) < 4:
            return None
        # skip timestamp at index 0, then groups of 3 values.
        group_start = 1 + (servo_id - 1) * 3
        if group_start + 2 >= len(vals):
            return None
        pos = int(vals[group_start + 1])
        return pos
    except Exception:
        return None


def wait_for_motion_towards(q, servo_id, target, timeout_s, tolerance):
    """
    Observe several svo frames after a command.
    Returns: (saw_any, reached_or_approached, last_pos, samples)
    """
    deadline = time.time() + timeout_s
    saw_any = False
    samples = []
    best_dist = None
    reached = False
    last_pos = None

    while time.time() < deadline:
        remaining = max(0.0, deadline - time.time())
        got = wait_for_queue(q, min(0.25, remaining))
        if got is None:
            continue
        pos = parse_svo_position(got, servo_id)
        if pos is None:
            continue
        saw_any = True
        last_pos = pos
        samples.append((pos, got.strip()))
        dist = abs(target - pos)
        if best_dist is None or dist < best_dist:
            best_dist = dist
        if dist <= tolerance:
            reached = True
            break

    improved = best_dist is not None and best_dist <= (2 * tolerance)
    return saw_any, (reached or improved), last_pos, samples


def main():
    args = parse_args()
    state = DiagState()
    started_ti = False
    start_id = str(datetime.now())

    client = mqtt.Client(client_id=f"servo-diag-{int(time.time())}")

    def on_connect(_client, _userdata, _flags, rc):
        if rc == 0:
            state.connected = True
            _client.subscribe(TOPIC_SVO)
            _client.subscribe(TOPIC_INFO)
            _client.subscribe(TOPIC_MASTER)
            print(f"% Connected to MQTT {args.host}:{args.port}")
        else:
            print(f"% MQTT connect failed rc={rc}")

    def on_message(_client, _userdata, msg):
        payload = msg.payload.decode(errors="replace")
        if msg.topic == TOPIC_SVO:
            state.last_svo = payload
            state.svo_queue.put(payload)
        elif msg.topic == TOPIC_INFO:
            state.last_info = payload
            state.info_queue.put(payload)
        elif msg.topic == TOPIC_MASTER:
            state.last_master = payload
            state.master_queue.put(payload)

    def alive_loop():
        while not state.stop_alive:
            client.publish(TOPIC_CMD_TI, f"alive {start_id}")
            time.sleep(0.5)

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        if not args.skip_start_ti:
            start_teensy_interface()
            started_ti = True
            print("% Started teensy_interface")
            time.sleep(0.8)

        client.connect(args.host, args.port)
        client.loop_start()

        t0 = time.time()
        while not state.connected and (time.time() - t0) < 3.0:
            time.sleep(0.05)

        if not state.connected:
            print("DIAG: FAIL - Cannot connect to local MQTT broker")
            return 2

        alive_th = threading.Thread(target=alive_loop, daemon=True)
        alive_th.start()

        # Allow baseline messages to arrive.
        time.sleep(0.4)
        baseline_svo = state.last_svo
        print(f"% Baseline svo: {baseline_svo if baseline_svo else '<none yet>'}")

        commands = [
            f"servo {args.servo} {args.pos_a} {args.speed}",
            f"servo {args.servo} {args.pos_b} {args.speed}",
            f"servo {args.servo} {args.pos_a} {args.speed}",
        ]

        got_svo_after_cmd = []
        pos_matches = []
        for i, cmd in enumerate(commands, start=1):
            drain_queue(state.svo_queue)
            print(f"% TX[{i}] {cmd}")
            r = client.publish(TOPIC_CMD_T0, cmd)
            if r.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"% Publish failed rc={r.rc}")
            if args.hold_s > 0:
                time.sleep(args.hold_s)
            wanted = args.pos_a if i in (1, 3) else args.pos_b
            saw_any, ok, last_pos, samples = wait_for_motion_towards(
                state.svo_queue,
                args.servo,
                wanted,
                args.reach_timeout,
                args.tolerance,
            )
            got_svo_after_cmd.append(saw_any)
            pos_matches.append(ok)
            if not saw_any:
                print(f"% RX[{i}] no svo within {args.reach_timeout:.1f}s")
            else:
                print(
                    f"% RX[{i}] servo{args.servo} last_pos={last_pos} "
                    f"wanted={wanted} ok={ok} samples={len(samples)}"
                )
                if samples:
                    print(f"% RX[{i}] last raw svo {samples[-1][1]}")

        any_svo = any(got_svo_after_cmd)
        all_svo = all(got_svo_after_cmd)
        all_pos = len(pos_matches) == 3 and all(pos_matches)

        print("\n=== Diagnosis ===")
        if not any_svo:
            print("DIAG: FAIL - No svo feedback after servo commands.")
            print("CAUSE: command path to Teensy or Teensy telemetry is broken.")
            print("NEXT: inspect teensy_interface log and USB/power stability first.")
            return 1
        if not all_svo:
            print("DIAG: WARN - Intermittent svo feedback (some commands got no reply).")
            print("CAUSE: likely unstable Teensy link/power or process contention.")
            return 1
        if not all_pos:
            print("DIAG: FAIL - svo is alive, but servo target position does not follow commands.")
            print("CAUSE: Teensy/firmware command handling or servo channel hardware issue.")
            print("NEXT: test another servo ID and inspect servo power/connector integrity.")
            return 1

        print("DIAG: PASS - MQTT -> Teensy -> svo feedback pipeline is alive.")
        print("CAUSE if no physical motion: likely servo power/wiring/mechanical issue.")
        return 0

    finally:
        state.stop_alive = True
        try:
            client.loop_stop()
        except Exception:
            pass
        try:
            client.disconnect()
        except Exception:
            pass
        if started_ti:
            stop_teensy_interface()
            print("% Stopped teensy_interface")


if __name__ == "__main__":
    sys.exit(main())
