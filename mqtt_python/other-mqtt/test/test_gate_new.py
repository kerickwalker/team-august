#!/usr/bin/env python3

# Gate entry driving test with turn-in-place alignment.
# Based on test_gate1_entry_drive.py but adds an explicit align phase:
# when lateral error exceeds GATE1_ALIGN_THRESHOLD the robot rotates in
# place (v=0) until aligned, then switches to normal approach.
#
# Usage:
#   python3 test/test_gate_new.py <robot-ip> [mqtt-host]
#   python3 test/test_gate_new.py 10.197.217.81
#   python3 test/test_gate_new.py 10.197.217.81 localhost
#
# Env vars (all optional):
#   GATE1_TARGET_X          target bar x position [-1..1]  (default 0.302)
#   GATE1_TARGET_W          target bar width px            (default 128.0)
#   GATE1_STOP_W            stop when bar wider than this  (default 80.0)
#   GATE1_MAX_V             max forward speed m/s          (default 0.08)
#   GATE1_MAX_W             max turn rate rad/s            (default 0.45)
#   GATE1_ALIGN_THRESHOLD   lateral error to trigger align (default 0.25)
#   GATE1_ALIGN_W           turn rate during align rad/s   (default 0.45)
#   GATE1_TEST_TIMEOUT_S    max run time seconds           (default 20)
#   GATE1_STOP_ON_LOSS_FRAMES  consecutive loss before stop (default 5)
#   GATE1_TURN_ASSIST_W_MIN  |w| threshold for crawl inject (default 0.12)
#   GATE1_TURN_ASSIST_V     crawl speed during pure turns  (default 0.03)
#
# Keys:
#   Ctrl+C — stop immediately (always publishes rc 0 0)

import os
import sys
import time
import threading
import cv2 as cv
from datetime import datetime
from paho.mqtt import client as mqtt_client

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sgate_1 import gate1, gate1_ctrl


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def alive_loop(mqtt, alive_id, stop_event):
    """Send 'alive <id>' heartbeat every 0.5 s so the C++ teensy_interface
    keeps the master registered and does not emergency-stop the robot."""
    while not stop_event.is_set():
        mqtt.publish("robobot/cmd/ti", f"alive {alive_id}")
        stop_event.wait(0.5)


def make_mqtt_client(client_id):
    try:
        return mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, client_id=client_id)
    except Exception:
        return mqtt_client.Client(client_id)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test/test_gate_new.py <robot-ip> [mqtt-host]")
        sys.exit(1)

    robot_host = sys.argv[1]
    mqtt_host  = sys.argv[2] if len(sys.argv) >= 3 else "localhost"
    stream_url = f"http://{robot_host}:7123/stream.mjpg"

    print(f"% Connecting video stream: {stream_url}")
    cap = cv.VideoCapture(stream_url)
    if not cap.isOpened():
        print("% Error: could not open stream")
        sys.exit(1)

    print(f"% Connecting MQTT broker: {mqtt_host}:1883")
    mqtt = make_mqtt_client("gate1-new-drive-test")
    mqtt.connect(mqtt_host, 1883, 60)
    mqtt.loop_start()

    alive_id   = f"gate1-new-{int(time.time())}"
    stop_alive = threading.Event()
    alive_thread = threading.Thread(target=alive_loop, args=(mqtt, alive_id, stop_alive), daemon=True)
    alive_thread.start()

    gate1.setup()
    gate1_ctrl.reset()

    # Safe conservative defaults for first run.
    gate1_ctrl.min_v = 0.03
    gate1_ctrl.max_v = 0.08
    gate1_ctrl.max_w = 0.45

    # Optional tuning from environment variables.
    def _ef(name):
        v = os.getenv(name)
        return float(v) if v is not None else None

    if (v := _ef("GATE1_TARGET_X"))   is not None: gate1_ctrl.target_x_norm  = v
    if (v := _ef("GATE1_TARGET_W"))   is not None: gate1_ctrl.target_width_px = v
    if (v := _ef("GATE1_STOP_W"))     is not None: gate1_ctrl.stop_width_px   = v
    if (v := _ef("GATE1_MAX_V"))      is not None: gate1_ctrl.max_v           = v
    if (v := _ef("GATE1_MAX_W"))      is not None: gate1_ctrl.max_w           = v

    align_threshold     = float(os.getenv("GATE1_ALIGN_THRESHOLD",      "0.25"))
    align_w             = float(os.getenv("GATE1_ALIGN_W",              "0.45"))
    max_test_time_s     = float(os.getenv("GATE1_TEST_TIMEOUT_S",       "20"))
    stop_on_loss_frames = int(  os.getenv("GATE1_STOP_ON_LOSS_FRAMES",  "5"))
    turn_assist_w_min   = float(os.getenv("GATE1_TURN_ASSIST_W_MIN",    "0.12"))
    turn_assist_v       = float(os.getenv("GATE1_TURN_ASSIST_V",        "0.03"))

    print("% DRIVING ENABLED - keep robot lifted for first run if possible.")
    print(f"% target_x={gate1_ctrl.target_x_norm:+.3f}  target_w={gate1_ctrl.target_width_px:.1f}  stop_w={gate1_ctrl.stop_width_px:.1f}")
    print(f"% max_v={gate1_ctrl.max_v:.3f}  max_w={gate1_ctrl.max_w:.3f}  timeout={max_test_time_s:.1f}s")
    print(f"% align_threshold={align_threshold:.3f}  align_w={align_w:.3f} rad/s")
    print(f"% turn_assist: crawl v={turn_assist_v:.3f} when |w|>={turn_assist_w_min:.3f} (APPROACH/SEARCH only)")
    print("% Ctrl+C to stop immediately")

    start_t     = time.time()
    last_t      = start_t
    t_print     = start_t
    loss_frames = 0
    frame_count = 0
    last_frame  = None

    # Colour map for mode overlay.
    MODE_COLOR = {
        "ALIGN":    (0, 220, 220),   # yellow
        "APPROACH": (0, 200, 200),   # cyan
        "SEARCH":   (0, 80,  255),   # red-orange
    }

    try:
        while True:
            now = time.time()
            if now - start_t > max_test_time_s:
                print("% Timeout reached; stopping test")
                break

            ret, frame = cap.read()
            if not ret:
                print("% Lost stream")
                break
            frame_count += 1

            dt     = max(now - last_t, 1e-3)
            last_t = now

            gate1.detect(frame)
            v_cmd, w_cmd, pose = gate1_ctrl.command(gate1, dt=dt)

            # --- Mode selection ---
            if pose["valid"]:
                loss_frames = 0
                e_x = pose["lateral_error"]
                if abs(e_x) > align_threshold:
                    # Turn in place: pure rotation until aligned.
                    sign  = 1.0 if e_x > 0 else -1.0
                    v_cmd = 0.0
                    w_cmd = sign * align_w
                    mode  = "ALIGN"
                else:
                    mode = "APPROACH"
            else:
                loss_frames += 1
                mode = "SEARCH"

            # Deadman: stop if target missing for several consecutive frames.
            if loss_frames >= stop_on_loss_frames:
                v_cmd = 0.0
                w_cmd = 0.0

            # Turn assist: inject a tiny crawl so the drive stack can pivot/arc.
            # Skipped in ALIGN mode — pure rotation is intentional there.
            if mode != "ALIGN" and abs(v_cmd) < 1e-4 and abs(w_cmd) >= turn_assist_w_min:
                v_cmd = max(v_cmd, turn_assist_v)

            # Final safety clamps.
            v_cmd = clamp(v_cmd, 0.0, gate1_ctrl.max_v)
            w_cmd = clamp(w_cmd, -gate1_ctrl.max_w, gate1_ctrl.max_w)

            mqtt.publish("robobot/cmd/ti", f"rc {v_cmd:.3f} {w_cmd:.3f}")

            # --- Annotate frame ---
            gate1.paint(frame)
            color = MODE_COLOR.get(mode, (200, 200, 200))
            cv.putText(frame, f"[{mode}] v={v_cmd:+.3f} w={w_cmd:+.3f}",
                       (10, 58), cv.FONT_HERSHEY_PLAIN, 1.4, color, thickness=2)
            last_frame = frame

            # --- Terminal status at ~2 Hz ---
            if now - t_print >= 0.5:
                t_print = now
                if pose["valid"]:
                    print(f"% [{mode:<7}] Gate1 FOUND "
                          f"cx={gate1.bar_cx} w={gate1.bar_width_px}px "
                          f"e_x={pose['lateral_error']:+.3f} e_w={pose['depth_error']:+.3f} "
                          f"-> rc {v_cmd:+.3f} {w_cmd:+.3f}")
                else:
                    print(f"% [{mode:<7}] Gate1 not found ({loss_frames} lost) "
                          f"-> rc {v_cmd:+.3f} {w_cmd:+.3f}")

    except KeyboardInterrupt:
        pass
    finally:
        stop_alive.set()
        mqtt.publish("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.1)
        mqtt.loop_stop()
        mqtt.disconnect()
        cap.release()
        gate1.terminate()

    if last_frame is not None:
        ts = datetime.now().strftime("%Y_%b_%d_%H%M%S")
        fn = os.path.join(os.path.dirname(__file__), f"gate_new_{ts}_{frame_count:04d}.jpg")
        cv.imwrite(fn, last_frame)
        print(f"% Saved final frame to {fn}")

    print(f"% Done - processed {frame_count} frames  gate detections: {gate1.detCnt}")


if __name__ == "__main__":
    main()
