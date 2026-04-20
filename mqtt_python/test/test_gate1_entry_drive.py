#!/usr/bin/env python3

import os
import sys
import time
import cv2 as cv
from datetime import datetime
from paho.mqtt import client as mqtt_client

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sgate_1 import gate1, gate1_ctrl


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def make_mqtt_client(client_id):
    """Create MQTT client compatible with both old and new paho APIs."""
    try:
        # Newer paho expects callback API version enum first.
        return mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, client_id=client_id)
    except Exception:
        # Older paho expects client_id as first positional argument.
        return mqtt_client.Client(client_id)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test/test_gate1_entry_drive.py <robot-ip> [mqtt-host]")
        sys.exit(1)

    robot_host = sys.argv[1]
    mqtt_host = sys.argv[2] if len(sys.argv) >= 3 else "localhost"
    stream_url = f"http://{robot_host}:7123/stream.mjpg"

    print(f"% Connecting video stream: {stream_url}")
    cap = cv.VideoCapture(stream_url)
    if not cap.isOpened():
        print("% Error: could not open stream")
        sys.exit(1)

    print(f"% Connecting MQTT broker: {mqtt_host}:1883")
    mqtt = make_mqtt_client("gate1-entry-drive-test")
    mqtt.connect(mqtt_host, 1883, 60)
    mqtt.loop_start()

    gate1.setup()
    gate1_ctrl.reset()

    # Safe defaults for first movement trial.
    gate1_ctrl.min_v = 0.03
    gate1_ctrl.max_v = 0.08
    gate1_ctrl.max_w = 0.45

    # Optional tuning from environment variables.
    tx = os.getenv("GATE1_TARGET_X")
    tw = os.getenv("GATE1_TARGET_W")
    sw = os.getenv("GATE1_STOP_W")
    mv = os.getenv("GATE1_MAX_V")
    mw = os.getenv("GATE1_MAX_W")
    if tx is not None:
        gate1_ctrl.target_x_norm = float(tx)
    if tw is not None:
        gate1_ctrl.target_width_px = float(tw)
    if sw is not None:
        gate1_ctrl.stop_width_px = float(sw)
    if mv is not None:
        gate1_ctrl.max_v = float(mv)
    if mw is not None:
        gate1_ctrl.max_w = float(mw)

    max_test_time_s = float(os.getenv("GATE1_TEST_TIMEOUT_S", "20"))
    stop_on_loss_frames = int(os.getenv("GATE1_STOP_ON_LOSS_FRAMES", "5"))
    turn_assist_w_min = float(os.getenv("GATE1_TURN_ASSIST_W_MIN", "0.12"))
    turn_assist_v = float(os.getenv("GATE1_TURN_ASSIST_V", "0.03"))

    print("% DRIVING ENABLED - keep robot lifted for first run if possible.")
    print("% Default calibration reference: good pose at cx~534, w~128 (unless env overrides)")
    print(f"% target_x={gate1_ctrl.target_x_norm:+.3f} target_w={gate1_ctrl.target_width_px:.1f} stop_w={gate1_ctrl.stop_width_px:.1f}")
    print(f"% max_v={gate1_ctrl.max_v:.3f} max_w={gate1_ctrl.max_w:.3f} timeout={max_test_time_s:.1f}s")
    print(f"% turn mode=crawl: assist_v={turn_assist_v:.3f} when |w|>={turn_assist_w_min:.3f} and v~0")
    print("% Ctrl+C to stop immediately")

    start_t = time.time()
    last_t = start_t
    t_print = start_t
    loss_frames = 0
    frame_count = 0
    last_frame = None

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

            dt = max(now - last_t, 1e-3)
            last_t = now

            gate1.detect(frame)
            v_cmd, w_cmd, pose = gate1_ctrl.command(gate1, dt=dt)

            if not pose["valid"]:
                loss_frames += 1
            else:
                loss_frames = 0

            # Deadman: stop if target is missing for several consecutive frames.
            if loss_frames >= stop_on_loss_frames:
                v_cmd = 0.0
                w_cmd = 0.0

            # Some drive stacks ignore pure turn commands with v=0.
            # Inject a tiny crawl speed so the robot can pivot/arc.
            if abs(v_cmd) < 1e-4 and abs(w_cmd) >= turn_assist_w_min:
                v_cmd = max(v_cmd, turn_assist_v)

            # Final safety clamps.
            v_cmd = clamp(v_cmd, 0.0, gate1_ctrl.max_v)
            w_cmd = clamp(w_cmd, -gate1_ctrl.max_w, gate1_ctrl.max_w)

            mqtt.publish("robobot/cmd/ti", f"rc {v_cmd:.3f} {w_cmd:.3f}")

            gate1.paint(frame)
            cv.putText(frame, f"DRIVE v={v_cmd:+.3f} w={w_cmd:+.3f}",
                       (10, 58), cv.FONT_HERSHEY_PLAIN, 1.4, (0, 200, 200), thickness=2)
            last_frame = frame

            if now - t_print >= 0.5:
                if pose["valid"]:
                    print("% Gate1 FOUND "
                          f"cx={gate1.bar_cx} w={gate1.bar_width_px}px "
                          f"e_x={pose['lateral_error']:+.3f} e_w={pose['depth_error']:+.3f} "
                          f"-> rc {v_cmd:+.3f} {w_cmd:+.3f}")
                else:
                    print(f"% Gate1 not found ({loss_frames} lost) -> rc {v_cmd:+.3f} {w_cmd:+.3f}")
                t_print = now

    except KeyboardInterrupt:
        pass
    finally:
        mqtt.publish("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.1)
        mqtt.loop_stop()
        mqtt.disconnect()
        cap.release()
        gate1.terminate()

    if last_frame is not None:
        ts = datetime.now().strftime("%Y_%b_%d_%H%M%S")
        fn = os.path.join(os.path.dirname(__file__), f"gate1_entry_drive_{ts}_{frame_count:04d}.jpg")
        cv.imwrite(fn, last_frame)
        print(f"% Saved final frame to {fn}")

    print(f"% Done - processed {frame_count} frames")


if __name__ == "__main__":
    main()
