#!/usr/bin/env python3

import os
import sys
import time
import cv2 as cv
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sgate_1 import gate1, gate1_ctrl


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test/test_gate1_entry.py <robot-ip>")
        sys.exit(1)

    host = sys.argv[1]
    url = f"http://{host}:7123/stream.mjpg"
    print(f"% Connecting to {url} ...")
    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% Error: could not open stream at {url}")
        sys.exit(1)

    gate1.setup()
    gate1_ctrl.reset()

    # Optional tuning from environment variables.
    tx = os.getenv("GATE1_TARGET_X")
    tw = os.getenv("GATE1_TARGET_W")
    sw = os.getenv("GATE1_STOP_W")
    if tx is not None:
        gate1_ctrl.target_x_norm = float(tx)
    if tw is not None:
        gate1_ctrl.target_width_px = float(tw)
    if sw is not None:
        gate1_ctrl.stop_width_px = float(sw)

    print("% Running gate1 entry controller (headless). Ctrl+C to quit")
    print("% Default calibration reference: good pose at cx~534, w~128 (unless env overrides)")
    print(f"% target_x_norm={gate1_ctrl.target_x_norm:+.3f}  "
          f"target_width_px={gate1_ctrl.target_width_px:.1f}  "
          f"stop_width_px={gate1_ctrl.stop_width_px:.1f}")

    frame_count = 0
    last_frame = None
    t_last = time.time()
    t_print = t_last

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("% Lost stream")
                break

            now = time.time()
            dt = now - t_last
            t_last = now

            frame_count += 1
            gate1.detect(frame)
            v_cmd, w_cmd, pose = gate1_ctrl.command(gate1, dt=max(dt, 1e-3))
            gate1.paint(frame)

            status = f"v={v_cmd:+.3f} w={w_cmd:+.3f}"
            cv.putText(frame, status, (10, 56), cv.FONT_HERSHEY_PLAIN, 1.4, (0, 200, 200), thickness=2)
            last_frame = frame

            if now - t_print >= 0.5:
                if pose["valid"]:
                    print(
                        "% Gate1 FOUND "
                        f"cx={gate1.bar_cx} w={gate1.bar_width_px}px h={gate1.bar_height_px}px "
                        f"e_x={pose['lateral_error']:+.3f} "
                        f"e_w={pose['depth_error']:+.3f} "
                        f"conf={pose['confidence']:.2f} -> "
                        f"v={v_cmd:+.3f} w={w_cmd:+.3f}"
                    )
                else:
                    print(f"% Gate1 not found -> v={v_cmd:+.3f} w={w_cmd:+.3f}")
                t_print = now

    except KeyboardInterrupt:
        pass

    if last_frame is not None:
        ts = datetime.now().strftime("%Y_%b_%d_%H%M%S")
        fn = os.path.join(os.path.dirname(__file__), f"gate1_entry_{ts}_{frame_count:04d}.jpg")
        cv.imwrite(fn, last_frame)
        print(f"% Saved final frame to {fn}")

    cap.release()
    gate1.terminate()
    print(f"% Done - {frame_count} frames processed")


if __name__ == "__main__":
    main()
