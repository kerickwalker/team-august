#!/usr/bin/env python3

# Standalone field test for gate detection (sgate.py) and vision-based
# white line detection (svline.py).
# No MQTT or robot connection required — opens the camera stream directly.
# Runs headless — the final annotated frame is saved to test/ on exit.
#
# Usage:
#   python3 test/test_gate.py <robot-ip>
#   python3 test/test_gate.py 10.197.217.81
#
# Keys:
#   Ctrl+C — quit

import sys
import os
import time
import cv2 as cv
from datetime import datetime

# Allow importing modules from the parent directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sgate import gate
from svline import vline


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test/test_gate.py <robot-ip>")
        sys.exit(1)

    host = sys.argv[1]
    url  = f"http://{host}:7123/stream.mjpg"
    print(f"% Connecting to {url} ...")

    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% Error: could not open stream at {url}")
        sys.exit(1)

    gate.setup()
    vline.setup()

    # Read one frame to confirm stream is live and print resolution
    ret, frame = cap.read()
    if not ret:
        print("% Error: stream opened but first frame failed")
        sys.exit(1)
    h, w = frame.shape[:2]
    print(f"% Stream live — {w}x{h}")
    print("% Running headless — press Ctrl+C to stop and save final frame")

    save_dir     = os.path.dirname(__file__)  # save into test/
    frame_count  = 0
    last_frame   = None
    t_last_print = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("% Lost stream")
                break

            frame_count += 1
            gate.detect(frame)
            gate.paint(frame)
            vline.detect(frame)
            vline.paint(frame)
            last_frame = frame

            # Print detection status to terminal at ~2 Hz
            now = time.time()
            if now - t_last_print >= 0.5:
                if gate.detected:
                    print(f"% Gate  FOUND  width={gate.gate_width_px}px  "
                          f"cx={gate.gate_cx}  steeringErr={gate.steeringError():+.3f}")
                else:
                    print("% Gate  not found")
                if vline.lineValid:
                    print(f"% Line  FOUND  offset={vline.lineOffset:+.3f}  "
                          f"conf={vline.lineValidCnt}/20")
                else:
                    print(f"% Line  not found  conf={vline.lineValidCnt}/20")
                t_last_print = now

    except KeyboardInterrupt:
        pass

    # Save the final annotated frame
    if last_frame is not None:
        ts = datetime.now().strftime('%Y_%b_%d_%H%M%S')
        fn = os.path.join(save_dir, f"gate_{ts}_{frame_count:04d}.jpg")
        cv.imwrite(fn, last_frame)
        print(f"% Saved final frame to {fn}")

    cap.release()
    gate.terminate()
    vline.terminate()
    print(f"% Done — {frame_count} frames processed")
    print(f"%         gate detections: {gate.detCnt}  line detections: {vline.detCnt}")


if __name__ == "__main__":
    main()
