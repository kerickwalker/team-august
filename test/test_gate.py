#!/usr/bin/env python3

# Standalone field test for gate detection (sgate.py).
# No MQTT or robot connection required — opens the camera stream directly.
#
# Usage:
#   python3 test/test_gate.py <robot-ip>
#   python3 test/test_gate.py 10.197.217.81
#
# Keys:
#   q — quit
#   s — save current annotated frame to test/

import sys
import os
import time
import cv2 as cv
from datetime import datetime

# Allow importing sgate from the parent directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sgate import gate


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

    # Read one frame to confirm stream is live and print resolution
    ret, frame = cap.read()
    if not ret:
        print("% Error: stream opened but first frame failed")
        sys.exit(1)
    h, w = frame.shape[:2]
    print(f"% Stream live — {w}x{h}")
    print("% Press q to quit, s to save current frame")

    save_dir = os.path.dirname(__file__)  # save into test/
    frame_count = 0
    t_last_print = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("% Lost stream")
            break

        frame_count += 1
        gate.detect(frame)
        gate.paint(frame)

        # Print detection status to terminal at ~2 Hz
        now = time.time()
        if now - t_last_print >= 0.5:
            if gate.detected:
                print(f"% Gate FOUND  area={gate.gate_area:.0f}px  "
                      f"cx={gate.gate_cx}  steeringErr={gate.steeringError():+.3f}")
            else:
                print("% Gate not found")
            t_last_print = now

        cv.imshow("Gate detection test  (q=quit  s=save)", frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            ts  = datetime.now().strftime('%Y_%b_%d_%H%M%S')
            fn  = os.path.join(save_dir, f"gate_{ts}_{frame_count:04d}.jpg")
            cv.imwrite(fn, frame)
            print(f"% Saved {fn}")

    cap.release()
    cv.destroyAllWindows()
    gate.terminate()
    print(f"% Done — {frame_count} frames processed, {gate.detCnt} with gate detected")


if __name__ == "__main__":
    main()
