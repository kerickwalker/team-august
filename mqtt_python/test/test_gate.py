#!/usr/bin/env python3

# Standalone field test for all vision detection modules:
#   sgate.py  — orange gate uprights
#   svline.py — white line on ground
#   sball.py  — white golf ball
#   shole.py  — golf hole (dark circle)
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
import cv2 as cv
from datetime import datetime

# Allow importing modules from the parent directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sgate import gate
from svline import vline
from sball import ball
from shole import hole


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
    ball.setup()
    hole.setup()

    # Read one frame to confirm stream is live and print resolution
    ret, frame = cap.read()
    if not ret:
        print("% Error: stream opened but first frame failed")
        sys.exit(1)
    h, w = frame.shape[:2]
    print(f"% Stream live — {w}x{h}")

    save_dir = os.path.dirname(__file__)  # save into test/

    # Flush a few buffered frames so we get a fresh one
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()
    if not ret:
        print("% Error: could not read frame")
        cap.release()
        sys.exit(1)

    gate.detect(frame);  gate.paint(frame)
    vline.detect(frame); vline.paint(frame)
    ball.detect(frame);  ball.paint(frame)
    hole.detect(frame);  hole.paint(frame)

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
    if ball.detected:
        print(f"% Ball  FOUND  r={ball.radius}px  "
              f"cx={ball.cx}  steeringErr={ball.steeringError():+.3f}  "
              f"close={ball.isClose()}")
    else:
        print("% Ball  not found")
    if hole.detected:
        print(f"% Hole  FOUND  r={hole.radius}px  "
              f"cx={hole.cx}  steeringErr={hole.steeringError():+.3f}  "
              f"close={hole.isClose()}")
    else:
        print("% Hole  not found")

    ts = datetime.now().strftime('%Y_%b_%d_%H%M%S')
    fn = os.path.join(save_dir, f"vision_{ts}_0001.jpg")
    cv.imwrite(fn, frame)
    print(f"% Saved frame to {fn}")

    cap.release()
    gate.terminate()
    vline.terminate()
    ball.terminate()
    hole.terminate()
    print(f"% Done — gate={gate.detCnt}  line={vline.detCnt}  "
          f"ball={ball.detCnt}  hole={hole.detCnt}")


if __name__ == "__main__":
    main()
