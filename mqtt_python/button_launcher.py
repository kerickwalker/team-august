#!/usr/bin/env python3
"""Press green button (GPIO 13) to launch mqtt-full-mission-simple.py -s -e --now"""
import subprocess
import time
import os
from sgpio import gpio

SCRIPT = ["python3", "mqtt-full-mission-simple.py", "-s", "-e", "--now"]
HERE = os.path.dirname(os.path.abspath(__file__))
POLL_S = 0.05   # polling interval
HOLD_S = 0.1    # button must be held this long to register (debounce)

print("% Button launcher ready — press green button (GPIO 13) to start mission")
proc = None

while True:
    if gpio.test_start_button():
        time.sleep(HOLD_S)
        if gpio.test_start_button():  # still held after debounce
            if proc is None or proc.poll() is not None:  # no mission running
                print("% Green button pressed — launching mission")
                proc = subprocess.Popen(SCRIPT, cwd=HERE)
            else:
                print("% Green button pressed — mission already running, ignoring")
            while gpio.test_start_button():  # wait for release
                time.sleep(POLL_S)
    time.sleep(POLL_S)
