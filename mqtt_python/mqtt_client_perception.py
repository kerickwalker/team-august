#!/usr/bin/env python3

from setproctitle import setproctitle
from uservice import service
from live_perception_overlay import perception_thread

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client already running - terminating")
    else:
        setproctitle("mqtt-client")
        service.setup("localhost")
        if service.connected:
            perception_thread(show_window=False)
        service.terminate()
    print("% Main Terminated")