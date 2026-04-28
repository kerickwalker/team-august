#!/usr/bin/env python3
"""Interactive creep-forward tester.

Usage:
    python3 -m control.tasks.creep_test [robot-ip]

Prompts for distance (m) and speed (m/s); drives that far on tripB and stops.
Type 'q' to quit. Stdout/stderr from the MQTT stack are silenced so the
prompt stays usable; status lines are written directly to /dev/tty.
"""
from __future__ import annotations
import os
import sys
import time as t

DEFAULT_IP = "10.197.219.117"


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    sys.argv = ["creep_test", "-i", ip, "--allow-no-hbt", "-s"]

    from util.uservice import service
    from sensors.spose import pose

    service.setup(ip)
    t.sleep(1.5)

    tty = open("/dev/tty", "w")
    def say(msg: str) -> None:
        tty.write(msg + "\n")
        tty.flush()

    devnull = open(os.devnull, "w")
    sys.stdout = devnull
    # keep stderr on the terminal so exceptions are visible

    say(f"\n[creep] connected to {ip}.  'q' to quit. format: <dist_m> [speed_m/s]")
    say(f"[creep] service.stop={service.stop}  pose.x={pose.x:.2f} y={pose.y:.2f}\n")

    while True:
        tty.write("dist speed > ")
        tty.flush()
        try:
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            break
        except Exception as exc:
            say(f"  ! input error: {exc!r}")
            break
        if not line or line.lower() in ("q", "quit", "exit"):
            break

        parts = line.split()
        try:
            dist = float(parts[0])
            speed = float(parts[1]) if len(parts) > 1 else 0.05
        except (ValueError, IndexError):
            say("  ! enter:  <dist_m> [speed_m/s]   e.g.  0.08 0.05")
            continue

        if abs(dist) > 0.5 or abs(speed) > 0.2:
            say("  ! safety cap: |dist|<=0.5 m, |speed|<=0.2 m/s")
            continue

        if service.stop:
            say("  ! service.stop is True — uservice signaled shutdown; aborting")
            break

        try:
            pose.tripBreset()
        except Exception as exc:
            say(f"  ! tripBreset failed: {exc!r}")
            continue
        x0, y0, h0 = pose.x, pose.y, pose.h
        say(f"  start: x={x0:+.3f} y={y0:+.3f} yaw_deg={h0*57.3:+.1f}")

        timeout_s = abs(dist) / max(abs(speed), 0.01) * 3.0 + 3.0
        t0 = t.time()
        last_log = t0
        service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.000")
        try:
            while not service.stop and pose.tripB < abs(dist):
                t.sleep(0.05)
                service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.000")
                now = t.time()
                if now - last_log > 0.5:
                    say(f"  ... tripB={pose.tripB:.3f}  x={pose.x:+.3f} y={pose.y:+.3f}")
                    last_log = now
                if now - t0 > timeout_s:
                    say(f"  ! timeout {timeout_s:.1f}s — stopping")
                    break
        except KeyboardInterrupt:
            pass
        service.send("robobot/cmd/ti", "rc 0.000 0.000")
        t.sleep(0.3)

        dx, dy = pose.x - x0, pose.y - y0
        dh_deg = (pose.h - h0) * 57.3
        say(f"  end:   x={pose.x:+.3f} y={pose.y:+.3f} yaw_deg={pose.h*57.3:+.1f}")
        say(f"  delta: dx={dx:+.3f} dy={dy:+.3f} dyaw={dh_deg:+.1f}  tripB={pose.tripB:.3f} m\n")

    service.send("robobot/cmd/ti", "rc 0.000 0.000")
    service.terminate()
    say("[creep] done.")


if __name__ == "__main__":
    main()
